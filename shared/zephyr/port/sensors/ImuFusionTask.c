/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Zephyr-owned IMU task.  The old board InsTask.c files are deliberately not
 * linked: they rely on HAL DMA callbacks and FreeRTOS notifications.
 */
#include "InsTask.h"
#include "Ahrs.h"
#include "RobotTargetConfig.h"

#ifndef ROBOT_EXTERNAL_IMU_HI14
#define ROBOT_EXTERNAL_IMU_HI14 0
#endif

#if ROBOT_EXTERNAL_IMU_HI14
#include "BspExternalImuUart.h"
#include "Hi14Parser.h"
#else
#include "Bmi088Driver.h"
#include "BspBmi088Port.h"
#include "CalibrateTask.h"
#include "ControlInput.h"
#include "GyroZeroCali.h"
#include "ImuFrame.h"
#include "ManualInputSnapshot.h"
#include "Mpu6500.h"
#include "RobotMode.h"
#include "UserLib.h"
#include "ImuAttitude.h"
#include "ImuCalStore.h"
#if defined(CONFIG_BOARD_DJI_C_F407)
#include "BspIst8310Port.h"
#include "Ist8310Driver.h"
#include "Pid.h"
static PidTypeDef InsTempPid;
static uint8_t InsTempPidReady, InsHeaterMode;
static uint16_t InsWarmSamples;
static uint8_t InsMagReady;
static uint32_t InsMagTick;
#endif
#if defined(CONFIG_BOARD_DJI_A_F427)
#include "ImuThermalA.h"
static ImuThermalA InsThermal;
#endif
#endif

#include "BspImuPwm.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "SdLog.h"
#if ROBOT_TASK_BUILD_HEALTH_MONITOR || defined(CONFIG_ARBATOS_MINI_BENCH)
#include "DetectTask.h"
#endif

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_ARBATOS_BENCH_TELEMETRY)
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_bench, LOG_LEVEL_INF);
#endif

#define IMU_DT_FALLBACK_S 0.001f
#define IMU_DT_MIN_S 0.0002f
#define IMU_DT_MAX_S 0.0200f
#define IMU_G 9.80665f
#define IMU_DEG_TO_RAD 0.01745329251994329577f

fp32 INS_angle[3];
static fp32 InsQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static fp32 InsGyro[3];
static fp32 InsAccel[3];
static fp32 InsMag[3];
static fp32 InsTemp;
static uint16_t InsHeaterPwm;
static fp32 InsHeaterPidOut;
#if !ROBOT_EXTERNAL_IMU_HI14
static ImuAttitude InsAttitude;
static fp32 InsGyroOffset[3];
static GyroZeroCaliRuntimeState InsGyroCaliState;
static uint8_t InsHeaterStable;
static uint8_t InsGyroApplyDelta;
#endif

__weak int8_t get_control_temperature(void)
{
#if defined(CONFIG_BOARD_DJI_A_F427) && !ROBOT_EXTERNAL_IMU_HI14
    return (int8_t)IMU_A_TARGET_C;
#else
    return 40;
#endif
}

#define INS_quat InsQuat
#define INS_gyro InsGyro
#define INS_accel InsAccel
#define INS_mag InsMag
#include "InsSnapshotStore.inc"
#undef INS_mag
#undef INS_accel
#undef INS_gyro
#undef INS_quat

#if ROBOT_EXTERNAL_IMU_HI14
static void ImuEulerUpdate(void)
{
    get_angle(InsQuat, &INS_angle[0], &INS_angle[2], &INS_angle[1]);
}
#endif

#if !ROBOT_EXTERNAL_IMU_HI14
static void ImuRotateVector(fp32 rotated[3], const fp32 raw[3])
{
    ImuFrameRotate(rotated, raw);
}

static float ImuSampleDt(void)
{
    static uint32_t last_cycles;
    const uint32_t now_cycles = k_cycle_get_32();

    if (last_cycles == 0u)
    {
        last_cycles = now_cycles;
        return IMU_DT_FALLBACK_S;
    }

    const uint32_t elapsed_cycles = now_cycles - last_cycles;
    last_cycles = now_cycles;
    const float dt = (float)k_cyc_to_ns_floor64(elapsed_cycles) * 1.0e-9f;
    return (dt >= IMU_DT_MIN_S && dt <= IMU_DT_MAX_S) ? dt : IMU_DT_FALLBACK_S;
}

static void ImuHeaterUpdate(float temp)
{
    if (ImuCalStoreIsWriting()) {
        InsHeaterPwm = 0u;
        InsHeaterStable = 0u;
        imu_pwm_set(0u);
        return;
    }
#if defined(CONFIG_BOARD_DJI_A_F427)
    ImuThermalAUpdate(&InsThermal, temp, k_uptime_get_32(), g_config.imu.imu_temp_pwm_max);
    InsHeaterPwm = InsThermal.pwm;
    InsHeaterPidOut = InsHeaterPwm;
    InsHeaterStable = InsThermal.stable;
    imu_pwm_set(InsHeaterPwm);
    if (imu_pwm_last_error() != 0) InsHeaterStable = 0u;
    return;
#endif
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    extern volatile uint32_t MPreflightHeatEnable;
    if (MPreflightHeatEnable == 0u) {
        InsHeaterPwm = 0u;
        InsHeaterStable = 0u;
        imu_pwm_set(0u);
        return;
    }
#endif
    if (!isfinite(temp) || temp < -40.0f || temp > 60.0f) {
        InsHeaterPwm = 0u;
        InsHeaterPidOut = 0.0f;
        InsHeaterStable = 0u;
        imu_pwm_set(0u);
        return;
    }
    const float target = get_control_temperature();
    const float max = g_config.imu.imu_temp_pwm_max;
    const float err = target - temp;
#if defined(CONFIG_BOARD_DM_MC02_H7)
    /* MC02 电阻直接接输入电源。24 V 实测原始大占空比会严重过冲。
     * 使用 100 ms 温控周期、慢积分及 2% 硬上限，保留原配置更小的限制。 */
    static uint32_t lastUpdate;
    static float integral;
    uint32_t now = k_uptime_get_32();
    if (now - lastUpdate >= 100u) {
        float dt = fminf((now - lastUpdate) * 0.001f, 0.2f);
        float limit = fminf(max, 200.0f);
        lastUpdate = now;
        float candidate = integral + err * 2.0f * dt;
        if (candidate < 0) candidate = 0;
        if (candidate > limit) candidate = limit;
        if (err <= 0 || 20.0f * err + candidate < limit) integral = candidate;
        InsHeaterPidOut = fminf(fmaxf(20.0f * err + integral, 0.0f), limit);
        if (temp >= target + 1.0f) { InsHeaterPidOut = 0; integral = 0; }
        InsHeaterPwm = (uint16_t)InsHeaterPidOut;
    }
#elif defined(CONFIG_BOARD_DJI_C_F407)
    if (!InsTempPidReady) {
        const float gains[3] = {g_config.imu.temperature_pid.kp,
                               g_config.imu.temperature_pid.ki,
                               g_config.imu.temperature_pid.kd};
        PID_init(&InsTempPid, PID_POSITION, gains,
                 g_config.imu.temperature_pid_max_out, g_config.imu.temperature_pid_max_iout);
        InsTempPidReady = 1u;
    }
    /* C板沿用预热→PID稳温；所有阶段仍服从配置及板级PWM上限。
     * M板24V加热器和A板的专用稳温保护保留各自实现。 */
    if (!InsHeaterMode) {
        InsWarmSamples = temp > target ? InsWarmSamples + 1u : 0u;
        if (InsWarmSamples > 200u) {
            InsHeaterMode = 1u;
            InsTempPid.Iout = fminf(max * 0.5f, g_config.imu.temperature_pid_max_iout);
        }
    }
    InsHeaterPidOut = InsHeaterMode ? PID_calc(&InsTempPid, temp, target) : max;
    InsHeaterPwm = (uint16_t)fminf(fmaxf(InsHeaterPidOut, 0.0f), max);
#else
    InsHeaterPidOut = err * g_config.imu.temperature_pid.kp;
    if (InsHeaterPidOut < 0.0f) InsHeaterPidOut = 0.0f;
    if (InsHeaterPidOut > max) InsHeaterPidOut = max;
    InsHeaterPwm = (uint16_t)InsHeaterPidOut;
#endif
    InsHeaterStable = (fabsf(err) <= GYRO_ZERO_CALI_TEMP_ERR_C) ? 1u : 0u;
#if defined(CONFIG_BOARD_DM_MC02_H7)
    static uint32_t stableWindowMs;
    static float lowTemp = 100.0f, highTemp = -100.0f;
    static uint8_t slowTemperature;
    lowTemp = fminf(lowTemp, temp);
    highTemp = fmaxf(highTemp, temp);
    if (now - stableWindowMs >= 2000u) {
        slowTemperature = (highTemp - lowTemp <= 0.5f);
        lowTemp = highTemp = temp;
        stableWindowMs = now;
    }
    /* 温度经过目标附近但仍在快速升降时，不能启动零偏采样。 */
    InsHeaterStable = InsHeaterStable && slowTemperature;
#endif
    imu_pwm_set(InsHeaterPwm);
}

#if !defined(CONFIG_BOARD_DJI_A_F427)
static int ImuReadBmi(fp32 gyro_raw[3], fp32 accel_raw[3], fp32 *temp)
{
    uint32_t errors = Bmi088PortErrorCount();
    BMI088_read(gyro_raw, accel_raw, temp);
    bmi088_diag_t diag;
    BMI088_get_diag(&diag);
    return diag.gyro_read_ok != 0u && Bmi088PortErrorCount() == errors ? 0 : -1;
}
#endif

#if defined(CONFIG_BOARD_DJI_A_F427)
static int ImuReadMpu(fp32 gyro_raw[3], fp32 accel_raw[3], fp32 *temp)
{
    mpu6500_raw_t raw;
    if (mpu6500_read_raw(&raw) != 0) return -1;
    for (int i = 0; i < 3; ++i) {
        gyro_raw[i] = (fp32)raw.gyro[i] * (IMU_DEG_TO_RAD / 32.8f);
        accel_raw[i] = (fp32)raw.accel[i] * (IMU_G / 4096.0f);
    }
    *temp = mpu6500_temp_c(raw.temp);
    return 0;
}
#endif

static void ImuApplyGyroOffset(const fp32 offset[3])
{
    if (offset == NULL)
    {
        return;
    }

    for (int i = 0; i < 3; ++i)
    {
        InsGyroOffset[i] = offset[i];
    }
}

__weak bool_t CalibrateGyroOffsetSave(const fp32 offset[3])
{
    bool_t saved = ImuCalStoreSave(offset, InsTemp) == 0;
    if (saved) ImuApplyGyroOffset(offset);
    return saved;
}

static void ImuApplyGyroOffsetCallback(const fp32 offset[3], void *ctx)
{
    ARG_UNUSED(ctx);
    if (InsGyroApplyDelta) {
        for (unsigned i = 0; i < 3; ++i) InsGyroOffset[i] += offset[i];
    } else {
        ImuApplyGyroOffset(offset);
    }
}

static void ImuCopyCorrected(fp32 rotated[3], const fp32 corrected[3])
{
    memcpy(rotated, corrected, 3u * sizeof(fp32));
}

static void ImuCalibrationSampleDone(const GyroZeroCaliSampleState *samples, void *ctx)
{
    ARG_UNUSED(ctx);
    ImuAttitudeSetAccRef(&InsAttitude, GyroZeroCaliAccelNormAvg(samples, IMU_G));
}

static bool_t ImuSaveGyroOffsetCallback(const fp32 offset[3], void *ctx)
{
    ARG_UNUSED(ctx);
    return CalibrateGyroOffsetSave(offset);
}

static uint8_t ImuBootAdjustAllowed(void)
{
    ManualInputSnapshot input;
    if (ManualInputSnapshotRead(&input) == 0u || input.online == 0u)
    {
        return 1u;
    }

    const uint8_t gimbal_safe = ControlInputSwitchIsPos(
        input.control.sw[INPUT_SW_GIMBAL_MODE], input.semantics.GimbalSafePos);
    const uint8_t chassis_safe = ControlInputSwitchIsPos(
        input.control.sw[INPUT_SW_CHASSIS_MODE], input.semantics.ChassisSafePos);
    return (gimbal_safe != 0u && chassis_safe != 0u) ? 1u : 0u;
}

#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
#include "ImuService.inc"
#endif

static void ImuGyroCalibrationUpdate(const fp32 gyro_raw[3],
                                     const fp32 accel_raw[3],
                                     uint32_t now_ms)
{
    const uint8_t test = robot_mode_is_calibration(ROBOT_CALI_TARGET_IMU_GYRO);
    InsGyroApplyDelta = !test;
    const GyroZeroCaliRuntimeCfg cfg = {
        .test_mode_active = test,
        .temp_c = InsTemp,
        .target_temp_c = (fp32)get_control_temperature(),
        .heater_stable = InsHeaterStable,
        .boot_adjust_allowed = ImuBootAdjustAllowed(),
        .now_ms = now_ms,
        .rotate_gyro = test ? ImuRotateVector : ImuCopyCorrected,
        .rotate_accel = ImuRotateVector,
        .apply_offset = ImuApplyGyroOffsetCallback,
        .save_offset = ImuSaveGyroOffsetCallback,
        .sample_done = ImuCalibrationSampleDone,
        .ctx = NULL,
    };
    GyroZeroCaliRuntimeUpdate(&InsGyroCaliState, &cfg, test ? gyro_raw : InsGyro, accel_raw);
    InsGyroApplyDelta = 0u;
}

#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_SDLOG
static struct {
    sdlog_imu_base_stream_header_t header;
    sdlog_imu_base_sample_t samples[16];
} InsLogStream;
static uint32_t InsLogLastTick;
static uint16_t InsLogDivCount;

static void ImuLogFlush(void)
{
    if (InsLogStream.header.sample_count != 0u) {
        const uint16_t size = sizeof(InsLogStream.header) +
            InsLogStream.header.sample_count * sizeof(InsLogStream.samples[0]);
        SdLogWrite(SDLOG_TAG_IMU_BASE_STREAM, &InsLogStream, size);
        InsLogStream.header.sample_count = 0u;
    }
}

static void ImuLogSample(uint32_t now, float dt)
{
    const uint8_t div = SdLogHighRateDiv();
    if (div > 1u && InsLogDivCount++ % div != 0u) return;
    const uint32_t period = (uint32_t)(dt * 1000000.0f + 0.5f) * (div ? div : 1u);
    if (InsLogStream.header.sample_count &&
        (period != InsLogStream.header.period_us ||
         now - InsLogLastTick != (period + 999u) / 1000u)) {
        /* 实际周期变化就分包，不能把丢样伪装成等间隔采样。 */
        ImuLogFlush();
    }
    if (InsLogStream.header.sample_count == 0u) {
        InsLogStream.header.start_tick_ms = now;
        InsLogStream.header.period_us = period > UINT16_MAX ? UINT16_MAX : period;
        InsLogStream.header.version = SDLOG_IMU_BASE_STREAM_VERSION;
    }
    SdLogImuBaseSampleSet(&InsLogStream.samples[InsLogStream.header.sample_count++],
                         InsQuat, InsGyro, InsAccel, InsTemp);
    InsLogLastTick = now;
    if (InsLogStream.header.sample_count == 16u) ImuLogFlush();
}

static void ImuLogTrust(uint32_t now)
{
    static uint32_t last;
    if (now - last < 10u) return;
    last = now;
    const ImuAccTrust *acc = &InsAttitude.acc;
    const sdlog_imu_trust_t trust = {
        .acc_norm_g = acc->accNormG, .acc_ref_g = acc->accRefG,
        .norm_err_g = acc->normErrG, .angle_deg = acc->angleDeg,
        .trust = acc->trust, .kp_gain = acc->kpGain,
        .gyro_norm_dps = sqrtf(InsGyro[0] * InsGyro[0] + InsGyro[1] * InsGyro[1] +
                               InsGyro[2] * InsGyro[2]) / IMU_DEG_TO_RAD,
        .acc_healthy = acc->healthy, .acc_rejected = acc->rejected,
        .fusion_mode = InsAttitude.mode,
    };
    SdLogWrite(SDLOG_TAG_IMU_TRUST, &trust, sizeof(trust));
    const sdlog_pid_runtime_t temperature = {
        .PidId = SDLOG_PID_IMU_TEMP, .mode = 0u,
        .set = get_control_temperature(), .fdb = InsTemp, .out = InsHeaterPidOut,
    };
    SdLogWrite(SDLOG_TAG_PID, &temperature, sizeof(temperature));
}
#endif
#endif

#if ROBOT_EXTERNAL_IMU_HI14
#define HI14_LINK_BAUD 115200u
#define HI14_DMA_RX_SIZE 256u

static struct k_spinlock Hi14Lock;
static Hi14ByteRing Hi14RxRing;
static Hi14Parser Hi14TaskParser;
static Hi14SampleGate Hi14TaskGate;
static uint8_t Hi14DmaRx[HI14_DMA_RX_SIZE];
static uint8_t Hi14TaskBytes[64];
static uint32_t Hi14TaskReceiveTicksMs[64];
static uint16_t Hi14DmaLastPos;
static volatile uint8_t Hi14LinkActive;
static volatile uint8_t Hi14DmaActive;

static void Hi14LinkOnRxEvent(uint16_t size, BspAuxLinkRxEvent event)
{
    if (Hi14LinkActive == 0u || Hi14DmaActive == 0u)
    {
        return;
    }
    if (size > HI14_DMA_RX_SIZE)
    {
        k_spinlock_key_t invalid_key = k_spin_lock(&Hi14Lock);
        Hi14ByteRingDiscard(&Hi14RxRing);
        Hi14DmaLastPos = 0u;
        k_spin_unlock(&Hi14Lock, invalid_key);
        return;
    }
    /* Zephyr 串口在满缓冲事件后可能再报告一次 size==capacity 的 IDLE。 */
    if (event == BSP_AUX_LINK_RXEVENT_IDLE && size == HI14_DMA_RX_SIZE)
    {
        return;
    }

    const uint32_t receive_tick_ms = k_uptime_get_32();
    k_spinlock_key_t key = k_spin_lock(&Hi14Lock);
    uint16_t begin = Hi14DmaLastPos;
    if (size < begin)
    {
        Hi14ByteRingDiscard(&Hi14RxRing);
        begin = 0u;
    }
    if (size > begin)
    {
        (void)Hi14ByteRingPush(&Hi14RxRing,
                               &Hi14DmaRx[begin],
                               (size_t)(size - begin),
                               receive_tick_ms);
    }
    Hi14DmaLastPos = (event == BSP_AUX_LINK_RXEVENT_TC || size == HI14_DMA_RX_SIZE)
                            ? 0u
                            : size;
    k_spin_unlock(&Hi14Lock, key);
}

static void Hi14LinkOnRxByte(uint8_t byte)
{
    if (Hi14LinkActive == 0u || Hi14DmaActive != 0u)
    {
        return;
    }
    const uint32_t receive_tick_ms = k_uptime_get_32();
    k_spinlock_key_t key = k_spin_lock(&Hi14Lock);
    (void)Hi14ByteRingPush(&Hi14RxRing, &byte, 1u, receive_tick_ms);
    k_spin_unlock(&Hi14Lock, key);
}

static uint8_t Hi14LinkOnError(void)
{
    k_spinlock_key_t key = k_spin_lock(&Hi14Lock);
    Hi14ByteRingDiscard(&Hi14RxRing);
    Hi14DmaLastPos = 0u;
    k_spin_unlock(&Hi14Lock, key);
    /* 返回 0，让串口端口执行原有的接收重启。 */
    return 0u;
}

static int Hi14LinkStart(void)
{
    const uint8_t dma_available = BspExternalImuLinkRxHasDma();
    if (BspExternalImuLinkGetBaudrate() != HI14_LINK_BAUD)
    {
        return -1;
    }

    Hi14LinkActive = 0u;
    Hi14DmaActive = 0u;
    Hi14DmaLastPos = 0u;
    if (dma_available != 0u &&
        BspExternalImuLinkRxToIdleDmaStart(Hi14DmaRx, HI14_DMA_RX_SIZE) == 0)
    {
        Hi14DmaActive = 1u;
        Hi14LinkActive = 1u;
        return 0;
    }
    if (BspExternalImuLinkRxItStart() == 0)
    {
        Hi14LinkActive = 1u;
        return 0;
    }
    return -1;
}

static size_t Hi14TakeBytes(uint8_t *bytes,
                            uint32_t *receive_ticks_ms,
                            size_t capacity,
                            uint32_t *epoch)
{
    k_spinlock_key_t key = k_spin_lock(&Hi14Lock);
    const size_t count = Hi14ByteRingPop(&Hi14RxRing,
                                         bytes,
                                         receive_ticks_ms,
                                         capacity,
                                         epoch);
    k_spin_unlock(&Hi14Lock, key);
    return count;
}

static uint32_t Hi14RingEpoch(void)
{
    k_spinlock_key_t key = k_spin_lock(&Hi14Lock);
    const uint32_t epoch = Hi14RxRing.epoch;
    k_spin_unlock(&Hi14Lock, key);
    return epoch;
}

static void Hi14Publish(const Hi14Sample *sample, uint32_t now_ms)
{
    /*
     * 外置 HI14 使用模块默认正装坐标。它已经输出 body->world 的 WXYZ
     * 四元数，不能再套用 HERO 板载 IMU 的 ImuFrame 安装旋转。
     */
    for (int i = 0; i < 3; ++i)
    {
        InsAccel[i] = sample->accel_g[i] * IMU_G;
        InsGyro[i] = sample->gyro_dps[i] * IMU_DEG_TO_RAD;
        InsMag[i] = sample->mag_ut[i];
    }
    memcpy(InsQuat, sample->quat, sizeof(InsQuat));
    InsTemp = (fp32)sample->temperature_c;
    ImuEulerUpdate();
    InsSnapshotPublishCurrent(now_ms, InsTemp);

#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    static uint32_t last_log_ms;
    if (now_ms - last_log_ms >= 10u)
    {
        sdlog_imu_t log_sample;
        memcpy(log_sample.quat, InsQuat, sizeof(log_sample.quat));
        memcpy(log_sample.gyro, InsGyro, sizeof(log_sample.gyro));
        memcpy(log_sample.accel, InsAccel, sizeof(log_sample.accel));
        log_sample.temp = InsTemp;
        SdLogWrite(SDLOG_TAG_IMU, &log_sample, sizeof(log_sample));
        last_log_ms = now_ms;
    }
#endif
}
#endif

void ImuFusionTask(void const *pvParameters)
{
#if ROBOT_EXTERNAL_IMU_HI14
    ARG_UNUSED(pvParameters);
    WatchImuSetStage(WATCH_IMU_STAGE_ENTER);
    WatchTaskWait(WATCH_TASK_IMU);
    k_msleep(g_config.imu.task_init_time_ms);
    WatchImuSetStage(WATCH_IMU_STAGE_INIT_DELAY_DONE);

    /* 外置姿态源不使用板载加热和零偏保存。 */
    InsHeaterPwm = 0u;
    InsHeaterPidOut = 0.0f;
    imu_pwm_set(0u);
    Hi14ByteRingInit(&Hi14RxRing);
    BspExternalImuLinkSetRxEventCb(Hi14LinkOnRxEvent);
    BspExternalImuLinkSetRxByteCb(Hi14LinkOnRxByte);
    BspExternalImuLinkSetErrorCb(Hi14LinkOnError);
    while (Hi14LinkStart() != 0)
    {
        WatchTaskError(WATCH_TASK_IMU);
        k_msleep(100);
    }
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_OK);

    uint32_t parser_epoch = 0u;
    Hi14ParserInit(&Hi14TaskParser);
    Hi14SampleGateInit(&Hi14TaskGate);
    for (;;)
    {
        uint32_t batch_epoch = parser_epoch;
        const size_t count = Hi14TakeBytes(Hi14TaskBytes,
                                           Hi14TaskReceiveTicksMs,
                                           sizeof(Hi14TaskBytes),
                                           &batch_epoch);
        if (batch_epoch != parser_epoch)
        {
            Hi14ParserInit(&Hi14TaskParser);
            parser_epoch = batch_epoch;
        }

        Hi14Sample sample;
        uint32_t sample_receive_tick_ms = 0u;
        uint8_t sample_ready = 0u;
        for (size_t i = 0u; i < count; ++i)
        {
            if (Hi14ParserFeedByte(&Hi14TaskParser, Hi14TaskBytes[i], &sample))
            {
                sample_receive_tick_ms = Hi14TaskReceiveTicksMs[i];
                sample_ready = 1u;
            }
        }

        const uint32_t current_epoch = Hi14RingEpoch();
        if (current_epoch != batch_epoch)
        {
            /* 环形缓冲出现缺口后，丢弃本批结果并从新字节重新找帧头。 */
            Hi14ParserInit(&Hi14TaskParser);
            parser_epoch = current_epoch;
            sample_ready = 0u;
        }
        if (sample_ready != 0u && Hi14SampleGateAccept(&Hi14TaskGate, &sample))
        {
            Hi14Publish(&sample, sample_receive_tick_ms);
            WatchImuSetStage(WATCH_IMU_STAGE_FUSION_LOOP);
        }
        /* 无新帧时不重复发布，快照年龄会自然反映链路超时。 */
        WatchTaskBeat(WATCH_TASK_IMU);
        k_sleep(K_MSEC(1));
    }
#else
    int init;
    ARG_UNUSED(pvParameters);
    WatchImuSetStage(WATCH_IMU_STAGE_ENTER);
    WatchTaskWait(WATCH_TASK_IMU);
    k_msleep(g_config.imu.task_init_time_ms);
    WatchImuSetStage(WATCH_IMU_STAGE_INIT_DELAY_DONE);

#if defined(CONFIG_BOARD_DJI_A_F427)
    init = mpu6500_init();
#else
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_TRY);
    init = BMI088_init();
#endif
    while (init != 0) {
        WatchTaskError(WATCH_TASK_IMU);
        k_msleep(100);
#if defined(CONFIG_BOARD_DJI_A_F427)
        init = mpu6500_init();
#else
        WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_RETRY);
        init = BMI088_init();
#endif
    }
    ImuAttitudeInit(&InsAttitude);
    GyroZeroCaliRuntimeReset(&InsGyroCaliState);
    imu_pwm_set(0u);
    (void)ImuCalStoreLoad(InsGyroOffset);
#if defined(CONFIG_BOARD_DJI_C_F407)
    /* 六轴工作不因磁力计缺失而停住；九轴只使用读成功且未过期的磁场。 */
    InsMagReady = ist8310_init() == IST8310_NO_ERROR;
#endif
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    uint8_t offsetQueued = 0u;
#endif
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_OK);

    for (;;) {
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
        ImuServiceModeUpdate(robot_mode_is_calibration(ROBOT_CALI_TARGET_IMU_GYRO));
#endif
#if defined(CONFIG_BOARD_DJI_C_F407)
        /* C板沿用陀螺数据就绪触发；超时不把旧寄存器值当新姿态发布。 */
        if (Bmi088PortWaitSample(20u) != 0) {
            InsHeaterPwm = 0u;
            InsHeaterStable = 0u;
            imu_pwm_set(0u);
            WatchTaskError(WATCH_TASK_IMU);
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
            const fp32 invalid[3] = {0};
            ImuServiceUpdate(invalid, invalid, k_uptime_get_32(), 0u);
            ImuServicePublish(k_uptime_get_32(), 0u);
#endif
            k_msleep(1);
            continue;
        }
#endif
        fp32 gyro_raw[3] = {0};
        fp32 accel_raw[3] = {0};
#if defined(CONFIG_BOARD_DJI_A_F427)
        const int read = ImuReadMpu(gyro_raw, accel_raw, &InsTemp);
#else
        const int read = ImuReadBmi(gyro_raw, accel_raw, &InsTemp);
#endif
        if (read != 0) {
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_SDLOG
            ImuLogFlush();
#endif
            /* 读失败不能继续沿用加热占空比，也不能把未初始化值发布为姿态。 */
#if defined(CONFIG_BOARD_DJI_A_F427)
            InsThermal = (ImuThermalA){0};
#endif
            InsHeaterPwm = 0u;
            InsHeaterStable = 0u;
            imu_pwm_set(0u);
            WatchTaskError(WATCH_TASK_IMU);
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
            ImuServiceUpdate(gyro_raw, accel_raw, k_uptime_get_32(), 0u);
            ImuServicePublish(k_uptime_get_32(), 0u);
#endif
            k_msleep(2);
            continue;
        }
        ImuRotateVector(InsGyro, gyro_raw);
        ImuRotateVector(InsAccel, accel_raw);
#if ROBOT_TASK_BUILD_HEALTH_MONITOR || defined(CONFIG_ARBATOS_MINI_BENCH)
        /* 仅成功读取的新数据续期；加速度可信度拒绝不等于设备离线。 */
        DetectHook(BOARD_GYRO_TOE);
        DetectHook(BOARD_ACCEL_TOE);
#endif
        for (int i = 0; i < 3; ++i) InsGyro[i] += InsGyroOffset[i];
        ImuHeaterUpdate(InsTemp);
        const uint32_t now_ms = k_uptime_get_32();
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
        ImuServiceUpdate(gyro_raw, accel_raw, now_ms, 1u);
        if (!ImuOutputHeld && !robot_mode_is_calibration(ROBOT_CALI_TARGET_IMU_GYRO)) {
            ImuGyroCalibrationUpdate(gyro_raw, accel_raw, now_ms);
        }
#else
        ImuGyroCalibrationUpdate(gyro_raw, accel_raw, now_ms);
#endif
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
        if (offsetQueued == 0u && ins_is_gyro_boot_calibrated()) {
            ImuCalStoreQueue(InsGyroOffset, InsTemp);
            offsetQueued = 1u;
        }
#endif
        const fp32 *mag = NULL;
#if defined(CONFIG_BOARD_DJI_C_F407)
        static uint32_t lastMagRead;
        if (InsMagReady && now_ms - lastMagRead >= 5u) {
            fp32 rawMag[3];
            lastMagRead = now_ms;
            if (BspIst8310Read(rawMag) == 0) {
                /* C板两颗芯片的原生轴不同，先转到BMI088原生轴再套车型安装矩阵。 */
                const fp32 bmiAxes[3] = {-rawMag[1], rawMag[0], rawMag[2]};
                ImuRotateVector(InsMag, bmiAxes);
                InsMagTick = now_ms;
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
                DetectHook(BOARD_MAG_TOE);
#endif
            }
        }
        if (InsMagTick != 0u && now_ms - InsMagTick <= 100u) mag = InsMag;
#endif
        const float dt = ImuSampleDt();
        if (!ImuAttitudeUpdate(&InsAttitude, dt, now_ms, InsGyro, InsAccel, mag, g_config.imu.fusion_mode)) {
            WatchTaskError(WATCH_TASK_IMU);
            k_msleep(1);
            continue;
        }
        memcpy(InsQuat, InsAttitude.quat, sizeof(InsQuat));
        memcpy(INS_angle, InsAttitude.angle, sizeof(INS_angle));
        InsSnapshotPublishCurrent(now_ms, InsTemp);
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_SDLOG
        ImuLogSample(now_ms, dt);
        ImuLogTrust(now_ms);
#endif
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_IMU
        ImuServicePublish(now_ms, 1u);
#endif
#if defined(CONFIG_ARBATOS_BENCH_TELEMETRY)
        /* 延后打印，采样线程不等待串口；角速度/角度用微弧度，保留零漂分辨率。 */
        static uint32_t lastBenchMs;
        if (now_ms - lastBenchMs >= 100u) {
            LOG_INF("@IMU,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u",
                    now_ms, (int)(InsTemp * 1000.0f),
                    (int)(InsGyro[0] * 1000000.0f), (int)(InsGyro[1] * 1000000.0f),
                    (int)(InsGyro[2] * 1000000.0f),
                    (int)(INS_angle[0] * 1000000.0f), (int)(INS_angle[1] * 1000000.0f),
                    (int)(INS_angle[2] * 1000000.0f),
                    (int)(InsAccel[0] * 1000.0f), (int)(InsAccel[1] * 1000.0f),
                    (int)(InsAccel[2] * 1000.0f), InsHeaterPwm, ins_is_gyro_boot_calibrated());
            LOG_INF("@BIAS,%u,%d,%d,%d,%u", now_ms,
                    (int)(InsGyroOffset[0] * 1000000.0f), (int)(InsGyroOffset[1] * 1000000.0f),
                    (int)(InsGyroOffset[2] * 1000000.0f), ins_is_gyro_boot_calibrating());
            lastBenchMs = now_ms;
        }
#endif
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
        static uint32_t lastLogMs;
        if (now_ms - lastLogMs >= 10u) {
            sdlog_imu_t sample;
            memcpy(sample.quat, InsQuat, sizeof(sample.quat));
            memcpy(sample.gyro, InsGyro, sizeof(sample.gyro));
            memcpy(sample.accel, InsAccel, sizeof(sample.accel));
            sample.temp = InsTemp;
            SdLogWrite(SDLOG_TAG_IMU, &sample, sizeof(sample));
            lastLogMs = now_ms;
        }
#endif
        WatchImuSetStage(WATCH_IMU_STAGE_FUSION_LOOP);
        WatchTaskBeat(WATCH_TASK_IMU);
#if !defined(CONFIG_BOARD_DJI_C_F407)
        k_sleep(K_MSEC(1));
#endif
    }
#endif
}

void InsTask(void const *pvParameters) { ImuFusionTask(pvParameters); }
const fp32 *get_INS_quat_point(void) { return InsQuat; }
const fp32 *get_INS_angle_point(void) { return INS_angle; }
const fp32 *get_gyro_data_point(void) { return InsGyro; }
const fp32 *get_accel_data_point(void) { return InsAccel; }
const fp32 *get_mag_data_point(void) { return InsMag; }
fp32 ins_get_imu_temperature_c(void) { return InsTemp; }
uint16_t ins_get_imu_heater_pwm(void) { return InsHeaterPwm; }
uint8_t ins_get_imu_heater_mode(void)
{
#if defined(CONFIG_BOARD_DJI_C_F407) && !ROBOT_EXTERNAL_IMU_HI14
    return InsHeaterMode;
#else
    return InsHeaterPwm > 0 ? 1u : 0u;
#endif
}
fp32 ins_get_imu_heater_pid_out(void) { return InsHeaterPidOut; }
#if ROBOT_EXTERNAL_IMU_HI14
__weak bool_t CalibrateGyroOffsetSave(const fp32 offset[3])
{
    ARG_UNUSED(offset);
    return 0u;
}

void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    ARG_UNUSED(cali_scale);
    ARG_UNUSED(cali_offset);
    ARG_UNUSED(time_count);
}

void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    ARG_UNUSED(cali_scale);
    ARG_UNUSED(cali_offset);
}

bool_t ins_is_gyro_boot_calibrated(void)
{
    return 1u;
}

bool_t ins_is_gyro_boot_calibrating(void)
{
    return 0u;
}

ins_gyro_boot_init_result_e ins_get_gyro_boot_initial_result(void)
{
    return INS_GYRO_BOOT_INIT_SUCCESS;
}
#else
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (cali_scale == NULL || cali_offset == NULL || time_count == NULL)
    {
        return;
    }

    for (int i = 0; i < 3; ++i)
    {
        if (*time_count == 0u)
        {
            cali_offset[i] = InsGyroOffset[i];
        }
        cali_offset[i] -= 0.0003f * InsGyro[i];
        cali_scale[i] = 1.0f;
    }
    (*time_count)++;
    ImuApplyGyroOffset(cali_offset);
}
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    ARG_UNUSED(cali_scale);
    ImuApplyGyroOffset(cali_offset);
}
bool_t ins_is_gyro_boot_calibrated(void)
{
    return GyroZeroCaliRuntimeIsCalibrated(&InsGyroCaliState);
}
bool_t ins_is_gyro_boot_calibrating(void)
{
    return GyroZeroCaliRuntimeIsCalibrating(&InsGyroCaliState);
}
ins_gyro_boot_init_result_e ins_get_gyro_boot_initial_result(void)
{
    return (ins_gyro_boot_init_result_e)GyroZeroCaliRuntimeResult(&InsGyroCaliState);
}
#endif
