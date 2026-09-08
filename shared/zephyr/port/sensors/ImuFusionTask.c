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
#if defined(CONFIG_BOARD_DM_MC02_H7)
#include "ImuCalStore.h"
#endif
#endif

#include "BspImuPwm.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "SdLog.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>

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
static mahony_imu_t InsMahony;
static fp32 InsGyroOffset[3];
static GyroZeroCaliRuntimeState InsGyroCaliState;
static uint8_t InsHeaterStable;
#endif

__weak int8_t get_control_temperature(void)
{
    return 40;
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

static void ImuEulerUpdate(void)
{
    get_angle(InsQuat, &INS_angle[0], &INS_angle[2], &INS_angle[1]);
}

#if !ROBOT_EXTERNAL_IMU_HI14
static bool_t ImuAccelHealthy(const fp32 a[3])
{
    const float g2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (IMU_G * IMU_G);
    return (g2 > 0.81f && g2 < 1.21f) ? 1u : 0u;
}

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
#if defined(CONFIG_BOARD_DM_MC02_H7)
    if (ImuCalStoreIsWriting()) {
        InsHeaterPwm = 0u;
        InsHeaterStable = 0u;
        imu_pwm_set(0u);
        return;
    }
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
#else
    /* 原 PID 仍由控制层提供；这里先保留硬件安全的比例加热边界。 */
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
#if defined(CONFIG_BOARD_DM_MC02_H7)
    bool_t saved = ImuCalStoreSave(offset, InsTemp) == 0;
    if (saved) ImuApplyGyroOffset(offset);
    return saved;
#else
    /*
     * Zephyr 迁移阶段尚未开放 Flash 写入。先应用本次运行的零偏，并如实返回
     * “未持久保存”；开机静止修正仍可完成，主动校准则会报告保存失败。
     */
    ImuApplyGyroOffset(offset);
    return 0u;
#endif
}

static void ImuApplyGyroOffsetCallback(const fp32 offset[3], void *ctx)
{
    ARG_UNUSED(ctx);
    ImuApplyGyroOffset(offset);
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

static void ImuGyroCalibrationUpdate(const fp32 gyro_raw[3],
                                     const fp32 accel_raw[3],
                                     uint32_t now_ms)
{
    const GyroZeroCaliRuntimeCfg cfg = {
        .test_mode_active = robot_mode_is_calibration(ROBOT_CALI_TARGET_IMU_GYRO),
        .temp_c = InsTemp,
        .target_temp_c = (fp32)get_control_temperature(),
        .heater_stable = InsHeaterStable,
        .boot_adjust_allowed = ImuBootAdjustAllowed(),
        .now_ms = now_ms,
        .rotate_gyro = ImuRotateVector,
        .rotate_accel = ImuRotateVector,
        .apply_offset = ImuApplyGyroOffsetCallback,
        .save_offset = ImuSaveGyroOffsetCallback,
        .sample_done = NULL,
        .ctx = NULL,
    };
    GyroZeroCaliRuntimeUpdate(&InsGyroCaliState, &cfg, gyro_raw, accel_raw);
}
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
    mahony_imu_init(&InsMahony, 0.002f);
    GyroZeroCaliRuntimeReset(&InsGyroCaliState);
#if defined(CONFIG_BOARD_DM_MC02_H7)
    imu_pwm_set(0u);
    (void)ImuCalStoreLoad(InsGyroOffset);
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    uint8_t offsetQueued = 0u;
#endif
#endif
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_OK);

    for (;;) {
        fp32 gyro_raw[3] = {0};
        fp32 accel_raw[3] = {0};
#if defined(CONFIG_BOARD_DJI_A_F427)
        const int read = ImuReadMpu(gyro_raw, accel_raw, &InsTemp);
#else
        const int read = ImuReadBmi(gyro_raw, accel_raw, &InsTemp);
#endif
        if (read != 0) {
            /* 读失败不能继续沿用加热占空比，也不能把未初始化值发布为姿态。 */
            InsHeaterPwm = 0u;
            InsHeaterStable = 0u;
            imu_pwm_set(0u);
            WatchTaskError(WATCH_TASK_IMU);
            k_msleep(2);
            continue;
        }
        ImuRotateVector(InsGyro, gyro_raw);
        ImuRotateVector(InsAccel, accel_raw);
        for (int i = 0; i < 3; ++i) InsGyro[i] += InsGyroOffset[i];
        ImuHeaterUpdate(InsTemp);
        const uint32_t now_ms = k_uptime_get_32();
        ImuGyroCalibrationUpdate(gyro_raw, accel_raw, now_ms);
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
        if (offsetQueued == 0u && ins_is_gyro_boot_calibrated()) {
            ImuCalStoreQueue(InsGyroOffset, InsTemp);
            offsetQueued = 1u;
        }
#endif
        mahony_imu_update(&InsMahony,
                          ImuSampleDt(),
                          InsGyro,
                          InsAccel,
                          ImuAccelHealthy(InsAccel),
                          0.25f);
        for (int i = 0; i < 4; ++i) InsQuat[i] = InsMahony.quat[i];
        ImuEulerUpdate();
        InsSnapshotPublishCurrent(now_ms, InsTemp);
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
        k_sleep(K_MSEC(1));
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
uint8_t ins_get_imu_heater_mode(void) { return InsHeaterPwm > 0 ? 1u : 0u; }
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
