/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Zephyr-owned IMU task.  The old board InsTask.c files are deliberately not
 * linked: they rely on HAL DMA callbacks and FreeRTOS notifications.
 */
#include "InsTask.h"
#include "Ahrs.h"
#include "Bmi088Driver.h"
#include "BspImuPwm.h"
#include "CalibrateTask.h"
#include "ControlInput.h"
#include "GyroZeroCali.h"
#include "ManualInputSnapshot.h"
#include "Mpu6500.h"
#include "RobotConfig.h"
#include "RobotMode.h"
#include "UserLib.h"
#include "Watch.h"

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
static mahony_imu_t InsMahony;
static fp32 InsGyroOffset[3];
static GyroZeroCaliRuntimeState InsGyroCaliState;
static uint8_t InsHeaterStable;

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

static bool_t ImuAccelHealthy(const fp32 a[3])
{
    const float g2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (IMU_G * IMU_G);
    return (g2 > 0.81f && g2 < 1.21f) ? 1u : 0u;
}

static void ImuRotateVector(fp32 rotated[3], const fp32 raw[3])
{
    /*
     * 三块原板使用同一安装矩阵：
     * [ 0  1  0 ]
     * [-1  0  0 ]
     * [ 0  0  1 ]
     */
    rotated[0] = raw[1];
    rotated[1] = -raw[0];
    rotated[2] = raw[2];
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
    const float target = get_control_temperature();
    const float max = g_config.imu.imu_temp_pwm_max;
    const float err = target - temp;
    /* 原 PID 仍由控制层提供；这里先保留硬件安全的比例加热边界。 */
    InsHeaterPidOut = err * g_config.imu.temperature_pid.kp;
    if (InsHeaterPidOut < 0.0f) InsHeaterPidOut = 0.0f;
    if (InsHeaterPidOut > max) InsHeaterPidOut = max;
    InsHeaterPwm = (uint16_t)InsHeaterPidOut;
    InsHeaterStable = (fabsf(err) <= GYRO_ZERO_CALI_TEMP_ERR_C) ? 1u : 0u;
    imu_pwm_set(InsHeaterPwm);
}

#if !defined(CONFIG_ARBATOS_TARGET_INFANTRY_A) && !defined(CONFIG_ARBATOS_TARGET_CARRIER_A)
static int ImuReadBmi(fp32 gyro_raw[3], fp32 accel_raw[3], fp32 *temp)
{
    BMI088_read(gyro_raw, accel_raw, temp);
    return 0;
}
#endif

#if defined(CONFIG_ARBATOS_TARGET_INFANTRY_A) || defined(CONFIG_ARBATOS_TARGET_CARRIER_A)
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
    /*
     * Zephyr 迁移阶段尚未开放 Flash 写入。先应用本次运行的零偏，并如实返回
     * “未持久保存”；开机静止修正仍可完成，主动校准则会报告保存失败。
     */
    ImuApplyGyroOffset(offset);
    return 0u;
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

void ImuFusionTask(void const *pvParameters)
{
    int init;
    ARG_UNUSED(pvParameters);
    WatchImuSetStage(WATCH_IMU_STAGE_ENTER);
    WatchTaskWait(WATCH_TASK_IMU);
    k_msleep(g_config.imu.task_init_time_ms);
    WatchImuSetStage(WATCH_IMU_STAGE_INIT_DELAY_DONE);

#if defined(CONFIG_ARBATOS_TARGET_INFANTRY_A) || defined(CONFIG_ARBATOS_TARGET_CARRIER_A)
    init = mpu6500_init();
#else
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_TRY);
    init = BMI088_init();
#endif
    while (init != 0) {
        WatchTaskError(WATCH_TASK_IMU);
        k_msleep(100);
#if defined(CONFIG_ARBATOS_TARGET_INFANTRY_A) || defined(CONFIG_ARBATOS_TARGET_CARRIER_A)
        init = mpu6500_init();
#else
        WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_RETRY);
        init = BMI088_init();
#endif
    }
    mahony_imu_init(&InsMahony, 0.002f);
    GyroZeroCaliRuntimeReset(&InsGyroCaliState);
    WatchImuSetStage(WATCH_IMU_STAGE_BMI088_INIT_OK);

    for (;;) {
        fp32 gyro_raw[3];
        fp32 accel_raw[3];
#if defined(CONFIG_ARBATOS_TARGET_INFANTRY_A) || defined(CONFIG_ARBATOS_TARGET_CARRIER_A)
        const int read = ImuReadMpu(gyro_raw, accel_raw, &InsTemp);
#else
        const int read = ImuReadBmi(gyro_raw, accel_raw, &InsTemp);
#endif
        if (read != 0) { WatchTaskError(WATCH_TASK_IMU); k_msleep(2); continue; }
        ImuRotateVector(InsGyro, gyro_raw);
        ImuRotateVector(InsAccel, accel_raw);
        for (int i = 0; i < 3; ++i) InsGyro[i] += InsGyroOffset[i];
        ImuHeaterUpdate(InsTemp);
        const uint32_t now_ms = k_uptime_get_32();
        ImuGyroCalibrationUpdate(gyro_raw, accel_raw, now_ms);
        mahony_imu_update(&InsMahony,
                          ImuSampleDt(),
                          InsGyro,
                          InsAccel,
                          ImuAccelHealthy(InsAccel),
                          0.25f);
        for (int i = 0; i < 4; ++i) InsQuat[i] = InsMahony.quat[i];
        ImuEulerUpdate();
        InsSnapshotPublishCurrent(now_ms, InsTemp);
        WatchImuSetStage(WATCH_IMU_STAGE_FUSION_LOOP);
        WatchTaskBeat(WATCH_TASK_IMU);
        k_sleep(K_MSEC(1));
    }
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
