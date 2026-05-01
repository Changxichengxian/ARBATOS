
#include "INS_task.h"

#include "main.h"

#include "cmsis_os2.h"

#include "bmi088driver.h"
#include "bsp_imu_pwm.h"
#include "bsp_time.h"
#include "detect_task.h"
#include "pid.h"
#include "sdlog.h"
#include "config.h"
#include "tim.h"
#include "user_lib.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f},                  \
    {-1.0f, 0.0f, 0.0f},                 \
    {0.0f, 0.0f, 1.0f}

#define MAHONY_KP_DEFAULT             0.25f
#define MAHONY_KI_DEFAULT             0.002f
#define ATTITUDE_RESET_GYRO_LIMIT_DPS 15.0f
#define ATTITUDE_RESET_QUIET_TIME_MS  250U
#define ATTITUDE_RESET_ACTIVE_TIME_MS 500U
#define EULER_LPF_FC_HZ               60.0f
#define GYRO_BOOT_CALIB_SAMPLES       2000U
#define GYRO_BOOT_CALIB_MOVING_LIMIT_DPS 5.0f
#define GYRO_BOOT_CALIB_DELAY_MS      1U
#define GRAVITY_EARTH                 9.80665f
#define ACC_HEALTH_MIN_G2             0.81f
#define ACC_HEALTH_MAX_G2             1.21f
#define IMU_TEMP_PID_OUTPUT_FULL_SCALE 5000.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD 0.01745329251994329577f

__weak int8_t get_control_temperature(void)
{
    return 40;
}

void imu_fusion_task(void const *pvParameters)
{
    INS_task(pvParameters);
}

static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], const fp32 gyro_raw[3], const fp32 accel_raw[3]);
static bool gyro_boot_calibration(void);
static void imu_temp_control(fp32 temp);
static uint16_t imu_temp_output_to_pwm(fp32 out);
static float imu_calc_dt_s(void);
static bool_t imu_acc_is_healthy(const fp32 acc[3]);
static float imu_calc_dynamic_kp(bool_t use_acc, const fp32 gyro[3], uint32_t now_ms);
static void imu_update_euler_from_quat(const fp32 quat[4], fp32 euler[3]);
static void imu_fusion_reset(void);

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

static uint8_t first_temperate = 0u;
static const imu_config_t *const imu_cfg = &g_config.imu;
static pid_type_def imu_temp_pid;

static fp32 accel_filter_out[3] = {0.0f, 0.0f, 0.0f};
static second_order_filter_type_t accel_filter[3];
static const fp32 accel_filter_num[3] = {
    1.929454039488895f,
    -0.93178349823448126f,
    0.002329458745586203f,
};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};

static mahony_imu_t imu_mahony;
static enum {IMU_ST_DISARMED, IMU_ST_QUIET, IMU_ST_RESET} imu_gain_state = IMU_ST_DISARMED;
static uint32_t imu_state_timeout_ms = 0u;
static float imu_kp = MAHONY_KP_DEFAULT;
static float imu_ki = MAHONY_KI_DEFAULT;
static bool_t gyro_boot_calibrated = 0;
static bool_t gyro_boot_calibrating = 0;
static float imu_yaw_continuous = 0.0f;
static bool_t imu_yaw_inited = 0;
static fp32 imu_angle_lpf[3] = {0.0f, 0.0f, 0.0f};
static bool_t imu_angle_lpf_inited = 0;
static float imu_last_dt_s = 0.001f;
static volatile fp32 g_ins_temp_c = 0.0f;
static volatile fp32 g_ins_heater_pid_out = 0.0f;
static volatile uint16_t g_ins_heater_pwm = 0u;
static volatile uint8_t g_ins_heater_mode = 0u;

void INS_task(void const *pvParameters)
{
    (void)pvParameters;

    osDelay(imu_cfg->task_init_time_ms);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    while (BMI088_init() != 0u)
    {
        osDelay(100u);
    }

    bmi088_real_data_t raw = {0};
    BMI088_read(raw.gyro, raw.accel, &raw.temp);

    gyro_boot_calibrating = 1;
    gyro_boot_calibrated = gyro_boot_calibration();
    gyro_boot_calibrating = 0;

    imu_cali_solve(INS_gyro, INS_accel, raw.gyro, raw.accel);
    for (uint8_t i = 0u; i < 3u; i++)
    {
        accel_filter_out[i] = INS_accel[i];
        second_order_filter_init(&accel_filter[i], accel_filter_num, INS_accel[i]);
    }

    const fp32 imu_temp_PID[3] = {
        imu_cfg->temperature_pid.kp,
        imu_cfg->temperature_pid.ki,
        imu_cfg->temperature_pid.kd,
    };
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, imu_cfg->temperature_pid_max_out, imu_cfg->temperature_pid_max_iout);
    imu_fusion_reset();

    while (1)
    {
        BMI088_read(raw.gyro, raw.accel, &raw.temp);
        detect_hook(BOARD_GYRO_TOE);
        detect_hook(BOARD_ACCEL_TOE);

        imu_cali_solve(INS_gyro, INS_accel, raw.gyro, raw.accel);
        imu_temp_control(raw.temp);

        for (uint8_t i = 0u; i < 3u; i++)
        {
            accel_filter_out[i] = second_order_filter_cali(&accel_filter[i], INS_accel[i]);
        }

        const float dt = imu_calc_dt_s();
        const bool_t acc_healthy = imu_acc_is_healthy(accel_filter_out);
        const float kp_gain = imu_calc_dynamic_kp(acc_healthy, INS_gyro, bsp_time_get_tick_ms());
        mahony_imu_update(&imu_mahony, dt, INS_gyro, accel_filter_out, acc_healthy, kp_gain);

        INS_quat[0] = imu_mahony.quat[0];
        INS_quat[1] = imu_mahony.quat[1];
        INS_quat[2] = imu_mahony.quat[2];
        INS_quat[3] = imu_mahony.quat[3];
        imu_update_euler_from_quat(INS_quat, INS_angle);

        sdlog_imu_t pkt = {0};
        pkt.temp = raw.temp;
        for (uint8_t i = 0u; i < 4u; i++)
        {
            pkt.quat[i] = INS_quat[i];
        }
        for (uint8_t i = 0u; i < 3u; i++)
        {
            pkt.gyro[i] = INS_gyro[i];
            pkt.accel[i] = INS_accel[i];
        }
        sdlog_write(SDLOG_TAG_IMU, &pkt, (uint16_t)sizeof(pkt));

        sdlog_pid_snapshot_t pidlog = {0};
        pidlog.pid_id = SDLOG_PID_IMU_TEMP;
        pidlog.mode = imu_temp_pid.mode;
        pidlog.Kp = imu_temp_pid.Kp;
        pidlog.Ki = imu_temp_pid.Ki;
        pidlog.Kd = imu_temp_pid.Kd;
        pidlog.max_out = imu_temp_pid.max_out;
        pidlog.max_iout = imu_temp_pid.max_iout;
        pidlog.set = imu_temp_pid.set;
        pidlog.fdb = imu_temp_pid.fdb;
        pidlog.out = imu_temp_pid.out;
        pidlog.Pout = imu_temp_pid.Pout;
        pidlog.Iout = imu_temp_pid.Iout;
        pidlog.Dout = imu_temp_pid.Dout;
        memcpy((void *)pidlog.Dbuf, (const void *)imu_temp_pid.Dbuf, sizeof(pidlog.Dbuf));
        memcpy((void *)pidlog.error, (const void *)imu_temp_pid.error, sizeof(pidlog.error));
        sdlog_write(SDLOG_TAG_PID, &pidlog, (uint16_t)sizeof(pidlog));

        osDelay(1u);
    }
}

static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], const fp32 gyro_raw[3], const fp32 accel_raw[3])
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        gyro[i] = gyro_raw[0] * gyro_scale_factor[i][0] +
                  gyro_raw[1] * gyro_scale_factor[i][1] +
                  gyro_raw[2] * gyro_scale_factor[i][2] +
                  gyro_offset[i];
        accel[i] = accel_raw[0] * accel_scale_factor[i][0] +
                   accel_raw[1] * accel_scale_factor[i][1] +
                   accel_raw[2] * accel_scale_factor[i][2] +
                   accel_offset[i];
    }
}

static uint16_t imu_temp_output_to_pwm(fp32 out)
{
    const uint16_t pwm_max = MPU6500_TEMP_PWM_MAX;
    if (pwm_max <= 1u)
    {
        return 0u;
    }

    fp32 v = out;
    if (v < 0.0f)
    {
        v = 0.0f;
    }
    if (v > IMU_TEMP_PID_OUTPUT_FULL_SCALE)
    {
        v = IMU_TEMP_PID_OUTPUT_FULL_SCALE;
    }

    const fp32 scaled = v * (fp32)(pwm_max - 1u) / IMU_TEMP_PID_OUTPUT_FULL_SCALE;
    uint32_t pwm = (uint32_t)(scaled + 0.5f);
    if (pwm > (uint32_t)(pwm_max - 1u))
    {
        pwm = (uint32_t)(pwm_max - 1u);
    }
    return (uint16_t)pwm;
}

static void imu_temp_control(fp32 temp)
{
    static uint8_t temp_constant_time = 0u;
    g_ins_temp_c = temp;

    if (first_temperate != 0u)
    {
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        g_ins_heater_pid_out = imu_temp_pid.out;

        if (imu_temp_pid.Iout < 0.0f)
        {
            imu_temp_pid.Iout = 0.0f;
        }

        fp32 out = imu_temp_pid.Pout + imu_temp_pid.Iout + imu_temp_pid.Dout;
        if (out < 0.0f)
        {
            out = 0.0f;
        }
        if (out > imu_cfg->temperature_pid_max_out)
        {
            out = imu_cfg->temperature_pid_max_out;
        }

        g_ins_heater_mode = 1u;
        g_ins_heater_pwm = imu_temp_output_to_pwm(out);
        imu_pwm_set(g_ins_heater_pwm);
        return;
    }

    g_ins_heater_mode = 0u;
    if (temp > get_control_temperature())
    {
        temp_constant_time++;
        if (temp_constant_time > 200u)
        {
            first_temperate = 1u;
            imu_temp_pid.Iout = imu_cfg->temperature_pid_max_out / 2.0f;
        }
    }

    g_ins_heater_pid_out = imu_cfg->temperature_pid_max_out;
    g_ins_heater_pwm = imu_temp_output_to_pwm(imu_cfg->temperature_pid_max_out);
    imu_pwm_set(g_ins_heater_pwm);
}

void gyro_offset_calc(fp32 gyro_offset_in[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset_in == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    gyro_offset_in[0] -= 0.0003f * gyro[0];
    gyro_offset_in[1] -= 0.0003f * gyro[1];
    gyro_offset_in[2] -= 0.0003f * gyro[2];
    (*offset_time_count)++;
}

void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (cali_scale == NULL || cali_offset == NULL || time_count == NULL)
    {
        return;
    }

    if (*time_count == 0u)
    {
        gyro_offset[0] = gyro_cali_offset[0];
        gyro_offset[1] = gyro_cali_offset[1];
        gyro_offset[2] = gyro_cali_offset[2];
    }

    gyro_offset_calc(gyro_offset, INS_gyro, time_count);
    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;
}

void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    (void)cali_scale;
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

static bool gyro_boot_calibration(void)
{
    fp32 gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    bmi088_real_data_t raw = {0};
    const fp32 move_limit_rad = GYRO_BOOT_CALIB_MOVING_LIMIT_DPS * DEG_TO_RAD;

    for (uint16_t i = 0u; i < GYRO_BOOT_CALIB_SAMPLES; i++)
    {
        BMI088_read(raw.gyro, raw.accel, &raw.temp);

        fp32 gyro_rot[3];
        gyro_rot[0] = raw.gyro[0] * gyro_scale_factor[0][0] + raw.gyro[1] * gyro_scale_factor[0][1] + raw.gyro[2] * gyro_scale_factor[0][2];
        gyro_rot[1] = raw.gyro[0] * gyro_scale_factor[1][0] + raw.gyro[1] * gyro_scale_factor[1][1] + raw.gyro[2] * gyro_scale_factor[1][2];
        gyro_rot[2] = raw.gyro[0] * gyro_scale_factor[2][0] + raw.gyro[1] * gyro_scale_factor[2][1] + raw.gyro[2] * gyro_scale_factor[2][2];

        if (fabsf(gyro_rot[0]) > move_limit_rad || fabsf(gyro_rot[1]) > move_limit_rad || fabsf(gyro_rot[2]) > move_limit_rad)
        {
            return false;
        }

        gyro_sum[0] += gyro_rot[0];
        gyro_sum[1] += gyro_rot[1];
        gyro_sum[2] += gyro_rot[2];
        osDelay(GYRO_BOOT_CALIB_DELAY_MS);
    }

    const fp32 inv_samples = 1.0f / (fp32)GYRO_BOOT_CALIB_SAMPLES;
    gyro_cali_offset[0] = gyro_offset[0] = -gyro_sum[0] * inv_samples;
    gyro_cali_offset[1] = gyro_offset[1] = -gyro_sum[1] * inv_samples;
    gyro_cali_offset[2] = gyro_offset[2] = -gyro_sum[2] * inv_samples;
    return true;
}

bool_t ins_is_gyro_boot_calibrated(void)
{
    return gyro_boot_calibrated;
}

bool_t ins_is_gyro_boot_calibrating(void)
{
    return gyro_boot_calibrating;
}

fp32 ins_get_imu_temperature_c(void)
{
    return g_ins_temp_c;
}

uint16_t ins_get_imu_heater_pwm(void)
{
    return g_ins_heater_pwm;
}

uint8_t ins_get_imu_heater_mode(void)
{
    return g_ins_heater_mode;
}

fp32 ins_get_imu_heater_pid_out(void)
{
    return g_ins_heater_pid_out;
}

static float imu_calc_dt_s(void)
{
    static uint32_t last_tick_us = 0u;
    const uint32_t now_us = bsp_time_get_tick_us();
    float dt = 0.001f;

    if (last_tick_us != 0u)
    {
        dt = (float)(now_us - last_tick_us) * 1e-6f;
    }
    last_tick_us = now_us;

    if (dt <= 0.0001f || dt > 0.01f)
    {
        dt = 0.001f;
    }
    imu_last_dt_s = dt;
    return dt;
}

static bool_t imu_acc_is_healthy(const fp32 acc[3])
{
    const float ax_g = acc[0] / GRAVITY_EARTH;
    const float ay_g = acc[1] / GRAVITY_EARTH;
    const float az_g = acc[2] / GRAVITY_EARTH;
    const float acc_g2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    return (acc_g2 > ACC_HEALTH_MIN_G2) && (acc_g2 < ACC_HEALTH_MAX_G2);
}

static float imu_calc_dynamic_kp(bool_t use_acc, const fp32 gyro[3], uint32_t now_ms)
{
    const float gyro_limit = ATTITUDE_RESET_GYRO_LIMIT_DPS * DEG_TO_RAD;

    if (!use_acc ||
        fabsf(gyro[0]) > gyro_limit ||
        fabsf(gyro[1]) > gyro_limit ||
        fabsf(gyro[2]) > gyro_limit)
    {
        imu_gain_state = IMU_ST_QUIET;
        imu_state_timeout_ms = now_ms + ATTITUDE_RESET_QUIET_TIME_MS;
        return imu_kp;
    }

    switch (imu_gain_state)
    {
    case IMU_ST_QUIET:
        if (now_ms >= imu_state_timeout_ms)
        {
            imu_gain_state = IMU_ST_RESET;
            imu_state_timeout_ms = now_ms + ATTITUDE_RESET_ACTIVE_TIME_MS;
        }
        return imu_kp;
    case IMU_ST_RESET:
        if (now_ms >= imu_state_timeout_ms)
        {
            imu_gain_state = IMU_ST_DISARMED;
        }
        return imu_kp * 100.0f;
    case IMU_ST_DISARMED:
    default:
        return imu_kp * 10.0f;
    }
}

static void imu_update_euler_from_quat(const fp32 quat[4], fp32 euler[3])
{
    const float qw = quat[0];
    const float qx = quat[1];
    const float qy = quat[2];
    const float qz = quat[3];

    euler[INS_ROLL_ADDRESS_OFFSET] = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    euler[INS_PITCH_ADDRESS_OFFSET] = asinf(sinp);

    const float yaw_raw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    if (!imu_yaw_inited)
    {
        imu_yaw_continuous = yaw_raw;
        imu_yaw_inited = 1;
        imu_angle_lpf_inited = 0;
    }
    else
    {
        float delta = yaw_raw - euler[INS_YAW_ADDRESS_OFFSET];
        if (delta > M_PI)
        {
            delta -= 2.0f * M_PI;
        }
        else if (delta < -M_PI)
        {
            delta += 2.0f * M_PI;
        }
        imu_yaw_continuous += delta;
    }
    euler[INS_YAW_ADDRESS_OFFSET] = imu_yaw_continuous;

    if (!imu_angle_lpf_inited)
    {
        imu_angle_lpf[0] = euler[0];
        imu_angle_lpf[1] = euler[1];
        imu_angle_lpf[2] = euler[2];
        imu_angle_lpf_inited = 1;
    }
    else
    {
        const float alpha = 2.0f * (float)M_PI * EULER_LPF_FC_HZ * imu_last_dt_s;
        const float a = (alpha > 1.0f) ? 1.0f : alpha;
        for (uint8_t i = 0u; i < 3u; i++)
        {
            imu_angle_lpf[i] += a * (euler[i] - imu_angle_lpf[i]);
        }
    }

    euler[0] = imu_angle_lpf[0];
    euler[1] = imu_angle_lpf[1];
    euler[2] = imu_angle_lpf[2];
}

static void imu_fusion_reset(void)
{
    mahony_imu_init(&imu_mahony, imu_ki);
    imu_gain_state = IMU_ST_DISARMED;
    imu_state_timeout_ms = 0u;
    INS_quat[0] = 1.0f;
    INS_quat[1] = 0.0f;
    INS_quat[2] = 0.0f;
    INS_quat[3] = 0.0f;
    INS_angle[0] = 0.0f;
    INS_angle[1] = 0.0f;
    INS_angle[2] = 0.0f;
    imu_yaw_continuous = 0.0f;
    imu_yaw_inited = 0;
    imu_angle_lpf[0] = 0.0f;
    imu_angle_lpf[1] = 0.0f;
    imu_angle_lpf[2] = 0.0f;
    imu_angle_lpf_inited = 0;
    imu_last_dt_s = 0.001f;
}
