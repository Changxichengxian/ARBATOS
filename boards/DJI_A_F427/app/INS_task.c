/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#include "INS_task.h"

#include "main.h"

#include "cmsis_os2.h"

#include "AHRS.h"
#include "bsp_imu_pwm.h"
#include "detect_task.h"
#include "mpu6500.h"
#include "pid.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "sdlog.h"

#if defined(IMU_WATCH_ENABLE) && (IMU_WATCH_ENABLE != 0)
#include "watch.h"
#define IMU_WATCH_ERROR() watch_task_error(WATCH_TASK_IMU)
#define IMU_WATCH_WAIT() watch_task_wait(WATCH_TASK_IMU)
#define IMU_WATCH_BEAT() watch_task_beat(WATCH_TASK_IMU)
#define IMU_WATCH_TIMEOUT() watch_task_timeout(WATCH_TASK_IMU)
#else
#define IMU_WATCH_ERROR() ((void)0)
#define IMU_WATCH_WAIT() ((void)0)
#define IMU_WATCH_BEAT() ((void)0)
#define IMU_WATCH_TIMEOUT() ((void)0)
#endif


#define IMU_BOARD_INSTALL_SPIN_MATRIX      \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

// Mahony attitude filter defaults.
#define MAHONY_KP_DEFAULT             0.25f
#define MAHONY_KI_DEFAULT             0.002f
#define ATTITUDE_RESET_GYRO_LIMIT_DPS 15.0f
#define ATTITUDE_RESET_QUIET_TIME_MS  250U
#define ATTITUDE_RESET_ACTIVE_TIME_MS 500U

#define EULER_LPF_FC_HZ               60.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f

// Accelerometer health thresholds (about 0.9g to 1.1g).
#define ACC_HEALTH_MIN_G2 0.81f
#define ACC_HEALTH_MAX_G2 1.21f
#define GRAVITY_EARTH     9.80665f

#define MPU6500_GYRO_LSB_TO_RAD_S  (DEG_TO_RAD / 32.8f)        // +-1000 dps
#define MPU6500_ACCEL_LSB_TO_M_S2 (GRAVITY_EARTH / 4096.0f)    // +-8g

__weak int8_t get_control_temperature(void)
{
    return 40;
}

void imu_fusion_task(void const *pvParameters)
{
    INS_task(pvParameters);
}

// Boot-time gyro zero-offset calibration.
#define GYRO_BOOT_CALIB_SAMPLES        2000U
#define GYRO_BOOT_CALIB_MOVING_LIMIT_DPS 5.0f
#define GYRO_BOOT_CALIB_DELAY_MS       1U
#define GYRO_BOOT_CALIB_ACC_TOL_G      0.05f
#define GYRO_BOOT_TEMP_STABLE_ERR_C    1.0f
#define GYRO_BOOT_TEMP_STABLE_TIME_MS  1000U
#define IMU_SDLOG_BASE_STREAM_MAX_SAMPLES 16u
#define IMU_SDLOG_PID_PERIOD_MS        10u
#define IMU_TEMP_VALID_MIN_C           (-20.0f)
#define IMU_TEMP_VALID_MAX_C           85.0f
#define IMU_TEMP_INVALID_MAX_SAMPLES   5u
#define IMU_TEMP_PREHEAT_TIMEOUT_MS    30000u


/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          Rotate IMU axes and apply offsets.
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], const fp32 gyro_raw[3], const fp32 accel_raw[3]);
/**
  * @brief          Average gyro samples at boot to estimate zero offset.
  * @retval         true when calibration updates gyro_offset; false if motion is detected.
  */
static void gyro_boot_retry_reset(void);
static void gyro_boot_retry_update(const fp32 gyro[3], const fp32 acc[3]);
static bool_t gyro_boot_temp_ready(fp32 temp);

static void imu_temp_control(fp32 temp);
static uint16_t imu_temp_pwm_limit(void);
static bool imu_temp_is_valid(fp32 temp);
static float imu_calc_dt_s(void);
static bool_t imu_acc_is_healthy(const fp32 acc[3]);
static float imu_calc_dynamic_kp(bool_t use_acc, const fp32 gyro[3], uint32_t now_ms);
static void imu_update_euler_from_quat(const fp32 quat[4], fp32 euler[3]);
static void imu_fusion_reset(imu_fusion_mode_e mode, const fp32 acc[3], const fp32 mag[3]);


fp32 gyro_scale_factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

static uint8_t first_temperate;
static const imu_config_t *const imu_cfg = &g_config.imu;
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   // task run time, unit s


// Low-pass filter state for accelerometer fusion input.
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static second_order_filter_type_t accel_filter[3];
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      // euler angle, unit rad

// Mahony attitude state.
static mahony_imu_t imu_mahony;
typedef struct
{
    fp32 gyro_sum[3];
    fp32 accel_norm_sum;
    uint16_t sample_count;
} gyro_boot_retry_state_t;
static enum {IMU_ST_DISARMED, IMU_ST_QUIET, IMU_ST_RESET} imu_gain_state = IMU_ST_DISARMED;
static uint32_t imu_state_timeout_ms = 0;
static float imu_kp = MAHONY_KP_DEFAULT;
static float imu_ki = MAHONY_KI_DEFAULT;
static bool_t gyro_boot_calibrated = 0;
static bool_t gyro_boot_calibrating = 0;
static ins_gyro_boot_init_result_e gyro_boot_initial_result = INS_GYRO_BOOT_INIT_PENDING;
static fp32 gyro_boot_accel_norm_ref = GRAVITY_EARTH;
static bool_t gyro_boot_accel_norm_valid = 0;
static gyro_boot_retry_state_t gyro_boot_retry_state = {0};
static float imu_yaw_continuous = 0.0f;
static bool_t imu_yaw_inited = 0;
static fp32 imu_angle_lpf[3] = {0.0f, 0.0f, 0.0f};
static bool_t imu_angle_lpf_inited = 0;
static float imu_last_dt_s = 0.001f;
static imu_fusion_mode_e imu_fusion_mode_active = IMU_FUSION_MAHONY_6AXIS;

static TaskHandle_t mpu6500_dma_notify_task = NULL;
static volatile int8_t mpu6500_dma_last_status = 0;
static SemaphoreHandle_t imu_drdy_sem = NULL;
static StaticSemaphore_t imu_drdy_sem_buf;
static uint32_t imu_pid_log_tick_ms = 0u;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    sdlog_imu_base_sample_t samples[IMU_SDLOG_BASE_STREAM_MAX_SAMPLES];
} imu_sdlog_base_stream_state_t;

static imu_sdlog_base_stream_state_t s_imu_sdlog_base_stream = {0};

static uint32_t imu_sdlog_period_us_from_dt(float dt_s)
{
    uint32_t period_us = 1000u;

    if (dt_s > 0.0f)
    {
        const float period_f = dt_s * 1000000.0f;
        if (period_f > 0.5f)
        {
            period_us = (uint32_t)(period_f + 0.5f);
        }
    }

    if (period_us == 0u)
    {
        period_us = 1000u;
    }
    return period_us;
}

static void imu_sdlog_begin_base_stream(uint32_t now_ms, uint32_t period_us)
{
    s_imu_sdlog_base_stream.start_tick_ms = now_ms;
    s_imu_sdlog_base_stream.last_tick_ms = now_ms;
    s_imu_sdlog_base_stream.period_us = period_us;
    s_imu_sdlog_base_stream.sample_count = 0u;
}

static void imu_sdlog_flush_base_stream(void)
{
    if (s_imu_sdlog_base_stream.sample_count == 0u)
    {
        return;
    }

    sdlog_imu_base_stream_header_t header = {0};
    uint8_t payload[sizeof(header) + sizeof(s_imu_sdlog_base_stream.samples)];
    const uint16_t payload_len =
        (uint16_t)(sizeof(header) + (uint32_t)s_imu_sdlog_base_stream.sample_count * sizeof(sdlog_imu_base_sample_t));

    header.start_tick_ms = s_imu_sdlog_base_stream.start_tick_ms;
    header.period_us = (s_imu_sdlog_base_stream.period_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_imu_sdlog_base_stream.period_us;
    header.sample_count = (uint8_t)s_imu_sdlog_base_stream.sample_count;
    header.version = SDLOG_IMU_BASE_STREAM_VERSION;

    memcpy(payload, &header, sizeof(header));
    memcpy(&payload[sizeof(header)],
           s_imu_sdlog_base_stream.samples,
           (uint32_t)s_imu_sdlog_base_stream.sample_count * sizeof(sdlog_imu_base_sample_t));
    sdlog_write(SDLOG_TAG_IMU_BASE_STREAM, payload, payload_len);
    s_imu_sdlog_base_stream.sample_count = 0u;
}

static void imu_sdlog_append_base_sample(const sdlog_imu_base_sample_t *sample,
                                         uint32_t now_ms,
                                         uint32_t period_us)
{
    if (sample == NULL)
    {
        return;
    }

    if (period_us == 0u)
    {
        period_us = 1000u;
    }

    if (s_imu_sdlog_base_stream.sample_count == 0u)
    {
        imu_sdlog_begin_base_stream(now_ms, period_us);
    }
    else
    {
        const uint32_t expected_dt_ms = (s_imu_sdlog_base_stream.period_us + 999u) / 1000u;
        const uint32_t actual_dt_ms = now_ms - s_imu_sdlog_base_stream.last_tick_ms;
        if (s_imu_sdlog_base_stream.period_us != period_us || actual_dt_ms != expected_dt_ms)
        {
            imu_sdlog_flush_base_stream();
            imu_sdlog_begin_base_stream(now_ms, period_us);
        }
    }

    s_imu_sdlog_base_stream.samples[s_imu_sdlog_base_stream.sample_count++] = *sample;
    s_imu_sdlog_base_stream.last_tick_ms = now_ms;

    if (s_imu_sdlog_base_stream.sample_count >= IMU_SDLOG_BASE_STREAM_MAX_SAMPLES)
    {
        imu_sdlog_flush_base_stream();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL || hspi->Instance != SPI5)
    {
        return;
    }

    mpu6500_dma_last_status = (hspi->ErrorCode == HAL_SPI_ERROR_NONE) ? 0 : -1;
    mpu6500_read_raw_dma_release_cs();

    if (mpu6500_dma_notify_task != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(mpu6500_dma_notify_task, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL || hspi->Instance != SPI5)
    {
        return;
    }

    mpu6500_dma_last_status = -1;
    mpu6500_read_raw_dma_release_cs();

    if (mpu6500_dma_notify_task != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(mpu6500_dma_notify_task, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IMU_INT_Pin || imu_drdy_sem == NULL)
    {
        return;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        BaseType_t hpw = pdFALSE;
        (void)xSemaphoreGiveFromISR(imu_drdy_sem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}





/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          IMU task: initialize MPU6500 and calculate Euler angles.
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void INS_task(void const *pvParameters)
{
    (void)pvParameters;

    osDelay(imu_cfg->task_init_time_ms);

    mpu6500_dma_notify_task = xTaskGetCurrentTaskHandle();
    if (imu_drdy_sem == NULL)
    {
        imu_drdy_sem = xSemaphoreCreateBinaryStatic(&imu_drdy_sem_buf);
    }
    if (imu_drdy_sem != NULL)
    {
        (void)xSemaphoreTake(imu_drdy_sem, 0u);
    }
    while (mpu6500_init() != 0)
    {
        IMU_WATCH_ERROR();
        osDelay(100);
    }

    mpu6500_raw_t raw = {0};
    while (mpu6500_read_raw(&raw) != 0)
    {
        IMU_WATCH_ERROR();
        osDelay(10);
    }

    fp32 gyro_raw[3];
    fp32 accel_raw[3];

    gyro_raw[0] = (fp32)raw.gyro[0] * MPU6500_GYRO_LSB_TO_RAD_S;
    gyro_raw[1] = (fp32)raw.gyro[1] * MPU6500_GYRO_LSB_TO_RAD_S;
    gyro_raw[2] = (fp32)raw.gyro[2] * MPU6500_GYRO_LSB_TO_RAD_S;

    accel_raw[0] = (fp32)raw.accel[0] * MPU6500_ACCEL_LSB_TO_M_S2;
    accel_raw[1] = (fp32)raw.accel[1] * MPU6500_ACCEL_LSB_TO_M_S2;
    accel_raw[2] = (fp32)raw.accel[2] * MPU6500_ACCEL_LSB_TO_M_S2;

    imu_cali_slove(INS_gyro, INS_accel, gyro_raw, accel_raw);
    INS_mag[0] = 0.0f;
    INS_mag[1] = 0.0f;
    INS_mag[2] = 0.0f;

    const fp32 imu_temp_PID[3] = {imu_cfg->temperature_pid.kp, imu_cfg->temperature_pid.ki, imu_cfg->temperature_pid.kd};
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, imu_cfg->temperature_pid_max_out, imu_cfg->temperature_pid_max_iout);

    for (uint8_t i = 0u; i < 3u; i++)
    {
        accel_fliter_3[i] = INS_accel[i];
        second_order_filter_init(&accel_filter[i], fliter_num, INS_accel[i]);
    }

    imu_fusion_mode_e mode_init = (imu_fusion_mode_e)imu_cfg->fusion_mode;
    if (mode_init == IMU_FUSION_AHRS_9AXIS)
    {
        // No magnetometer integrated in this port yet -> fallback to 6-axis.
        mode_init = IMU_FUSION_MAHONY_6AXIS;
    }
    imu_fusion_reset(mode_init, accel_fliter_3, INS_mag);

    while (1)
    {
        if (imu_drdy_sem != NULL)
        {
            IMU_WATCH_WAIT();
            (void)xSemaphoreTake(imu_drdy_sem, portMAX_DELAY);
        }

        bool_t got_raw = 0;
        bool_t raw_timeout = 0;
        bool_t raw_error = 0;

        // Try SPI5 DMA first; fallback to blocking SPI if DMA not configured.
        (void)ulTaskNotifyTake(pdTRUE, 0u);
        mpu6500_dma_last_status = -1;
        const int dma_start = mpu6500_read_raw_dma_start();
        if (dma_start == 0)
        {
            IMU_WATCH_WAIT();
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2u)) != 0u && mpu6500_dma_last_status == 0)
            {
                got_raw = (mpu6500_read_raw_dma_finish(&raw) == 0) ? 1 : 0;
                raw_error = !got_raw;
            }
            else
            {
                raw_timeout = 1;
                (void)HAL_SPI_Abort(&hspi5);
                mpu6500_read_raw_dma_release_cs();
            }
        }
        else if (dma_start == -2)
        {
            got_raw = (mpu6500_read_raw(&raw) == 0) ? 1 : 0;
            raw_error = !got_raw;
        }
        else
        {
            raw_error = 1;
            (void)HAL_SPI_Abort(&hspi5);
            mpu6500_read_raw_dma_release_cs();
            got_raw = (mpu6500_read_raw(&raw) == 0) ? 1 : 0;
            raw_error = !got_raw;
        }

        if (got_raw)
        {
            IMU_WATCH_BEAT();
            detect_hook(BOARD_GYRO_TOE);
            detect_hook(BOARD_ACCEL_TOE);

            gyro_raw[0] = (fp32)raw.gyro[0] * MPU6500_GYRO_LSB_TO_RAD_S;
            gyro_raw[1] = (fp32)raw.gyro[1] * MPU6500_GYRO_LSB_TO_RAD_S;
            gyro_raw[2] = (fp32)raw.gyro[2] * MPU6500_GYRO_LSB_TO_RAD_S;

            accel_raw[0] = (fp32)raw.accel[0] * MPU6500_ACCEL_LSB_TO_M_S2;
            accel_raw[1] = (fp32)raw.accel[1] * MPU6500_ACCEL_LSB_TO_M_S2;
            accel_raw[2] = (fp32)raw.accel[2] * MPU6500_ACCEL_LSB_TO_M_S2;

            imu_cali_slove(INS_gyro, INS_accel, gyro_raw, accel_raw);

            const float temp_c = mpu6500_temp_c(raw.temp);
            imu_temp_control(temp_c);

            // Accelerometer low-pass filter.
            for (uint8_t i = 0u; i < 3u; i++)
            {
                accel_fliter_3[i] = second_order_filter_cali(&accel_filter[i], INS_accel[i]);
            }
            if (!gyro_boot_calibrated)
            {
                if (gyro_boot_temp_ready(temp_c) != 0u)
                {
                    gyro_boot_retry_update(INS_gyro, accel_fliter_3);
                }
                else
                {
                    gyro_boot_retry_reset();
                }
            }

            imu_fusion_mode_e mode = (imu_fusion_mode_e)imu_cfg->fusion_mode;
            if (mode == IMU_FUSION_AHRS_9AXIS)
            {
                // No magnetometer integrated in this port yet -> fallback to 6-axis.
                mode = IMU_FUSION_MAHONY_6AXIS;
            }
            if (mode != imu_fusion_mode_active)
            {
                imu_fusion_reset(mode, accel_fliter_3, INS_mag);
            }

            // Calculate dt from millisecond tick; fall back to 1 ms on outliers.
            const uint32_t now_ms = HAL_GetTick();
            const float dt = imu_calc_dt_s();

            if (imu_fusion_mode_active == IMU_FUSION_AHRS_9AXIS)
            {
                (void)AHRS_update(INS_quat, dt, INS_gyro, accel_fliter_3, INS_mag);
                imu_update_euler_from_quat(INS_quat, INS_angle);
            }
            else
            {
                const bool_t acc_healthy = imu_acc_is_healthy(accel_fliter_3);
                const float kp_gain = imu_calc_dynamic_kp(acc_healthy, INS_gyro, now_ms);

                mahony_imu_update(&imu_mahony, dt, INS_gyro, accel_fliter_3, acc_healthy, kp_gain);

                INS_quat[0] = imu_mahony.quat[0];
                INS_quat[1] = imu_mahony.quat[1];
                INS_quat[2] = imu_mahony.quat[2];
                INS_quat[3] = imu_mahony.quat[3];
                imu_update_euler_from_quat(INS_quat, INS_angle);
            }

            sdlog_imu_base_sample_t pkt = {0};
            for (uint8_t i = 0; i < 4u; i++)
            {
                pkt.quat[i] = INS_quat[i];
            }
            for (uint8_t i = 0; i < 3u; i++)
            {
                pkt.gyro[i] = INS_gyro[i];
                pkt.accel[i] = INS_accel[i];
            }
            pkt.temp = temp_c;
            imu_sdlog_append_base_sample(&pkt, now_ms, imu_sdlog_period_us_from_dt(dt));

            if ((uint32_t)(now_ms - imu_pid_log_tick_ms) >= IMU_SDLOG_PID_PERIOD_MS)
            {
                imu_pid_log_tick_ms = now_ms;

                sdlog_pid_runtime_t pidlog = {0};
                pidlog.pid_id = SDLOG_PID_IMU_TEMP;
                pidlog.mode = imu_temp_pid.mode;
                pidlog.set = imu_temp_pid.set;
                pidlog.fdb = imu_temp_pid.fdb;
                pidlog.out = imu_temp_pid.out;
                sdlog_write(SDLOG_TAG_PID, &pidlog, (uint16_t)sizeof(pidlog));
            }
        }
        else if (raw_timeout)
        {
            IMU_WATCH_TIMEOUT();
        }
        else if (raw_error)
        {
            IMU_WATCH_ERROR();
        }
    }
}




/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          Rotate IMU axes and apply offsets.
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], const fp32 gyro_raw[3], const fp32 accel_raw[3])
{
    for (uint8_t i = 0; i < 3u; i++)
    {
        gyro[i] = gyro_raw[0] * gyro_scale_factor[i][0] + gyro_raw[1] * gyro_scale_factor[i][1] + gyro_raw[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = accel_raw[0] * accel_scale_factor[i][0] + accel_raw[1] * accel_scale_factor[i][1] + accel_raw[2] * accel_scale_factor[i][2] + accel_offset[i];
    }
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          Control MPU6500 temperature.
  * @param[in]      temp: current MPU6500 temperature.
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    static uint8_t temp_invalid_count = 0;
    static uint8_t temp_fault = 0;
    static uint32_t preheat_start_tick_ms = 0u;
    const uint16_t pwm_limit = imu_temp_pwm_limit();

    if (temp_fault)
    {
        imu_pwm_set(0u);
        return;
    }

    if (!imu_temp_is_valid(temp))
    {
        imu_pwm_set(0u);
        PID_clear(&imu_temp_pid);
        temp_constant_time = 0u;
        preheat_start_tick_ms = 0u;
        if (temp_invalid_count < IMU_TEMP_INVALID_MAX_SAMPLES)
        {
            temp_invalid_count++;
        }
        if (temp_invalid_count >= IMU_TEMP_INVALID_MAX_SAMPLES)
        {
            temp_fault = 1u;
        }
        IMU_WATCH_ERROR();
        return;
    }

    temp_invalid_count = 0u;

    if (first_temperate)
    {
        preheat_start_tick_ms = 0u;
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        if (imu_temp_pid.Iout < 0.0f)
        {
            imu_temp_pid.Iout = 0.0f;
        }
        else if (imu_temp_pid.Iout > (fp32)pwm_limit)
        {
            imu_temp_pid.Iout = (fp32)pwm_limit;
        }
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        else if (imu_temp_pid.out > (fp32)pwm_limit)
        {
            imu_temp_pid.out = (fp32)pwm_limit;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        imu_pwm_set(tempPWM);
    }
    else
    {
        if (preheat_start_tick_ms == 0u)
        {
            preheat_start_tick_ms = HAL_GetTick();
        }

        // Heat at full power until the target temperature is reached.
        if (temp > get_control_temperature())
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // Preload the integral term to speed convergence after warm-up.
                first_temperate = 1;
                imu_temp_pid.Iout = (fp32)pwm_limit / 2.0f;
                preheat_start_tick_ms = 0u;
            }
        }
        else
        {
            temp_constant_time = 0u;
        }

        if (first_temperate)
        {
            imu_pwm_set(pwm_limit);
            return;
        }

        if ((uint32_t)(HAL_GetTick() - preheat_start_tick_ms) >= IMU_TEMP_PREHEAT_TIMEOUT_MS)
        {
            imu_pwm_set(0u);
            PID_clear(&imu_temp_pid);
            temp_constant_time = 0u;
            temp_fault = 1u;
            IMU_WATCH_ERROR();
            return;
        }

        imu_pwm_set(pwm_limit);
    }
}

static uint16_t imu_temp_pwm_limit(void)
{
    const uint16_t max_pwm = MPU6500_TEMP_PWM_MAX;
    if (max_pwm == 0u)
    {
        return 0u;
    }
    return (uint16_t)(max_pwm - 1u);
}

static bool imu_temp_is_valid(fp32 temp)
{
    return (temp == temp) && (temp >= IMU_TEMP_VALID_MIN_C) && (temp <= IMU_TEMP_VALID_MAX_C);
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          Calculate gyro zero drift.
  * @param[out]     gyro_offset: calculated zero drift.
  * @param[in]      gyro: angular velocity data.
  * @param[out]     offset_time_count: incremented on each sample.
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc
  * @retval         none
  */
/**
  * @brief          Calibrate gyro.
  * @param[out]     cali_scale: gyro scale factor; defaults to 1.0f.
  * @param[out]     cali_offset: gyro zero drift sampled while stationary.
  * @param[out]     time_count: incremented on each gyro_offset_calc call.
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
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

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift,
  * @retval         none
  */
/**
  * @brief          Set gyro calibration values from flash or another source.
  * @param[in]      cali_scale: gyro scale factor; defaults to 1.0f.
  * @param[in]      cali_offset: gyro zero drift.
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
  * @brief          get the euler angle, 0:yaw, 1:roll, 2:pitch unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          Get Euler angle, unit rad.
  * @param[in]      none
  * @retval         pointer to INS_angle.
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          Get gyro data, unit rad/s.
  * @param[in]      none
  * @retval         pointer to INS_gyro.
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
/**
  * @brief          Get accelerometer data, unit m/s2.
  * @param[in]      none
  * @retval         pointer to INS_accel.
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          Get magnetometer data, unit uT.
  * @param[in]      none
  * @retval         pointer to INS_mag.
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


static void gyro_boot_retry_reset(void)
{
    memset(&gyro_boot_retry_state, 0, sizeof(gyro_boot_retry_state));
    gyro_boot_calibrating = 0;
}

static void gyro_boot_retry_update(const fp32 gyro[3], const fp32 acc[3])
{
    if (gyro == NULL || acc == NULL)
    {
        gyro_boot_retry_reset();
        return;
    }

    const fp32 move_limit_rad = GYRO_BOOT_CALIB_MOVING_LIMIT_DPS * DEG_TO_RAD;
    const fp32 acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    const fp32 acc_ref = gyro_boot_accel_norm_valid ? gyro_boot_accel_norm_ref : GRAVITY_EARTH;
    const fp32 acc_tol = acc_ref * GYRO_BOOT_CALIB_ACC_TOL_G;

    if (fabsf(gyro[0]) > move_limit_rad ||
        fabsf(gyro[1]) > move_limit_rad ||
        fabsf(gyro[2]) > move_limit_rad ||
        fabsf(acc_norm - acc_ref) > acc_tol)
    {
        gyro_boot_retry_reset();
        return;
    }

    gyro_boot_calibrating = 1;
    gyro_boot_retry_state.gyro_sum[0] += gyro[0];
    gyro_boot_retry_state.gyro_sum[1] += gyro[1];
    gyro_boot_retry_state.gyro_sum[2] += gyro[2];
    gyro_boot_retry_state.accel_norm_sum += acc_norm;
    gyro_boot_retry_state.sample_count++;

    if (gyro_boot_retry_state.sample_count < GYRO_BOOT_CALIB_SAMPLES)
    {
        return;
    }

    const fp32 inv_samples = 1.0f / (fp32)gyro_boot_retry_state.sample_count;
    gyro_cali_offset[0] = gyro_offset[0] = gyro_offset[0] - gyro_boot_retry_state.gyro_sum[0] * inv_samples;
    gyro_cali_offset[1] = gyro_offset[1] = gyro_offset[1] - gyro_boot_retry_state.gyro_sum[1] * inv_samples;
    gyro_cali_offset[2] = gyro_offset[2] = gyro_offset[2] - gyro_boot_retry_state.gyro_sum[2] * inv_samples;
    gyro_boot_accel_norm_ref = gyro_boot_retry_state.accel_norm_sum * inv_samples;
    gyro_boot_accel_norm_valid = 1;
    gyro_boot_initial_result = INS_GYRO_BOOT_INIT_SUCCESS;
    gyro_boot_calibrated = 1;
    gyro_boot_retry_reset();
}

static bool_t gyro_boot_temp_ready(fp32 temp)
{
    static uint32_t stable_start_ms = 0u;
    const fp32 target = (fp32)get_control_temperature();

    if (first_temperate == 0u || !imu_temp_is_valid(temp) || fabsf(temp - target) > GYRO_BOOT_TEMP_STABLE_ERR_C)
    {
        stable_start_ms = 0u;
        return 0u;
    }

    const uint32_t now_ms = HAL_GetTick();
    if (stable_start_ms == 0u)
    {
        stable_start_ms = now_ms;
    }

    return ((uint32_t)(now_ms - stable_start_ms) >= GYRO_BOOT_TEMP_STABLE_TIME_MS) ? 1u : 0u;
}

bool_t ins_is_gyro_boot_calibrated(void)
{
    return gyro_boot_calibrated;
}

bool_t ins_is_gyro_boot_calibrating(void)
{
    return gyro_boot_calibrating;
}

ins_gyro_boot_init_result_e ins_get_gyro_boot_initial_result(void)
{
    return gyro_boot_initial_result;
}


static float imu_calc_dt_s(void)
{
    static uint32_t last_tick_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    float dt;
    if (last_tick_ms == 0)
    {
        dt = timing_time;
    }
    else
    {
        dt = (now_ms - last_tick_ms) * 0.001f;
    }
    last_tick_ms = now_ms;

    // Clamp abnormal dt caused by scheduling jitter.
    if (dt <= 0.0001f || dt > 0.01f)
    {
        dt = timing_time;
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

    // Euler convention: yaw-Z, pitch-Y, roll-X.
    euler[INS_ROLL_ADDRESS_OFFSET] = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    euler[INS_PITCH_ADDRESS_OFFSET] = asinf(sinp);

    const float yaw_raw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)); // [-pi, pi]
    if (!imu_yaw_inited)
    {
        imu_yaw_continuous = yaw_raw;
        imu_yaw_inited = 1;
        imu_angle_lpf_inited = 0;
    }
    else
    {
        float delta = yaw_raw - euler[INS_YAW_ADDRESS_OFFSET];
        // Unwrap yaw to keep it continuous.
        if (delta > M_PI) delta -= 2.0f * M_PI;
        else if (delta < -M_PI) delta += 2.0f * M_PI;
        imu_yaw_continuous += delta;
    }
    euler[INS_YAW_ADDRESS_OFFSET] = imu_yaw_continuous;

    // Apply a one-stage low-pass filter to the Euler angles.
    float raw_angles[3];
    raw_angles[INS_ROLL_ADDRESS_OFFSET] = euler[INS_ROLL_ADDRESS_OFFSET];
    raw_angles[INS_PITCH_ADDRESS_OFFSET] = euler[INS_PITCH_ADDRESS_OFFSET];
    raw_angles[INS_YAW_ADDRESS_OFFSET] = euler[INS_YAW_ADDRESS_OFFSET];

    if (!imu_angle_lpf_inited)
    {
        imu_angle_lpf[0] = raw_angles[0];
        imu_angle_lpf[1] = raw_angles[1];
        imu_angle_lpf[2] = raw_angles[2];
        imu_angle_lpf_inited = 1;
    }
    else
    {
        const float alpha = (2.0f * M_PI * EULER_LPF_FC_HZ * imu_last_dt_s);
        const float a = (alpha > 1.0f) ? 1.0f : alpha;
        for (int i = 0; i < 3; i++)
        {
            imu_angle_lpf[i] += a * (raw_angles[i] - imu_angle_lpf[i]);
        }
    }

    euler[INS_ROLL_ADDRESS_OFFSET] = imu_angle_lpf[INS_ROLL_ADDRESS_OFFSET];
    euler[INS_PITCH_ADDRESS_OFFSET] = imu_angle_lpf[INS_PITCH_ADDRESS_OFFSET];
    euler[INS_YAW_ADDRESS_OFFSET] = imu_angle_lpf[INS_YAW_ADDRESS_OFFSET];
}

static void imu_fusion_reset(imu_fusion_mode_e mode, const fp32 acc[3], const fp32 mag[3])
{
    mahony_imu_init(&imu_mahony, imu_ki);
    imu_gain_state = IMU_ST_DISARMED;
    imu_state_timeout_ms = 0;

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
    imu_last_dt_s = timing_time;

    if (mode == IMU_FUSION_AHRS_9AXIS && acc != NULL && mag != NULL)
    {
        AHRS_init(INS_quat, acc, mag);
        imu_update_euler_from_quat(INS_quat, INS_angle);
    }

    imu_fusion_mode_active = mode;
}
