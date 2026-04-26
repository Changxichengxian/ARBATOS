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

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "calibrate_task.h"
#include "detect_task.h"
#include "app_config.h"
#include "bsp_key.h"
#include "AHRS.h"
#include "sdlog.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

// Mahony 姿态滤波参数（取无人机代码默认值）
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

// 加速度计可信度判定
#define ACC_HEALTH_MIN_G2 0.81f
#define ACC_HEALTH_MAX_G2 1.21f
#define ACC_TRUST_FULL_ANGLE_DEG  6.0f
#define ACC_TRUST_ZERO_ANGLE_DEG  18.0f
#define ACC_TRUST_FULL_ERR_G      0.02f
#define ACC_TRUST_ZERO_ERR_G      0.08f
#define ACC_DISTURB_HOLD_MS       180U
#define GRAVITY_EARTH             9.80665f

// 上电静止陀螺零偏校准（取无人机代码思路）
#define GYRO_BOOT_CALIB_SAMPLES        2000U
#define GYRO_BOOT_CALIB_MOVING_LIMIT_DPS 5.0f
#define GYRO_BOOT_CALIB_DELAY_MS       1U
#define GYRO_BOOT_CALIB_ACC_TOL_G      0.05f


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
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
 */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
 */
static void imu_cmd_spi_dma(void);

/**
  * @brief          上电静止时对陀螺仪做均值零偏校准（取无人机姿态解算中的做法）
  * @retval         true 校准成功并写入 gyro_offset；false 检测到运动，未更新
  */
static bool gyro_boot_calibration(void);
static void gyro_boot_retry_reset(void);
static void gyro_boot_retry_update(const fp32 gyro[3], const fp32 acc[3]);

static float imu_calc_dt_s(void);
static bool_t imu_acc_is_healthy(const fp32 acc[3]);
typedef struct
{
    float acc_norm_g;
    float acc_ref_g;
    float norm_err_g;
    float angle_deg;
    float trust;
    bool_t acc_healthy;
    bool_t acc_rejected;
} imu_acc_debug_t;
static float imu_calc_acc_trust(const fp32 quat[4], const fp32 acc[3], uint32_t now_ms, imu_acc_debug_t *debug);
static void imu_get_gravity_dir_from_quat(const fp32 quat[4], fp32 gravity_dir[3]);
static void imu_mix_accel_for_fusion(const fp32 quat[4], const fp32 raw_acc[3], fp32 fused_acc[3], float trust);
static float imu_calc_dynamic_kp(bool_t use_acc, const fp32 gyro[3], uint32_t now_ms);
static void imu_update_euler_from_quat(const fp32 quat[4], fp32 euler[3]);
static void imu_fusion_reset(imu_fusion_mode_e mode, const fp32 acc[3], const fp32 mag[3]);



extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const imu_config_t *const imu_cfg = &g_app_config.imu;
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s


//加速度计低通滤波
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static second_order_filter_type_t accel_filter[3];
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad

// Mahony 状态
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
static uint32_t imu_acc_reject_until_ms = 0;
static fp32 accel_fusion_3[3] = {0.0f, 0.0f, 0.0f};
static float imu_yaw_continuous = 0.0f;
static bool_t imu_yaw_inited = 0;
static fp32 imu_angle_lpf[3] = {0.0f, 0.0f, 0.0f};
static bool_t imu_angle_lpf_inited = 0;
static float imu_last_dt_s = 0.001f;
static imu_fusion_mode_e imu_fusion_mode_active = IMU_FUSION_MAHONY_6AXIS;
static uint32_t imu_trust_log_tick_ms = 0;





/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void INS_task(void const *pvParameters)
{
    //wait a time
    osDelay(imu_cfg->task_init_time_ms);
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    ist8310_read_mag(ist8310_real_data.mag);
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    // 上电静止均值零偏校准：静止且陀螺 <5 dps 时累积 2000 次
    // 运动时校准返回 false，保留原偏置（默认 0 或 flash 传入）
    gyro_boot_calibrating = 1;
    gyro_boot_calibrated = gyro_boot_calibration();
    gyro_boot_calibrating = 0;
    gyro_boot_initial_result = gyro_boot_calibrated ? INS_GYRO_BOOT_INIT_SUCCESS : INS_GYRO_BOOT_INIT_FAILED;

    //rotate and zero drift 
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    const fp32 imu_temp_PID[3] = {imu_cfg->temperature_pid.kp, imu_cfg->temperature_pid.ki, imu_cfg->temperature_pid.kd};
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, imu_cfg->temperature_pid_max_out, imu_cfg->temperature_pid_max_iout);

    for (uint8_t i = 0u; i < 3u; i++)
    {
        accel_fliter_3[i] = INS_accel[i];
        accel_fusion_3[i] = INS_accel[i];
        second_order_filter_init(&accel_filter[i], fliter_num, INS_accel[i]);
    }
    imu_fusion_reset(imu_cfg->fusion_mode, accel_fliter_3, INS_mag);
    //get the handle of task
    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    
    while (1)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        if (mag_update_flag & (1 << IMU_DR_SHFITS))
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            ist8310_read_mag(ist8310_real_data.mag);
        }

        //rotate and zero drift 
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


        //加速度计低通滤波
        //accel low-pass filter
        for (uint8_t i = 0u; i < 3u; i++)
        {
            accel_fliter_3[i] = second_order_filter_cali(&accel_filter[i], INS_accel[i]);
        }
        if (!gyro_boot_calibrated)
        {
            gyro_boot_retry_update(INS_gyro, accel_fliter_3);
        }

        const imu_fusion_mode_e mode = (imu_fusion_mode_e)imu_cfg->fusion_mode;
        if (mode != imu_fusion_mode_active)
        {
            if (mode == IMU_FUSION_AHRS_9AXIS)
            {
                ist8310_read_mag(ist8310_real_data.mag);
                imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
            }
            imu_fusion_reset(mode, accel_fliter_3, INS_mag);
        }


        // dt 计算（ms tick），异常值回退到 1ms
        const uint32_t now_ms = HAL_GetTick();
        const float dt = imu_calc_dt_s();
        imu_acc_debug_t acc_debug = {0};
        const float acc_trust = imu_calc_acc_trust(INS_quat, accel_fliter_3, now_ms, &acc_debug);
        imu_mix_accel_for_fusion(INS_quat, accel_fliter_3, accel_fusion_3, acc_trust);
        float kp_gain = 0.0f;

        if (imu_fusion_mode_active == IMU_FUSION_AHRS_9AXIS)
        {
            (void)AHRS_update(INS_quat, dt, INS_gyro, accel_fusion_3, INS_mag);
            imu_update_euler_from_quat(INS_quat, INS_angle);
        }
        else
        {
            const bool_t acc_healthy = (acc_trust > 0.05f) ? 1 : 0;
            kp_gain = imu_calc_dynamic_kp(acc_healthy, INS_gyro, now_ms) * acc_trust;

            mahony_imu_update(&imu_mahony, dt, INS_gyro, accel_fusion_3, acc_healthy, kp_gain);

            INS_quat[0] = imu_mahony.quat[0];
            INS_quat[1] = imu_mahony.quat[1];
            INS_quat[2] = imu_mahony.quat[2];
            INS_quat[3] = imu_mahony.quat[3];
            imu_update_euler_from_quat(INS_quat, INS_angle);
        }

        sdlog_imu_t pkt = {0};
        for (uint8_t i = 0; i < 4u; i++)
        {
            pkt.quat[i] = INS_quat[i];
        }
        for (uint8_t i = 0; i < 3u; i++)
        {
            pkt.gyro[i] = INS_gyro[i];
            pkt.accel[i] = INS_accel[i];
        }
        pkt.temp = bmi088_real_data.temp;
        sdlog_write(SDLOG_TAG_IMU, &pkt, (uint16_t)sizeof(pkt));

        sdlog_pid_runtime_t pidlog = {0};
        pidlog.pid_id = SDLOG_PID_IMU_TEMP;
        pidlog.mode = imu_temp_pid.mode;
        pidlog.set = imu_temp_pid.set;
        pidlog.fdb = imu_temp_pid.fdb;
        pidlog.out = imu_temp_pid.out;
        sdlog_write(SDLOG_TAG_PID, &pidlog, (uint16_t)sizeof(pidlog));

        if ((uint32_t)(now_ms - imu_trust_log_tick_ms) >= 10u)
        {
            imu_trust_log_tick_ms = now_ms;

            const float gyro_norm_dps = sqrtf(INS_gyro[0] * INS_gyro[0] +
                                              INS_gyro[1] * INS_gyro[1] +
                                              INS_gyro[2] * INS_gyro[2]) * RAD_TO_DEG;

            sdlog_imu_trust_t trust_log = {0};
            trust_log.acc_norm_g = acc_debug.acc_norm_g;
            trust_log.acc_ref_g = acc_debug.acc_ref_g;
            trust_log.norm_err_g = acc_debug.norm_err_g;
            trust_log.angle_deg = acc_debug.angle_deg;
            trust_log.trust = acc_debug.trust;
            trust_log.kp_gain = kp_gain;
            trust_log.gyro_norm_dps = gyro_norm_dps;
            trust_log.acc_healthy = (uint8_t)acc_debug.acc_healthy;
            trust_log.acc_rejected = (uint8_t)acc_debug.acc_rejected;
            trust_log.fusion_mode = (uint8_t)imu_fusion_mode_active;
            sdlog_write(SDLOG_TAG_IMU_TRUST, &trust_log, (uint16_t)sizeof(trust_log));
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
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > get_control_temperature())
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
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
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
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
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
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
/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
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
  * @brief          获取欧拉角, 0:yaw, 1:roll, 2:pitch 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
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
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
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
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
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
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        // Board key (EXTI0 mapped to PG0).
        bsp_key_exti0_callback();
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    



    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);

            // Wake INS_task directly from DMA ISR (avoid occupying EXTI0/GPIO0).
            if (INS_task_local_handler != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}


static bool gyro_boot_calibration(void)
{
    fp32 gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    fp32 gyro_raw[3];
    fp32 accel_raw[3];
    fp32 accel_norm_sum = 0.0f;
    fp32 temp;
    const fp32 move_limit_rad = GYRO_BOOT_CALIB_MOVING_LIMIT_DPS * DEG_TO_RAD;
    gyro_boot_accel_norm_ref = GRAVITY_EARTH;
    gyro_boot_accel_norm_valid = 0;

    // 静止均值法：若检测到运动则退出并不更新偏置
    for (uint16_t i = 0; i < GYRO_BOOT_CALIB_SAMPLES; i++)
    {
        BMI088_read(gyro_raw, accel_raw, &temp);

        // 旋转到板载坐标系（与运行时一致）
        fp32 gyro_rot[3];
        gyro_rot[0] = gyro_raw[0] * gyro_scale_factor[0][0] + gyro_raw[1] * gyro_scale_factor[0][1] + gyro_raw[2] * gyro_scale_factor[0][2];
        gyro_rot[1] = gyro_raw[0] * gyro_scale_factor[1][0] + gyro_raw[1] * gyro_scale_factor[1][1] + gyro_raw[2] * gyro_scale_factor[1][2];
        gyro_rot[2] = gyro_raw[0] * gyro_scale_factor[2][0] + gyro_raw[1] * gyro_scale_factor[2][1] + gyro_raw[2] * gyro_scale_factor[2][2];
        fp32 accel_rot[3];
        accel_rot[0] = accel_raw[0] * accel_scale_factor[0][0] + accel_raw[1] * accel_scale_factor[0][1] + accel_raw[2] * accel_scale_factor[0][2] + accel_offset[0];
        accel_rot[1] = accel_raw[0] * accel_scale_factor[1][0] + accel_raw[1] * accel_scale_factor[1][1] + accel_raw[2] * accel_scale_factor[1][2] + accel_offset[1];
        accel_rot[2] = accel_raw[0] * accel_scale_factor[2][0] + accel_raw[1] * accel_scale_factor[2][1] + accel_raw[2] * accel_scale_factor[2][2] + accel_offset[2];

        if (fabsf(gyro_rot[0]) > move_limit_rad || fabsf(gyro_rot[1]) > move_limit_rad || fabsf(gyro_rot[2]) > move_limit_rad)
        {
            return false;
        }

        gyro_sum[0] += gyro_rot[0];
        gyro_sum[1] += gyro_rot[1];
        gyro_sum[2] += gyro_rot[2];
        accel_norm_sum += sqrtf(accel_rot[0] * accel_rot[0] +
                                accel_rot[1] * accel_rot[1] +
                                accel_rot[2] * accel_rot[2]);

        osDelay(GYRO_BOOT_CALIB_DELAY_MS);
    }

    const fp32 inv_samples = 1.0f / (fp32)GYRO_BOOT_CALIB_SAMPLES;
    gyro_cali_offset[0] = gyro_offset[0] = -gyro_sum[0] * inv_samples;
    gyro_cali_offset[1] = gyro_offset[1] = -gyro_sum[1] * inv_samples;
    gyro_cali_offset[2] = gyro_offset[2] = -gyro_sum[2] * inv_samples;
    gyro_boot_accel_norm_ref = accel_norm_sum * inv_samples;
    gyro_boot_accel_norm_valid = 1;

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

ins_gyro_boot_init_result_e ins_get_gyro_boot_initial_result(void)
{
    return gyro_boot_initial_result;
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
    gyro_cali_offset[0] = gyro_offset[0] = -gyro_boot_retry_state.gyro_sum[0] * inv_samples;
    gyro_cali_offset[1] = gyro_offset[1] = -gyro_boot_retry_state.gyro_sum[1] * inv_samples;
    gyro_cali_offset[2] = gyro_offset[2] = -gyro_boot_retry_state.gyro_sum[2] * inv_samples;
    gyro_boot_accel_norm_ref = gyro_boot_retry_state.accel_norm_sum * inv_samples;
    gyro_boot_accel_norm_valid = 1;
    gyro_boot_calibrated = 1;
    gyro_boot_retry_reset();
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

    // 防止调度抖动导致异常 dt
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

static void imu_get_gravity_dir_from_quat(const fp32 quat[4], fp32 gravity_dir[3])
{
    if (gravity_dir == NULL)
    {
        return;
    }

    if (quat == NULL)
    {
        gravity_dir[0] = 0.0f;
        gravity_dir[1] = 0.0f;
        gravity_dir[2] = 1.0f;
        return;
    }

    const float qw = quat[0];
    const float qx = quat[1];
    const float qy = quat[2];
    const float qz = quat[3];

    gravity_dir[0] = 2.0f * (qx * qz - qw * qy);
    gravity_dir[1] = 2.0f * (qw * qx + qy * qz);
    gravity_dir[2] = qw * qw - qx * qx - qy * qy + qz * qz;

    const float norm = sqrtf(gravity_dir[0] * gravity_dir[0] +
                             gravity_dir[1] * gravity_dir[1] +
                             gravity_dir[2] * gravity_dir[2]);
    if (norm > 1e-6f)
    {
        const float inv_norm = 1.0f / norm;
        gravity_dir[0] *= inv_norm;
        gravity_dir[1] *= inv_norm;
        gravity_dir[2] *= inv_norm;
    }
    else
    {
        gravity_dir[0] = 0.0f;
        gravity_dir[1] = 0.0f;
        gravity_dir[2] = 1.0f;
    }
}

static float imu_calc_acc_trust(const fp32 quat[4], const fp32 acc[3], uint32_t now_ms, imu_acc_debug_t *debug)
{
    const float acc_ref = gyro_boot_accel_norm_valid ? gyro_boot_accel_norm_ref : GRAVITY_EARTH;

    if (debug != NULL)
    {
        memset(debug, 0, sizeof(*debug));
        debug->acc_ref_g = acc_ref / GRAVITY_EARTH;
    }

    if (acc == NULL)
    {
        imu_acc_reject_until_ms = now_ms + ACC_DISTURB_HOLD_MS;
        if (debug != NULL)
        {
            debug->acc_rejected = 1;
        }
        return 0.0f;
    }

    const bool_t acc_healthy = imu_acc_is_healthy(acc);
    if (debug != NULL)
    {
        const float acc_norm_now = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        debug->acc_norm_g = acc_norm_now / GRAVITY_EARTH;
        debug->norm_err_g = fabsf(acc_norm_now - acc_ref) / GRAVITY_EARTH;
        debug->acc_healthy = acc_healthy;
    }

    if (!acc_healthy)
    {
        imu_acc_reject_until_ms = now_ms + ACC_DISTURB_HOLD_MS;
        if (debug != NULL)
        {
            debug->acc_rejected = 1;
        }
        return 0.0f;
    }

    const float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (acc_norm < 1e-3f)
    {
        imu_acc_reject_until_ms = now_ms + ACC_DISTURB_HOLD_MS;
        if (debug != NULL)
        {
            debug->acc_rejected = 1;
        }
        return 0.0f;
    }

    fp32 gravity_dir[3];
    imu_get_gravity_dir_from_quat(quat, gravity_dir);

    float dot = (acc[0] * gravity_dir[0] + acc[1] * gravity_dir[1] + acc[2] * gravity_dir[2]) / acc_norm;
    if (dot > 1.0f)
    {
        dot = 1.0f;
    }
    else if (dot < -1.0f)
    {
        dot = -1.0f;
    }

    const float angle_deg = acosf(dot) * RAD_TO_DEG;
    const float norm_err_g = fabsf(acc_norm - acc_ref) / GRAVITY_EARTH;

    if (debug != NULL)
    {
        debug->acc_norm_g = acc_norm / GRAVITY_EARTH;
        debug->norm_err_g = norm_err_g;
        debug->angle_deg = angle_deg;
    }

    float angle_trust;
    if (angle_deg <= ACC_TRUST_FULL_ANGLE_DEG)
    {
        angle_trust = 1.0f;
    }
    else if (angle_deg >= ACC_TRUST_ZERO_ANGLE_DEG)
    {
        angle_trust = 0.0f;
    }
    else
    {
        angle_trust = (ACC_TRUST_ZERO_ANGLE_DEG - angle_deg) /
                      (ACC_TRUST_ZERO_ANGLE_DEG - ACC_TRUST_FULL_ANGLE_DEG);
    }

    float norm_trust;
    if (norm_err_g <= ACC_TRUST_FULL_ERR_G)
    {
        norm_trust = 1.0f;
    }
    else if (norm_err_g >= ACC_TRUST_ZERO_ERR_G)
    {
        norm_trust = 0.0f;
    }
    else
    {
        norm_trust = (ACC_TRUST_ZERO_ERR_G - norm_err_g) /
                     (ACC_TRUST_ZERO_ERR_G - ACC_TRUST_FULL_ERR_G);
    }

    float trust = (angle_trust < norm_trust) ? angle_trust : norm_trust;

    if (trust < 0.35f)
    {
        imu_acc_reject_until_ms = now_ms + ACC_DISTURB_HOLD_MS;
    }

    if (now_ms < imu_acc_reject_until_ms)
    {
        trust = 0.0f;
    }

    if (debug != NULL)
    {
        debug->trust = trust;
        debug->acc_rejected = (uint8_t)(now_ms < imu_acc_reject_until_ms);
    }

    return trust;
}

static void imu_mix_accel_for_fusion(const fp32 quat[4], const fp32 raw_acc[3], fp32 fused_acc[3], float trust)
{
    if (raw_acc == NULL || fused_acc == NULL)
    {
        return;
    }

    if (trust <= 0.0f)
    {
        trust = 0.0f;
    }
    else if (trust >= 1.0f)
    {
        trust = 1.0f;
    }

    fp32 gravity_dir[3];
    imu_get_gravity_dir_from_quat(quat, gravity_dir);
    const float acc_ref = gyro_boot_accel_norm_valid ? gyro_boot_accel_norm_ref : GRAVITY_EARTH;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        const float gravity_acc = gravity_dir[i] * acc_ref;
        fused_acc[i] = gravity_acc + trust * (raw_acc[i] - gravity_acc);
    }
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
        return imu_kp * 100.0f; // 高速收敛
    case IMU_ST_DISARMED:
    default:
        return imu_kp * 10.0f; // 日常抗漂
    }
}

static void imu_update_euler_from_quat(const fp32 quat[4], fp32 euler[3])
{
    const float qw = quat[0];
    const float qx = quat[1];
    const float qy = quat[2];
    const float qz = quat[3];

    // 参考航空角定义：yaw-Z, pitch-Y, roll-X
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
        imu_angle_lpf_inited = 0; // 将首次滤波推迟到 yaw 连续基准就绪
    }
    else
    {
        float delta = yaw_raw - euler[INS_YAW_ADDRESS_OFFSET];
        // 解包角，保持连续
        if (delta > M_PI) delta -= 2.0f * M_PI;
        else if (delta < -M_PI) delta += 2.0f * M_PI;
        imu_yaw_continuous += delta;
    }
    euler[INS_YAW_ADDRESS_OFFSET] = imu_yaw_continuous;

    // 一阶低通滤波欧拉角，减少抖动
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
    imu_acc_reject_until_ms = 0;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        accel_fusion_3[i] = (acc != NULL) ? acc[i] : 0.0f;
    }

    if (acc != NULL && mag != NULL)
    {
        AHRS_init(INS_quat, acc, mag);
        imu_mahony.quat[0] = INS_quat[0];
        imu_mahony.quat[1] = INS_quat[1];
        imu_mahony.quat[2] = INS_quat[2];
        imu_mahony.quat[3] = INS_quat[3];
        imu_update_euler_from_quat(INS_quat, INS_angle);
    }

    imu_fusion_mode_active = mode;
}
