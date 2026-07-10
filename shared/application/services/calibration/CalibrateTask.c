/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "CalibrateTask.h"
#include "string.h"
#include "cmsis_os.h"

#include "BspAdc.h"
#include "BspBuzzer.h"
#include "BspFlash.h"

#include "CanReceive.h"
#include "ManualInputSnapshot.h"
#include "InsTask.h"
#include "GimbalControlTask.h"
#include "RobotTaskProfile.h"


//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGTH) devices, need data length + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGTH  (sizeof(head_cali_t) + sizeof(GimbalCali) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGTH * 4)




/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void);

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void);


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          head device calibration.
  * @param[in][out] cali: head calibration data pointer.
  * @param[in]      cmd: CALI_FUNC_CMD_INIT initializes from saved data; CALI_FUNC_CMD_ON runs calibration.
  * @retval         0: calibration not finished
                    1: calibration finished
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //header device cali function

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: means calibration is required
  * @retval         0: calibration not finished
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //gyro device cali function

/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: means calibration is required
  * @retval         0: calibration not finished
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //gimbal device cali function

static uint8_t cali_gimbal_profile_enabled(void);

__weak void set_cali_gimbal_hook(const uint16_t yaw_offset,
                                 const uint16_t pitch_offset,
                                 const fp32 max_yaw,
                                 const fp32 min_yaw,
                                 const fp32 max_pitch,
                                 const fp32 min_pitch)
{
    (void)yaw_offset;
    (void)pitch_offset;
    (void)max_yaw;
    (void)min_yaw;
    (void)max_pitch;
    (void)min_pitch;
}

__weak bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset,
                                   uint16_t *pitch_offset,
                                   fp32 *max_yaw,
                                   fp32 *min_yaw,
                                   fp32 *max_pitch,
                                   fp32 *min_pitch)
{
    (void)yaw_offset;
    (void)pitch_offset;
    (void)max_yaw;
    (void)min_yaw;
    (void)max_pitch;
    (void)min_pitch;
    return 1;
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t CalibrateTaskStack;
#endif


static head_cali_t     head_cali;       //head cali data
static GimbalCali   s_gimbal_cal_data;     //gimbal cali data
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
static imu_cali_t      mag_cali;        //mag cali data


static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGTH];

cali_sensor_t cali_sensor[CALI_LIST_LENGTH];

static const uint8_t cali_name[CALI_LIST_LENGTH][3] = {
        {'H', 'D', 0u},
        {'G', 'M', 0u},
        {'G', 'Y', 'R'},
        {'A', 'C', 'C'},
        {'M', 'A', 'G'}};

//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGTH] = {
        (uint32_t *)&head_cali, (uint32_t *)&s_gimbal_cal_data,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};


static uint8_t cali_sensor_size[CALI_LIST_LENGTH] =
    {
        sizeof(head_cali_t) / 4, sizeof(GimbalCali) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGTH] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

static uint32_t CalibrateSystemTick;
static uint8_t manual_cali_buzzer_enable = 0u;

static uint8_t cali_gimbal_profile_enabled(void)
{
    return (uint8_t)(RobotProfileNeedSingleGimbalControlTask() ||
                     RobotProfileNeedDualGimbalControlTask());
}


/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          校准任务，由main函数创建
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void CalibrateTask(void const *pvParameters)
{
    static uint8_t i = 0;


    while (1)
    {
        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGTH; i++)
        {
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {

                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //write
                        cali_data_write();
                    }
                }
            }
        }
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        CalibrateTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          get imu control temperature, unit ℃
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
int8_t get_control_temperature(void)
{
    return 40;
}

/**
  * @brief          get latitude, default 22.0f
  * @param[out]     latitude: the point to fp32
  * @retval         none
  */
/**
  * @brief          获取纬度,默认22.0f
  * @param[out]     latitude:fp32指针
  * @retval         none
  */
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL)
    {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = 22.0f;
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void)
{
    static const uint8_t BEGIN_FLAG   = 1;
    static const uint8_t GIMBAL_FLAG  = 2;
    static const uint8_t CHASSIS_FLAG = 4;

    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t BuzzerTime       = 0;
    static uint16_t rc_cmd_time       = 0;
    static uint8_t  rc_action_flag    = 0;
    static uint8_t  rc_buzzer_owned   = 0u;
    static uint32_t last_authority_seq = 0u;
    static uint32_t last_semantics_seq = 0u;
    ManualInputSnapshot manualInput;
    const uint8_t inputValid = ManualInputSnapshotRead(&manualInput);

    //if something is calibrating, return
    for (uint8_t sensorIndex = 0u; sensorIndex < CALI_LIST_LENGTH; sensorIndex++)
    {
        if (cali_sensor[sensorIndex].cali_cmd)
        {
            BuzzerTime = 0;
            rc_cmd_time = 0;
            rc_action_flag = 0;
            if (rc_buzzer_owned != 0u)
            {
                cali_buzzer_off();
                rc_buzzer_owned = 0u;
            }

            return;
        }
    }

    /* 离线帧不能延续上一段长按计时，也不能在恢复时补触发校准。 */
    if (inputValid == 0u || manualInput.online == 0u)
    {
        BuzzerTime = 0u;
        rc_cmd_time = 0u;
        rc_cmd_systemTick = 0u;
        rc_action_flag = 0u;
        if (rc_buzzer_owned != 0u)
        {
            cali_buzzer_off();
            rc_buzzer_owned = 0u;
        }
        return;
    }

    /* 真正控制来源或解释变化后，旧来源的半段校准手势不能接着计时。 */
    if (last_authority_seq != manualInput.authoritySeq ||
        last_semantics_seq != manualInput.semanticsSeq)
    {
        last_authority_seq = manualInput.authoritySeq;
        last_semantics_seq = manualInput.semanticsSeq;
        BuzzerTime = 0u;
        rc_cmd_time = 0u;
        rc_cmd_systemTick = 0u;
        rc_action_flag = 0u;
        if (rc_buzzer_owned != 0u)
        {
            cali_buzzer_off();
            rc_buzzer_owned = 0u;
        }
        return;
    }

    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_flag = BEGIN_FLAG;
        rc_cmd_time = 0;
    }
    else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //gimbal cali,
        rc_action_flag = 0;
        rc_cmd_time = 0;
        if (cali_gimbal_profile_enabled() != 0u)
        {
            cali_sensor[CALI_GIMBAL].cali_cmd = 1;
            manual_cali_buzzer_enable = 1u;
        }
        if (rc_buzzer_owned != 0u)
        {
            cali_buzzer_off();
            rc_buzzer_owned = 0u;
        }
    }
    else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_action_flag = 0;
        rc_cmd_time = 0;
        if (RobotProfileNeedClassicChassisControlTask() != 0u)
        {
            //send CAN reset ID cmd to M3508
            //发送CAN重设ID命令到3508
            CAN_cmd_chassis_reset_ID();
            CAN_cmd_chassis_reset_ID();
            CAN_cmd_chassis_reset_ID();
        }
        if (rc_buzzer_owned != 0u)
        {
            cali_buzzer_off();
            rc_buzzer_owned = 0u;
        }
    }

    const int16_t cali_ch0 = manualInput.control.axis[INPUT_AXIS_CALIB_0];
    const int16_t cali_ch1 = manualInput.control.axis[INPUT_AXIS_CALIB_1];
    const int16_t cali_ch2 = manualInput.control.axis[INPUT_AXIS_CALIB_2];
    const int16_t cali_ch3 = manualInput.control.axis[INPUT_AXIS_CALIB_3];
    const uint8_t cali_sw_l = manualInput.control.sw[INPUT_SW_CALIB_L];
    const uint8_t cali_sw_r = manualInput.control.sw[INPUT_SW_CALIB_R];

    if (cali_ch0 < -RC_CALI_VALUE_HOLE && cali_ch1 < -RC_CALI_VALUE_HOLE && cali_ch2 > RC_CALI_VALUE_HOLE && cali_ch3 < -RC_CALI_VALUE_HOLE && switch_is_down(cali_sw_l) && switch_is_down(cali_sw_r) && rc_action_flag == 0)
    {
        //two rockers set to  \../, hold for 2 seconds,
        //两个摇杆打成 \../,保持2s
        rc_cmd_time++;
    }
    else if (cali_ch0 > RC_CALI_VALUE_HOLE && cali_ch1 > RC_CALI_VALUE_HOLE && cali_ch2 < -RC_CALI_VALUE_HOLE && cali_ch3 > RC_CALI_VALUE_HOLE && switch_is_down(cali_sw_l) && switch_is_down(cali_sw_r) && rc_action_flag != 0)
    {
        //two rockers set '\/', hold for 2 seconds
        //两个摇杆打成'\/',保持2s
        rc_cmd_time++;
        rc_action_flag = GIMBAL_FLAG;
    }
    else if (cali_ch0 > RC_CALI_VALUE_HOLE && cali_ch1 < -RC_CALI_VALUE_HOLE && cali_ch2 < -RC_CALI_VALUE_HOLE && cali_ch3 < -RC_CALI_VALUE_HOLE && switch_is_down(cali_sw_l) && switch_is_down(cali_sw_r) && rc_action_flag != 0)
    {
        //two rocker set to ./\., hold for 2 seconds
        //两个摇杆打成./\.,保持2s
        rc_cmd_time = 0;
        rc_action_flag = 0;
    }
    else if (cali_ch0 < -RC_CALI_VALUE_HOLE && cali_ch1 > RC_CALI_VALUE_HOLE && cali_ch2 > RC_CALI_VALUE_HOLE && cali_ch3 > RC_CALI_VALUE_HOLE && switch_is_down(cali_sw_l) && switch_is_down(cali_sw_r) && rc_action_flag != 0)
    {
        //two rocker set to /''\, hold for 2 seconds
        //两个摇杆打成/''\,保持2s
        rc_cmd_time++;
        rc_action_flag = CHASSIS_FLAG;
    }
    else
    {
        rc_cmd_time = 0;
    }

    CalibrateSystemTick = xTaskGetTickCount();

    if (CalibrateSystemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //over 20 seconds, end
        //超过20s,停止
        rc_action_flag = 0;
        if (rc_buzzer_owned != 0u)
        {
            cali_buzzer_off();
            rc_buzzer_owned = 0u;
        }
        return;
    }
    else if (CalibrateSystemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_middle_on();
        rc_buzzer_owned = 1u;
    }
    else if (CalibrateSystemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_start_on();
        rc_buzzer_owned = 1u;
    }

    if (rc_action_flag != 0)
    {
        BuzzerTime++;
    }

    if (BuzzerTime > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        BuzzerTime = 0;
    }
    if (BuzzerTime > RC_CALI_BUZZER_PAUSE_TIME &&
        rc_action_flag != 0 && rc_buzzer_owned != 0u)
    {
        cali_buzzer_off();
        rc_buzzer_owned = 0u;
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    if (cali_gimbal_profile_enabled() == 0u)
    {
        cali_sensor[CALI_GIMBAL].cali_hook = NULL;
        cali_sensor[CALI_GIMBAL].cali_cmd = 0u;
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {

        //read the data in flash,
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);

        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);

        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL && i != (uint8_t)CALI_GYRO)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;


    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
        //copy the data of device calibration data
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }

    //erase the page
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGTH + 3) / 4);
}

bool_t CalibrateGyroOffsetSave(const fp32 offset[3])
{
    if (offset == NULL)
    {
        return 0;
    }
    if (cali_sensor[CALI_GYRO].flash_buf == NULL || cali_sensor[CALI_GYRO].flash_len == 0u)
    {
        return 0;
    }

    gyro_cali.offset[0] = offset[0];
    gyro_cali.offset[1] = offset[1];
    gyro_cali.offset[2] = offset[2];
    gyro_cali.scale[0] = 1.0f;
    gyro_cali.scale[1] = 1.0f;
    gyro_cali.scale[2] = 1.0f;

    cali_sensor[CALI_GYRO].name[0] = cali_name[CALI_GYRO][0];
    cali_sensor[CALI_GYRO].name[1] = cali_name[CALI_GYRO][1];
    cali_sensor[CALI_GYRO].name[2] = cali_name[CALI_GYRO][2];
    cali_sensor[CALI_GYRO].cali_done = CALIED_FLAG;
    cali_sensor[CALI_GYRO].cali_cmd = 0u;

    gyro_set_cali(gyro_cali.scale, gyro_cali.offset);
    cali_data_write();
    return 1;
}


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: means calibration is required
  * @retval         0: calibration not finished
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *head = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
//        memcpy(&head_cali, head, sizeof(head_cali_t));

        return 1;
    }
    // self id
    head->self_id = SELF_ID;
    //imu control temperature
    head->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (head->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        head->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

    head->firmware_version = FIRMWARE_VERSION;
    //shenzhen latitude
    head->latitude = 22.0f;

    return 1;
}

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: means calibration is required
  * @retval         0: calibration not finished
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *imu = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(imu->scale, imu->offset);

        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(imu->scale, imu->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            manual_cali_buzzer_enable = 0u;
            gyro_cali_enable_control();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //disable the remote control to make robot no move
            if (manual_cali_buzzer_enable != 0u)
            {
                imu_start_buzzer();
            }

            return 0;
        }
    }

    return 0;
}

/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: means calibration is required
  * @retval         0: calibration not finished
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    GimbalCali *gimbal = (GimbalCali *)cali;
    if (cali_gimbal_profile_enabled() == 0u)
    {
        return 1;
    }

    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(gimbal->yaw_offset, gimbal->pitch_offset,
                             gimbal->yaw_max_angle, gimbal->yaw_min_angle,
                             gimbal->pitch_max_angle, gimbal->pitch_min_angle);

        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&gimbal->yaw_offset, &gimbal->pitch_offset,
                                 &gimbal->yaw_max_angle, &gimbal->yaw_min_angle,
                                 &gimbal->pitch_max_angle, &gimbal->pitch_min_angle))
        {
            cali_buzzer_off();
            manual_cali_buzzer_enable = 0u;

            return 1;
        }
        else
        {
            if (manual_cali_buzzer_enable != 0u)
            {
                GimbalStartBuzzer();
            }

            return 0;
        }
    }

    return 0;
}
