/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "can_receive.h"
#include "manual_input.h"
#include "control_input.h"
#include "INS_task.h"
#include "gimbal_control_task.h"


//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)




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

typedef struct
{
    uint16_t freq_hz;
    uint16_t on_ms;
    uint16_t off_ms;
} boot_sound_step_t;

typedef enum
{
    BOOT_SOUND_WAIT_RESULT = 0,
    BOOT_SOUND_PLAYING = 1,
    BOOT_SOUND_DONE = 2,
} boot_sound_state_e;

static void boot_sound_update(void);
static uint8_t boot_sound_has_active_calibration(void);
static void boot_sound_begin(const boot_sound_step_t *seq, uint8_t len);
static uint8_t boot_sound_volume(void);



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif


static head_cali_t     head_cali;       //head cali data
static gimbal_cali_t   gimbal_cali;     //gimbal cali data
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
static imu_cali_t      mag_cali;        //mag cali data


static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};


static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

static uint32_t calibrate_systemTick;
static const boot_sound_step_t boot_sound_success_seq[] = {
    {659u, 105u, 34u},
    {784u, 110u, 36u},
    {988u, 220u, 0u},
};
static boot_sound_state_e boot_sound_state = BOOT_SOUND_WAIT_RESULT;
static const boot_sound_step_t *boot_sound_seq = NULL;
static uint8_t boot_sound_seq_len = 0u;
static uint8_t boot_sound_seq_idx = 0u;
static uint16_t boot_sound_wait_ms = 0u;
static uint8_t boot_sound_note_on = 0u;
static uint8_t manual_cali_buzzer_enable = 0u;


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
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;


    while (1)
    {
        boot_sound_update();

        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
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
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static uint8_t boot_sound_has_active_calibration(void)
{
    for (uint8_t i = 0u; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd != 0u)
        {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t boot_sound_volume(void)
{
    uint8_t volume = (uint8_t)g_config.buzzer.pcm.volume;
    if (volume == 0u)
    {
        volume = 255u;
    }
    return volume;
}

static void boot_sound_begin(const boot_sound_step_t *seq, uint8_t len)
{
    boot_sound_seq = seq;
    boot_sound_seq_len = len;
    boot_sound_seq_idx = 0u;
    boot_sound_wait_ms = 0u;
    boot_sound_note_on = 0u;
    boot_sound_state = BOOT_SOUND_PLAYING;
}

static void boot_sound_update(void)
{
    if (boot_sound_state == BOOT_SOUND_DONE)
    {
        return;
    }

    if (boot_sound_has_active_calibration() != 0u)
    {
        if (boot_sound_note_on != 0u)
        {
            buzzer_tone_stop();
            boot_sound_note_on = 0u;
        }
        return;
    }

    if (boot_sound_state == BOOT_SOUND_WAIT_RESULT)
    {
        const ins_gyro_boot_init_result_e result = ins_get_gyro_boot_initial_result();
        if (result == INS_GYRO_BOOT_INIT_PENDING)
        {
            return;
        }

        if (result == INS_GYRO_BOOT_INIT_SUCCESS)
        {
            boot_sound_begin(boot_sound_success_seq, (uint8_t)(sizeof(boot_sound_success_seq) / sizeof(boot_sound_success_seq[0])));
        }
        else
        {
            boot_sound_state = BOOT_SOUND_DONE;
            return;
        }
    }

    if (boot_sound_state != BOOT_SOUND_PLAYING || boot_sound_seq == NULL || boot_sound_seq_len == 0u)
    {
        boot_sound_state = BOOT_SOUND_DONE;
        return;
    }

    if (boot_sound_wait_ms != 0u)
    {
        boot_sound_wait_ms--;
        return;
    }

    if (boot_sound_note_on == 0u)
    {
        if (boot_sound_seq_idx >= boot_sound_seq_len)
        {
            boot_sound_state = BOOT_SOUND_DONE;
            return;
        }

        (void)buzzer_tone_start_hz(boot_sound_seq[boot_sound_seq_idx].freq_hz, boot_sound_volume());
        boot_sound_note_on = 1u;
        boot_sound_wait_ms = boot_sound_seq[boot_sound_seq_idx].on_ms;
        if (boot_sound_wait_ms == 0u)
        {
            boot_sound_wait_ms = 1u;
        }
        return;
    }

    buzzer_tone_stop();
    boot_sound_note_on = 0u;
    boot_sound_wait_ms = boot_sound_seq[boot_sound_seq_idx].off_ms;
    boot_sound_seq_idx++;

    if (boot_sound_seq_idx >= boot_sound_seq_len && boot_sound_wait_ms == 0u)
    {
        boot_sound_state = BOOT_SOUND_DONE;
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

    return head_cali.temperature;
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
    static const uint8_t GYRO_FLAG    = 3;
    static const uint8_t CHASSIS_FLAG = 4;

    static uint8_t  i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time       = 0;
    static uint16_t rc_cmd_time       = 0;
    static uint8_t  rc_action_flag    = 0;

    //if something is calibrating, return
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_flag = 0;

            return;
        }
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
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
        manual_cali_buzzer_enable = 1u;
        cali_buzzer_off();
    }
    else if (rc_action_flag == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //gyro cali
        rc_action_flag = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        manual_cali_buzzer_enable = 1u;
        //update control temperature
        head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
        cali_buzzer_off();
    }
    else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_action_flag = 0;
        rc_cmd_time = 0;
        //send CAN reset ID cmd to M3508
        //发送CAN重设ID命令到3508
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        cali_buzzer_off();
    }

    const int16_t cali_ch0 = input_axis(INPUT_AXIS_CALIB_0);
    const int16_t cali_ch1 = input_axis(INPUT_AXIS_CALIB_1);
    const int16_t cali_ch2 = input_axis(INPUT_AXIS_CALIB_2);
    const int16_t cali_ch3 = input_axis(INPUT_AXIS_CALIB_3);
    const uint8_t cali_sw_l = input_switch(INPUT_SW_CALIB_L);
    const uint8_t cali_sw_r = input_switch(INPUT_SW_CALIB_R);

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
        rc_cmd_time++;
        rc_action_flag = GYRO_FLAG;
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

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //over 20 seconds, end
        //超过20s,停止
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }

    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
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

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
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
    for (i = 0; i < CALI_LIST_LENGHT; i++)
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

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
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


    for (i = 0; i < CALI_LIST_LENGHT; i++)
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
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
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
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
//        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

    local_cali_t->firmware_version = FIRMWARE_VERSION;
    //shenzhen latitude
    local_cali_t->latitude = 22.0f;

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
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);

        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
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

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);

        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        {
            cali_buzzer_off();
            manual_cali_buzzer_enable = 0u;

            return 1;
        }
        else
        {
            if (manual_cali_buzzer_enable != 0u)
            {
                gimbal_start_buzzer();
            }

            return 0;
        }
    }

    return 0;
}
