/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#include "gimbal_task.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "motor_config.h"
#include "user_lib.h"
#include "axis_current_conditioner.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "usb_task.h"
#include "app_watch.h"
#include "sdlog.h"
#include "pitch_cali.h"

#include <string.h>

//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                             \
    {                                                                    \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);            \
                                                                           \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);          \
    }

#define PITCH_KICK_ERR_OFF_RAD 0.008f
#define PITCH_KICK_ERR_ON_RAD  0.025f

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode angle to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          limit angle set in encoder angle mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          在编码角度模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      maxout: pid最大输出
  * @param[in]      intergral_limit: pid最大积分输出
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @param[in]      error_delta: 角速度
  * @retval         pid 输出
  */

/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
/**
  * @brief          云台校准计算
  * @param[in]      gimbal_cali: 校准数据
  * @param[out]     yaw_offset:yaw电机云台中值
  * @param[out]     pitch_offset:pitch 电机云台中值
  * @param[out]     max_yaw:yaw 电机最大机械角度
  * @param[out]     min_yaw: yaw 电机最小机械角度
  * @param[out]     max_pitch: pitch 电机最大机械角度
  * @param[out]     min_pitch: pitch 电机最小机械角度
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);


#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
static fp32 gimbal_pitch_kick_scale(fp32 angle_err);
#endif




//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;
// 调试计数：确认 gimbal_task 主循环是否在运行
volatile uint32_t gimbal_loop_counter = 0;

//motor current 
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;
// 调试 watch：记录最终下发的 yaw/pitch 电流，便于 easy test 等模式观察
volatile int16_t gimbal_watch_yaw_current = 0;
volatile int16_t gimbal_watch_pitch_current = 0;
// Yaw easy test 可调电流（调试器可实时修改），默认 1000
volatile int16_t gimbal_yaw_easytest_current = 3000;

extern shoot_control_t shoot_control;

static void sdlog_pack_pid(sdlog_pid_runtime_t *out, uint16_t pid_id, const pid_type_def *pid)
{
    if (out == NULL || pid == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->pid_id = pid_id;
    out->mode = pid->mode;
    out->set = pid->set;
    out->fdb = pid->fdb;
    out->out = pid->out;
}

static fp32 gimbal_pitch_kick_scale(fp32 angle_err)
{
    const fp32 err_abs = fabsf(angle_err);

    if (err_abs <= PITCH_KICK_ERR_OFF_RAD)
    {
        return 0.0f;
    }
    if (err_abs >= PITCH_KICK_ERR_ON_RAD)
    {
        return 1.0f;
    }

    return (err_abs - PITCH_KICK_ERR_OFF_RAD) / (PITCH_KICK_ERR_ON_RAD - PITCH_KICK_ERR_OFF_RAD);
}

static void sdlog_pack_gimbal_pid(sdlog_pid_runtime_t *out, uint16_t pid_id, const gimbal_PID_t *pid)
{
    if (out == NULL || pid == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->pid_id = pid_id;
    out->set = pid->set;
    out->fdb = pid->get;
    out->out = pid->out;
}


static test_mode_e current_test_mode(void)
{
    return (test_mode_e)g_app_config.test.mode;
}

static void gimbal_apply_test_mode(int16_t *yaw_current, int16_t *pitch_current)
{
    const test_mode_e mode = current_test_mode();

    // 安全模式（遥控下档/离线等）下直接清零，避免测试模式覆盖安全输出
    if (gimbal_behaviour_watch == GIMBAL_ZERO_FORCE)
    {
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }

    switch (mode)
    {
    case TEST_MODE_NONE:
    case TEST_MODE_GIMBAL_DUAL:
    case TEST_MODE_ENTERTAIN:
    case TEST_MODE_PITCH_CALI:
        return;
    case TEST_MODE_YAW_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
        *pitch_current = 0;
        return;
    case TEST_MODE_PITCH_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
        *yaw_current = 0;
        return;
    case TEST_MODE_YAW_EASY_TEST:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = gimbal_yaw_easytest_current;
        *pitch_current = 0;
        return;
    default:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }
}

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    TickType_t last_wake = 0;
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //云台初始化
    gimbal_init(&gimbal_control);
    //shoot init
    //射击初始化
    shoot_init();
    pitch_cali_boot_load();
    last_wake = xTaskGetTickCount();
    // 不再因电机掉线阻塞：仅遥控掉线需要安全保护

    while (1)
    {
        app_watch_task_beat(APP_WATCH_TASK_GIMBAL);
        gimbal_loop_counter++;
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        pitch_cali_tick_pre(&gimbal_control, gimbal_behaviour_watch, current_test_mode());
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
        gimbal_set_control(&gimbal_control);                 //设置云台控制量
        gimbal_control_loop(&gimbal_control);                //云台控制PID计算
        pitch_cali_tick_post(&gimbal_control, gimbal_behaviour_watch, current_test_mode());
        shoot_can_set_current = shoot_control_loop();        // 拨盘电流
        if (YAW_TURN)
        {
            yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
        }
        else
        {
            yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        }

        if (PITCH_TURN)
        {
            pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
        }
        else
        {
            pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
        }

        gimbal_apply_test_mode(&yaw_can_set_current, &pitch_can_set_current);

        yaw_can_set_current = motor_cfg_limit_current_node(&g_app_config.motor.yaw, yaw_can_set_current);
        pitch_can_set_current = motor_cfg_limit_current_node(&g_app_config.motor.pitch, pitch_can_set_current);
        shoot_can_set_current = motor_cfg_limit_current_node(&g_app_config.motor.trigger, shoot_can_set_current);

        sdlog_gimbal_loop_t log = {0};
        log.loop_cnt = gimbal_loop_counter;
        log.yaw_current = yaw_can_set_current;
        log.pitch_current = pitch_can_set_current;
        log.trigger_current = shoot_can_set_current;
        log.yaw_mode = (uint8_t)gimbal_control.gimbal_yaw_motor.gimbal_motor_mode;
        log.pitch_mode = (uint8_t)gimbal_control.gimbal_pitch_motor.gimbal_motor_mode;
        log.shoot_mode = (uint8_t)shoot_control.shoot_mode;
        sdlog_pack_gimbal_pid(&log.yaw_angle_pid, SDLOG_PID_GIMBAL_YAW_ANGLE, &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
        sdlog_pack_pid(&log.yaw_speed_pid, SDLOG_PID_GIMBAL_YAW_SPEED, &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
        sdlog_pack_gimbal_pid(&log.pitch_angle_pid, SDLOG_PID_GIMBAL_PITCH_ANGLE, &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
        sdlog_pack_pid(&log.pitch_speed_pid, SDLOG_PID_GIMBAL_PITCH_SPEED, &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
        sdlog_pack_pid(&log.shoot_trigger_pid, SDLOG_PID_SHOOT_TRIGGER, &shoot_control.trigger_motor_pid);
        if (SDLOG_FRIC_NUM >= 4u)
        {
            sdlog_pack_pid(&log.shoot_fric_pid[0], SDLOG_PID_SHOOT_FRIC1_SPEED, &shoot_control.fric_speed_pid[0]);
            sdlog_pack_pid(&log.shoot_fric_pid[1], SDLOG_PID_SHOOT_FRIC2_SPEED, &shoot_control.fric_speed_pid[1]);
            sdlog_pack_pid(&log.shoot_fric_pid[2], SDLOG_PID_SHOOT_FRIC3_SPEED, &shoot_control.fric_speed_pid[2]);
            sdlog_pack_pid(&log.shoot_fric_pid[3], SDLOG_PID_SHOOT_FRIC4_SPEED, &shoot_control.fric_speed_pid[3]);
        }
        sdlog_write(SDLOG_TAG_GIMBAL_LOOP, &log, (uint16_t)sizeof(log));

        // watch 输出：观察最终下发电流（含测试模式、安全模式及方向翻转后的值）
        gimbal_watch_yaw_current = yaw_can_set_current;
        gimbal_watch_pitch_current = pitch_can_set_current;

        // 仅保留遥控掉线保护：遥控掉线则下发零力矩，其余离线状态不拦截发送
        taskENTER_CRITICAL();
        actuator_cmd_set_trigger_current_can1(shoot_can_set_current);
        actuator_cmd_set_yaw_current_can1(yaw_can_set_current);
        actuator_cmd_set_pitch_current_can1(pitch_can_set_current);
        taskEXIT_CRITICAL();

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        {
            const uint16_t period_ms = (GIMBAL_CONTROL_TIME == 0u) ? 1u : GIMBAL_CONTROL_TIME;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
        }

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          gimbal cali data, set motor offset encode, max and min angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max angle
  * @param[in]      min_yaw:yaw min angle
  * @param[in]      max_yaw:pitch max angle
  * @param[in]      min_yaw:pitch min angle
  * @retval         none
  */
/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_angle = min_pitch;
}


/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

    if (YAW_TURN)
    {
        temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

        if (temp_ecd < 0)
        {
        temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    }
    else
    {

        temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

        if (temp_ecd < 0)
        {
            temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
        
        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    }

    if (PITCH_TURN)

    {
        temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
        temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        if (temp_max_ecd > temp_min_ecd)
        {
            temp_min_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
        *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
    }
    else
    {
        temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
        temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
        *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
    }
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{

    const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //陀螺仪/姿态数据指针获取
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_INS_angle_point = get_INS_angle_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_ENCODE_ANGLE_PID_MAX_OUT, YAW_ENCODE_ANGLE_PID_MAX_IOUT, YAW_ENCODE_ANGLE_PID_KP, YAW_ENCODE_ANGLE_PID_KI, YAW_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_ENCODE_ANGLE_PID_MAX_OUT, PITCH_ENCODE_ANGLE_PID_MAX_IOUT, PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    // 速度环输出直接映射为电流，且后续还会做一次 PITCH_CURRENT_LIMIT 限幅；
    // 若 PID 的 max_out/max_iout 远大于电流限幅，会导致积分风up、抖动明显。
    {
        const fp32 limit = fabsf(PITCH_CURRENT_LIMIT);
        if (limit > 0.0f)
        {
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out = limit;
            }
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout = limit;
            }
        }
    }

    //清除所有PID
    gimbal_total_pid_clear(init);

    // yaw 编码器中位（车身对齐）：常用调参项，统一放到 app_config 里
    init->gimbal_yaw_motor.offset_ecd = g_app_config.gimbal.yaw_middle_ecd;

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.angle_set = init->gimbal_yaw_motor.angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


    init->gimbal_pitch_motor.angle_set = init->gimbal_pitch_motor.angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

    // 默认限幅：若未校准或限幅异常，给 yaw 角设置安全范围，避免 0/脏值卡死
    if ((init->gimbal_yaw_motor.max_angle <= init->gimbal_yaw_motor.min_angle) ||
        ((init->gimbal_yaw_motor.max_angle == 0.0f) && (init->gimbal_yaw_motor.min_angle == 0.0f)) ||
        (fabsf(init->gimbal_yaw_motor.max_angle) > 100.0f) ||
        (fabsf(init->gimbal_yaw_motor.min_angle) > 100.0f))
    {
        init->gimbal_yaw_motor.max_angle = PI;
        init->gimbal_yaw_motor.min_angle = -PI;
    }

    // pitch 软限位：优先用 app_config（配置允许不按大小排序）；若未配置则沿用校准值/兜底放开
    {
        const fp32 cfg_up = PITCH_SOFT_LIMIT_UP;
        const fp32 cfg_down = PITCH_SOFT_LIMIT_DOWN;
        fp32 cfg_min = cfg_up;
        fp32 cfg_max = cfg_down;
        if (cfg_max < cfg_min)
        {
            const fp32 t = cfg_max;
            cfg_max = cfg_min;
            cfg_min = t;
        }
        if ((cfg_max > cfg_min) && (fabsf(cfg_max) < 100.0f) && (fabsf(cfg_min) < 100.0f))
        {
            init->gimbal_pitch_motor.max_angle = cfg_max;
            init->gimbal_pitch_motor.min_angle = cfg_min;
        }
        else
        {
            if ((init->gimbal_pitch_motor.max_angle <= init->gimbal_pitch_motor.min_angle) ||
                ((init->gimbal_pitch_motor.max_angle == 0.0f) && (init->gimbal_pitch_motor.min_angle == 0.0f)) ||
                (fabsf(init->gimbal_pitch_motor.max_angle) > 100.0f) ||
                (fabsf(init->gimbal_pitch_motor.min_angle) > 100.0f))
            {
                init->gimbal_pitch_motor.max_angle = 4.0f * PI;
                init->gimbal_pitch_motor.min_angle = -4.0f * PI;
            }
        }
    }

}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    // pitch：使用 IMU 作为双环反馈
    // 注意：本项目 IMU 安装坐标中，云台 pitch 轴对应 INS_ROLL / gyro X（而不是 INS_PITCH / gyro Y）。
    {
        fp32 pitch_angle_raw = 0.0f;
        fp32 pitch_speed_raw = 0.0f;

        if (feedback_update->gimbal_INS_angle_point != NULL)
        {
            pitch_angle_raw = feedback_update->gimbal_INS_angle_point[INS_ROLL_ADDRESS_OFFSET];
        }
        if (feedback_update->gimbal_INT_gyro_point != NULL)
        {
            pitch_speed_raw = feedback_update->gimbal_INT_gyro_point[INS_GYRO_X_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_pitch_motor.angle = PITCH_TURN ? -pitch_angle_raw : pitch_angle_raw;
        feedback_update->gimbal_pitch_motor.motor_gyro = PITCH_TURN ? -pitch_speed_raw : pitch_speed_raw;
        // 方向约定（以 gimbal_pitch_motor.angle / UART1-VOFA ch0 为准）：
        // - 抬头为正、低头为负；motor_gyro 为 angle 的导数（rad/s）
        // - INS_angle_point[] 是 IMU 原始坐标欧拉角；此处会按 PITCH_TURN 做符号适配，因此可能与 app_watch.imu.angle_deg[] 符号相反
    }

    // yaw 角度/速度：优先 IMU（角度 INS_YAW，角速度 gyro Z）
    {
        fp32 yaw_angle_raw = 0.0f;
        fp32 yaw_speed_raw = 0.0f;

        if (feedback_update->gimbal_INS_angle_point != NULL)
        {
            yaw_angle_raw = feedback_update->gimbal_INS_angle_point[INS_YAW_ADDRESS_OFFSET];
        }

        if (feedback_update->gimbal_INT_gyro_point != NULL)
        {
            yaw_speed_raw = feedback_update->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_yaw_motor.angle = YAW_TURN ? -yaw_angle_raw : yaw_angle_raw;
        feedback_update->gimbal_yaw_motor.motor_gyro = YAW_TURN ? -yaw_speed_raw : yaw_speed_raw;
    }
}

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.angle_set = gimbal_mode_change->gimbal_yaw_motor.angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.angle_set = gimbal_mode_change->gimbal_pitch_motor.angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);

    // Vision -> gimbal: when a new frame arrives (mode 1/2), override yaw/pitch targets.
    VisionToGimbal vision_cmd;
    const uint8_t image_manual_active = (remote_control_get_active_source() == MANUAL_INPUT_SRC_IMAGE) ? 1u : 0u;
    const uint8_t image_auto_aim_on = uart1_image_auto_aim_requested() ? 1u : 0u;
    if (vision_take_latest(&vision_cmd) && (vision_cmd.mode == 1U || vision_cmd.mode == 2U) &&
        (image_manual_active == 0u || image_auto_aim_on != 0u) &&
        set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE &&
        set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE &&
        gimbal_behaviour_watch == GIMBAL_ANGLE &&
        !gimbal_turnaround_is_active())
    {
        set_control->gimbal_yaw_motor.angle_set = rad_format(vision_cmd.yaw);
        // CDC pitch command is inverted to match the board's pitch direction.
        set_control->gimbal_pitch_motor.angle_set = fp32_constrain(-vision_cmd.pitch,
                                                                   set_control->gimbal_pitch_motor.min_angle,
                                                                   set_control->gimbal_pitch_motor.max_angle);
        add_yaw_angle = 0.0f;
        add_pitch_angle = 0.0f;
    }

    // yaw：按模式走角度控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        gimbal_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    // pitch：编码角度控制（外环角度，内环速度），add_pitch_angle 为角度增量
    {
        if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
        {
            set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
        }
        else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
        {
            gimbal_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
        }
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encoder angle to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->angle_set += add;

    // Yaw uses IMU Euler angle in (-PI, PI). To support continuous rotation, always wrap the setpoint
    // instead of clamping at +/- PI.
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_motor->angle_set = rad_format(gimbal_motor->angle_set);
        return;
    }
    // 是否超过最大 最小值
    if (gimbal_motor->angle_set > gimbal_motor->max_angle)
    {
        gimbal_motor->angle_set = gimbal_motor->max_angle;
    }
    else if (gimbal_motor->angle_set < gimbal_motor->min_angle)
    {
        gimbal_motor->angle_set = gimbal_motor->min_angle;
    }
}
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode angle to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->angle, gimbal_motor->angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    // pitch 起步/静摩擦补偿电流（持续叠加到 PID 输出上）：
    // - motor_gyro_set > 0：抬头方向（angle 增大）
    // - motor_gyro_set < 0：低头方向（angle 减小）
    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        const fp32 pitch_err = gimbal_motor->angle_set - gimbal_motor->angle;
        fp32 kick_up = fabsf(PITCH_KICK_UP_CURRENT);
        fp32 kick_down = fabsf(PITCH_KICK_DOWN_CURRENT);

        // Optional pitch feedforward (gravity hold + static friction compensation) from SD calibration.
        fp32 ff_hold = 0.0f;
        fp32 ff_kick_up = 0.0f;
        fp32 ff_kick_down = 0.0f;
        if (pitch_cali_get_comp(gimbal_motor->angle, &ff_hold, &ff_kick_up, &ff_kick_down))
        {
            gimbal_motor->current_set += ff_hold;
            kick_up = fabsf(ff_kick_up);
            kick_down = fabsf(ff_kick_down);
        }

        // 静止附近只保留重力维持电流，避免目标角速度一过零就整块叠加起步电流导致来回颤。
        const fp32 kick_scale = gimbal_pitch_kick_scale(pitch_err);
        kick_up *= kick_scale;
        kick_down *= kick_scale;

        const fp32 current_limit = fabsf(PITCH_CURRENT_LIMIT);

        axis_current_conditioner_info_t limit_info = {0};
        gimbal_motor->current_set = axis_current_conditioner_apply(
            gimbal_motor->current_set,
            gimbal_motor->motor_gyro_set,
            kick_up,
            kick_down,
            gimbal_motor->angle,
            gimbal_motor->min_angle,
            gimbal_motor->max_angle,
            current_limit,
            &gimbal_motor->gimbal_motor_angle_pid.Iout,
            &gimbal_motor->gimbal_motor_gyro_pid.Iout,
            &limit_info);

        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

        if (limit_info.soft_limited || limit_info.current_limited)
        {
            sdlog_gimbal_limit_t log = {0};
            log.axis = 1u;
            log.soft_limited = limit_info.soft_limited;
            log.current_limited = limit_info.current_limited;
            log.angle = gimbal_motor->angle;
            log.angle_min = gimbal_motor->min_angle;
            log.angle_max = gimbal_motor->max_angle;
            log.gyro_set = gimbal_motor->motor_gyro_set;
            log.current_before = limit_info.current_before;
            log.current_after = limit_info.current_after;
            log.current_limit = limit_info.current_limit;
            sdlog_write(SDLOG_TAG_GIMBAL_LIMIT, &log, (uint16_t)sizeof(log));
        }
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_angle_int_1000, pitch_angle_int_1000;
int32_t yaw_angle_set_1000, pitch_angle_set_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_angle_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle * 1000);
    yaw_angle_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_angle_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle * 1000);
    pitch_angle_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
}

#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_tune_get_yaw_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const pid_type_def *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_yaw_angle_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const gimbal_PID_t *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid_type_def *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    gimbal_PID_t *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_yaw_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const pid_type_def *pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_angle_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const gimbal_PID_t *pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid_type_def *dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    {
        fp32 max_out = fabsf(pid->max_out);
        fp32 max_iout = fabsf(pid->max_iout);
        const fp32 limit = fabsf(PITCH_CURRENT_LIMIT);
        if (limit > 0.0f)
        {
            if (max_out > limit)
            {
                max_out = limit;
            }
            if (max_iout > limit)
            {
                max_iout = limit;
            }
        }
        dst->max_out = max_out;
        dst->max_iout = max_iout;
    }
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    gimbal_PID_t *dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_pitch_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}
