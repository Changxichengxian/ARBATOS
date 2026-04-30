/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "remote_control.h"
#include "config.h"
//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        (g_config.gimbal.pitch_speed_pid.kp)
#define PITCH_SPEED_PID_KI        (g_config.gimbal.pitch_speed_pid.ki)
#define PITCH_SPEED_PID_KD        (g_config.gimbal.pitch_speed_pid.kd)
#define PITCH_SPEED_PID_MAX_OUT   (g_config.gimbal.pitch_speed_pid.max_out)
#define PITCH_SPEED_PID_MAX_IOUT  (g_config.gimbal.pitch_speed_pid.max_iout)

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        (g_config.gimbal.yaw_speed_pid.kp)
#define YAW_SPEED_PID_KI        (g_config.gimbal.yaw_speed_pid.ki)
#define YAW_SPEED_PID_KD        (g_config.gimbal.yaw_speed_pid.kd)
#define YAW_SPEED_PID_MAX_OUT   (g_config.gimbal.yaw_speed_pid.max_out)
#define YAW_SPEED_PID_MAX_IOUT  (g_config.gimbal.yaw_speed_pid.max_iout)

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_ANGLE_PID_KP (g_config.gimbal.pitch_encode_angle_pid.kp)
#define PITCH_ENCODE_ANGLE_PID_KI (g_config.gimbal.pitch_encode_angle_pid.ki)
#define PITCH_ENCODE_ANGLE_PID_KD (g_config.gimbal.pitch_encode_angle_pid.kd)

#define PITCH_ENCODE_ANGLE_PID_MAX_OUT (g_config.gimbal.pitch_encode_angle_pid.max_out)
#define PITCH_ENCODE_ANGLE_PID_MAX_IOUT (g_config.gimbal.pitch_encode_angle_pid.max_iout)

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_ANGLE_PID_KP        (g_config.gimbal.yaw_encode_angle_pid.kp)
#define YAW_ENCODE_ANGLE_PID_KI        (g_config.gimbal.yaw_encode_angle_pid.ki)
#define YAW_ENCODE_ANGLE_PID_KD        (g_config.gimbal.yaw_encode_angle_pid.kd)
#define YAW_ENCODE_ANGLE_PID_MAX_OUT   (g_config.gimbal.yaw_encode_angle_pid.max_out)
#define YAW_ENCODE_ANGLE_PID_MAX_IOUT  (g_config.gimbal.yaw_encode_angle_pid.max_iout)


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME (g_config.gimbal.task_init_time_ms)
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   (g_config.gimbal.channel_yaw)
#define PITCH_CHANNEL (g_config.gimbal.channel_pitch)
#define GIMBAL_MODE_CHANNEL (g_config.gimbal.channel_mode)

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD (g_config.gimbal.turn_key_mask)
//turn speed
//掉头云台速度
#define TURN_SPEED    (g_config.gimbal.turn_speed)
//测试按键尚未使用
#define TEST_KEYBOARD (g_config.gimbal.test_key_mask)
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   (g_config.gimbal.rc_deadband)


#define YAW_RC_SEN    (g_config.gimbal.yaw_rc_sen)
#define PITCH_RC_SEN  (g_config.gimbal.pitch_rc_sen)

#define YAW_MOUSE_SEN   (g_config.gimbal.yaw_mouse_sen)
#define PITCH_MOUSE_SEN (g_config.gimbal.pitch_mouse_sen)

#define YAW_ENCODE_SEN    (g_config.gimbal.yaw_encode_sen)
#define PITCH_ENCODE_SEN  (g_config.gimbal.pitch_encode_sen)

#define GIMBAL_CONTROL_TIME (g_config.gimbal.control_period_ms)

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  (g_config.gimbal.pitch_turn)
#define YAW_TURN    (g_config.gimbal.yaw_turn)

#define PITCH_KICK_UP_CURRENT   (g_config.gimbal.pitch_kick_up_current)
#define PITCH_KICK_DOWN_CURRENT (g_config.gimbal.pitch_kick_down_current)
#define PITCH_SOFT_LIMIT_UP     (g_config.gimbal.pitch_soft_limit_up)
#define PITCH_SOFT_LIMIT_DOWN   (g_config.gimbal.pitch_soft_limit_down)
#define PITCH_CURRENT_LIMIT     (g_config.gimbal.pitch_current_limit)

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     (g_config.gimbal.init_angle_error)
#define GIMBAL_INIT_STOP_TIME       (g_config.gimbal.init_stop_time_ms)
#define GIMBAL_INIT_TIME            (g_config.gimbal.init_time_ms)
#define GIMBAL_CALI_REDUNDANT_ANGLE (g_config.gimbal.cali_redundant_angle)
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     (g_config.gimbal.init_pitch_speed)
#define GIMBAL_INIT_YAW_SPEED       (g_config.gimbal.init_yaw_speed)

#define INIT_YAW_SET    (g_config.gimbal.init_yaw_set)
#define INIT_PITCH_SET  (g_config.gimbal.init_pitch_set)

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   (g_config.gimbal.cali_motor_set)
#define GIMBAL_CALI_STEP_TIME   (g_config.gimbal.cali_step_time_ms)
#define GIMBAL_CALI_GYRO_LIMIT  (g_config.gimbal.cali_gyro_limit)

#define GIMBAL_CALI_PITCH_MAX_STEP  (g_config.gimbal.cali_pitch_max_step)
#define GIMBAL_CALI_PITCH_MIN_STEP  (g_config.gimbal.cali_pitch_min_step)
#define GIMBAL_CALI_YAW_MAX_STEP    (g_config.gimbal.cali_yaw_max_step)
#define GIMBAL_CALI_YAW_MIN_STEP    (g_config.gimbal.cali_yaw_min_step)

#define GIMBAL_CALI_START_STEP  (g_config.gimbal.cali_start_step)
#define GIMBAL_CALI_END_STEP    (g_config.gimbal.cali_end_step)

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE (g_config.gimbal.motionless_rc_deadline)
#define GIMBAL_MOTIONLESS_TIME_MAX    (g_config.gimbal.motionless_time_max_ms)

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD (g_config.gimbal.motor_ecd_to_rad) //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_angle; //rad
    fp32 min_angle; //rad

    fp32 angle;     //rad
    fp32 angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_gyro_point;
    const fp32 *gimbal_INS_angle_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;

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
extern const gimbal_motor_t *get_yaw_motor_point(void);

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
extern const gimbal_motor_t *get_pitch_motor_point(void);

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

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          Yaw PID tuning API (runtime)
  * @note           These APIs update the live PID structs used by gimbal_task,
  *                 independent from compile-time defaults in g_config.
  */
extern void gimbal_tune_get_yaw_speed_pid(pid_param_t *out);
extern void gimbal_tune_get_yaw_angle_pid(pid_param_t *out);
extern void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_clear_yaw_pid(void);

/**
  * @brief          Pitch PID tuning API (runtime)
  */
extern void gimbal_tune_get_pitch_speed_pid(pid_param_t *out);
extern void gimbal_tune_get_pitch_angle_pid(pid_param_t *out);
extern void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_clear_pitch_pid(void);

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
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

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
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern volatile uint32_t gimbal_loop_counter;
extern volatile int16_t gimbal_watch_yaw_current;
extern volatile int16_t gimbal_watch_pitch_current;
extern volatile int16_t gimbal_yaw_easytest_current;
#endif
