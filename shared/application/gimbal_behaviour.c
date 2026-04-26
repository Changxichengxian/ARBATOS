/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "config.h"
#include "control_input.h"
#include "detect_task.h"
#include "pitch_cali.h"
#include "FreeRTOS.h"
#include "task.h"

#include "user_lib.h"

//when gimbal is in calibrating, set buzzer frequency and strenght
//当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_tone_start_legacy(g_config.buzzer.gimbal_warn_psc, g_config.buzzer.gimbal_warn_pwm)
#define gimbal_warn_buzzer_off() buzzer_tone_stop()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 gimbal will lift the pitch axis and rotate yaw axis.
  * @param[out]     yaw: yaw motor angle increment, unit rad.
  * @param[out]     pitch: pitch motor angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台初始化控制，先抬起 pitch 轴，再旋转 yaw 轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis,
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ANGLE, the function is called
  *                 and gimbal control mode is encoder angle mode.
  * @param[out]     yaw: yaw axia angle increment, unit rad
  * @param[out]     pitch: pitch axia angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode.
  * @param[out]     yaw: yaw axia angle increment,  unit rad
  * @param[out]     pitch: pitch axia angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
// 调试用可见镜像，便于 watch 窗口查看当前行为
volatile gimbal_behaviour_e gimbal_behaviour_watch = GIMBAL_ZERO_FORCE;

typedef struct
{
    volatile uint8_t active;
    volatile uint8_t flipped;
    uint8_t pending_flipped;
    uint8_t setpoint_done;
    uint8_t key_prev;
    fp32 target_yaw_relative;
    fp32 remaining_setpoint_rad;
    uint32_t start_ms;
} gimbal_turnaround_state_t;

static gimbal_turnaround_state_t s_turn = {0};
static const uint32_t GIMBAL_TURNAROUND_TIMEOUT_MS = 3000u;

static fp32 gimbal_get_yaw_relative_angle(const gimbal_motor_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->gimbal_motor_measure == NULL)
    {
        return 0.0f;
    }

    int32_t relative_ecd = (int32_t)yaw_motor->gimbal_motor_measure->ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return (YAW_TURN ? -1.0f : 1.0f) * ((fp32)relative_ecd * MOTOR_ECD_TO_RAD);
}

bool_t gimbal_turnaround_is_active(void)
{
    return (s_turn.active != 0u) ? 1 : 0;
}

fp32 gimbal_turnaround_chassis_follow_offset_rad(void)
{
    return (s_turn.flipped != 0u) ? PI : 0.0f;
}

bool_t gimbal_turnaround_get_frame_yaw_relative(fp32 *out_yaw_relative)
{
    if (out_yaw_relative == NULL)
    {
        return 0;
    }

    if (s_turn.active == 0u)
    {
        return 0;
    }

    *out_yaw_relative = s_turn.target_yaw_relative;
    return 1;
}

/**
  * @brief          the function is called by gimbal_set_mode function in gimbal_task.c
  *                 the function set gimbal_behaviour variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_PITCH_CALI)
    {
        // NOTE: pitch motor may be temporarily switched to RAW by the calibration state machine.
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }

    // 更新调试镜像，便于在 watch 中查看行为状态
    gimbal_behaviour_watch = gimbal_behaviour;
}

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 accoring to the gimbal_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }


    if (gimbal_behaviour != GIMBAL_ANGLE)
    {
        s_turn.active = 0u;
        s_turn.setpoint_done = 0u;
        s_turn.remaining_setpoint_rad = 0.0f;
        s_turn.key_prev = 0u;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ANGLE)
    {
        gimbal_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_PITCH_CALI)
    {
        pitch_cali_control(add_yaw, add_pitch, gimbal_control_set);
    }

}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    // Only stop chassis in safety/initialization/calibration.
    // NOTE: Do NOT stop chassis in GIMBAL_MOTIONLESS; otherwise chassis follow can't converge after you release yaw input.
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_PITCH_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_PITCH_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    const gimbal_control_snapshot_t *fast = &gimbal_mode_set->fast;
    const bool_t dbus_offline = fast->dbus_offline;
    const uint8_t gimbal_sw = fast->mode_sw;
    const bool_t switch_safe = input_switch_is_pos(gimbal_sw, fast->safe_pos);
    const bool_t yaw_only_mode = fast->test_mode == TEST_MODE_YAW_ONLY;
    const bool_t yaw_easy_test_mode = fast->test_mode == TEST_MODE_YAW_EASY_TEST;
    const bool_t pitch_cali_mode = fast->test_mode == TEST_MODE_PITCH_CALI;

    // 安全模式最高优先级：遥控上档或离线直接降为零力矩
    if (dbus_offline || switch_safe)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        return;
    }

    // yaw-only / yaw-easy 测试模式：直接启用 yaw 角度控制，跳过 pitch 零点/校准检查
    if (yaw_only_mode || yaw_easy_test_mode)
    {
        gimbal_behaviour = GIMBAL_ANGLE;
        // 若未校准或数据异常，给 yaw 角度限幅一个安全默认值，避免 max/min 为脏数据导致 set 被卡死
        if (gimbal_mode_set->gimbal_yaw_motor.max_angle <= gimbal_mode_set->gimbal_yaw_motor.min_angle ||
            fabsf(gimbal_mode_set->gimbal_yaw_motor.max_angle) > 1000.0f ||
            fabsf(gimbal_mode_set->gimbal_yaw_motor.min_angle) > 1000.0f)
        {
            gimbal_mode_set->gimbal_yaw_motor.max_angle = PI;
            gimbal_mode_set->gimbal_yaw_motor.min_angle = -PI;
        }
        return;
    }
    //in cali mode, return
    //校准行为，return 不会设置其他的模式
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }

    //if other operate make step change to start, means enter cali mode
    //如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP && !dbus_offline)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //init mode, judge if gimbal is in middle place
    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {

            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {

            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态：开关打上档，或者掉线
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_safe && !dbus_offline)
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //开关控制 云台状态
    if (switch_safe)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else
    {
        gimbal_behaviour = GIMBAL_ANGLE;
    }

    //enter init mode
    //判断进入init状态机
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

    // pitch 补偿校准模式：在非安全档运行；仍保留 INIT 状态机（上电/退出安全档先回中位）
    if (pitch_cali_mode)
    {
        if (gimbal_behaviour == GIMBAL_ANGLE || gimbal_behaviour == GIMBAL_MOTIONLESS)
        {
            gimbal_behaviour = GIMBAL_PITCH_CALI;
        }
    }



}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called.
  *                 gimbal will lift the pitch axis and rotate yaw axis.
  * @param[out]     yaw: yaw motor angle increment, unit rad.
  * @param[out]     pitch: pitch motor angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis,
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_ANGLE, the function is called
  *                 and gimbal control mode is encoder angle mode.
  * @param[out]     yaw: yaw axia angle increment, unit rad
  * @param[out]     pitch: pitch axia angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    const gimbal_control_snapshot_t *fast = &gimbal_control_set->fast;
    const test_mode_e test_mode = fast->test_mode;
    const bool_t yaw_test_mode = (test_mode == TEST_MODE_YAW_ONLY) || (test_mode == TEST_MODE_YAW_EASY_TEST);

    rc_deadband_limit(fast->yaw_axis, yaw_channel, fast->rc_deadband);
    rc_deadband_limit(fast->pitch_axis, pitch_channel, fast->rc_deadband);

    const uint16_t key_mask = fast->key_mask;
    const uint8_t turn_key_down = ((key_mask & fast->turn_key_mask) != 0u) ? 1u : 0u;
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (turn_key_down != 0u && s_turn.key_prev == 0u && s_turn.active == 0u)
    {
        s_turn.active = 1u;
        s_turn.setpoint_done = 0u;
        s_turn.pending_flipped = (s_turn.flipped != 0u) ? 0u : 1u;
        s_turn.remaining_setpoint_rad = PI;
        s_turn.target_yaw_relative = rad_format(gimbal_get_yaw_relative_angle(&gimbal_control_set->gimbal_yaw_motor) + PI);
        s_turn.start_ms = now_ms;
    }
    s_turn.key_prev = turn_key_down;

    // yaw 测试模式：放宽限幅防止 ±pi 卡死
    if (yaw_test_mode)
    {
        gimbal_control_set->gimbal_yaw_motor.max_angle = 100.0f;
        gimbal_control_set->gimbal_yaw_motor.min_angle = -100.0f;
    }

    fp32 yaw_add = -yaw_channel * fast->yaw_rc_sen - (fp32)fast->mouse_x * fast->yaw_mouse_sen;
    *pitch = pitch_channel * fast->pitch_rc_sen + (fp32)fast->mouse_y * fast->pitch_mouse_sen;

    if (s_turn.active != 0u)
    {
        yaw_add = 0.0f;

        if (s_turn.setpoint_done == 0u)
        {
            fp32 step = fabsf(fast->turn_speed);
            if (step < 0.000001f)
            {
                step = 0.04f;
            }
            if (step > s_turn.remaining_setpoint_rad)
            {
                step = s_turn.remaining_setpoint_rad;
            }

            yaw_add = (s_turn.pending_flipped != 0u) ? step : -step;
            s_turn.remaining_setpoint_rad -= step;
            if (s_turn.remaining_setpoint_rad <= 0.000001f)
            {
                s_turn.remaining_setpoint_rad = 0.0f;
                s_turn.setpoint_done = 1u;
            }
        }
        else
        {
            const fp32 yaw_rel = gimbal_get_yaw_relative_angle(&gimbal_control_set->gimbal_yaw_motor);
            const fp32 abs_err = fabsf(rad_format(yaw_rel - s_turn.target_yaw_relative));
            const fp32 abs_gyro = fabsf(gimbal_control_set->gimbal_yaw_motor.motor_gyro);
            const uint32_t elapsed_ms = now_ms - s_turn.start_ms;
            if (abs_err < 0.15f && abs_gyro < 0.3f)
            {
                s_turn.flipped = s_turn.pending_flipped;
                s_turn.active = 0u;
            }
            else if (elapsed_ms >= GIMBAL_TURNAROUND_TIMEOUT_MS)
            {
                s_turn.active = 0u;
            }
        }
    }

    *yaw = yaw_add;

}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode.
  * @param[out]     yaw: yaw axia angle increment,  unit rad
  * @param[out]     pitch: pitch axia angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
