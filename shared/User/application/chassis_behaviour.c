/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "app_input.h"
#include "detect_task.h"

#include <math.h>

// "Small gyro" (小陀螺) is implemented as a constant chassis yaw rate (wz) while:
// - NOT following gimbal yaw (open-loop rotation speed set-point)
// - Keeping translation (vx/vy) in the gimbal frame (rotate by -yaw_relative), so
//   the driver can still move normally while the chassis is spinning.

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

// Small gyro: keep vx/vy in gimbal frame, while commanding a constant chassis wz (no yaw follow).
static fp32 chassis_get_gimbal_yaw_relative_angle(const gimbal_motor_t *yaw_motor);
static void chassis_gyro_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_gyro_spin_var_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_swing_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);






//highlight, the variable chassis behaviour mode
//留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

static uint16_t chassis_get_effective_switch(uint16_t raw_sw, uint8_t manual_online)
{
    static uint8_t gate_inited = 0u;
    static uint8_t down_engaged = 0u;
    static uint16_t last_sw_raw = RC_SW_UP;
    uint16_t effective_sw = raw_sw;

    if (manual_online == 0u)
    {
        gate_inited = 0u;
        down_engaged = 0u;
        last_sw_raw = RC_SW_UP;
        return raw_sw;
    }

    if (gate_inited == 0u)
    {
        gate_inited = 1u;
        down_engaged = 0u;
        last_sw_raw = raw_sw;
    }

    if (!switch_is_down(raw_sw))
    {
        down_engaged = 0u;
    }
    else if (!switch_is_down(last_sw_raw))
    {
        down_engaged = 1u;
    }

    if (switch_is_down(raw_sw) && down_engaged == 0u)
    {
        effective_sw = (uint16_t)RC_SW_MID;
    }

    last_sw_raw = raw_sw;
    return effective_sw;
}


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    const uint8_t chassis_sw = app_input_switch(INPUT_SW_CHASSIS_MODE);
    const uint16_t chassis_sw_effective = chassis_get_effective_switch(chassis_sw, toe_is_error(DBUS_TOE) ? 0u : 1u);

    // 优先级最高的安全模式：拨杆上档立即停转所有底盘电机
    if (switch_is_up(chassis_sw))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
        return;
    }

    // chassis-only test mode: ignore gimbal follow and always use yaw stick to rotate chassis
    if (((test_mode_e)g_config.test.mode) == TEST_MODE_CHASSIS_ONLY)
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        return;
    }


    //remote control  set chassis behaviour mode
    //遥控器设置模式
    if (gimbal_turnaround_is_active())
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_mid(chassis_sw_effective))
    {
        // 普通模式：底盘跟随云台 yaw（让云台 yaw 回中，底盘转向对齐云台朝向）
        // CTRL 按下：进入小陀螺持续自转（不跟随云台角度）
        const uint16_t key_mask = chassis_move_mode->chassis_RC->key.v;
        const uint8_t want_swing = ((key_mask & CHASSIS_SWING_KEY) != 0u) ? 1u : 0u;
        const uint8_t want_gyro = ((key_mask & SWING_KEY) != 0u) ? 1u : 0u;
        const uint8_t want_gyro_var = ((key_mask & CHASSIS_GYRO_SPIN_VAR_KEY) != 0u) ? 1u : 0u;
        if (want_swing != 0u)
        {
            chassis_behaviour_mode = CHASSIS_SWING;
        }
        else if (want_gyro_var != 0u)
        {
            chassis_behaviour_mode = CHASSIS_GYRO_SPIN_VAR;
        }
        else
        {
            chassis_behaviour_mode = want_gyro ? CHASSIS_GYRO_SPIN : CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
    }
    else if (switch_is_down(chassis_sw_effective))
    {
        // 小陀螺：持续自转（类似 WH 的 SelfProtect），同时保留平移控制
        chassis_behaviour_mode = ((chassis_move_mode->chassis_RC->key.v & CHASSIS_GYRO_SPIN_VAR_KEY) != 0u) ?
                                     CHASSIS_GYRO_SPIN_VAR :
                                     CHASSIS_GYRO_SPIN;
    }

    //when gimbal in some mode, such as init mode, chassis must's move
    //当云台在某些模式下，像初始化， 底盘不动
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }


    //add your own logic to enter the new mode
    //添加自己的逻辑判断进入新模式


    //accord to beheviour mode, choose chassis control mode
    //根据行为模式选择一个底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_SWING)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_GYRO_SPIN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_GYRO_SPIN_VAR)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_SWING)
    {
        chassis_swing_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_GYRO_SPIN)
    {
        chassis_gyro_spin_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_GYRO_SPIN_VAR)
    {
        chassis_gyro_spin_var_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    if (gimbal_turnaround_is_active())
    {
        *angle_set = chassis_get_gimbal_yaw_relative_angle(chassis_move_rc_to_vector->chassis_yaw_motor);
    }
    else
    {
        *angle_set = gimbal_turnaround_chassis_follow_offset_rad();
    }
}

typedef struct
{
    uint8_t initialized;
    uint8_t center_idx;
    int8_t dir;
    uint8_t pending_center_advance;
    fp32 offset_rad;
    uint32_t last_ms;
    uint32_t next_center_jump_ms;
    uint32_t rng_state;
} chassis_swing_state_t;

static uint32_t chassis_swing_rng_next(chassis_swing_state_t *st)
{
    if (st == NULL)
    {
        return 0u;
    }

    uint32_t x = st->rng_state;
    if (x == 0u)
    {
        x = ((uint32_t)osKernelSysTick()) ^ 0x9E3779B9u;
        if (x == 0u)
        {
            x = 0x6C8E9CF5u;
        }
    }

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    st->rng_state = x;
    return x;
}

static uint32_t chassis_swing_rand_between_u32(chassis_swing_state_t *st, uint32_t min_v, uint32_t max_v)
{
    if (max_v < min_v)
    {
        const uint32_t t = min_v;
        min_v = max_v;
        max_v = t;
    }
    if (max_v == min_v)
    {
        return min_v;
    }

    const uint32_t span = max_v - min_v;
    const uint32_t r = chassis_swing_rng_next(st);
    return min_v + (r % (span + 1u));
}

static void chassis_swing_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    static chassis_swing_state_t st;
    static const fp32 centers_rad[4] = {
        0.7853981633974483f,
        2.3561944901923450f,
        -2.3561944901923450f,
        -0.7853981633974483f,
    };
    const fp32 center_step_rad = 1.5707963267948966f;
    const uint32_t now_ms = (uint32_t)osKernelSysTick();

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    // 摇摆时仍然尽量保持平移参考跟着云台，不然按键方向会发飘。
    const fp32 vx_raw = *vx_set;
    const fp32 vy_raw = *vy_set;
    const fp32 yaw_relative = chassis_get_gimbal_yaw_relative_angle(chassis_move_rc_to_vector->chassis_yaw_motor);
    const fp32 sin_yaw = arm_sin_f32(-yaw_relative);
    const fp32 cos_yaw = arm_cos_f32(-yaw_relative);
    *vx_set = cos_yaw * vx_raw + sin_yaw * vy_raw;
    *vy_set = -sin_yaw * vx_raw + cos_yaw * vy_raw;

    fp32 amp = CHASSIS_SWING_AMP_RAD;
    if (amp < 0.0f)
    {
        amp = -amp;
    }

    uint32_t half_period_ms = (uint32_t)CHASSIS_SWING_HALF_PERIOD_MS;
    if (half_period_ms < 10u)
    {
        half_period_ms = 10u;
    }

    uint32_t hold_min_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MIN_MS;
    uint32_t hold_max_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MAX_MS;
    if (hold_min_ms == 0u)
    {
        hold_min_ms = 5000u;
    }
    if (hold_max_ms == 0u)
    {
        hold_max_ms = 20000u;
    }

    if (st.initialized == 0u)
    {
        st.initialized = 1u;
        st.center_idx = (uint8_t)(chassis_swing_rng_next(&st) & 3u);
        st.dir = 1;
        st.pending_center_advance = 0u;
        st.offset_rad = 0.0f;
        st.last_ms = now_ms;
        st.next_center_jump_ms = now_ms + chassis_swing_rand_between_u32(&st, hold_min_ms, hold_max_ms);
    }

    if (amp <= 0.0f)
    {
        *angle_set = rad_format(centers_rad[st.center_idx]);
        return;
    }

    uint32_t dt_ms = now_ms - st.last_ms;
    st.last_ms = now_ms;
    if (dt_ms > 200u)
    {
        dt_ms = 0u;
    }

    if (st.pending_center_advance == 0u && now_ms >= st.next_center_jump_ms)
    {
        st.pending_center_advance = 1u;
    }

    const fp32 rate_rad_per_ms = (2.0f * amp) / (fp32)half_period_ms;
    fp32 offset = st.offset_rad + (fp32)st.dir * rate_rad_per_ms * (fp32)dt_ms;

    if (st.dir > 0)
    {
        fp32 max_offset = amp;
        if (st.pending_center_advance != 0u)
        {
            max_offset = amp + center_step_rad;
        }

        if (offset >= max_offset)
        {
            if (st.pending_center_advance != 0u && max_offset > amp)
            {
                st.center_idx = (uint8_t)((st.center_idx + 1u) & 3u);
                st.pending_center_advance = 0u;
                st.next_center_jump_ms = now_ms + chassis_swing_rand_between_u32(&st, hold_min_ms, hold_max_ms);
                offset = amp;
                st.dir = -1;
            }
            else
            {
                offset = amp;
                st.dir = -1;
            }
        }
    }
    else if (offset <= -amp)
    {
        offset = -amp;
        st.dir = 1;
    }

    st.offset_rad = offset;
    *angle_set = rad_format(centers_rad[st.center_idx] + offset);
}

static fp32 chassis_get_gimbal_yaw_relative_angle(const gimbal_motor_t *yaw_motor)
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

    fp32 angle = (fp32)relative_ecd * MOTOR_ECD_TO_RAD;
    return YAW_TURN ? -angle : angle;
}

static void chassis_gyro_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 1) Read translation command as usual (RC/keyboard), then rotate it into gimbal frame.
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    const fp32 vx_raw = *vx_set;
    const fp32 vy_raw = *vy_set;

    const fp32 yaw_relative_meas = chassis_get_gimbal_yaw_relative_angle(chassis_move_rc_to_vector->chassis_yaw_motor);
    fp32 yaw_frame = yaw_relative_meas;
    (void)gimbal_turnaround_get_frame_yaw_relative(&yaw_frame);
    const fp32 sin_yaw = arm_sin_f32(-yaw_frame);
    const fp32 cos_yaw = arm_cos_f32(-yaw_frame);
    *vx_set = cos_yaw * vx_raw + sin_yaw * vy_raw;
    *vy_set = -sin_yaw * vx_raw + cos_yaw * vy_raw;

    // 2) Constant spin rate (wz) like WH's "小陀螺" mode.
    // Use two presets so tuning can keep stability while moving.
    fp32 spin_wz = SWING_NO_MOVE_ANGLE;
    if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY) ||
        (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY) ||
        (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY) ||
        (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) ||
        (fabsf(vx_raw) > 0.05f) ||
        (fabsf(vy_raw) > 0.05f))
    {
        spin_wz = SWING_MOVE_ANGLE;
    }
    *wz_set = spin_wz;
}

static void chassis_gyro_spin_var_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_gyro_spin_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);

    const fp32 spin_wz_base = *wz_set;
    const fp32 sign = (spin_wz_base >= 0.0f) ? 1.0f : -1.0f;
    const fp32 base_abs = fabsf(spin_wz_base);
    const uint32_t now_ms = (uint32_t)osKernelSysTick();
    const fp32 t = (fp32)now_ms * 0.001f;

    static uint32_t s_rng = 0u;
    static uint32_t s_jump_next_ms = 0u;
    static fp32 s_jump = 0.0f;
    static fp32 s_walk = 0.0f;

    if (s_rng == 0u)
    {
        s_rng = now_ms ^ 0x9E3779B9u;
        if (s_rng == 0u)
        {
            s_rng = 0x6C8E9CF5u;
        }
    }

    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;

    if (now_ms >= s_jump_next_ms)
    {
        const fp32 r01 = (fp32)(s_rng & 0xFFFFu) * (1.0f / 65535.0f);
        s_jump = (r01 - 0.5f) * 0.36f;

        s_rng ^= s_rng << 13;
        s_rng ^= s_rng >> 17;
        s_rng ^= s_rng << 5;
        s_jump_next_ms = now_ms + 150u + (s_rng % 451u);
    }

    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    s_walk += (((fp32)(s_rng & 0xFFFFu) * (1.0f / 65535.0f)) - 0.5f) * 0.01f;
    s_walk = fp32_constrain(s_walk, -0.10f, 0.10f);

    fp32 factor = 0.75f + 0.25f * arm_sin_f32((2.0f * PI * 1.2f) * t) + s_jump + s_walk;
    factor = fp32_constrain(factor, 0.5f, 1.0f);
    *wz_set = sign * base_abs * factor;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * app_input_axis(INPUT_AXIS_CHASSIS_WZ));
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * app_input_axis(INPUT_AXIS_CHASSIS_WZ);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = app_input_axis(INPUT_AXIS_CHASSIS_X) * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -app_input_axis(INPUT_AXIS_CHASSIS_Y) * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -app_input_axis(INPUT_AXIS_CHASSIS_WZ) * CHASSIS_OPEN_RC_SCALE;
    return;
}
