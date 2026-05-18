/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_CONTROL_TASK_H
#define CHASSIS_CONTROL_TASK_H

#include "types.h"
#include "CAN_receive.h"
#include "gimbal_state.h"
#include "pid.h"
#include "manual_input.h"
#include "user_lib.h"
#include "config.h"

#define CHASSIS_TASK_INIT_TIME                 (g_config.chassis.task_init_time_ms)

#define CHASSIS_X_CHANNEL                      (g_config.chassis.channel_vx)
#define CHASSIS_Y_CHANNEL                      (g_config.chassis.channel_vy)
#define CHASSIS_WZ_CHANNEL                     (g_config.chassis.channel_wz)
#define CHASSIS_MODE_CHANNEL                   (g_config.chassis.channel_mode)

#define CHASSIS_VX_RC_SEN                      (g_config.chassis.vx_rc_sen)
#define CHASSIS_VY_RC_SEN                      (g_config.chassis.vy_rc_sen)
#define CHASSIS_ANGLE_Z_RC_SEN                 (g_config.chassis.angle_z_rc_sen)
#define CHASSIS_WZ_RC_SEN                      (g_config.chassis.wz_rc_sen)

#define CHASSIS_ACCEL_X_NUM                    (g_config.chassis.accel_x_first_order)
#define CHASSIS_ACCEL_Y_NUM                    (g_config.chassis.accel_y_first_order)
#define CHASSIS_RC_DEADLINE                    (g_config.chassis.rc_deadband)

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX        (g_config.chassis.motor_speed_to_chassis_vx)
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY        (g_config.chassis.motor_speed_to_chassis_vy)
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ        (g_config.chassis.motor_speed_to_chassis_wz)
#define MOTOR_DISTANCE_TO_CENTER               (g_config.chassis.motor_distance_to_center)

#define CHASSIS_CONTROL_TIME_MS                (g_config.chassis.control_period_ms)
#define CHASSIS_CONTROL_TIME                   (g_config.chassis.control_period_ms * 0.001f)
#define CHASSIS_CONTROL_FREQUENCE              500.0f

#define MAX_MOTOR_CAN_CURRENT                  (g_config.chassis.max_motor_can_current)

#define SWING_KEY                              (g_config.chassis.swing_key_mask)
#define CHASSIS_GYRO_SPIN_VAR_KEY              (g_config.chassis.gyro_spin_var_key_mask)
#define CHASSIS_SWING_KEY                      (g_config.chassis.swing_mode_key_mask)
#define CHASSIS_FRONT_KEY                      (g_config.chassis.key_front_mask)
#define CHASSIS_BACK_KEY                       (g_config.chassis.key_back_mask)
#define CHASSIS_LEFT_KEY                       (g_config.chassis.key_left_mask)
#define CHASSIS_RIGHT_KEY                      (g_config.chassis.key_right_mask)

#define M3508_MOTOR_RPM_TO_VECTOR              (g_config.chassis.rpm_to_vector)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN        M3508_MOTOR_RPM_TO_VECTOR

#define MAX_WHEEL_SPEED                        (g_config.chassis.max_wheel_speed)
#define NORMAL_MAX_CHASSIS_SPEED_X             (g_config.chassis.max_vx_forward)
#define NORMAL_MAX_CHASSIS_SPEED_Y             (g_config.chassis.max_vy_left)

#define CHASSIS_WZ_SET_SCALE                   (g_config.chassis.wz_set_scale)

#define SWING_NO_MOVE_ANGLE                    (g_config.chassis.swing_no_move_angle)
#define SWING_MOVE_ANGLE                       (g_config.chassis.swing_move_angle)
#define CHASSIS_SWING_AMP_RAD                  (g_config.chassis.swing_amp_rad)
#define CHASSIS_SWING_HALF_PERIOD_MS           (g_config.chassis.swing_half_period_ms)
#define CHASSIS_SWING_CENTER_HOLD_MIN_MS       (g_config.chassis.swing_center_hold_min_ms)
#define CHASSIS_SWING_CENTER_HOLD_MAX_MS       (g_config.chassis.swing_center_hold_max_ms)

#define M3505_MOTOR_SPEED_PID_KP               (g_config.chassis.motor_speed_pid.kp)
#define M3505_MOTOR_SPEED_PID_KI               (g_config.chassis.motor_speed_pid.ki)
#define M3505_MOTOR_SPEED_PID_KD               (g_config.chassis.motor_speed_pid.kd)
#define M3505_MOTOR_SPEED_PID_MAX_OUT          (g_config.chassis.motor_speed_pid.max_out)
#define M3505_MOTOR_SPEED_PID_MAX_IOUT         (g_config.chassis.motor_speed_pid.max_iout)

#define CHASSIS_FOLLOW_GIMBAL_PID_KP           (g_config.chassis.follow_gimbal_pid.kp)
#define CHASSIS_FOLLOW_GIMBAL_PID_KI           (g_config.chassis.follow_gimbal_pid.ki)
#define CHASSIS_FOLLOW_GIMBAL_PID_KD           (g_config.chassis.follow_gimbal_pid.kd)
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT      (g_config.chassis.follow_gimbal_pid.max_out)
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT     (g_config.chassis.follow_gimbal_pid.max_iout)

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    CHASSIS_VECTOR_NO_FOLLOW_YAW,
    CHASSIS_VECTOR_RAW,
} chassis_mode_e;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

typedef struct
{
    bool_t manual_online;
    test_mode_e test_mode;
    uint16_t mode_sw;
    uint8_t safe_pos;
    uint8_t follow_pos;
    uint8_t spin_pos;
    int16_t axis_x;
    int16_t axis_y;
    int16_t axis_wz;
    int16_t rc_deadband;
    uint16_t key_mask;
    uint16_t front_key_mask;
    uint16_t back_key_mask;
    uint16_t left_key_mask;
    uint16_t right_key_mask;
    uint16_t gyro_spin_key_mask;
    uint16_t gyro_spin_var_key_mask;
    uint16_t swing_key_mask;
    fp32 vx_rc_sen;
    fp32 vy_rc_sen;
    fp32 angle_z_rc_sen;
    fp32 wz_rc_sen;
    fp32 open_rc_scale;
    fp32 swing_no_move_angle;
    fp32 swing_move_angle;
    fp32 swing_amp_rad;
    uint32_t swing_half_period_ms;
    uint32_t swing_center_hold_min_ms;
    uint32_t swing_center_hold_max_ms;
} chassis_control_snapshot_t;

typedef struct
{
    const manual_input_state_t *chassis_RC;
    uint8_t gimbal_state_valid;
    gimbal_motor_state_t chassis_yaw_motor;
    gimbal_motor_state_t chassis_pitch_motor;
    const fp32 *chassis_INS_angle;
    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;
    chassis_motor_t motor_chassis[4];
    pid_type_def motor_speed_pid[4];
    pid_type_def chassis_angle_pid;

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;
    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 chassis_yaw_offset;
    fp32 chassis_yaw_offset_set;
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
    chassis_control_snapshot_t fast;
} chassis_move_t;

extern void chassis_control_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern const chassis_move_t *get_chassis_move_point(void);

extern void chassis_tune_get_follow_pid(pid_param_t *out);
extern void chassis_tune_set_follow_pid(const pid_param_t *pid, bool_t clear_state);
extern void chassis_tune_clear_follow_pid(void);
extern void chassis_tune_get_motor_speed_pid(pid_param_t *out);
extern void chassis_tune_set_motor_speed_pid(const pid_param_t *pid, bool_t clear_state);

#endif
