/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_CONTROL_TASK_H
#define GIMBAL_CONTROL_TASK_H

#include "types.h"
#include "CAN_receive.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "manual_input.h"
#include "config.h"

// pitch speed close-loop PID params, max out and max iout
#define PITCH_SPEED_PID_KP            (g_config.gimbal.pitch_speed_pid.kp)
#define PITCH_SPEED_PID_KI            (g_config.gimbal.pitch_speed_pid.ki)
#define PITCH_SPEED_PID_KD            (g_config.gimbal.pitch_speed_pid.kd)
#define PITCH_SPEED_PID_MAX_OUT       (g_config.gimbal.pitch_speed_pid.max_out)
#define PITCH_SPEED_PID_MAX_IOUT      (g_config.gimbal.pitch_speed_pid.max_iout)

// yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP              (g_config.gimbal.yaw_speed_pid.kp)
#define YAW_SPEED_PID_KI              (g_config.gimbal.yaw_speed_pid.ki)
#define YAW_SPEED_PID_KD              (g_config.gimbal.yaw_speed_pid.kd)
#define YAW_SPEED_PID_MAX_OUT         (g_config.gimbal.yaw_speed_pid.max_out)
#define YAW_SPEED_PID_MAX_IOUT        (g_config.gimbal.yaw_speed_pid.max_iout)

// pitch encode angle close-loop PID params, max out and max iout
#define PITCH_ENCODE_ANGLE_PID_KP     (g_config.gimbal.pitch_encode_angle_pid.kp)
#define PITCH_ENCODE_ANGLE_PID_KI     (g_config.gimbal.pitch_encode_angle_pid.ki)
#define PITCH_ENCODE_ANGLE_PID_KD     (g_config.gimbal.pitch_encode_angle_pid.kd)
#define PITCH_ENCODE_ANGLE_PID_MAX_OUT (g_config.gimbal.pitch_encode_angle_pid.max_out)
#define PITCH_ENCODE_ANGLE_PID_MAX_IOUT (g_config.gimbal.pitch_encode_angle_pid.max_iout)

// yaw encode angle close-loop PID params, max out and max iout
#define YAW_ENCODE_ANGLE_PID_KP       (g_config.gimbal.yaw_encode_angle_pid.kp)
#define YAW_ENCODE_ANGLE_PID_KI       (g_config.gimbal.yaw_encode_angle_pid.ki)
#define YAW_ENCODE_ANGLE_PID_KD       (g_config.gimbal.yaw_encode_angle_pid.kd)
#define YAW_ENCODE_ANGLE_PID_MAX_OUT  (g_config.gimbal.yaw_encode_angle_pid.max_out)
#define YAW_ENCODE_ANGLE_PID_MAX_IOUT (g_config.gimbal.yaw_encode_angle_pid.max_iout)

// task init delay
#define GIMBAL_TASK_INIT_TIME         (g_config.gimbal.task_init_time_ms)

// yaw, pitch control channels and mode channel
#define YAW_CHANNEL                   (g_config.gimbal.channel_yaw)
#define PITCH_CHANNEL                 (g_config.gimbal.channel_pitch)
#define GIMBAL_MODE_CHANNEL           (g_config.gimbal.channel_mode)

// turn 180 key and turn speed
#define TURN_KEYBOARD                 (g_config.gimbal.turn_key_mask)
#define TURN_SPEED                    (g_config.gimbal.turn_speed)
#define TEST_KEYBOARD                 (g_config.gimbal.test_key_mask)

// remote input deadband
#define RC_DEADBAND                   (g_config.gimbal.rc_deadband)

#define YAW_RC_SEN                    (g_config.gimbal.yaw_rc_sen)
#define PITCH_RC_SEN                  (g_config.gimbal.pitch_rc_sen)

#define YAW_MOUSE_SEN                 (g_config.gimbal.yaw_mouse_sen)
#define PITCH_MOUSE_SEN               (g_config.gimbal.pitch_mouse_sen)

#define YAW_ENCODE_SEN                (g_config.gimbal.yaw_encode_sen)
#define PITCH_ENCODE_SEN              (g_config.gimbal.pitch_encode_sen)

#define GIMBAL_CONTROL_TIME           (g_config.gimbal.control_period_ms)

// test mode, 0 close, 1 open
#define GIMBAL_TEST_MODE              0

#define PITCH_TURN                    (g_config.gimbal.pitch_turn)
#define YAW_TURN                      (g_config.gimbal.yaw_turn)

#define PITCH_KICK_UP_CURRENT         (g_config.gimbal.pitch_kick_up_current)
#define PITCH_KICK_DOWN_CURRENT       (g_config.gimbal.pitch_kick_down_current)
#define PITCH_SOFT_LIMIT_UP           (g_config.gimbal.pitch_soft_limit_up)
#define PITCH_SOFT_LIMIT_DOWN         (g_config.gimbal.pitch_soft_limit_down)
#define PITCH_CURRENT_LIMIT           (g_config.gimbal.pitch_current_limit)

// motor encoder range helpers
#define HALF_ECD_RANGE                4096
#define ECD_RANGE                     8191

#define GIMBAL_INIT_ANGLE_ERROR       (g_config.gimbal.init_angle_error)
#define GIMBAL_INIT_STOP_TIME         (g_config.gimbal.init_stop_time_ms)
#define GIMBAL_INIT_TIME              (g_config.gimbal.init_time_ms)
#define GIMBAL_CALI_REDUNDANT_ANGLE   (g_config.gimbal.cali_redundant_angle)
#define GIMBAL_INIT_PITCH_SPEED       (g_config.gimbal.init_pitch_speed)
#define GIMBAL_INIT_YAW_SPEED         (g_config.gimbal.init_yaw_speed)

#define INIT_YAW_SET                  (g_config.gimbal.init_yaw_set)
#define INIT_PITCH_SET                (g_config.gimbal.init_pitch_set)

#define GIMBAL_CALI_MOTOR_SET         (g_config.gimbal.cali_motor_set)
#define GIMBAL_CALI_STEP_TIME         (g_config.gimbal.cali_step_time_ms)
#define GIMBAL_CALI_GYRO_LIMIT        (g_config.gimbal.cali_gyro_limit)

#define GIMBAL_CALI_PITCH_MAX_STEP    (g_config.gimbal.cali_pitch_max_step)
#define GIMBAL_CALI_PITCH_MIN_STEP    (g_config.gimbal.cali_pitch_min_step)
#define GIMBAL_CALI_YAW_MAX_STEP      (g_config.gimbal.cali_yaw_max_step)
#define GIMBAL_CALI_YAW_MIN_STEP      (g_config.gimbal.cali_yaw_min_step)

#define GIMBAL_CALI_START_STEP        (g_config.gimbal.cali_start_step)
#define GIMBAL_CALI_END_STEP          (g_config.gimbal.cali_end_step)

#define GIMBAL_MOTIONLESS_RC_DEADLINE (g_config.gimbal.motionless_rc_deadline)
#define GIMBAL_MOTIONLESS_TIME_MAX    (g_config.gimbal.motionless_time_max_ms)

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD              (g_config.gimbal.motor_ecd_to_rad)
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,
    GIMBAL_MOTOR_ENCONDE,
} gimbal_motor_mode_e;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_angle;
    fp32 min_angle;

    fp32 angle;
    fp32 angle_set;
    fp32 motor_gyro;
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
    bool_t dbus_offline;
    test_mode_e test_mode;
    uint8_t mode_sw;
    uint8_t safe_pos;
    uint8_t active_source;
    uint8_t image_auto_aim_requested;
    int16_t yaw_axis;
    int16_t pitch_axis;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t rc_deadband;
    uint16_t key_mask;
    uint16_t turn_key_mask;
    fp32 turn_speed;
    fp32 yaw_rc_sen;
    fp32 pitch_rc_sen;
    fp32 yaw_mouse_sen;
    fp32 pitch_mouse_sen;
} gimbal_control_snapshot_t;

typedef struct
{
    const manual_input_state_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_gyro_point;
    const fp32 *gimbal_INS_angle_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    gimbal_control_snapshot_t fast;
} gimbal_control_t;

extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);

extern void gimbal_control_task(void const *pvParameters);

// runtime PID tuning helpers
extern void gimbal_tune_get_yaw_speed_pid(pid_param_t *out);
extern void gimbal_tune_get_yaw_angle_pid(pid_param_t *out);
extern void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_clear_yaw_pid(void);

extern void gimbal_tune_get_pitch_speed_pid(pid_param_t *out);
extern void gimbal_tune_get_pitch_angle_pid(pid_param_t *out);
extern void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state);
extern void gimbal_tune_clear_pitch_pid(void);

extern bool_t cmd_cali_gimbal_hook(
    uint16_t *yaw_offset,
    uint16_t *pitch_offset,
    fp32 *max_yaw,
    fp32 *min_yaw,
    fp32 *max_pitch,
    fp32 *min_pitch
);

extern void set_cali_gimbal_hook(
    const uint16_t yaw_offset,
    const uint16_t pitch_offset,
    const fp32 max_yaw,
    const fp32 min_yaw,
    const fp32 max_pitch,
    const fp32 min_pitch
);

extern volatile uint32_t gimbal_loop_counter;
extern volatile int16_t gimbal_watch_yaw_current;
extern volatile int16_t gimbal_watch_pitch_current;
extern volatile int16_t gimbal_yaw_easytest_current;

#endif
