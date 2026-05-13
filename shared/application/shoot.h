/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SHOOT_H
#define SHOOT_H

#include "struct_typedef.h"

#include "CAN_receive.h"
#include "pid.h"
#include "manual_input.h"
#include "user_lib.h"
#include "config.h"

// shoot mode channel
#define SHOOT_RC_MODE_CHANNEL        (g_config.shoot.rc_mode_channel)

#define SHOOT_CONTROL_TIME           (g_config.shoot.control_period_ms)

// friction wheel settings
#define FRIC_MOTOR_NUM               (4u)
#define SHOOT_FRIC_SPEED_RPM         (g_config.shoot.fric_speed_rpm)
#define SHOOT_FRIC_SPEED_OFF_RPM     (g_config.shoot.fric_speed_off_rpm)
#define SHOOT_FRIC_SPEED_STEP_RPM_S  (g_config.shoot.fric_speed_step_rpm_s)
#define SHOOT_FRIC_READY_RATIO       (g_config.shoot.fric_ready_ratio)
#define SHOOT_FRIC_DIR(i)            (g_config.shoot.fric_motor_dir[(i) & 0x03])

#define SHOOT_ON_KEYBOARD            (g_config.shoot.key_on_mask)
#define SHOOT_OFF_KEYBOARD           (g_config.shoot.key_off_mask)

// delay before considering the shot fully released
#define SHOOT_DONE_KEY_OFF_TIME      (g_config.shoot.shoot_done_key_off_time_ms)
#define PRESS_LONG_TIME              (g_config.shoot.press_long_time_ms)
#define RC_S_LONG_TIME               (g_config.shoot.rc_s_long_time_ms)
#define UP_ADD_TIME                  (g_config.shoot.up_add_time_ms)

// motor encoder helpers
#define HALF_ECD_RANGE               4096
#define ECD_RANGE                    8191
#define MOTOR_RPM_TO_SPEED           (g_config.shoot.motor_rpm_to_speed)
#define MOTOR_ECD_TO_ANGLE           (g_config.shoot.motor_ecd_to_angle)
#define FULL_COUNT                   (g_config.shoot.full_count)

#define TRIGGER_SPEED                (g_config.shoot.trigger_speed_single)
#define CONTINUE_TRIGGER_SPEED       (g_config.shoot.trigger_speed_continuous)
#define READY_TRIGGER_SPEED          (g_config.shoot.trigger_speed_ready)

#define KEY_OFF_JUGUE_TIME           (g_config.shoot.key_off_judge_time_ms)
#define SWITCH_TRIGGER_ON            (g_config.shoot.switch_trigger_on)
#define SWITCH_TRIGGER_OFF           (g_config.shoot.switch_trigger_off)

#define BLOCK_TRIGGER_SPEED          (g_config.shoot.block_trigger_speed)
#define BLOCK_TIME                   (g_config.shoot.block_time_ms)
#define REVERSE_TIME                 (g_config.shoot.reverse_time_ms)
#define REVERSE_SPEED_LIMIT          (g_config.shoot.reverse_speed_limit)

#define PI_FOUR                      (g_config.shoot.pi_over_four)
#define PI_TEN                       (g_config.shoot.pi_over_ten)

#define TRIGGER_ANGLE_PID_KP         (g_config.shoot.trigger_angle_pid.kp)
#define TRIGGER_ANGLE_PID_KI         (g_config.shoot.trigger_angle_pid.ki)
#define TRIGGER_ANGLE_PID_KD         (g_config.shoot.trigger_angle_pid.kd)

#define TRIGGER_BULLET_PID_MAX_OUT   (g_config.shoot.trigger_bullet_pid_max_out)
#define TRIGGER_BULLET_PID_MAX_IOUT  (g_config.shoot.trigger_bullet_pid_max_iout)

#define TRIGGER_READY_PID_MAX_OUT    (g_config.shoot.trigger_ready_pid_max_out)
#define TRIGGER_READY_PID_MAX_IOUT   (g_config.shoot.trigger_ready_pid_max_iout)

#define SHOOT_HEAT_REMAIN_VALUE      (g_config.shoot.heat_remain_value)

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    shoot_mode_e shoot_mode;
    const manual_input_state_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric_speed_ramp;
    fp32 fric_speed_set;
    pid_type_def fric_speed_pid[FRIC_MOTOR_NUM];
    int16_t fric_current_set[FRIC_MOTOR_NUM];
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
    uint8_t trigger_measure_ready;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint16_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

// shoot runs together with gimbal because they share CAN IDs
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern const shoot_control_t *get_shoot_control_point(void);

#endif
