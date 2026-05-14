/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈舜 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef APP_TOPICS_H
#define APP_TOPICS_H

#include "app_pubsub.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "struct_typedef.h"

#define APP_CHASSIS_MOTOR_COUNT 4u
#define APP_SHOOT_FRIC_MOTOR_COUNT 4u

typedef enum
{
    APP_CHASSIS_MODE_FOLLOW_GIMBAL_YAW = 0,
    APP_CHASSIS_MODE_FOLLOW_CHASSIS_YAW,
    APP_CHASSIS_MODE_NO_FOLLOW_YAW,
    APP_CHASSIS_MODE_RAW,
} app_chassis_mode_e;

typedef struct
{
    uint8_t valid;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    int16_t last_ecd;
} app_motor_measure_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t motor_mode;
    uint8_t last_motor_mode;
    uint16_t offset_ecd;
    fp32 max_angle;
    fp32 min_angle;
    app_motor_measure_state_t measure;
    gimbal_PID_t angle_pid;
    pid_type_def gyro_pid;
    fp32 angle;
    fp32 angle_set;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
} app_gimbal_motor_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t behaviour;
    uint8_t chassis_stop;
    uint8_t shoot_stop;
    uint8_t turnaround_active;
    uint8_t turnaround_frame_valid;
    fp32 turnaround_frame_yaw_relative;
    fp32 turnaround_follow_offset_rad;
    app_gimbal_motor_state_t yaw;
    app_gimbal_motor_state_t pitch;
} app_gimbal_state_t;

typedef struct
{
    app_motor_measure_state_t measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} app_chassis_motor_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t mode;
    uint8_t last_mode;
    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 chassis_yaw_offset;
    fp32 chassis_yaw_offset_set;
    fp32 chassis_yaw_set;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
    pid_type_def angle_pid;
    app_chassis_motor_state_t motor[APP_CHASSIS_MOTOR_COUNT];
} app_chassis_state_t;

typedef struct
{
    uint8_t valid;
    uint8_t mode;
    fp32 fric_speed_set;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
    uint8_t trigger_measure_ready;
    uint8_t press_l;
    uint8_t press_r;
    uint8_t last_press_l;
    uint8_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;
    uint16_t block_time;
    uint16_t reverse_time;
    uint8_t move_flag;
    uint8_t key;
    uint16_t key_time;
    uint16_t heat_limit;
    uint16_t heat;
    pid_type_def trigger_motor_pid;
    pid_type_def fric_speed_pid[APP_SHOOT_FRIC_MOTOR_COUNT];
    int16_t fric_current_set[APP_SHOOT_FRIC_MOTOR_COUNT];
} app_shoot_state_t;

typedef char app_gimbal_state_fits_pubsub[(sizeof(app_gimbal_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];
typedef char app_chassis_state_fits_pubsub[(sizeof(app_chassis_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];
typedef char app_shoot_state_fits_pubsub[(sizeof(app_shoot_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];

static inline uint8_t app_publish_gimbal_state(const app_gimbal_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_GIMBAL_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_gimbal_state(app_gimbal_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_GIMBAL_STATE, out, (uint16_t)sizeof(*out));
}

static inline uint8_t app_publish_chassis_state(const app_chassis_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_CHASSIS_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_chassis_state(app_chassis_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_CHASSIS_STATE, out, (uint16_t)sizeof(*out));
}

static inline uint8_t app_publish_shoot_state(const app_shoot_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_SHOOT_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_shoot_state(app_shoot_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_SHOOT_STATE, out, (uint16_t)sizeof(*out));
}

#endif
