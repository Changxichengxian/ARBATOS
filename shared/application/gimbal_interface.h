/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_INTERFACE_H
#define GIMBAL_INTERFACE_H

#include "app_interface.h"
#include "app_pubsub.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef char app_gimbal_state_fits_pubsub[(sizeof(app_gimbal_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];

static inline uint8_t app_publish_gimbal_state(const app_gimbal_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_GIMBAL_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_gimbal_state(app_gimbal_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_GIMBAL_STATE, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
