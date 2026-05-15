/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_INTERFACE_H
#define CHASSIS_INTERFACE_H

#include "app_interface.h"
#include "app_pubsub.h"
#include "pid.h"
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CHASSIS_MOTOR_COUNT 4u

typedef enum
{
    APP_CHASSIS_MODE_FOLLOW_GIMBAL_YAW = 0,
    APP_CHASSIS_MODE_FOLLOW_CHASSIS_YAW,
    APP_CHASSIS_MODE_NO_FOLLOW_YAW,
    APP_CHASSIS_MODE_RAW,
} app_chassis_mode_e;

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

typedef char app_chassis_state_fits_pubsub[(sizeof(app_chassis_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];

static inline uint8_t app_publish_chassis_state(const app_chassis_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_CHASSIS_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_chassis_state(app_chassis_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_CHASSIS_STATE, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
