/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SHOOT_INTERFACE_H
#define SHOOT_INTERFACE_H

#include "app_pubsub.h"
#include "pid.h"
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SHOOT_FRIC_MOTOR_COUNT 4u

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

typedef char app_shoot_state_fits_pubsub[(sizeof(app_shoot_state_t) <= APP_PUBSUB_MAX_PAYLOAD_BYTES) ? 1 : -1];

static inline uint8_t app_publish_shoot_state(const app_shoot_state_t *state)
{
    return app_pubsub_publish(APP_TOPIC_SHOOT_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t app_copy_shoot_state(app_shoot_state_t *out)
{
    return app_pubsub_copy(APP_TOPIC_SHOOT_STATE, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
