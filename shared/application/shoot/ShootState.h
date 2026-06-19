/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SHOOT_STATE_H
#define SHOOT_STATE_H

#include "StateStore.h"
#include "Pid.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHOOT_STATE_FRIC_MOTOR_COUNT 4u

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
    PidTypeDef trigger_motor_pid;
    PidTypeDef fric_speed_pid[SHOOT_STATE_FRIC_MOTOR_COUNT];
    int16_t fric_current_set[SHOOT_STATE_FRIC_MOTOR_COUNT];
} ShootState;

typedef char ShootStateFitsStore[(sizeof(ShootState) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t ShootStateWrite(const ShootState *state)
{
    return StateStoreWrite(STATE_SHOOT, state, (uint16_t)sizeof(*state));
}

static inline uint8_t ShootStateRead(ShootState *out)
{
    return StateStoreRead(STATE_SHOOT, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
