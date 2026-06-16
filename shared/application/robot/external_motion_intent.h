/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef EXTERNAL_MOTION_INTENT_H
#define EXTERNAL_MOTION_INTENT_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"

typedef enum
{
    EXTERNAL_MOTION_MODE_IDLE = 0u,
    EXTERNAL_MOTION_MODE_FOLLOW_GIMBAL = 1u,
    EXTERNAL_MOTION_MODE_NO_FOLLOW = 2u,
    EXTERNAL_MOTION_MODE_GYRO_SPIN = 3u,
    EXTERNAL_MOTION_MODE_STOP = 4u,
} external_motion_mode_e;

typedef enum
{
    EXTERNAL_MOTION_FRAME_GIMBAL = 0u,
    EXTERNAL_MOTION_FRAME_CHASSIS = 1u,
    EXTERNAL_MOTION_FRAME_FIELD = 2u,
} external_motion_frame_e;

#define EXTERNAL_MOTION_FLAG_VXY_VALID        (1u << 0)
#define EXTERNAL_MOTION_FLAG_WZ_VALID         (1u << 1)
#define EXTERNAL_MOTION_FLAG_YAW_OFFSET_VALID (1u << 2)
#define EXTERNAL_MOTION_FLAG_ACCEL_VALID      (1u << 3)

typedef struct
{
    uint8_t mode;
    uint8_t frame;
    uint16_t flags;
    uint16_t timeout_ms;
    fp32 vx_mps;
    fp32 vy_mps;
    fp32 wz_radps;
    fp32 yaw_offset_rad;
    fp32 ax_mps2;
    fp32 ay_mps2;
    fp32 wz_acc_radps2;
} external_motion_intent_t;

void external_motion_intent_clear(void);
void external_motion_intent_write_from_isr(const external_motion_intent_t *intent);
bool external_motion_intent_read_latest(external_motion_intent_t *out, uint32_t *age_ms);

#endif
