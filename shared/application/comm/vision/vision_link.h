/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈卓 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"

typedef struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2];
    uint8_t mode;        // 0: idle, 1: auto-aim, 2: small buff, 3: large buff
    float   q[4];        // wxyz
    float   yaw;
    float   yaw_vel;
    float   pitch;
    float   pitch_vel;
    float   bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision;

typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2];
    uint8_t mode;      // 0: idle, 1: control no-fire, 2: control and fire
    float   yaw;
    float   yaw_vel;
    float   yaw_acc;
    float   pitch;
    float   pitch_vel;
    float   pitch_acc;
    uint16_t crc16;
} VisionToGimbal;

void vision_link_init(const fp32 *quat, const fp32 *angle, const fp32 *gyro);
void vision_link_poll_tx(void);
bool vision_take_latest(VisionToGimbal *out);
void vision_link_rx_callback(uint8_t *buf, uint32_t len);

#endif
