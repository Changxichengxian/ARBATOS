/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
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
    uint8_t head[2];     // 'L','S'
    uint8_t mode;        // 0: idle, 1: auto-aim, 2: small buff, 3: large buff
    float   q[4];        // board attitude reference, wxyz
    float   yaw;         // aiming feedback yaw
    float   yaw_vel;     // aiming feedback yaw rate
    float   pitch;       // aiming feedback pitch
    float   pitch_vel;   // aiming feedback pitch rate
    float   bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision;

typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2];   // 'L','S'
    uint8_t mode;      // 0: idle, 1: control no-fire, 2: control and fire
    float   yaw;
    float   yaw_vel;
    float   yaw_acc;
    float   pitch;
    float   pitch_vel;
    float   pitch_acc;
    uint16_t crc16;
} VisionToGimbal;

void VisionLinkInit(const fp32 *quat, const fp32 *angle, const fp32 *gyro);
void VisionLinkPollTx(void);
bool VisionTakeLatest(VisionToGimbal *out);
void VisionLinkRxCallback(uint8_t *buf, uint32_t len);

#endif
