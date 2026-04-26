/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "struct_typedef.h"

typedef struct
{
    uint8_t online;
    uint8_t rx_dlc;
    uint16_t rx_id;
    uint32_t rx_count;
    uint32_t last_rx_tick;
    fp32 position;
    fp32 velocity;
    fp32 torque;
} arm_motor_feedback_t;

void arm_task(void const *argument);
const arm_motor_feedback_t *arm_get_feedback(uint8_t index);
