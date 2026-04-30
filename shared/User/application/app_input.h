/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>

#include "config.h"
#include "remote_control.h"

typedef struct
{
    int16_t axis[INPUT_AXIS_COUNT];
    uint8_t sw[INPUT_SW_COUNT];
} app_input_t;

void app_input_update_from_rc(const RC_ctrl_t *rc);
const app_input_t *app_input_get(void);
int16_t app_input_axis(app_axis_e axis);
uint8_t app_input_switch(app_switch_e sw);
