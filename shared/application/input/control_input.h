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
#include "manual_input.h"

typedef struct
{
    int16_t axis[INPUT_AXIS_COUNT];
    uint8_t sw[INPUT_SW_COUNT];
} control_input_state_t;

/*
 * `control_input.c` is the business-facing mapping layer:
 * - input source merge happens in `manual_input.c`
 * - axis/switch remap happens here, driven by `g_config.input`
 */
void control_input_update_from_manual_input(const manual_input_state_t *rc);
const control_input_state_t *control_input_get_state(void);
int16_t control_input_axis(input_axis_e axis);
uint8_t control_input_switch(input_switch_e sw);
uint8_t control_input_switch_pos_to_raw(uint8_t pos);
uint8_t control_input_switch_is_pos(uint16_t raw, uint8_t pos);

// Legacy compatibility names.
void input_update_from_rc(const manual_input_state_t *rc);
const control_input_state_t *input_get(void);
int16_t input_axis(input_axis_e axis);
uint8_t input_switch(input_switch_e sw);
uint8_t input_switch_pos_to_raw(uint8_t pos);
uint8_t input_switch_is_pos(uint16_t raw, uint8_t pos);
