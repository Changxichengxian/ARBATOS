/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>

#include "RobotConfig.h"
#include "ManualInput.h"

typedef struct
{
    int16_t axis[INPUT_AXIS_COUNT];
    uint8_t sw[INPUT_SW_COUNT];
} ControlInputState;

/*
 * `ControlInput.c` is the business-facing mapping layer:
 * - input source merge happens in `ManualInput.c`
 * - axis/switch remap happens here, driven by `g_config.input`
 */
void ControlInputBuild(const ManualInputState *manual,
                       const input_config_t *config,
                       ControlInputState *out);
void ControlInputUpdateFromManualInput(const ManualInputState *rc);
const ControlInputState *ControlInputGetState(void);
uint8_t ControlInputGetCopy(ControlInputState *out);
int16_t ControlInputAxis(input_axis_e axis);
uint8_t ControlInputSwitch(input_switch_e sw);
uint8_t ControlInputSwitchPosToRaw(uint8_t pos);
uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos);

// Legacy compatibility names.
void input_update_from_rc(const ManualInputState *rc);
const ControlInputState *input_get(void);
uint8_t input_get_copy(ControlInputState *out);
int16_t input_axis(input_axis_e axis);
uint8_t input_switch(input_switch_e sw);
uint8_t input_switch_pos_to_raw(uint8_t pos);
uint8_t input_switch_is_pos(uint16_t raw, uint8_t pos);
