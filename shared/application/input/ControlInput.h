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
 * `ControlInput.c` 是无状态映射层：来源合并在 `ManualInput.c` 完成，
 * 调用者把同代冻结的输入配置传入这里生成轴和开关。
 */
void ControlInputBuild(const ManualInputState *manual,
                       const input_config_t *config,
                       ControlInputState *out);
uint8_t ControlInputSwitchPosToRaw(uint8_t pos);
uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos);
