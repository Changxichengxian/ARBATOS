/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include <stdint.h>

#include "ControlMgr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t tickMs;
    uint16_t periodMs;
    uint8_t forceSafe;
} ChassisCtrlInput;

typedef struct
{
    int16_t motorCurrent[4];
} ChassisCtrlOutput;

const ControlController *ChassisCtrlDesc(void);
void ChassisCtrlPrepare(void);
ControlResult ChassisCtrlStep(const ChassisCtrlInput *input, ChassisCtrlOutput *output);

#ifdef __cplusplus
}
#endif

#endif
