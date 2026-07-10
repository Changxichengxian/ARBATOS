/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SHOOT_CTRL_H
#define SHOOT_CTRL_H

#include <stdint.h>

#include "ControlMgr.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ManualInputSnapshot;

typedef struct
{
    uint32_t tickMs;
    uint16_t periodMs;
    uint8_t forceSafe;
    const struct ManualInputSnapshot *manualInput;
} ShootCtrlInput;

typedef struct
{
    int16_t triggerCurrent;
} ShootCtrlOutput;

const ControlController *ShootCtrlDesc(void);
void ShootCtrlPrepare(void);
ControlResult ShootCtrlStep(const ShootCtrlInput *input, ShootCtrlOutput *output);

#ifdef __cplusplus
}
#endif

#endif
