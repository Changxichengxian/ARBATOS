/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

// Legacy PWM interface (no longer used by shoot): kept for compatibility.
// 摩擦轮已切换为 CAN2 速度环/电流输出；此处仅保留默认 PWM 值，避免旧代码编译报错。
#define FRIC_UP   (1400U)
#define FRIC_DOWN (1320U)
#define FRIC_OFF  (1000U)

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
