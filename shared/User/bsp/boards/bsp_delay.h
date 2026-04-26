/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_DELAY_H
#define BSP_DELAY_H
#include "struct_typedef.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);
#endif

