/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_DWT_Init(void);
uint64_t BSP_DWT_GetCycles(void);
uint64_t BSP_DWT_GetUs(void);
uint8_t BSP_DWT_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif
