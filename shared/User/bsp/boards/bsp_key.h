/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "struct_typedef.h"

// Board-specific pin/level is configured in bsp_key_cfg.h.

// Board key (level defined by BSP_KEY_ACTIVE_LOW).

extern void bsp_key_exti0_callback(void);
extern uint8_t bsp_key_read_raw_down(void);
extern uint32_t bsp_key_get_press_cnt(void);
extern uint32_t bsp_key_get_last_press_tick_ms(void);

#endif
