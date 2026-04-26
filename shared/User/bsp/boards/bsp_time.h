/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * Copyright (c) 2024-2026 陈轩 <2811158416@qq.com>
  * @brief      BSP：时间基准（tick ms）
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

// Get a monotonic millisecond tick since boot.
// - Currently implemented via HAL_GetTick() inside BSP.
extern uint32_t bsp_time_get_tick_ms(void);

// Get a microsecond tick since boot.
// - Uses DWT when available; otherwise falls back to ms tick * 1000.
extern uint32_t bsp_time_get_tick_us(void);

#endif
