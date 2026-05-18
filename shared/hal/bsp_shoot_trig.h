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
  * @brief      BSP：射击微动开关（触发开关）输入
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_SHOOT_TRIG_H
#define BSP_SHOOT_TRIG_H

#include "types.h"

// Read raw GPIO level (0/1) of shoot trigger micro-switch.
// - Returns 1 when the GPIO exists on this board and *out_level is valid.
// - Returns 0 when not available (application should fallback to SWITCH_TRIGGER_OFF).
extern bool_t bsp_shoot_trig_read_raw(uint8_t *out_level);

#endif
