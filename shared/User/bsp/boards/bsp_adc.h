/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temprate(void);
extern fp32 get_battery_voltage(void);
extern uint8_t get_hardware_version(void);
#endif
