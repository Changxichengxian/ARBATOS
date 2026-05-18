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
#include "types.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temprate(void);
extern fp32 get_battery_voltage(void);
extern uint8_t bsp_adc_start(void);
extern uint8_t bsp_adc_is_started(void);
extern uint16_t bsp_adc_get_raw(uint8_t index);
extern fp32 bsp_adc_get_channel_voltage(uint8_t index);
extern uint32_t bsp_adc_get_start_ok_count(void);
extern uint32_t bsp_adc_get_start_fail_count(void);
extern uint8_t get_hardware_version(void);
#endif
