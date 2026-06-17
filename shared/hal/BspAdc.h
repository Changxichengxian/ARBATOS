/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
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
extern uint8_t BspAdcStart(void);
extern uint8_t BspAdcIsStarted(void);
extern uint16_t BspAdcGetRaw(uint8_t index);
extern fp32 BspAdcGetChannelVoltage(uint8_t index);
extern uint32_t BspAdcGetStartOkCount(void);
extern uint32_t BspAdcGetStartFailCount(void);
extern uint8_t get_hardware_version(void);
#endif
