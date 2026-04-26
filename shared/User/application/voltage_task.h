/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"



/**
  * @brief          power ADC and calculate electricity percentage
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void battery_voltage_task(void const * argument);

/**
  * @brief          get electricity percentage
  * @param[in]      void
  * @retval         electricity percentage, unit 1, 1 = 1%
  */
/**
  * @brief          获取电量
  * @param[in]      void
  * @retval         电量, 单位 1, 1 = 1%
  */
extern uint16_t get_battery_percentage(void);
#endif
