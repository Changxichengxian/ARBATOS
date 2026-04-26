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
  * @brief      BSP：电源/复位相关（PVD/RCC CSR）
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_PWR_H
#define BSP_PWR_H

#include <stdint.h>

// Power Voltage Detector (PVD) init/read.
// - On STM32F4: PVDO indicates VDD below selected threshold.
extern void bsp_pwr_pvd_init(void);
extern uint8_t bsp_pwr_pvd_vdd_low(void);

// Raw reset/status flags (STM32F4 RCC->CSR).
extern uint32_t bsp_pwr_rcc_csr(void);

#endif
