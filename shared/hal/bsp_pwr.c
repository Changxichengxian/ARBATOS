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
#include "bsp_pwr.h"

#include <stdint.h>

#include "main.h"

static uint8_t s_pvd_inited = 0u;

void bsp_pwr_pvd_init(void)
{
    if (s_pvd_inited != 0u)
    {
        return;
    }
    s_pvd_inited = 1u;

    // Enable PVD and set threshold to the highest level (about 2.9V on STM32F4).
    // This cannot directly tell "3.3V -> 3.0V" sag, but it helps catch deeper brownout events.
    __HAL_RCC_PWR_CLK_ENABLE();
    MODIFY_REG(PWR->CR, PWR_CR_PLS, PWR_PVDLEVEL_7);
    SET_BIT(PWR->CR, PWR_CR_PVDE);
}
uint8_t bsp_pwr_pvd_vdd_low(void)
{
    return (uint8_t)((READ_BIT(PWR->CSR, PWR_CSR_PVDO) != 0U) ? 1u : 0u);
}

uint32_t bsp_pwr_rcc_csr(void)
{
    return RCC->CSR;
}
