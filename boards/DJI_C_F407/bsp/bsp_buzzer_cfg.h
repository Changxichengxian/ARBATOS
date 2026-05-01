/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_BUZZER_CFG_H
#define BSP_BUZZER_CFG_H

#include "bsp_board_ports.h"

#define BSP_BUZZER_TIM_HANDLE  BSP_BOARD_BUZZER_TIM_HANDLE
#define BSP_BUZZER_TIM_CHANNEL BSP_BOARD_BUZZER_TIM_CHANNEL
#define BSP_BUZZER_HAS_PCM     BSP_BOARD_BUZZER_HAS_PCM
#define BSP_BUZZER_PCM_USE_DMA BSP_BOARD_BUZZER_PCM_USE_DMA
#define BSP_BUZZER_DMA_ID      BSP_BOARD_BUZZER_DMA_ID

static inline uint32_t bsp_buzzer_tim_clock_hz(void)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t flash_latency = 0;
    HAL_RCC_GetClockConfig(&clkconfig, &flash_latency);

    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if (clkconfig.APB1CLKDivider == RCC_HCLK_DIV1)
    {
        return pclk1;
    }
    return 2u * pclk1;
}

#endif
