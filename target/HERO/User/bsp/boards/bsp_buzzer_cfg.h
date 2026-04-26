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

#define BSP_BUZZER_TIM_HANDLE  htim4
#define BSP_BUZZER_TIM_CHANNEL TIM_CHANNEL_3
#define BSP_BUZZER_HAS_PCM     1
#define BSP_BUZZER_PCM_USE_DMA 1
#define BSP_BUZZER_DMA_ID      TIM_DMA_ID_CC3

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
