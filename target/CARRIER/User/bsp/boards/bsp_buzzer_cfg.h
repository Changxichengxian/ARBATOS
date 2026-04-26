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

#define BSP_BUZZER_TIM_HANDLE  htim12
#define BSP_BUZZER_TIM_CHANNEL TIM_CHANNEL_1
#define BSP_BUZZER_HAS_PCM     1
#define BSP_BUZZER_PCM_USE_DMA 0

static inline uint32_t bsp_buzzer_tim_clock_hz(void)
{
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1);
    return (ppre1 == RCC_CFGR_PPRE1_DIV1) ? pclk1 : (pclk1 * 2u);
}

#endif
