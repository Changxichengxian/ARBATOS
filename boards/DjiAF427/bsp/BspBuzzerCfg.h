/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_BUZZER_CFG_H
#define BSP_BUZZER_CFG_H

#include "BspBoardPorts.h"

#define BSP_BUZZER_TIM_HANDLE  BSP_BOARD_BUZZER_TIM_HANDLE
#define BSP_BUZZER_TIM_CHANNEL BSP_BOARD_BUZZER_TIM_CHANNEL
#define BSP_BUZZER_HAS_PCM     BSP_BOARD_BUZZER_HAS_PCM
#define BSP_BUZZER_PCM_USE_DMA BSP_BOARD_BUZZER_PCM_USE_DMA

static inline uint32_t BspBuzzerTimClockHz(void)
{
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1);
    return (ppre1 == RCC_CFGR_PPRE1_DIV1) ? pclk1 : (pclk1 * 2u);
}

#endif
