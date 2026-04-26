#ifndef BSP_BUZZER_CFG_H
#define BSP_BUZZER_CFG_H

#include "bsp_board_ports.h"

#define BSP_BUZZER_TIM_HANDLE  BSP_BOARD_BUZZER_TIM_HANDLE
#define BSP_BUZZER_TIM_CHANNEL BSP_BOARD_BUZZER_TIM_CHANNEL
#define BSP_BUZZER_HAS_PCM     BSP_BOARD_BUZZER_HAS_PCM
#define BSP_BUZZER_PCM_USE_DMA BSP_BOARD_BUZZER_PCM_USE_DMA

static inline uint32_t bsp_buzzer_tim_clock_hz(void)
{
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t ppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1);
    return (ppre1 == RCC_D2CFGR_D2PPRE1_DIV1) ? pclk1 : (pclk1 * 2u);
}

#endif
