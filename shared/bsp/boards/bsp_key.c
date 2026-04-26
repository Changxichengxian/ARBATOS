/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "bsp_key.h"
#include "main.h"
#include "bsp_key_cfg.h"

#ifndef BSP_KEY_GPIO_Port
#error "BSP_KEY_GPIO_Port is not defined in bsp_key_cfg.h"
#endif
#ifndef BSP_KEY_Pin
#error "BSP_KEY_Pin is not defined in bsp_key_cfg.h"
#endif
#ifndef BSP_KEY_ACTIVE_LOW
#define BSP_KEY_ACTIVE_LOW 1u
#endif
#ifndef BSP_KEY_DEBOUNCE_MS
#define BSP_KEY_DEBOUNCE_MS 30u
#endif

static volatile uint32_t g_key_press_cnt = 0u;
static volatile uint32_t g_key_last_press_tick_ms = 0u;
static volatile uint32_t g_key_last_irq_tick_ms = 0u;

uint8_t bsp_key_read_raw_down(void)
{
    const GPIO_PinState state = HAL_GPIO_ReadPin(BSP_KEY_GPIO_Port, BSP_KEY_Pin);
#if (BSP_KEY_ACTIVE_LOW != 0u)
    return (state == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return (state == GPIO_PIN_SET) ? 1u : 0u;
#endif
}

uint32_t bsp_key_get_press_cnt(void)
{
    return g_key_press_cnt;
}

uint32_t bsp_key_get_last_press_tick_ms(void)
{
    return g_key_last_press_tick_ms;
}

void bsp_key_exti0_callback(void)
{
    const uint32_t now_ms = HAL_GetTick();

    if (g_key_last_irq_tick_ms != 0u && (uint32_t)(now_ms - g_key_last_irq_tick_ms) < BSP_KEY_DEBOUNCE_MS)
    {
        return;
    }
    g_key_last_irq_tick_ms = now_ms;

    if (bsp_key_read_raw_down() == 0u)
    {
        return;
    }

    g_key_press_cnt++;
    g_key_last_press_tick_ms = now_ms;
}
