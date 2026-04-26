/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_dwt.h"

#include "main.h"

static uint8_t s_dwt_state = 0u; /* 0=uninit 1=ready 2=unsupported */
static uint32_t s_dwt_last_cyccnt = 0u;
static uint32_t s_dwt_last_tick_ms = 0u;
static uint64_t s_dwt_wrap_count = 0u;
static uint32_t s_dwt_cpu_freq_hz = 0u;
static uint32_t s_dwt_cycles_per_ms = 0u;

static uint32_t bsp_dwt_get_primask(void)
{
#if defined(__CC_ARM) && !defined(__clang__)
    uint32_t state;
    __asm
    {
        mrs state, primask
    }
    return state;
#else
    return __get_PRIMASK();
#endif
}

static void bsp_dwt_restore_irq(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void bsp_dwt_sample_locked(uint64_t *cycles_out)
{
    const uint32_t now_cyccnt = DWT->CYCCNT;
    const uint32_t now_tick_ms = HAL_GetTick();
    const uint32_t elapsed_ms = now_tick_ms - s_dwt_last_tick_ms;
    uint64_t wrap_inc = 0u;

    if (s_dwt_cycles_per_ms != 0u)
    {
        wrap_inc = ((uint64_t)elapsed_ms * (uint64_t)s_dwt_cycles_per_ms) >> 32;
    }
    if (wrap_inc == 0u && now_cyccnt < s_dwt_last_cyccnt)
    {
        wrap_inc = 1u;
    }

    s_dwt_wrap_count += wrap_inc;
    s_dwt_last_cyccnt = now_cyccnt;
    s_dwt_last_tick_ms = now_tick_ms;

    if (cycles_out != NULL)
    {
        *cycles_out = (s_dwt_wrap_count << 32) | (uint64_t)now_cyccnt;
    }
}

void BSP_DWT_Init(void)
{
    const uint32_t primask = bsp_dwt_get_primask();

    __disable_irq();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u)
    {
        s_dwt_state = 2u;
        s_dwt_last_cyccnt = 0u;
        s_dwt_last_tick_ms = HAL_GetTick();
        s_dwt_wrap_count = 0u;
        s_dwt_cpu_freq_hz = 0u;
        s_dwt_cycles_per_ms = 0u;
        bsp_dwt_restore_irq(primask);
        return;
    }

    s_dwt_cpu_freq_hz = SystemCoreClock;
    if (s_dwt_cpu_freq_hz == 0u)
    {
        s_dwt_cpu_freq_hz = 1u;
    }

    s_dwt_cycles_per_ms = s_dwt_cpu_freq_hz / 1000u;
    s_dwt_wrap_count = 0u;
    s_dwt_last_cyccnt = DWT->CYCCNT;
    s_dwt_last_tick_ms = HAL_GetTick();
    s_dwt_state = 1u;

    bsp_dwt_restore_irq(primask);
}

uint64_t BSP_DWT_GetCycles(void)
{
    uint64_t cycles = 0u;
    const uint32_t primask = bsp_dwt_get_primask();

    if (s_dwt_state == 0u)
    {
        BSP_DWT_Init();
    }

    if (s_dwt_state != 1u)
    {
        return 0u;
    }

    __disable_irq();
    bsp_dwt_sample_locked(&cycles);
    bsp_dwt_restore_irq(primask);

    return cycles;
}

uint64_t BSP_DWT_GetUs(void)
{
    const uint64_t cycles = BSP_DWT_GetCycles();

    if (s_dwt_state != 1u)
    {
        return (uint64_t)HAL_GetTick() * 1000u;
    }

    return (cycles * 1000000u) / (uint64_t)s_dwt_cpu_freq_hz;
}

uint8_t BSP_DWT_IsReady(void)
{
    if (s_dwt_state == 0u)
    {
        BSP_DWT_Init();
    }

    return (s_dwt_state == 1u) ? 1u : 0u;
}
