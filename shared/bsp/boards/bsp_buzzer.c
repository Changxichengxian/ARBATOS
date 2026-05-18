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
  * @brief      BSP: buzzer implementation (shared logic + board cfg)
  *
  * Board-specific configuration is provided by bsp_buzzer_cfg.h.
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_buzzer.h"
#include "tim.h"
#include "bsp_buzzer_cfg.h"

#include <string.h>

#ifndef BSP_BUZZER_TIM_HANDLE
#error "BSP_BUZZER_TIM_HANDLE is not defined in bsp_buzzer_cfg.h"
#endif
#ifndef BSP_BUZZER_TIM_CHANNEL
#define BSP_BUZZER_TIM_CHANNEL TIM_CHANNEL_1
#endif
#ifndef BSP_BUZZER_HAS_PCM
#define BSP_BUZZER_HAS_PCM 0
#endif
#ifndef BSP_BUZZER_PCM_USE_DMA
#define BSP_BUZZER_PCM_USE_DMA 1
#endif
#if BSP_BUZZER_HAS_PCM && BSP_BUZZER_PCM_USE_DMA
#ifndef BSP_BUZZER_DMA_ID
#error "BSP_BUZZER_DMA_ID is not defined in bsp_buzzer_cfg.h"
#endif
#endif

extern TIM_HandleTypeDef BSP_BUZZER_TIM_HANDLE;

static uint8_t g_buzzer_enable_cfg = 1u;
static uint32_t g_buzzer_pcm_carrier_min_hz_cfg = 0u;
static uint16_t g_buzzer_pcm_stream_gain_q8_cfg = 0u;

static uint32_t buzzer_tim_clock_hz(void)
{
    return bsp_buzzer_tim_clock_hz();
}

static void buzzer_pwm_stop(void)
{
    (void)HAL_TIM_PWM_Stop(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);
}

void buzzer_set_enable(uint8_t enable)
{
    g_buzzer_enable_cfg = (enable != 0u) ? 1u : 0u;
    if (g_buzzer_enable_cfg == 0u)
    {
        buzzer_pcm_stop();
        buzzer_pwm_stop();
    }
}

void buzzer_pcm_set_carrier_min_hz(uint32_t carrier_min_hz)
{
    g_buzzer_pcm_carrier_min_hz_cfg = carrier_min_hz;
}

void buzzer_pcm_set_stream_gain_q8(uint16_t gain_q8)
{
    g_buzzer_pcm_stream_gain_q8_cfg = gain_q8;
}

#if BSP_BUZZER_HAS_PCM
#if BSP_BUZZER_PCM_USE_DMA
#define BUZZER_PCM_DMA_BUF_LEN 1024u
#endif
#define BUZZER_PCM_CARRIER_HZ_MIN_DEFAULT 48000u
#define BUZZER_PCM_GAIN_Q8_DEFAULT 256u
#define BUZZER_PCM_GAIN_Q8_MAX 4096u

#define BUZZER_PCM_STREAM_RING_SIZE (16u * 1024u)
#define BUZZER_PCM_STREAM_RING_MASK (BUZZER_PCM_STREAM_RING_SIZE - 1u)
typedef char _check_buzzer_pcm_ring_pow2[(BUZZER_PCM_STREAM_RING_SIZE & BUZZER_PCM_STREAM_RING_MASK) == 0u ? 1 : -1];

#define BUZZER_TONE_SAMPLE_RATE_HZ 24000u
#define BUZZER_TONE_PCM_BUF_MAX 512u

static volatile uint8_t g_buzzer_pcm_running = 0u;
static volatile uint8_t g_buzzer_pcm_stream_mode = 0u;
static volatile uint8_t g_buzzer_pcm_loop = 0u;
static volatile uint8_t g_buzzer_pcm_volume = 255u;
static const uint8_t *g_buzzer_pcm_data = NULL;
static volatile uint32_t g_buzzer_pcm_len = 0u;
static volatile uint32_t g_buzzer_pcm_pos = 0u;
static volatile uint32_t g_buzzer_pcm_sample_rate_hz = 0u;
static volatile uint32_t g_buzzer_pcm_carrier_hz = BUZZER_PCM_CARRIER_HZ_MIN_DEFAULT;
static volatile uint32_t g_buzzer_pcm_phase = 0u;

static uint16_t g_buzzer_pcm_saved_psc = 0u;
static uint16_t g_buzzer_pcm_saved_arr = 0u;
static uint8_t g_buzzer_pcm_saved_oc_preload = 0u;
static uint16_t g_buzzer_pcm_arr = 0u;
__attribute__((section(".ccmram"))) static uint16_t g_buzzer_pcm_duty_lut[256];
#if BSP_BUZZER_PCM_USE_DMA
static uint16_t g_buzzer_pcm_buf[BUZZER_PCM_DMA_BUF_LEN];
#endif

static uint8_t g_buzzer_tone_pcm[BUZZER_TONE_PCM_BUF_MAX];
static uint32_t g_buzzer_tone_len = 0u;

__attribute__((section(".ccmram"))) static uint8_t g_buzzer_pcm_stream_ring[BUZZER_PCM_STREAM_RING_SIZE];
static volatile uint32_t g_buzzer_pcm_stream_head = 0u;
static volatile uint32_t g_buzzer_pcm_stream_tail = 0u;
static volatile uint8_t g_buzzer_pcm_stream_cur = 128u;

static uint32_t buzzer_pcm_carrier_min_hz(void)
{
    uint32_t hz = g_buzzer_pcm_carrier_min_hz_cfg;
    if (hz == 0u)
    {
        hz = BUZZER_PCM_CARRIER_HZ_MIN_DEFAULT;
    }
    return hz;
}

static uint16_t buzzer_pcm_stream_gain_q8(void)
{
    uint32_t gain = g_buzzer_pcm_stream_gain_q8_cfg;
    if (gain == 0u)
    {
        gain = BUZZER_PCM_GAIN_Q8_DEFAULT;
    }
    else if (gain > BUZZER_PCM_GAIN_Q8_MAX)
    {
        gain = BUZZER_PCM_GAIN_Q8_MAX;
    }
    return (uint16_t)gain;
}

static uint8_t buzzer_pcm_is_oc_preload_enabled(void)
{
    switch (BSP_BUZZER_TIM_CHANNEL)
    {
    case TIM_CHANNEL_1:
        return (BSP_BUZZER_TIM_HANDLE.Instance->CCMR1 & TIM_CCMR1_OC1PE) ? 1u : 0u;
    case TIM_CHANNEL_2:
        return (BSP_BUZZER_TIM_HANDLE.Instance->CCMR1 & TIM_CCMR1_OC2PE) ? 1u : 0u;
    case TIM_CHANNEL_3:
        return (BSP_BUZZER_TIM_HANDLE.Instance->CCMR2 & TIM_CCMR2_OC3PE) ? 1u : 0u;
    case TIM_CHANNEL_4:
        return (BSP_BUZZER_TIM_HANDLE.Instance->CCMR2 & TIM_CCMR2_OC4PE) ? 1u : 0u;
    default:
        return 0u;
    }
}

static uint16_t buzzer_pcm_silence_duty(uint16_t arr)
{
    return (uint16_t)((arr + 1u) / 2u);
}

static uint16_t buzzer_pcm_u8_to_duty(uint8_t sample_u8, uint16_t arr, uint8_t volume, uint16_t gain_q8)
{
    int32_t centered = (int32_t)sample_u8 - 128;
    centered = (centered * (int32_t)volume) / 255;
    centered = (centered * (int32_t)gain_q8) / 256;
    int32_t scaled_u8 = centered + 128;
    if (scaled_u8 < 0)
    {
        scaled_u8 = 0;
    }
    else if (scaled_u8 > 255)
    {
        scaled_u8 = 255;
    }

    uint32_t duty = ((uint32_t)scaled_u8 * (uint32_t)(arr + 1u)) >> 8;
    if (arr > 1u)
    {
        if (duty == 0u)
        {
            duty = 1u;
        }
        else if (duty >= (uint32_t)arr)
        {
            duty = (uint32_t)arr - 1u;
        }
    }
    return (uint16_t)duty;
}

static void buzzer_pcm_update_lut(uint16_t arr, uint8_t volume, uint16_t gain_q8)
{
    for (uint32_t i = 0u; i < 256u; i++)
    {
        g_buzzer_pcm_duty_lut[i] = buzzer_pcm_u8_to_duty((uint8_t)i, arr, volume, gain_q8);
    }
}

static uint32_t buzzer_pcm_stream_used(uint32_t head, uint32_t tail)
{
    return head - tail;
}

static uint32_t buzzer_pcm_stream_free(uint32_t head, uint32_t tail)
{
    return BUZZER_PCM_STREAM_RING_SIZE - buzzer_pcm_stream_used(head, tail);
}

static uint8_t buzzer_pcm_stream_pop_u8(uint8_t *out_sample)
{
    if (out_sample == NULL)
    {
        return 0u;
    }

    uint32_t tail = g_buzzer_pcm_stream_tail;
    const uint32_t head = g_buzzer_pcm_stream_head;
    if (tail == head)
    {
        return 0u;
    }

    const uint8_t sample = g_buzzer_pcm_stream_ring[tail & BUZZER_PCM_STREAM_RING_MASK];
    g_buzzer_pcm_stream_tail = tail + 1u;
    *out_sample = sample;
    return 1u;
}

static void buzzer_pcm_fill(uint16_t *dst, uint32_t count)
{
    uint32_t i;
    const uint8_t *data = g_buzzer_pcm_data;
    uint32_t len = g_buzzer_pcm_len;
    uint32_t pos = g_buzzer_pcm_pos;
    uint8_t loop = g_buzzer_pcm_loop;
    uint16_t arr = g_buzzer_pcm_arr;
    uint16_t silence = buzzer_pcm_silence_duty(arr);
    uint32_t sample_rate_hz = g_buzzer_pcm_sample_rate_hz;
    uint32_t carrier_hz = g_buzzer_pcm_carrier_hz;
    uint32_t phase = g_buzzer_pcm_phase;

    if (g_buzzer_pcm_stream_mode != 0u)
    {
        uint8_t cur = g_buzzer_pcm_stream_cur;
        silence = g_buzzer_pcm_duty_lut[128u];

        for (i = 0; i < count; i++)
        {
            uint16_t duty = silence;

            if (sample_rate_hz != 0u && carrier_hz != 0u)
            {
                duty = g_buzzer_pcm_duty_lut[cur];
                dst[i] = duty;

                phase += sample_rate_hz;
                if (phase >= carrier_hz)
                {
                    phase -= carrier_hz;

                    uint8_t next = 128u;
                    if (buzzer_pcm_stream_pop_u8(&next) != 0u)
                    {
                        cur = next;
                    }
                    else
                    {
                        cur = 128u;
                    }
                }
            }
            else
            {
                dst[i] = duty;
            }
        }

        g_buzzer_pcm_stream_cur = cur;
        g_buzzer_pcm_phase = phase;
        return;
    }

    for (i = 0; i < count; i++)
    {
        uint16_t duty = silence;

        if (data != NULL && len != 0u && sample_rate_hz != 0u && carrier_hz != 0u)
        {
            if (pos >= len)
            {
                if (loop != 0u)
                {
                    pos = 0;
                }
                else
                {
                    dst[i] = duty;
                    continue;
                }
            }

            duty = g_buzzer_pcm_duty_lut[data[pos]];
            dst[i] = duty;

            phase += sample_rate_hz;
            if (phase >= carrier_hz)
            {
                phase -= carrier_hz;
                pos++;
                if (pos >= len)
                {
                    if (loop != 0u)
                    {
                        pos = 0;
                    }
                    else
                    {
                        pos = len;
                    }
                }
            }
        }
        else
        {
            dst[i] = duty;
        }
    }

    g_buzzer_pcm_pos = pos;
    g_buzzer_pcm_phase = phase;
}

int buzzer_tone_start_hz(uint32_t freq_hz, uint8_t volume)
{
    if (freq_hz == 0u)
    {
        buzzer_tone_stop();
        return 0;
    }

    uint32_t sample_rate_hz = BUZZER_TONE_SAMPLE_RATE_HZ;
    if (freq_hz >= (sample_rate_hz / 2u))
    {
        freq_hz = (sample_rate_hz / 2u) - 1u;
    }

    uint32_t period_samples = sample_rate_hz / freq_hz;
    if (period_samples < 2u)
    {
        period_samples = 2u;
    }
    if (period_samples > BUZZER_TONE_PCM_BUF_MAX)
    {
        period_samples = BUZZER_TONE_PCM_BUF_MAX;
    }

    uint32_t half = period_samples / 2u;
    for (uint32_t i = 0; i < period_samples; i++)
    {
        g_buzzer_tone_pcm[i] = (i < half) ? 0u : 255u;
    }
    g_buzzer_tone_len = period_samples;

    return buzzer_pcm_start_u8(g_buzzer_tone_pcm, g_buzzer_tone_len, sample_rate_hz, 1u, volume);
}

int buzzer_tone_start_legacy(uint16_t psc, uint16_t pwm)
{
    if (psc == 0xFFFFu || pwm == 0u)
    {
        buzzer_tone_stop();
        return 0;
    }

    uint32_t arr_base = (uint32_t)BSP_BUZZER_TIM_HANDLE.Init.Period;
    uint32_t denom = ((uint32_t)psc + 1u) * (arr_base + 1u);
    if (denom == 0u)
    {
        return -1;
    }

    uint32_t tim_clk = buzzer_tim_clock_hz();
    uint32_t freq_hz = tim_clk / denom;
    if (freq_hz == 0u)
    {
        freq_hz = 1u;
    }

    uint32_t duty = (uint32_t)pwm;
    if (duty > arr_base)
    {
        duty = arr_base;
    }
    uint32_t diff = (duty * 2u > arr_base) ? (duty * 2u - arr_base) : (arr_base - duty * 2u);
    uint8_t volume = 0u;
    if (arr_base > 0u && diff < arr_base)
    {
        volume = (uint8_t)(((arr_base - diff) * 255u) / arr_base);
    }

    return buzzer_tone_start_hz(freq_hz, volume);
}

uint16_t buzzer_legacy_pwm_half(void)
{
    uint32_t arr_base = (uint32_t)BSP_BUZZER_TIM_HANDLE.Init.Period;
    uint32_t duty = arr_base / 2u;
    if (duty == 0u)
    {
        duty = 10000u;
    }
    if (duty > 0xFFFFu)
    {
        duty = 0xFFFFu;
    }
    return (uint16_t)duty;
}

void buzzer_tone_stop(void)
{
    buzzer_pcm_stop();
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);
}

int buzzer_pcm_start_u8(const uint8_t *pcm_u8, uint32_t len, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume)
{
    if (sample_rate_hz == 0u)
    {
        return -1;
    }

    if (g_buzzer_enable_cfg == 0u)
    {
        buzzer_pcm_stop();
        __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);
        return -2;
    }

    buzzer_pcm_stop();
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);

#if BSP_BUZZER_PCM_USE_DMA
    if (BSP_BUZZER_TIM_HANDLE.hdma[BSP_BUZZER_DMA_ID] == NULL)
    {
        return -4;
    }
#endif

    uint32_t tim_clk = buzzer_tim_clock_hz();
    const uint32_t carrier_min_hz = buzzer_pcm_carrier_min_hz();
    uint32_t carrier_hz = (sample_rate_hz > carrier_min_hz) ? sample_rate_hz : carrier_min_hz;
    uint32_t arr32 = (tim_clk / carrier_hz);
    if (arr32 == 0u)
    {
        return -1;
    }
    arr32 -= 1u;
    if (arr32 > 0xFFFFu)
    {
        return -1;
    }

    g_buzzer_pcm_saved_psc = (uint16_t)BSP_BUZZER_TIM_HANDLE.Instance->PSC;
    g_buzzer_pcm_saved_arr = (uint16_t)BSP_BUZZER_TIM_HANDLE.Instance->ARR;
    g_buzzer_pcm_saved_oc_preload = buzzer_pcm_is_oc_preload_enabled();

    g_buzzer_pcm_arr = (uint16_t)arr32;
    buzzer_pcm_update_lut(g_buzzer_pcm_arr, volume, BUZZER_PCM_GAIN_Q8_DEFAULT);
    g_buzzer_pcm_sample_rate_hz = sample_rate_hz;
    g_buzzer_pcm_carrier_hz = carrier_hz;
    g_buzzer_pcm_phase = 0u;
    g_buzzer_pcm_stream_mode = 0u;
    g_buzzer_pcm_data = pcm_u8;
    g_buzzer_pcm_len = len;
    g_buzzer_pcm_pos = 0u;
    g_buzzer_pcm_loop = (loop != 0u) ? 1u : 0u;
    g_buzzer_pcm_volume = volume;

    (void)HAL_TIM_PWM_Stop(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);

    __HAL_TIM_PRESCALER(&BSP_BUZZER_TIM_HANDLE, 0);
    __HAL_TIM_SET_AUTORELOAD(&BSP_BUZZER_TIM_HANDLE, g_buzzer_pcm_arr);
    __HAL_TIM_ENABLE_OCxPRELOAD(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);

#if BSP_BUZZER_PCM_USE_DMA
    buzzer_pcm_fill(&g_buzzer_pcm_buf[0], BUZZER_PCM_DMA_BUF_LEN);

    if (HAL_TIM_PWM_Start_DMA(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, (uint32_t *)g_buzzer_pcm_buf, BUZZER_PCM_DMA_BUF_LEN) != HAL_OK)
    {
        buzzer_pcm_stop();
        return -3;
    }
#else
    uint16_t duty = 0u;
    buzzer_pcm_fill(&duty, 1u);
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, duty);

    if (HAL_TIM_PWM_Start_IT(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL) != HAL_OK)
    {
        buzzer_pcm_stop();
        return -3;
    }
#endif

    g_buzzer_pcm_running = 1u;
    return 0;
}

int buzzer_pcm_start_stream_u8(uint32_t sample_rate_hz, uint8_t volume)
{
    if (sample_rate_hz == 0u)
    {
        return -1;
    }

    if (g_buzzer_enable_cfg == 0u)
    {
        buzzer_pcm_stop();
        __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);
        return -2;
    }

    buzzer_pcm_stop();
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);

#if BSP_BUZZER_PCM_USE_DMA
    if (BSP_BUZZER_TIM_HANDLE.hdma[BSP_BUZZER_DMA_ID] == NULL)
    {
        return -4;
    }
#endif

    uint32_t tim_clk = buzzer_tim_clock_hz();
    const uint32_t carrier_min_hz = buzzer_pcm_carrier_min_hz();
    uint32_t carrier_hz = (sample_rate_hz > carrier_min_hz) ? sample_rate_hz : carrier_min_hz;
    uint32_t arr32 = (tim_clk / carrier_hz);
    if (arr32 == 0u)
    {
        return -1;
    }
    arr32 -= 1u;
    if (arr32 > 0xFFFFu)
    {
        return -1;
    }

    g_buzzer_pcm_saved_psc = (uint16_t)BSP_BUZZER_TIM_HANDLE.Instance->PSC;
    g_buzzer_pcm_saved_arr = (uint16_t)BSP_BUZZER_TIM_HANDLE.Instance->ARR;
    g_buzzer_pcm_saved_oc_preload = buzzer_pcm_is_oc_preload_enabled();

    g_buzzer_pcm_arr = (uint16_t)arr32;
    buzzer_pcm_update_lut(g_buzzer_pcm_arr, volume, buzzer_pcm_stream_gain_q8());
    g_buzzer_pcm_sample_rate_hz = sample_rate_hz;
    g_buzzer_pcm_carrier_hz = carrier_hz;
    g_buzzer_pcm_phase = 0u;

    g_buzzer_pcm_stream_head = 0u;
    g_buzzer_pcm_stream_tail = 0u;
    g_buzzer_pcm_stream_cur = 128u;
    g_buzzer_pcm_stream_mode = 1u;

    g_buzzer_pcm_data = NULL;
    g_buzzer_pcm_len = 0u;
    g_buzzer_pcm_pos = 0u;
    g_buzzer_pcm_loop = 0u;
    g_buzzer_pcm_volume = volume;

    (void)HAL_TIM_PWM_Stop(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);

    __HAL_TIM_PRESCALER(&BSP_BUZZER_TIM_HANDLE, 0);
    __HAL_TIM_SET_AUTORELOAD(&BSP_BUZZER_TIM_HANDLE, g_buzzer_pcm_arr);
    __HAL_TIM_ENABLE_OCxPRELOAD(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);

#if BSP_BUZZER_PCM_USE_DMA
    buzzer_pcm_fill(&g_buzzer_pcm_buf[0], BUZZER_PCM_DMA_BUF_LEN);

    if (HAL_TIM_PWM_Start_DMA(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, (uint32_t *)g_buzzer_pcm_buf, BUZZER_PCM_DMA_BUF_LEN) != HAL_OK)
    {
        buzzer_pcm_stop();
        return -3;
    }
#else
    uint16_t duty = 0u;
    buzzer_pcm_fill(&duty, 1u);
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, duty);

    if (HAL_TIM_PWM_Start_IT(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL) != HAL_OK)
    {
        buzzer_pcm_stop();
        return -3;
    }
#endif

    g_buzzer_pcm_running = 1u;
    return 0;
}

uint32_t buzzer_pcm_stream_write_u8(const uint8_t *pcm_u8, uint32_t len)
{
    if (pcm_u8 == NULL || len == 0u)
    {
        return 0u;
    }
    if (g_buzzer_pcm_running == 0u || g_buzzer_pcm_stream_mode == 0u)
    {
        return 0u;
    }

    const uint32_t head = g_buzzer_pcm_stream_head;
    const uint32_t tail = g_buzzer_pcm_stream_tail;
    uint32_t free = buzzer_pcm_stream_free(head, tail);
    if (free == 0u)
    {
        return 0u;
    }
    if (len > free)
    {
        len = free;
    }

    const uint32_t idx = head & BUZZER_PCM_STREAM_RING_MASK;
    const uint32_t to_end = BUZZER_PCM_STREAM_RING_SIZE - idx;
    if (len <= to_end)
    {
        memcpy(&g_buzzer_pcm_stream_ring[idx], pcm_u8, len);
    }
    else
    {
        memcpy(&g_buzzer_pcm_stream_ring[idx], pcm_u8, to_end);
        const uint32_t left = len - to_end;
        memcpy(&g_buzzer_pcm_stream_ring[0], pcm_u8 + to_end, left);
    }

    __DMB();
    g_buzzer_pcm_stream_head = head + len;
    return len;
}

uint32_t buzzer_pcm_stream_get_used(void)
{
    const uint32_t head = g_buzzer_pcm_stream_head;
    const uint32_t tail = g_buzzer_pcm_stream_tail;
    return buzzer_pcm_stream_used(head, tail);
}

uint32_t buzzer_pcm_stream_get_free(void)
{
    const uint32_t head = g_buzzer_pcm_stream_head;
    const uint32_t tail = g_buzzer_pcm_stream_tail;
    return buzzer_pcm_stream_free(head, tail);
}

uint8_t buzzer_pcm_is_stream_mode(void)
{
    return (g_buzzer_pcm_stream_mode != 0u) ? 1u : 0u;
}

void buzzer_pcm_stop(void)
{
    if (g_buzzer_pcm_running == 0u)
    {
        return;
    }

    g_buzzer_pcm_running = 0u;

#if BSP_BUZZER_PCM_USE_DMA
    (void)HAL_TIM_PWM_Stop_DMA(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
#else
    (void)HAL_TIM_PWM_Stop_IT(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
#endif
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, 0u);

    __HAL_TIM_PRESCALER(&BSP_BUZZER_TIM_HANDLE, g_buzzer_pcm_saved_psc);
    __HAL_TIM_SET_AUTORELOAD(&BSP_BUZZER_TIM_HANDLE, g_buzzer_pcm_saved_arr);
    if (g_buzzer_pcm_saved_oc_preload != 0u)
    {
        __HAL_TIM_ENABLE_OCxPRELOAD(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
    }
    else
    {
        __HAL_TIM_DISABLE_OCxPRELOAD(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
    }

    (void)HAL_TIM_PWM_Start(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);

    g_buzzer_pcm_data = NULL;
    g_buzzer_pcm_len = 0u;
    g_buzzer_pcm_pos = 0u;
    g_buzzer_pcm_loop = 0u;
    g_buzzer_pcm_sample_rate_hz = 0u;
    g_buzzer_pcm_carrier_hz = buzzer_pcm_carrier_min_hz();
    g_buzzer_pcm_phase = 0u;
    g_buzzer_pcm_stream_mode = 0u;
}

uint8_t buzzer_pcm_is_running(void)
{
    return g_buzzer_pcm_running;
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
#if BSP_BUZZER_PCM_USE_DMA
    if (htim->Instance != BSP_BUZZER_TIM_HANDLE.Instance)
    {
        return;
    }
    if (g_buzzer_pcm_running == 0u)
    {
        return;
    }

    buzzer_pcm_fill(&g_buzzer_pcm_buf[0], BUZZER_PCM_DMA_BUF_LEN / 2u);
#else
    (void)htim;
#endif
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != BSP_BUZZER_TIM_HANDLE.Instance)
    {
        return;
    }
    if (g_buzzer_pcm_running == 0u)
    {
        return;
    }

#if BSP_BUZZER_PCM_USE_DMA
    buzzer_pcm_fill(&g_buzzer_pcm_buf[BUZZER_PCM_DMA_BUF_LEN / 2u], BUZZER_PCM_DMA_BUF_LEN / 2u);
#else
    uint16_t duty = 0u;
    buzzer_pcm_fill(&duty, 1u);
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, duty);
#endif
}

#else

int buzzer_tone_start_hz(uint32_t freq_hz, uint8_t volume)
{
    (void)g_buzzer_pcm_carrier_min_hz_cfg;
    (void)g_buzzer_pcm_stream_gain_q8_cfg;

    if (g_buzzer_enable_cfg == 0u || freq_hz == 0u || volume == 0u)
    {
        buzzer_pwm_stop();
        return 0;
    }

    const uint32_t tim_clk = buzzer_tim_clock_hz();
    uint32_t ticks = tim_clk / freq_hz;
    if (ticks == 0u)
    {
        ticks = 1u;
    }

    uint32_t psc = (ticks - 1u) / 65536u;
    if (psc > 0xFFFFu)
    {
        psc = 0xFFFFu;
    }
    uint32_t arr = (ticks / (psc + 1u));
    if (arr == 0u)
    {
        arr = 1u;
    }
    arr -= 1u;
    if (arr > 0xFFFFu)
    {
        arr = 0xFFFFu;
    }

    const uint32_t half = (arr + 1u) / 2u;
    uint32_t duty = (half * (uint32_t)volume) / 255u;
    if (duty > arr)
    {
        duty = arr;
    }

    (void)HAL_TIM_PWM_Stop(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL);
    __HAL_TIM_SET_PRESCALER(&BSP_BUZZER_TIM_HANDLE, (uint16_t)psc);
    __HAL_TIM_SET_AUTORELOAD(&BSP_BUZZER_TIM_HANDLE, (uint16_t)arr);
    __HAL_TIM_SET_COUNTER(&BSP_BUZZER_TIM_HANDLE, 0u);
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL, duty);
    if (HAL_TIM_PWM_Start(&BSP_BUZZER_TIM_HANDLE, BSP_BUZZER_TIM_CHANNEL) != HAL_OK)
    {
        buzzer_pwm_stop();
        return -1;
    }

    return 0;
}

int buzzer_tone_start_legacy(uint16_t psc, uint16_t pwm)
{
    if (psc == 0xFFFFu || pwm == 0u)
    {
        buzzer_tone_stop();
        return 0;
    }

    const uint32_t arr_base = (uint32_t)BSP_BUZZER_TIM_HANDLE.Init.Period;
    const uint32_t denom = ((uint32_t)psc + 1u) * (arr_base + 1u);
    if (denom == 0u)
    {
        return -1;
    }

    const uint32_t tim_clk = buzzer_tim_clock_hz();
    uint32_t freq_hz = tim_clk / denom;
    if (freq_hz == 0u)
    {
        freq_hz = 1u;
    }

    uint32_t duty = (uint32_t)pwm;
    if (duty > arr_base)
    {
        duty = arr_base;
    }
    const uint32_t diff = (duty * 2u > arr_base) ? (duty * 2u - arr_base) : (arr_base - duty * 2u);
    uint8_t volume = 0u;
    if (arr_base > 0u && diff < arr_base)
    {
        volume = (uint8_t)(((arr_base - diff) * 255u) / arr_base);
    }

    return buzzer_tone_start_hz(freq_hz, volume);
}

uint16_t buzzer_legacy_pwm_half(void)
{
    uint32_t arr_base = (uint32_t)BSP_BUZZER_TIM_HANDLE.Init.Period;
    uint32_t duty = (arr_base + 1u) / 2u;
    if (duty == 0u)
    {
        duty = 1u;
    }
    if (duty > 0xFFFFu)
    {
        duty = 0xFFFFu;
    }
    return (uint16_t)duty;
}

void buzzer_tone_stop(void)
{
    buzzer_pwm_stop();
}

int buzzer_pcm_start_u8(const uint8_t *pcm_u8, uint32_t len, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume)
{
    (void)pcm_u8;
    (void)len;
    (void)sample_rate_hz;
    (void)loop;
    (void)volume;

    buzzer_pcm_stop();
    return -1;
}

int buzzer_pcm_start_stream_u8(uint32_t sample_rate_hz, uint8_t volume)
{
    (void)sample_rate_hz;
    (void)volume;

    buzzer_pcm_stop();
    return -1;
}

uint32_t buzzer_pcm_stream_write_u8(const uint8_t *pcm_u8, uint32_t len)
{
    (void)pcm_u8;
    (void)len;
    return 0u;
}

uint32_t buzzer_pcm_stream_get_used(void)
{
    return 0u;
}

uint32_t buzzer_pcm_stream_get_free(void)
{
    return 0u;
}

uint8_t buzzer_pcm_is_stream_mode(void)
{
    return 0u;
}

void buzzer_pcm_stop(void)
{
    buzzer_pwm_stop();
}

uint8_t buzzer_pcm_is_running(void)
{
    return 0u;
}

#endif
