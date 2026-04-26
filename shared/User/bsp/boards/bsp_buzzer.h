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
  * @brief      BSP：bsp_buzzer 头文
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"

// Board-specific configuration is provided by bsp_buzzer_cfg.h.

// Runtime configuration (owned by application; BSP must not depend on g_robot_config).
extern void buzzer_set_enable(uint8_t enable);
extern void buzzer_pcm_set_carrier_min_hz(uint32_t carrier_min_hz);
extern void buzzer_pcm_set_stream_gain_q8(uint16_t gain_q8);

// PWM duty update "PCM" mode (DMA on boards that support it; IRQ fallback otherwise).
// - Expects unsigned 8-bit PCM (0..255), 128 as mid-level.
// - sample_rate_hz is the audio sample rate; PWM carrier runs at ultrasonic rate and samples are held/upsampled internally.
// - When DMA is enabled, uses a circular buffer internally; IRQ mode updates one sample per PWM period.
// - When loop==0, playback continues with silence after data ends; call buzzer_pcm_stop() to fully stop.
extern int buzzer_pcm_start_u8(const uint8_t *pcm_u8, uint32_t len, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume);

// Streaming mode (caller feeds bytes continuously; do not call from ISR).
extern int buzzer_pcm_start_stream_u8(uint32_t sample_rate_hz, uint8_t volume);
extern uint32_t buzzer_pcm_stream_write_u8(const uint8_t *pcm_u8, uint32_t len);
extern uint32_t buzzer_pcm_stream_get_used(void);
extern uint32_t buzzer_pcm_stream_get_free(void);
extern uint8_t buzzer_pcm_is_stream_mode(void);

extern void buzzer_pcm_stop(void);
extern uint8_t buzzer_pcm_is_running(void);

// Simple tone helpers (implemented via PCM internally; legacy "PSC/CCR" direct mode removed).
extern int buzzer_tone_start_hz(uint32_t freq_hz, uint8_t volume);
extern int buzzer_tone_start_legacy(uint16_t psc, uint16_t pwm);
extern void buzzer_tone_stop(void);

// Helper for legacy callers: 50% duty (based on the underlying timer period).
extern uint16_t buzzer_legacy_pwm_half(void);

#endif
