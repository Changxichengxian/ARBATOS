/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * Copyright (c) 2024-2026 陈轩 <2811158416@qq.com>
  * @brief      BSP：BspBuzzer 头文
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "types.h"

// Board-specific configuration is provided by BspBuzzerCfg.h.

// Runtime configuration (owned by application; BSP must not depend on g_config).
extern void BuzzerSetEnable(uint8_t enable);
extern void BuzzerPcmSetCarrierMinHz(uint32_t carrier_min_hz);
extern void BuzzerPcmSetStreamGainQ8(uint16_t gain_q8);

// PWM duty update "PCM" mode (DMA on boards that support it; IRQ fallback otherwise).
// - Expects unsigned 8-bit PCM (0..255), 128 as mid-level.
// - sample_rate_hz is the audio sample rate; PWM carrier runs at ultrasonic rate and samples are held/upsampled internally.
// - When DMA is enabled, uses a circular buffer internally; IRQ mode updates one sample per PWM period.
// - When loop==0, playback continues with silence after data ends; call BuzzerPcmStop() to fully stop.
extern int BuzzerPcmStartU8(const uint8_t *pcm_u8, uint32_t len, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume);

// Streaming mode (caller feeds bytes continuously; do not call from ISR).
extern int BuzzerPcmStartStreamU8(uint32_t sample_rate_hz, uint8_t volume);
extern uint32_t BuzzerPcmStreamWriteU8(const uint8_t *pcm_u8, uint32_t len);
extern uint32_t BuzzerPcmStreamGetUsed(void);
extern uint32_t BuzzerPcmStreamGetFree(void);
extern uint8_t BuzzerPcmIsStreamMode(void);

extern void BuzzerPcmStop(void);
extern uint8_t BuzzerPcmIsRunning(void);

// Simple tone helpers (implemented via PCM internally; legacy "PSC/CCR" direct mode removed).
extern int BuzzerToneStartHz(uint32_t freq_hz, uint8_t volume);
extern int BuzzerToneStartLegacy(uint16_t psc, uint16_t pwm);
extern void BuzzerToneStop(void);

// Helper for legacy callers: 50% duty (based on the underlying timer period).
extern uint16_t BuzzerLegacyPwmHalf(void);

#endif
