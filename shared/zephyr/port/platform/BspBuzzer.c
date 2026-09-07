/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "ArbatosDt.h"
#include "BspBuzzer.h"

#define ARB_PLATFORM_NODE DT_PATH(arbatos_platform)
#define BUZZER_STREAM_CAPACITY 512u

#if DT_NODE_EXISTS(ARB_PLATFORM_NODE) && DT_NODE_HAS_PROP(ARB_PLATFORM_NODE, buzzer_pwms)
static const struct pwm_dt_spec BuzzerPwm =
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(ARB_PLATFORM_NODE, buzzer_pwms, 0);
#define BUZZER_HAS_PWM 1
#else
#define BUZZER_HAS_PWM 0
#endif

static struct k_timer BuzzerTimer;
static struct k_work BuzzerWork;
static struct k_spinlock BuzzerLock;
static uint8_t BuzzerEnabled;
static uint8_t BuzzerRunning;
static uint8_t BuzzerStreamMode;
static uint8_t BuzzerVolume;
static const uint8_t *BuzzerData;
static uint32_t BuzzerLength;
static uint32_t BuzzerPos;
static uint8_t BuzzerLoop;
static uint8_t BuzzerStream[BUZZER_STREAM_CAPACITY];
static uint16_t BuzzerHead;
static uint16_t BuzzerTail;

static int BuzzerApplySample(uint8_t sample)
{
#if BUZZER_HAS_PWM
    uint32_t pulse;
    uint32_t period;

    if (!BuzzerEnabled || !pwm_is_ready_dt(&BuzzerPwm)) {
        return -ENODEV;
    }
    period = BuzzerPwm.period;
    pulse = ((uint32_t)sample * BuzzerVolume * period) / (255u * 255u);
    return pwm_set_dt(&BuzzerPwm, period, pulse);
#else
    (void)sample;
    return -ENODEV;
#endif
}

static uint32_t BuzzerStreamUsedInternal(void)
{
    return (BuzzerHead >= BuzzerTail) ? (BuzzerHead - BuzzerTail) :
        (BUZZER_STREAM_CAPACITY - BuzzerTail + BuzzerHead);
}

static void BuzzerWorkHandler(struct k_work *work)
{
    uint8_t sample = 128u;
    k_spinlock_key_t key;

    ARG_UNUSED(work);
    key = k_spin_lock(&BuzzerLock);
    if (!BuzzerRunning) {
        k_spin_unlock(&BuzzerLock, key);
        return;
    }
    if (BuzzerStreamMode) {
        if (BuzzerHead != BuzzerTail) {
            sample = BuzzerStream[BuzzerTail++];
            if (BuzzerTail == BUZZER_STREAM_CAPACITY) BuzzerTail = 0u;
        }
    } else if (BuzzerData != NULL && BuzzerPos < BuzzerLength) {
        sample = BuzzerData[BuzzerPos++];
        if (BuzzerPos == BuzzerLength && BuzzerLoop) BuzzerPos = 0u;
    }
    k_spin_unlock(&BuzzerLock, key);
    (void)BuzzerApplySample(sample);
}

static void BuzzerTimerExpired(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    (void)k_work_submit(&BuzzerWork);
}

void BuzzerSetEnable(uint8_t enable)
{
    BuzzerEnabled = enable != 0u;
    if (!BuzzerEnabled) BuzzerPcmStop();
}

void BuzzerPcmSetCarrierMinHz(uint32_t carrier_min_hz) { ARG_UNUSED(carrier_min_hz); }
void BuzzerPcmSetStreamGainQ8(uint16_t gain_q8) { ARG_UNUSED(gain_q8); }

int BuzzerPcmStartU8(const uint8_t *pcm_u8, uint32_t len, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume)
{
    if (!BUZZER_HAS_PWM || pcm_u8 == NULL || len == 0u || sample_rate_hz == 0u || sample_rate_hz > 1000000u) return -EINVAL;
    BuzzerData = pcm_u8; BuzzerLength = len; BuzzerPos = 0u; BuzzerLoop = loop != 0u;
    BuzzerVolume = volume; BuzzerStreamMode = 0u; BuzzerRunning = 1u;
    k_timer_start(&BuzzerTimer, K_NO_WAIT, K_USEC(1000000u / sample_rate_hz));
    return 0;
}

int BuzzerPcmStartStreamU8(uint32_t sample_rate_hz, uint8_t volume)
{
    if (!BUZZER_HAS_PWM || sample_rate_hz == 0u || sample_rate_hz > 1000000u) return -EINVAL;
    BuzzerHead = 0u; BuzzerTail = 0u; BuzzerVolume = volume; BuzzerStreamMode = 1u; BuzzerRunning = 1u;
    k_timer_start(&BuzzerTimer, K_NO_WAIT, K_USEC(1000000u / sample_rate_hz));
    return 0;
}

uint32_t BuzzerPcmStreamWriteU8(const uint8_t *pcm_u8, uint32_t len)
{
    uint32_t count = 0u;
    k_spinlock_key_t key;
    if (!BuzzerStreamMode || pcm_u8 == NULL) return 0u;
    key = k_spin_lock(&BuzzerLock);
    while (count < len && BuzzerStreamUsedInternal() < BUZZER_STREAM_CAPACITY - 1u) {
        BuzzerStream[BuzzerHead++] = pcm_u8[count++];
        if (BuzzerHead == BUZZER_STREAM_CAPACITY) BuzzerHead = 0u;
    }
    k_spin_unlock(&BuzzerLock, key);
    return count;
}

uint32_t BuzzerPcmStreamGetUsed(void) { return BuzzerStreamUsedInternal(); }
uint32_t BuzzerPcmStreamGetFree(void) { return BUZZER_STREAM_CAPACITY - 1u - BuzzerStreamUsedInternal(); }
uint8_t BuzzerPcmIsStreamMode(void) { return BuzzerStreamMode; }

void BuzzerPcmStop(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerRunning = 0u;
    BuzzerStreamMode = 0u;
    BuzzerData = NULL;
    k_spin_unlock(&BuzzerLock, key);
    k_timer_stop(&BuzzerTimer);
#if BUZZER_HAS_PWM
    if (pwm_is_ready_dt(&BuzzerPwm)) (void)pwm_set_dt(&BuzzerPwm, BuzzerPwm.period, 0u);
#endif
}

uint8_t BuzzerPcmIsRunning(void) { return BuzzerRunning; }

int BuzzerToneStartHz(uint32_t freq_hz, uint8_t volume)
{
#if BUZZER_HAS_PWM
    uint32_t period;
    if (!BuzzerEnabled || !pwm_is_ready_dt(&BuzzerPwm) || freq_hz == 0u) return -EINVAL;
    period = 1000000000u / freq_hz;
    BuzzerVolume = volume; BuzzerRunning = 1u; BuzzerStreamMode = 0u; k_timer_stop(&BuzzerTimer);
    return pwm_set_dt(&BuzzerPwm, period, ((uint64_t)period * volume) / (2u * 255u));
#else
    ARG_UNUSED(freq_hz); ARG_UNUSED(volume); return -ENODEV;
#endif
}

int BuzzerToneStartLegacy(uint16_t psc, uint16_t pwm) { ARG_UNUSED(psc); return BuzzerToneStartHz(1000u, pwm > 255u ? 255u : (uint8_t)pwm); }
void BuzzerToneStop(void) { BuzzerPcmStop(); }
uint16_t BuzzerLegacyPwmHalf(void) { return 128u; }

int BspBuzzerPlatformInit(void)
{
    k_timer_init(&BuzzerTimer, BuzzerTimerExpired, NULL);
    k_work_init(&BuzzerWork, BuzzerWorkHandler);
#if BUZZER_HAS_PWM
    return pwm_is_ready_dt(&BuzzerPwm) ? 0 : -ENODEV;
#else
    return -ENODEV;
#endif
}
