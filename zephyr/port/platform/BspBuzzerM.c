/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <stm32_ll_tim.h>

#include "ArbatosDt.h"
#include "BspBuzzer.h"
#include "BspBuzzerM.h"

#define BUZZER_STREAM_CAPACITY 8192u
#define BUZZER_MAX_SAMPLE_HZ 48000u
#define BUZZER_PWM_NODE DT_NODELABEL(pwm12)

static const struct pwm_dt_spec BuzzerPwm =
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0);
static const struct device *const BuzzerClock = DEVICE_DT_GET(DT_ALIAS(buzzer_sample_timer));
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0),
                         BUZZER_PWM_NODE), "M PCM requires TIM12");
BUILD_ASSERT(DT_PHA_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0, channel) == 2,
             "M PCM requires TIM12 channel 2");

static K_MUTEX_DEFINE(BuzzerControl);
static struct k_spinlock BuzzerLock;
static uint8_t BuzzerEnabled;
static uint8_t BuzzerArmed;
static uint8_t BuzzerVolume;
static uint8_t BuzzerLoop;
static uint8_t BuzzerDraining;
static uint8_t BuzzerStream[BUZZER_STREAM_CAPACITY];
static uint32_t BuzzerHead;
static uint32_t BuzzerTail;
static const uint8_t *BuzzerData;
static uint32_t BuzzerLength;
static uint32_t BuzzerPosition;
static uint32_t BuzzerPeriod;
static uint32_t BuzzerCarrierMin = 40000u;
static uint16_t BuzzerGain = 256u;
static uint32_t BuzzerTonePhase;
static uint32_t BuzzerToneStep;

volatile BuzzerPcmDiagnostics BuzzerPcmDiag = {.magic = 0x4250434Du};

static uint32_t BuzzerUsed(void)
{
    return (BuzzerHead - BuzzerTail) & (BUZZER_STREAM_CAPACITY - 1u);
}

static void BuzzerSample(const struct device *dev, void *userData)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(userData);
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    if (!BuzzerEnabled || !BuzzerArmed || BuzzerPcmDiag.mode == 0u) {
        k_spin_unlock(&BuzzerLock, key);
        return;
    }

    uint8_t sample = 128u;
    if (BuzzerPcmDiag.mode == 2u) {
        if (BuzzerHead != BuzzerTail) {
            sample = BuzzerStream[BuzzerTail];
            BuzzerTail = (BuzzerTail + 1u) & (BUZZER_STREAM_CAPACITY - 1u);
        } else if (BuzzerDraining) {
            /* 文件结束后的正常收尾不计为数据断供；线程随后统一停定时器。 */
            BuzzerArmed = 0u;
            LL_TIM_OC_SetCompareCH2(TIM12, 0u);
            k_spin_unlock(&BuzzerLock, key);
            return;
        } else {
            BuzzerPcmDiag.underruns++;
        }
    } else if (BuzzerPcmDiag.mode == 3u) {
        BuzzerTonePhase += BuzzerToneStep;
        sample = (BuzzerTonePhase & 0x80000000u) ? 255u : 0u;
    } else if (BuzzerData != NULL && BuzzerPosition < BuzzerLength) {
        sample = BuzzerData[BuzzerPosition++];
        if (BuzzerLoop && BuzzerPosition == BuzzerLength) {
            BuzzerPosition = 0u;
        }
    }

    /* 音量围绕静音中点缩放；中断只更新 CCR2，不重复配置 PWM 驱动。 */
    int32_t value = 128 + (((int32_t)sample - 128) * BuzzerVolume * BuzzerGain) / (255 * 256);
    value = CLAMP(value, 0, 255);
    LL_TIM_OC_SetCompareCH2(TIM12, ((uint32_t)value * BuzzerPeriod) / 256u);
    BuzzerPcmDiag.samples++;
    BuzzerPcmDiag.queued = BuzzerUsed();
    k_spin_unlock(&BuzzerLock, key);
}

/* 调用方持有控制互斥量；关中断的小临界区避免停止后残留一次 PWM 更新。 */
static void BuzzerStopLocked(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerArmed = 0u;
    BuzzerDraining = 0u;
    BuzzerPcmDiag.mode = 0u;
    BuzzerData = NULL;
    BuzzerHead = 0u;
    BuzzerTail = 0u;
    BuzzerPcmDiag.queued = 0u;
    (void)counter_stop(BuzzerClock);
    LL_TIM_OC_SetCompareCH2(TIM12, 0u);
    k_spin_unlock(&BuzzerLock, key);
    (void)pwm_set_dt(&BuzzerPwm, BuzzerPwm.period, 0u);
    BuzzerPcmDiag.stops++;
}

static int BuzzerPrepare(uint32_t rate, uint8_t volume, uint32_t mode)
{
    BuzzerStopLocked();
    if (!BuzzerEnabled || rate == 0u || rate > BUZZER_MAX_SAMPLE_HZ) {
        return -EINVAL;
    }
    uint32_t frequency = counter_get_frequency(BuzzerClock);
    uint32_t ticks = (frequency + rate / 2u) / rate;
    if (ticks < 2u) {
        return -ERANGE;
    }
    struct counter_top_cfg top = {.ticks = ticks - 1u, .callback = BuzzerSample};
    int ret = counter_set_top_value(BuzzerClock, &top);
    uint32_t carrier = MAX(BuzzerCarrierMin, rate * 2u);
    if (ret == 0) {
        ret = pwm_set_dt(&BuzzerPwm, 1000000000u / carrier, 500000000u / carrier);
    }
    if (ret != 0) {
        BuzzerPcmDiag.lastError = ret;
        return ret;
    }
    BuzzerPeriod = LL_TIM_GetAutoReload(TIM12) + 1u;
    BuzzerVolume = volume;
    BuzzerPcmDiag.sampleHz = rate;
    BuzzerPcmDiag.actualHz = frequency / ticks;
    BuzzerPcmDiag.carrierHz = carrier;
    BuzzerPcmDiag.lastError = 0;
    BuzzerPcmDiag.mode = mode;
    BuzzerPcmDiag.starts++;
    return 0;
}

void BuzzerPcmStreamFinish(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerDraining = 1u;
    k_spin_unlock(&BuzzerLock, key);
}

static int BuzzerArm(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerArmed = 1u;
    int ret = counter_start(BuzzerClock);
    BuzzerPcmDiag.lastError = ret;
    if (ret != 0) {
        BuzzerArmed = 0u;
        BuzzerPcmDiag.mode = 0u;
        LL_TIM_OC_SetCompareCH2(TIM12, 0u);
    }
    k_spin_unlock(&BuzzerLock, key);
    return ret;
}

void BuzzerSetEnable(uint8_t enable)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerEnabled = enable != 0u;
    if (!BuzzerEnabled) {
        BuzzerStopLocked();
    }
    k_mutex_unlock(&BuzzerControl);
}

void BuzzerPcmSetCarrierMinHz(uint32_t carrierMinHz)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerCarrierMin = CLAMP(carrierMinHz, 40000u, 100000u);
    k_mutex_unlock(&BuzzerControl);
}

void BuzzerPcmSetStreamGainQ8(uint16_t gainQ8)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerGain = MIN(gainQ8, 1024u);
    k_spin_unlock(&BuzzerLock, key);
}

int BuzzerPcmStartU8(const uint8_t *data, uint32_t length, uint32_t rate, uint8_t loop, uint8_t volume)
{
    if (data == NULL || length == 0u) {
        return -EINVAL;
    }
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    int ret = BuzzerPrepare(rate, volume, 1u);
    if (ret == 0) {
        BuzzerData = data;
        BuzzerLength = length;
        BuzzerPosition = 0u;
        BuzzerLoop = loop != 0u;
        ret = BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return ret;
}

int BuzzerPcmStartStreamU8(uint32_t rate, uint8_t volume)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    int ret = BuzzerPrepare(rate, volume, 2u);
    /* 第一次写入音频后才启动采样，SD 打开和预读期间不会持续欠载。 */
    k_mutex_unlock(&BuzzerControl);
    return ret;
}

uint32_t BuzzerPcmStreamWriteU8(const uint8_t *data, uint32_t length)
{
    if (data == NULL) {
        return 0u;
    }
    uint32_t written = 0u;
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    while (written < length && BuzzerPcmDiag.mode == 2u) {
        k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
        uint32_t count = MIN(length - written, BUZZER_STREAM_CAPACITY - 1u - BuzzerUsed());
        count = MIN(count, 64u);
        for (uint32_t i = 0u; i < count; i++) {
            BuzzerStream[BuzzerHead] = data[written++];
            BuzzerHead = (BuzzerHead + 1u) & (BUZZER_STREAM_CAPACITY - 1u);
        }
        BuzzerPcmDiag.queued = BuzzerUsed();
        k_spin_unlock(&BuzzerLock, key);
        if (count == 0u) {
            break;
        }
    }
    if (written != 0u && !BuzzerArmed) {
        (void)BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return written;
}

uint32_t BuzzerPcmStreamGetUsed(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    uint32_t used = BuzzerUsed();
    k_spin_unlock(&BuzzerLock, key);
    return used;
}

uint32_t BuzzerPcmStreamGetFree(void) { return BUZZER_STREAM_CAPACITY - 1u - BuzzerPcmStreamGetUsed(); }
uint8_t BuzzerPcmIsStreamMode(void) { return BuzzerPcmDiag.mode == 2u; }
uint8_t BuzzerPcmIsRunning(void) { return BuzzerPcmDiag.mode != 0u; }

void BuzzerPcmStop(void)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerStopLocked();
    k_mutex_unlock(&BuzzerControl);
}

int BuzzerToneStartHz(uint32_t frequency, uint8_t volume)
{
    if (frequency == 0u || frequency > BUZZER_MAX_SAMPLE_HZ / 2u) {
        return -EINVAL;
    }
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    int ret = BuzzerPrepare(BUZZER_MAX_SAMPLE_HZ, volume, 3u);
    if (ret == 0) {
        BuzzerTonePhase = 0u;
        BuzzerToneStep = (uint32_t)(((uint64_t)frequency << 32) / BuzzerPcmDiag.actualHz);
        ret = BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return ret;
}

int BuzzerToneStartLegacy(uint16_t psc, uint16_t pwm)
{
    ARG_UNUSED(psc);
    return BuzzerToneStartHz(1000u, MIN(pwm, 255u));
}

void BuzzerToneStop(void) { BuzzerPcmStop(); }
uint16_t BuzzerLegacyPwmHalf(void) { return 128u; }

int BspBuzzerPlatformInit(void)
{
    if (!pwm_is_ready_dt(&BuzzerPwm) || !device_is_ready(BuzzerClock)) {
        return -ENODEV;
    }
    BuzzerPcmStop();
    return 0;
}
