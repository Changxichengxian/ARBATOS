/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <stm32_ll_tim.h>

#include "ArbatosDt.h"
#include "BspBuzzer.h"
#include "BspBuzzerC.h"

#define BUZZER_TIMER_NODE DT_NODELABEL(timers4)
#define BUZZER_DMA_CHANNEL DT_DMAS_CELL_BY_NAME(BUZZER_TIMER_NODE, buzzer, channel)
#define BUZZER_HALF_SIZE 512u
/* 12 kHz 单声道8位音频可缓冲约341 ms，给C板完整服务保留内存。 */
#define BUZZER_RING_SIZE 4096u
#define BUZZER_MAX_RATE 48000u
#if DT_HAS_CHOSEN(zephyr_dtcm)
#define BUZZER_CPU_BSS __dtcm_bss_section
#else
#define BUZZER_CPU_BSS
#endif

static const struct pwm_dt_spec BuzzerPwm =
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0);
static const struct device *const BuzzerDma =
    DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(BUZZER_TIMER_NODE, buzzer));
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0),
                         DT_NODELABEL(pwm4)), "C PCM requires TIM4");
BUILD_ASSERT(DT_PHA_BY_IDX(DT_PATH(arbatos_platform), buzzer_pwms, 0, channel) == 3,
             "C PCM requires TIM4 channel 3");
BUILD_ASSERT(BUZZER_DMA_CHANNEL == 7 &&
             DT_DMAS_CELL_BY_NAME(BUZZER_TIMER_NODE, buzzer, slot) == 2 &&
             DT_SAME_NODE(DT_DMAS_CTLR_BY_NAME(BUZZER_TIMER_NODE, buzzer), DT_NODELABEL(dma1)),
             "C PCM requires DMA1 Stream7 Channel2");

K_MUTEX_DEFINE(BuzzerControl);
static struct k_spinlock BuzzerLock;
static struct k_work BuzzerEndWork;
/* DMA 缓冲必须在普通 SRAM；F407 的 CCM 不能被 DMA 访问。 */
static uint16_t BuzzerBuffer[2][BUZZER_HALF_SIZE] __aligned(4);
static uint16_t BuzzerLut[256] BUZZER_CPU_BSS;
static uint8_t BuzzerRing[BUZZER_RING_SIZE] BUZZER_CPU_BSS;
static uint32_t BuzzerHead, BuzzerTail;
static const uint8_t *BuzzerData;
static uint32_t BuzzerLength, BuzzerPos, BuzzerPhase;
static uint32_t BuzzerRate, BuzzerCarrier, BuzzerGeneration;
static uint32_t BuzzerTonePhase, BuzzerToneStep;
static uint32_t BuzzerCarrierMin = 48000u;
static uint16_t BuzzerGain = 256u;
static uint8_t BuzzerEnabled, BuzzerMode, BuzzerArmed, BuzzerDmaConfigured;
static uint8_t BuzzerLoop, BuzzerCurrent, BuzzerNeedSample;
static uint8_t BuzzerDraining, BuzzerSourceDone, BuzzerEndSilence;
static uint8_t BuzzerHalfFinal[2];
static uint8_t BuzzerExpectedHalf;

static uint32_t BuzzerUsed(void)
{
    return BuzzerHead - BuzzerTail;
}

/* 仅寄存器写入；中断不调用带硬件等待的 dma_stop/pwm_set。 */
static void BuzzerMute(void)
{
    LL_TIM_DisableDMAReq_CC3(TIM4);
    LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_DisableCounter(TIM4);
    LL_TIM_OC_SetCompareCH3(TIM4, 0u);
    BuzzerArmed = 0u;
}

static void BuzzerStopLocked(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerGeneration++;
    BuzzerMute();
    BuzzerMode = 0u;
    BuzzerData = NULL;
    BuzzerHead = BuzzerTail = 0u;
    k_spin_unlock(&BuzzerLock, key);
    if (BuzzerDmaConfigured) {
        (void)dma_stop(BuzzerDma, BUZZER_DMA_CHANNEL);
        BuzzerDmaConfigured = 0u;
    }
    /* 工作函数只尝试取得控制锁，故这里同步等待不会形成互相等待。 */
    struct k_work_sync sync;
    (void)k_work_cancel_sync(&BuzzerEndWork, &sync);
}

static void BuzzerEndWorkHandler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (k_mutex_lock(&BuzzerControl, K_NO_WAIT) != 0) {
        return;
    }
    if (!BuzzerArmed && BuzzerMode == 0u && BuzzerDmaConfigured) {
        (void)dma_stop(BuzzerDma, BUZZER_DMA_CHANNEL);
        BuzzerDmaConfigured = 0u;
    }
    k_mutex_unlock(&BuzzerControl);
}

static uint16_t BuzzerNextDuty(void)
{
    if (BuzzerMode == 3u) {
        uint16_t duty = BuzzerLut[(BuzzerTonePhase & 0x80000000u) ? 255u : 0u];
        BuzzerTonePhase += BuzzerToneStep;
        return duty;
    }
    if (BuzzerNeedSample && !BuzzerSourceDone) {
        BuzzerCurrent = 128u;
        if (BuzzerMode == 2u) {
            if (BuzzerHead != BuzzerTail) {
                BuzzerCurrent = BuzzerRing[BuzzerTail++ & (BUZZER_RING_SIZE - 1u)];
            } else if (BuzzerDraining) {
                BuzzerSourceDone = 1u;
            }
        } else if (BuzzerPos < BuzzerLength) {
            BuzzerCurrent = BuzzerData[BuzzerPos++];
            if (BuzzerLoop && BuzzerPos == BuzzerLength) {
                BuzzerPos = 0u;
            }
        } else {
            BuzzerSourceDone = 1u;
        }
        BuzzerNeedSample = 0u;
    }
    uint16_t duty = BuzzerLut[BuzzerSourceDone ? 128u : BuzzerCurrent];
    if (BuzzerSourceDone && BuzzerEndSilence < 2u) {
        BuzzerEndSilence++;
    }
    BuzzerPhase += BuzzerRate;
    if (BuzzerPhase >= BuzzerCarrier) {
        BuzzerPhase -= BuzzerCarrier;
        BuzzerNeedSample = 1u;
    }
    return duty;
}

static void BuzzerFillHalf(uint8_t half)
{
    for (uint32_t i = 0u; i < BUZZER_HALF_SIZE; i++) {
        BuzzerBuffer[half][i] = BuzzerNextDuty();
    }
    /* 留出 CCR 预加载流水的两个静音周期，避免最后一个样本被提前关掉。 */
    BuzzerHalfFinal[half] = BuzzerEndSilence >= 2u;
}

static void BuzzerDmaCallback(const struct device *dev, void *userData, uint32_t channel, int status)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(channel);
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    if (!BuzzerArmed || (uint32_t)(uintptr_t)userData != BuzzerGeneration) {
        k_spin_unlock(&BuzzerLock, key);
        return;
    }
    uint8_t half = status == DMA_STATUS_BLOCK ? 0u : 1u;
    uint32_t remaining = DMA1_Stream7->NDTR;
    /* 延迟超过半缓冲时宁可停播，不能改写 DMA 正在读取的半区。 */
    uint8_t writable = half == 0u ? (remaining >= 128u && remaining <= BUZZER_HALF_SIZE) :
        (remaining >= BUZZER_HALF_SIZE + 128u && remaining <= 2u * BUZZER_HALF_SIZE);
    if (status < 0 || (status != DMA_STATUS_BLOCK && status != DMA_STATUS_COMPLETE) ||
        half != BuzzerExpectedHalf || !writable || BuzzerHalfFinal[half]) {
        BuzzerMute();
        BuzzerMode = 0u;
        BuzzerData = NULL;
        (void)k_work_submit(&BuzzerEndWork);
    } else {
        BuzzerFillHalf(half);
        BuzzerExpectedHalf ^= 1u;
    }
    k_spin_unlock(&BuzzerLock, key);
}

static int BuzzerArm(void)
{
    BuzzerFillHalf(0u);
    BuzzerFillHalf(1u);
    BuzzerExpectedHalf = 0u;
    struct dma_block_config block = {
        .source_address = (uint32_t)(uintptr_t)BuzzerBuffer,
        .dest_address = (uint32_t)(uintptr_t)&TIM4->CCR3,
        .block_size = sizeof(BuzzerBuffer),
        .source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
        .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        .source_reload_en = 1u,
        .dest_reload_en = 1u,
    };
    struct dma_config config = {
        .dma_slot = DT_DMAS_CELL_BY_NAME(BUZZER_TIMER_NODE, buzzer, slot),
        .channel_direction = MEMORY_TO_PERIPHERAL,
        .source_data_size = 2u, .dest_data_size = 2u,
        .source_burst_length = 1u, .dest_burst_length = 1u,
        .channel_priority = 2u, .block_count = 1u, .head_block = &block,
        .dma_callback = BuzzerDmaCallback, .user_data = (void *)(uintptr_t)BuzzerGeneration,
    };
    int ret = dma_config(BuzzerDma, BUZZER_DMA_CHANNEL, &config);
    if (ret != 0) {
        BuzzerMode = 0u;
        return ret;
    }
    BuzzerDmaConfigured = 1u;
    ret = dma_start(BuzzerDma, BUZZER_DMA_CHANNEL);
    if (ret != 0) {
        BuzzerStopLocked();
        return ret;
    }
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    LL_TIM_OC_SetCompareCH3(TIM4, BuzzerBuffer[0][0]);
    LL_TIM_GenerateEvent_UPDATE(TIM4);
    LL_TIM_SetCounter(TIM4, 0u);
    LL_TIM_ClearFlag_CC3(TIM4);
    BuzzerArmed = 1u;
    LL_TIM_EnableDMAReq_CC3(TIM4);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableCounter(TIM4);
    k_spin_unlock(&BuzzerLock, key);
    return 0;
}

static int BuzzerPrepare(uint32_t rate, uint8_t volume, uint8_t mode)
{
    BuzzerStopLocked();
    if (!BuzzerEnabled || rate == 0u || rate > BUZZER_MAX_RATE || !device_is_ready(BuzzerDma)) {
        return -EINVAL;
    }
    uint32_t carrier = MAX(BuzzerCarrierMin, rate);
    uint64_t clock;
    int ret = pwm_get_cycles_per_sec(BuzzerPwm.dev, BuzzerPwm.channel, &clock);
    if (ret != 0) return ret;
    uint32_t cycles = (uint32_t)((clock + carrier / 2u) / carrier);
    if (cycles < 4u || cycles > 65535u) return -ERANGE;
    ret = pwm_set_cycles(BuzzerPwm.dev, BuzzerPwm.channel, cycles, cycles / 2u, BuzzerPwm.flags);
    if (ret != 0) return ret;
    BuzzerMute();
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
    BuzzerCarrier = (uint32_t)(clock / cycles);
    BuzzerRate = MIN(rate, BuzzerCarrier);
    for (uint32_t i = 0u; i < 256u; i++) {
        int32_t centered = ((int32_t)i - 128) * volume / 255;
        centered = centered * (int32_t)(mode == 2u ? BuzzerGain : 256u) / 256;
        uint32_t scaled = (uint32_t)CLAMP(centered + 128, 0, 255);
        BuzzerLut[i] = (uint16_t)CLAMP((scaled * cycles) / 256u, 1u, cycles - 2u);
    }
    BuzzerMode = mode;
    BuzzerPos = BuzzerPhase = 0u;
    BuzzerCurrent = 128u;
    BuzzerNeedSample = 1u;
    BuzzerSourceDone = BuzzerEndSilence = BuzzerDraining = 0u;
    BuzzerHalfFinal[0] = BuzzerHalfFinal[1] = 0u;
    return 0;
}

int BuzzerPcmStartU8(const uint8_t *data, uint32_t length, uint32_t rate, uint8_t loop, uint8_t volume)
{
    if (data == NULL || length == 0u) return -EINVAL;
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    int ret = BuzzerPrepare(rate, volume, 1u);
    if (ret == 0) {
        BuzzerData = data; BuzzerLength = length; BuzzerLoop = loop != 0u;
        ret = BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return ret;
}

int BuzzerPcmStartStreamU8(uint32_t rate, uint8_t volume)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    int ret = BuzzerPrepare(rate, volume, 2u);
    k_mutex_unlock(&BuzzerControl);
    return ret;
}

uint32_t BuzzerPcmStreamWriteU8(const uint8_t *data, uint32_t length)
{
    if (data == NULL) return 0u;
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    uint32_t written = 0u;
    while (written < length) {
        k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
        uint32_t count = BuzzerMode == 2u && !BuzzerDraining ?
            MIN(MIN(length - written, BUZZER_RING_SIZE - BuzzerUsed()), 64u) : 0u;
        for (uint32_t i = 0u; i < count; i++) {
            BuzzerRing[BuzzerHead++ & (BUZZER_RING_SIZE - 1u)] = data[written++];
        }
        k_spin_unlock(&BuzzerLock, key);
        if (count == 0u) break;
    }
    if (written != 0u && !BuzzerArmed) {
        (void)BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return written;
}

void BuzzerPcmStreamFinish(void)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    BuzzerDraining = 1u;
    if (BuzzerMode == 2u && !BuzzerArmed) BuzzerMode = 0u;
    k_spin_unlock(&BuzzerLock, key);
    k_mutex_unlock(&BuzzerControl);
}

uint32_t BuzzerPcmStreamGetUsed(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    uint32_t used = BuzzerUsed();
    k_spin_unlock(&BuzzerLock, key);
    return used;
}
uint32_t BuzzerPcmStreamGetFree(void) { return BUZZER_RING_SIZE - BuzzerPcmStreamGetUsed(); }
uint8_t BuzzerPcmIsStreamMode(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    uint8_t stream = BuzzerMode == 2u;
    k_spin_unlock(&BuzzerLock, key);
    return stream;
}
uint8_t BuzzerPcmIsRunning(void)
{
    k_spinlock_key_t key = k_spin_lock(&BuzzerLock);
    uint8_t running = BuzzerMode != 0u;
    k_spin_unlock(&BuzzerLock, key);
    return running;
}

void BuzzerPcmStop(void)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerStopLocked();
    k_mutex_unlock(&BuzzerControl);
}
void BuzzerSetEnable(uint8_t enable)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerEnabled = enable != 0u;
    if (!BuzzerEnabled) BuzzerStopLocked();
    k_mutex_unlock(&BuzzerControl);
}
void BuzzerPcmSetCarrierMinHz(uint32_t frequency)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerCarrierMin = frequency == 0u ? 48000u : CLAMP(frequency, 24000u, 100000u);
    k_mutex_unlock(&BuzzerControl);
}
void BuzzerPcmSetStreamGainQ8(uint16_t gain)
{
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    BuzzerGain = gain == 0u ? 256u : MIN(gain, 4096u);
    k_mutex_unlock(&BuzzerControl);
}

int BuzzerToneStartHz(uint32_t frequency, uint8_t volume)
{
    if (frequency == 0u || frequency > 24000u) return -EINVAL;
    k_mutex_lock(&BuzzerControl, K_FOREVER);
    /* TIM4 固定预分频供 PCM 使用；低音不能直接要求超出16位ARR的PWM周期。 */
    int ret = BuzzerPrepare(BUZZER_MAX_RATE, volume, 3u);
    if (ret == 0) {
        BuzzerTonePhase = 0u;
        BuzzerToneStep = (uint32_t)(((uint64_t)frequency << 32) / BuzzerCarrier);
        ret = BuzzerArm();
    }
    k_mutex_unlock(&BuzzerControl);
    return ret;
}
int BuzzerToneStartLegacy(uint16_t psc, uint16_t pwm)
{
    const uint32_t period = 20999u;
    uint32_t duty = MIN(pwm, period);
    uint32_t diff = duty * 2u > period ? duty * 2u - period : period - duty * 2u;
    uint8_t volume = (uint8_t)(((period - diff) * 255u) / period);
    if (psc == 0xffffu || pwm == 0u) { BuzzerToneStop(); return 0; }
    uint32_t frequency = 84000000u / (((uint32_t)psc + 1u) * (period + 1u));
    return BuzzerToneStartHz(MAX(frequency, 1u), volume);
}
void BuzzerToneStop(void) { BuzzerPcmStop(); }
uint16_t BuzzerLegacyPwmHalf(void) { return 10500u; }

int BspBuzzerPlatformInit(void)
{
    k_work_init(&BuzzerEndWork, BuzzerEndWorkHandler);
    if (!pwm_is_ready_dt(&BuzzerPwm) || !device_is_ready(BuzzerDma)) return -ENODEV;
    BuzzerPcmStop();
    return 0;
}
