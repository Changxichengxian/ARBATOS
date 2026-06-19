/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ElrsTask.h"

#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "DetectTask.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "ManualInput.h"
#include "SdLog.h"

extern void Error_Handler(void);

// ===== ELRS(CRSF) RX on aux link =====
#define CRSF_FRAME_LEN_MAX 64u // length byte: [type + payload + crc]
#define CRSF_FRAME_SIZE_MAX (CRSF_FRAME_LEN_MAX + 2u) // [addr + len] + len bytes
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_RC_FRAME_LEN 24u // length byte for RC_CHANNELS_PACKED
#define CRSF_RC_PAYLOAD_LEN 22u
#define CRSF_CHANNEL_VALUE_MIN 172u
#define CRSF_CHANNEL_VALUE_MID 992u
#define CRSF_CHANNEL_VALUE_MAX 1811u

#define ELRS_LINK_DMA_RX_BUF_SIZE 4096u
typedef char _check_elrs_link_dma_buf_size_u16[(ELRS_LINK_DMA_RX_BUF_SIZE <= 65535u) ? 1 : -1];

#define ELRS_LINK_NOTIFY_RX (1u << 0)
#define ELRS_LINK_NOTIFY_RESTART (1u << 1)

static uint8_t ElrsCrsfBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t ElrsCrsfPos = 0u;
static uint8_t ElrsCrsfExpected = 0u;
static volatile uint32_t ElrsLinkLastFrameTickMs = 0u;
static volatile uint32_t ElrsLinkFrameCnt = 0u;

static uint8_t ElrsLinkDmaRxBuf[ELRS_LINK_DMA_RX_BUF_SIZE];
static volatile uint16_t ElrsLinkDmaPos = 0u;
static uint16_t ElrsLinkDmaLastPos = 0u;
static volatile uint32_t ElrsLinkDmaWrapCnt = 0u;
static uint32_t ElrsLinkDmaLastWrapCnt = 0u;
static volatile uint8_t ElrsLinkDmaActive = 0u;
static volatile uint8_t ElrsLinkDmaRestartReq = 0u;

static TaskHandle_t ElrsLinkTaskHandle = NULL;
static volatile uint32_t ElrsLinkNotifyPending = 0u;

static void ElrsLinkRxStartInternal(void);
static void ElrsLinkReset(void);
static void ElrsLinkDmaProcessTo(uint16_t pos);
static void ElrsLinkNotifyFromIsr(uint32_t notify_bits);
static void ElrsLinkOnByte(uint8_t b);
static void ElrsLinkHandleFrame(const uint8_t *frame, uint8_t total_len);
static void ElrsLinkDecodeRcChannels(const uint8_t *payload, uint8_t payload_len);
static uint8_t ElrsLinkCrc8DvbS2(const uint8_t *data, uint8_t len);
static int16_t ElrsLinkMapAxis(uint16_t v);
static uint8_t ElrsLinkMapSwitch(uint16_t v);

void ElrsLinkTask(void const *argument)
{
    (void)argument;

    ElrsLinkTaskHandle = xTaskGetCurrentTaskHandle();

    // Drain any notifications posted before the task handle is ready.
    taskENTER_CRITICAL();
    const uint32_t pending = ElrsLinkNotifyPending;
    ElrsLinkNotifyPending = 0u;
    taskEXIT_CRITICAL();
    if (pending != 0u)
    {
        (void)xTaskNotify(ElrsLinkTaskHandle, pending, eSetBits);
    }

    while (1)
    {
        uint32_t bits = 0u;
        WatchTaskWait(WATCH_TASK_ELRS);
        (void)xTaskNotifyWait(0u, 0xFFFFFFFFu, &bits, portMAX_DELAY);
        WatchTaskBeat(WATCH_TASK_ELRS);

        if (BspAuxLinkGetBaudrate() != ELRS_LINK_BAUD)
        {
            continue;
        }

        if (ElrsLinkDmaRestartReq || (bits & ELRS_LINK_NOTIFY_RESTART) != 0u)
        {
            ElrsLinkDmaRestartReq = 0u;
            ElrsLinkRxStartInternal();
            continue;
        }

        if (!ElrsLinkDmaActive)
        {
            continue;
        }

        if ((bits & ELRS_LINK_NOTIFY_RX) != 0u)
        {
            const uint32_t wrap = ElrsLinkDmaWrapCnt;
            const uint16_t pos = ElrsLinkDmaPos;
            const uint32_t wraps = wrap - ElrsLinkDmaLastWrapCnt;
            if (wraps > 1u)
            {
                // Too late: DMA has wrapped multiple times before we could process.
                WatchTaskTimeout(WATCH_TASK_ELRS);
                ElrsLinkReset();
                ElrsLinkDmaLastWrapCnt = wrap;
                ElrsLinkDmaLastPos = pos;
                continue;
            }
            if (wraps == 1u)
            {
                // Process tail of the previous cycle up to end-of-buffer.
                ElrsLinkDmaProcessTo((uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE);
                ElrsLinkDmaLastWrapCnt = wrap;
            }

            ElrsLinkDmaProcessTo(pos);
        }
    }
}

void ElrsLinkStop(void)
{
    ElrsLinkDmaActive = 0u;
    ElrsLinkDmaRestartReq = 0u;
    ElrsLinkReset();
    ElrsLinkDmaPos = 0u;
    ElrsLinkDmaLastPos = 0u;
    ElrsLinkDmaWrapCnt = 0u;
    ElrsLinkDmaLastWrapCnt = 0u;
}

void ElrsLinkRxStart(void)
{
    ElrsLinkRxStartInternal();
}

void ElrsLinkOnRxEvent(uint16_t Size, BspAuxLinkRxEvent evt)
{
    if (BspAuxLinkGetBaudrate() != ELRS_LINK_BAUD || !ElrsLinkDmaActive)
    {
        return;
    }

    // In DMA circular mode, HAL reports an extra IDLE event with Size==RxXferSize
    // after a Transfer Complete; ignore it to avoid duplicate processing.
    if (evt == BSP_AUX_LINK_RXEVENT_IDLE && Size >= (uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE)
    {
        return;
    }

    if (evt == BSP_AUX_LINK_RXEVENT_TC)
    {
        ElrsLinkDmaWrapCnt++;
    }

    // Size is the DMA write index within the current cycle (0..RxXferSize).
    // Use 0 to represent "wrapped to start".
    ElrsLinkDmaPos = (Size >= (uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE) ? 0u : Size;
    ElrsLinkNotifyFromIsr(ELRS_LINK_NOTIFY_RX);
}

void ElrsLinkOnItByte(uint8_t b)
{
    ElrsLinkOnByte(b);
}

bool_t ElrsLinkOnUartError(void)
{
    WatchTaskError(WATCH_TASK_ELRS);
    ElrsLinkReset();
    if (!ElrsLinkDmaActive)
    {
        return 0;
    }

    ElrsLinkDmaRestartReq = 1u;
    ElrsLinkNotifyFromIsr(ELRS_LINK_NOTIFY_RESTART);
    return 1;
}

static void ElrsLinkRxStartInternal(void)
{
    ElrsLinkStop();

    if (!BspAuxLinkRxHasDma())
    {
        // Fallback (shouldn't happen): IT per byte.
        (void)BspAuxLinkRxItStart();
        return;
    }

    if (BspAuxLinkRxToIdleDmaStart(ElrsLinkDmaRxBuf, (uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE) != 0)
    {
        WatchTaskError(WATCH_TASK_ELRS);
        Error_Handler();
    }
    ElrsLinkDmaActive = 1u;
}

static void ElrsLinkReset(void)
{
    ElrsCrsfPos = 0u;
    ElrsCrsfExpected = 0u;
}

static void ElrsLinkDmaProcessTo(uint16_t pos)
{
    const uint16_t size = (uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE;
    uint16_t last = ElrsLinkDmaLastPos;

    if (pos > size)
    {
        pos = (uint16_t)(pos % size);
    }
    if (last >= size)
    {
        last = 0u;
    }

    if (pos == last)
    {
        return;
    }

    if (pos > last)
    {
        for (uint16_t i = last; i < pos; i++)
        {
            ElrsLinkOnByte(ElrsLinkDmaRxBuf[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            ElrsLinkOnByte(ElrsLinkDmaRxBuf[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            ElrsLinkOnByte(ElrsLinkDmaRxBuf[i]);
        }
    }

    ElrsLinkDmaLastPos = (pos == size) ? 0u : pos;
}

static void ElrsLinkNotifyFromIsr(uint32_t notify_bits)
{
    if (ElrsLinkTaskHandle == NULL)
    {
        ElrsLinkNotifyPending |= notify_bits;
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    (void)xTaskNotifyFromISR(ElrsLinkTaskHandle, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint8_t ElrsLinkCrc8DvbS2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0xD5u) : (uint8_t)(crc << 1u);
        }
    }
    return crc;
}

static int16_t ElrsLinkMapAxis(uint16_t v)
{
    if (v < CRSF_CHANNEL_VALUE_MIN)
    {
        v = CRSF_CHANNEL_VALUE_MIN;
    }
    if (v > CRSF_CHANNEL_VALUE_MAX)
    {
        v = CRSF_CHANNEL_VALUE_MAX;
    }

    if (v >= CRSF_CHANNEL_VALUE_MID)
    {
        const uint16_t delta = (uint16_t)(v - CRSF_CHANNEL_VALUE_MID);
        const uint16_t denom = (uint16_t)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
        return (int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) + (denom / 2u)) / denom);
    }

    const uint16_t delta = (uint16_t)(CRSF_CHANNEL_VALUE_MID - v);
    const uint16_t denom = (uint16_t)(CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN);
    return (int16_t)(-((int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) + (denom / 2u)) / denom)));
}

static uint8_t ElrsLinkMapSwitch(uint16_t v)
{
    if (v < CRSF_CHANNEL_VALUE_MIN)
    {
        v = CRSF_CHANNEL_VALUE_MIN;
    }
    if (v > CRSF_CHANNEL_VALUE_MAX)
    {
        v = CRSF_CHANNEL_VALUE_MAX;
    }

    const uint16_t t_down = (uint16_t)((CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MID) / 2u);
    const uint16_t t_up = (uint16_t)((CRSF_CHANNEL_VALUE_MID + CRSF_CHANNEL_VALUE_MAX) / 2u);

    if (v <= t_down)
    {
        return RC_SW_DOWN;
    }
    if (v >= t_up)
    {
        return RC_SW_UP;
    }
    return RC_SW_MID;
}

static void ElrsLinkDecodeRcChannels(const uint8_t *payload, uint8_t payload_len)
{
    if (payload == NULL || payload_len < CRSF_RC_PAYLOAD_LEN)
    {
        return;
    }

    uint16_t ch[16] = {0};
    uint32_t bitbuf = 0u;
    uint8_t bitcnt = 0u;
    uint8_t idx = 0u;

    for (uint8_t i = 0u; i < CRSF_RC_PAYLOAD_LEN; i++)
    {
        bitbuf |= ((uint32_t)payload[i]) << bitcnt;
        bitcnt = (uint8_t)(bitcnt + 8u);
        while (bitcnt >= 11u && idx < 16u)
        {
            ch[idx++] = (uint16_t)(bitbuf & 0x07FFu);
            bitbuf >>= 11u;
            bitcnt = (uint8_t)(bitcnt - 11u);
        }
    }

    if (idx < 4u)
    {
        return;
    }

    const input_config_t *cfg = &g_config.input;
    int16_t ch_raw[16] = {0};
    for (uint8_t i = 0u; i < 16u; i++)
    {
        ch_raw[i] = (int16_t)ch[i];
    }

    ManualInputState rc = {0};
    for (uint8_t i = 0u; i < 5u; i++)
    {
        uint8_t idx_ch = cfg->ElrsChMap[i];
        if (idx_ch >= 16u)
        {
            idx_ch = (i < 4u) ? i : 6u;
        }
        rc.rc.ch[i] = ElrsLinkMapAxis(ch[idx_ch]);
    }

    for (uint8_t i = 0u; i < 2u; i++)
    {
        uint8_t idx_sw = cfg->ElrsSwMap[i];
        if (idx_sw >= 16u)
        {
            idx_sw = (uint8_t)(4u + i);
        }
        rc.rc.s[i] = (char)ElrsLinkMapSwitch(ch[idx_sw]);
    }

    remote_control_log_raw_source(MANUAL_INPUT_SRC_ELRS,
                                  SDLOG_MANUAL_INPUT_PROTO_CRSF,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  16u,
                                  ch_raw,
                                  NULL,
                                  &rc);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &rc);

    sdlog_rc_crsf_t pkt = {0};
    for (uint8_t i = 0u; i < 16u; i++)
    {
        pkt.ch_raw[i] = ch[i];
    }
    const uint32_t rc_len = (uint32_t)sizeof(ManualInputState);
    const uint32_t copy_len = (rc_len < (uint32_t)sizeof(pkt.rc_ctrl)) ? rc_len : (uint32_t)sizeof(pkt.rc_ctrl);
    const uint8_t *rc_bytes = (const uint8_t *)&rc;
    for (uint32_t i = 0u; i < copy_len; i++)
    {
        pkt.rc_ctrl[i] = rc_bytes[i];
    }
    SdLogWrite(SDLOG_TAG_RC_CRSF, &pkt, (uint16_t)sizeof(pkt));

    ElrsLinkLastFrameTickMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    ElrsLinkFrameCnt++;
}

static void ElrsLinkHandleFrame(const uint8_t *frame, uint8_t total_len)
{
    if (frame == NULL || total_len < 4u)
    {
        return;
    }

    const uint8_t len = frame[1];
    if (len < 2u || (uint8_t)(len + 2u) != total_len)
    {
        return;
    }

    const uint8_t crc = frame[total_len - 1u];
    const uint8_t calc = ElrsLinkCrc8DvbS2(&frame[2], (uint8_t)(len - 1u));
    if (calc != crc)
    {
        return;
    }

    const uint8_t type = frame[2];
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && len == CRSF_RC_FRAME_LEN)
    {
        ElrsLinkDecodeRcChannels(&frame[3], CRSF_RC_PAYLOAD_LEN);
    }
}

static void ElrsLinkOnByte(uint8_t b)
{
    if (ElrsCrsfPos == 0u)
    {
        ElrsCrsfBuf[0] = b;
        ElrsCrsfPos = 1u;
        return;
    }

    if (ElrsCrsfPos == 1u)
    {
        if (b < 2u || b > CRSF_FRAME_LEN_MAX)
        {
            ElrsLinkReset();
            return;
        }

        ElrsCrsfBuf[1] = b;
        ElrsCrsfExpected = (uint8_t)(b + 2u);
        ElrsCrsfPos = 2u;
        return;
    }

    if (ElrsCrsfExpected < 4u || ElrsCrsfExpected > CRSF_FRAME_SIZE_MAX)
    {
        ElrsLinkReset();
        return;
    }

    ElrsCrsfBuf[ElrsCrsfPos++] = b;
    if (ElrsCrsfPos >= ElrsCrsfExpected)
    {
        ElrsLinkHandleFrame(ElrsCrsfBuf, ElrsCrsfExpected);
        ElrsLinkReset();
    }
}
