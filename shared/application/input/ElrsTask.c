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
#include "cmsis_compiler.h"
#include "FreeRTOS.h"
#include "task.h"

#include "DetectTask.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "ManualInput.h"
#include "ManualInputCrsf.h"
#include "SdLog.h"

// ===== ELRS(CRSF) RX on aux link =====
#define CRSF_FRAME_LEN_MAX 64u // length byte: [type + payload + crc]
#define CRSF_FRAME_SIZE_MAX (CRSF_FRAME_LEN_MAX + 2u) // [addr + len] + len bytes
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_RC_FRAME_LEN 24u // length byte for RC_CHANNELS_PACKED
#define CRSF_RC_PAYLOAD_LEN 22u
#define ELRS_LINK_DMA_RX_BUF_SIZE 4096u
#define ELRS_LINK_IT_RX_RING_SIZE 4096u
typedef char _check_elrs_link_dma_buf_size_u16[(ELRS_LINK_DMA_RX_BUF_SIZE <= 65535u) ? 1 : -1];
typedef char _check_elrs_link_it_ring_power_of_two[
    ((ELRS_LINK_IT_RX_RING_SIZE & (ELRS_LINK_IT_RX_RING_SIZE - 1u)) == 0u) ? 1 : -1];

#define ELRS_LINK_NOTIFY_RX (1u << 0)
#define ELRS_LINK_NOTIFY_RESTART (1u << 1)

typedef union
{
    uint8_t dma[ELRS_LINK_DMA_RX_BUF_SIZE];
    uint8_t itRing[ELRS_LINK_IT_RX_RING_SIZE];
} ElrsLinkRxStorage;

static uint8_t ElrsCrsfBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t ElrsCrsfPos = 0u;
static uint8_t ElrsCrsfExpected = 0u;
static volatile uint32_t ElrsLinkLastFrameTickMs = 0u;
static volatile uint32_t ElrsLinkFrameCnt = 0u;

/* DMA 与逐字节中断接收由同一串口模式决定，不会同时运行。 */
static ElrsLinkRxStorage ElrsLinkRx;
static volatile uint16_t ElrsLinkItRxHead = 0u;
static volatile uint16_t ElrsLinkItRxTail = 0u;
static volatile uint8_t ElrsLinkItRxOverflow = 0u;
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
static void ElrsLinkItDrain(void);
static void ElrsLinkNotifyFromIsr(uint32_t notify_bits);
static void ElrsLinkOnByte(uint8_t b);
static void ElrsLinkHandleFrame(const uint8_t *frame, uint8_t total_len);
static void ElrsLinkDecodeRcChannels(const uint8_t *payload, uint8_t payload_len);
static uint8_t ElrsLinkCrc8DvbS2(const uint8_t *data, uint8_t len);

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

        if (ElrsLinkDmaActive != 0u &&
            (ElrsLinkDmaRestartReq || (bits & ELRS_LINK_NOTIFY_RESTART) != 0u))
        {
            WatchTaskError(WATCH_TASK_ELRS);
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
            if (BspAuxLinkRxHasDma() == 0u)
            {
                ElrsLinkItDrain();
                continue;
            }
            uint32_t wrap;
            uint16_t pos;
            taskENTER_CRITICAL();
            wrap = ElrsLinkDmaWrapCnt;
            pos = ElrsLinkDmaPos;
            taskEXIT_CRITICAL();
            const uint32_t wraps = wrap - ElrsLinkDmaLastWrapCnt;
            if (wraps > 1u ||
                (wraps == 1u && pos > ElrsLinkDmaLastPos))
            {
                // Too late: DMA has wrapped multiple times before we could process.
                WatchTaskTimeout(WATCH_TASK_ELRS);
                ElrsLinkReset();
                ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
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
    __DMB();
    ElrsLinkDmaRestartReq = 0u;
    ElrsLinkReset();
    ElrsLinkDmaPos = 0u;
    ElrsLinkDmaLastPos = 0u;
    ElrsLinkDmaWrapCnt = 0u;
    ElrsLinkDmaLastWrapCnt = 0u;
    BspAuxLinkRxItStop();
    ElrsLinkItRxHead = 0u;
    ElrsLinkItRxTail = 0u;
    ElrsLinkItRxOverflow = 0u;
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
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
    const uint16_t head = ElrsLinkItRxHead;
    const uint16_t next =
        (uint16_t)((head + 1u) & (uint16_t)(ELRS_LINK_IT_RX_RING_SIZE - 1u));
    const uint8_t was_empty = (head == ElrsLinkItRxTail) ? 1u : 0u;

    if (BspAuxLinkGetBaudrate() != ELRS_LINK_BAUD || ElrsLinkDmaActive == 0u)
    {
        return;
    }
    if (next == ElrsLinkItRxTail)
    {
        ElrsLinkItRxOverflow = 1u;
        return;
    }
    ElrsLinkRx.itRing[head] = b;
    __DMB();
    ElrsLinkItRxHead = next;
    if (was_empty != 0u)
    {
        ElrsLinkNotifyFromIsr(ELRS_LINK_NOTIFY_RX);
    }
}

bool_t ElrsLinkOnUartError(void)
{
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
        ElrsLinkItRxHead = 0u;
        ElrsLinkItRxTail = 0u;
        ElrsLinkItRxOverflow = 0u;
        ElrsLinkDmaActive = 1u;
        if (BspAuxLinkRxItStart() != 0)
        {
            ElrsLinkDmaActive = 0u;
            WatchTaskError(WATCH_TASK_ELRS);
        }
        return;
    }

    if (BspAuxLinkRxToIdleDmaStart(ElrsLinkRx.dma, (uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE) != 0)
    {
        WatchTaskError(WATCH_TASK_ELRS);
        /* 备用输入链路启动失败只撤销 ELRS，不能复位仍可由 DBUS/Image 控制的整机。 */
        ElrsLinkDmaActive = 0u;
        return;
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
            ElrsLinkOnByte(ElrsLinkRx.dma[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            ElrsLinkOnByte(ElrsLinkRx.dma[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            ElrsLinkOnByte(ElrsLinkRx.dma[i]);
        }
    }

    ElrsLinkDmaLastPos = (pos == size) ? 0u : pos;
}

static void ElrsLinkItDrain(void)
{
    if (ElrsLinkItRxOverflow != 0u)
    {
        taskENTER_CRITICAL();
        ElrsLinkItRxOverflow = 0u;
        ElrsLinkItRxTail = ElrsLinkItRxHead;
        taskEXIT_CRITICAL();
        ElrsLinkReset();
        WatchTaskTimeout(WATCH_TASK_ELRS);
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        return;
    }

    while (ElrsLinkItRxTail != ElrsLinkItRxHead)
    {
        const uint16_t tail = ElrsLinkItRxTail;
        const uint8_t value = ElrsLinkRx.itRing[tail];
        ElrsLinkItRxTail =
            (uint16_t)((tail + 1u) & (uint16_t)(ELRS_LINK_IT_RX_RING_SIZE - 1u));
        ElrsLinkOnByte(value);
    }
    if (ElrsLinkItRxOverflow != 0u)
    {
        taskENTER_CRITICAL();
        ElrsLinkItRxOverflow = 0u;
        ElrsLinkItRxTail = ElrsLinkItRxHead;
        taskEXIT_CRITICAL();
        ElrsLinkReset();
        WatchTaskTimeout(WATCH_TASK_ELRS);
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
    }
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

    int16_t ch_raw[16] = {0};
    for (uint8_t i = 0u; i < 16u; i++)
    {
        ch_raw[i] = (int16_t)ch[i];
    }

    ManualInputState rc = {0};
    ManualInputCrsfDecode(ch, &g_config.input, &rc);

    ManualInputLogRawSource(MANUAL_INPUT_SRC_ELRS,
                                  SDLOG_MANUAL_INPUT_PROTO_CRSF,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  16u,
                                  ch_raw,
                                  NULL,
                                  &rc);
    ManualInputUpdateElrsChannels(&rc, ch);

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
