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
#define CRSF_FRAME_SIZE_MAX 64u
#define CRSF_FRAME_LEN_MAX (CRSF_FRAME_SIZE_MAX - 2u) // [type + payload + crc]
#define CRSF_ADDRESS_BROADCAST 0x00u
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8u
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEAu
#define CRSF_ADDRESS_CRSF_RECEIVER 0xECu
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_RC_FRAME_LEN 24u // length byte for RC_CHANNELS_PACKED
#define CRSF_RC_PAYLOAD_LEN 22u
#define CRSF_LINK_STATS_FRAME_LEN 12u // type + 10-byte standard payload + crc
#define ELRS_LINK_PUBLISH_STALE 0u
#define ELRS_LINK_PUBLISH_OK 1u
#define ELRS_LINK_PUBLISH_CHANNEL_INVALID 2u
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

typedef struct
{
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint32_t session_gen;
    uint8_t valid;
} ElrsLinkPendingRcState;

typedef struct
{
    uint32_t session_gen;
    uint32_t transport_epoch;
} ElrsLinkManualCommitContext;

static ElrsLinkPendingRcState ElrsLinkPendingRc;
static ElrsLinkStats ElrsLinkDiag = {.state = ELRS_LINK_STATE_WAIT_STATS};
static TickType_t ElrsLinkLastStatsTick = 0u;
static uint32_t ElrsLinkLastStatsSessionGen = 0u;
static volatile uint32_t ElrsLinkSessionGen = 1u;
static volatile uint32_t ElrsLinkTransportEpoch = 1u;
static uint32_t ElrsLinkBatchSessionGen = 1u;
static uint32_t ElrsLinkBatchTransportEpoch = 1u;
static uint8_t ElrsLinkBatchActive = 0u;
static uint8_t ElrsLinkBatchForceInvalidate = 0u;

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
static void ElrsLinkSessionResetLocked(void);
static void ElrsLinkSessionReset(void);
static void ElrsLinkBatchBegin(uint32_t transport_epoch);
static void ElrsLinkBatchCommit(void);
static uint8_t ElrsLinkBatchContextCurrent(uint32_t session_gen);
static uint8_t ElrsLinkAbortTransportIfChanged(uint32_t session_gen);
static uint8_t ElrsLinkManualCommitGuard(void *context);
static void ElrsLinkExpireStats(void);
static void ElrsLinkMarkDown(uint8_t stats_timeout);
static void ElrsLinkTaskRunOnce(void);
static void ElrsLinkTaskWait(uint32_t *bits);
static uint8_t ElrsLinkDmaProcessTo(uint16_t pos);
static void ElrsLinkItDrain(void);
static void ElrsLinkNotifyFromIsr(uint32_t notify_bits);
static void ElrsLinkOnByte(uint8_t b);
static void ElrsLinkResync(uint8_t used);
static void ElrsLinkHandleFrame(const uint8_t *frame, uint8_t total_len);
static void ElrsLinkHandleStats(const uint8_t *payload, uint8_t payload_len);
static void ElrsLinkDecodeRcChannels(const uint8_t *payload, uint8_t payload_len);
static uint8_t ElrsLinkPublishRc(
    const uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
    uint32_t session_gen);
static uint8_t ElrsLinkAddressValid(uint8_t address);
static uint8_t ElrsLinkFrameCrcValid(const uint8_t *frame, uint8_t total_len);
static TickType_t ElrsLinkStatsTimeoutTicks(void);
static uint8_t ElrsLinkStatsFresh(TickType_t now_tick);
static TickType_t ElrsLinkWaitTicks(TickType_t now_tick);
static uint32_t ElrsLinkTickMs(TickType_t tick);
static uint32_t ElrsLinkSeqNext(uint32_t value);
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
        ElrsLinkTaskRunOnce();
    }
}

static void ElrsLinkTaskRunOnce(void)
{
    uint32_t bits = 0u;
    ElrsLinkExpireStats();
    ElrsLinkTaskWait(&bits);
    ElrsLinkExpireStats();

    if (BspAuxLinkGetBaudrate() != ELRS_LINK_BAUD)
    {
        return;
    }

    if (ElrsLinkDmaActive != 0u &&
        (ElrsLinkDmaRestartReq || (bits & ELRS_LINK_NOTIFY_RESTART) != 0u))
    {
        WatchTaskError(WATCH_TASK_ELRS);
        ElrsLinkDiag.restart_count++;
        ElrsLinkDmaRestartReq = 0u;
        ElrsLinkRxStartInternal();
        return;
    }

    if (!ElrsLinkDmaActive)
    {
        return;
    }
    if ((bits & ELRS_LINK_NOTIFY_RX) != 0u)
    {
        if (BspAuxLinkRxHasDma() == 0u)
        {
            ElrsLinkItDrain();
            return;
        }
        uint32_t wrap;
        uint32_t transport_epoch;
        uint16_t pos;
        taskENTER_CRITICAL();
        wrap = ElrsLinkDmaWrapCnt;
        pos = ElrsLinkDmaPos;
        transport_epoch = ElrsLinkTransportEpoch;
        taskEXIT_CRITICAL();
        const uint32_t wraps = wrap - ElrsLinkDmaLastWrapCnt;
        if (wraps > 1u ||
            (wraps == 1u && pos > ElrsLinkDmaLastPos))
        {
            // Too late: DMA has wrapped multiple times before we could process.
            WatchTaskTimeout(WATCH_TASK_ELRS);
            ElrsLinkDiag.overflow_count++;
            ElrsLinkSessionReset();
            ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
            ElrsLinkDmaLastWrapCnt = wrap;
            ElrsLinkDmaLastPos = pos;
            return;
        }
        /* wrap/pos/epoch 必须来自同一生产快照，已发生的正常跨环批仍可接受。 */
        ElrsLinkBatchBegin(transport_epoch);
        if (wraps == 1u)
        {
            // Process tail of the previous cycle up to end-of-buffer.
            if (ElrsLinkDmaProcessTo((uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE) == 0u)
            {
                return;
            }
            taskENTER_CRITICAL();
            if (ElrsLinkBatchSessionGen == ElrsLinkSessionGen)
            {
                ElrsLinkDmaLastWrapCnt = wrap;
            }
            taskEXIT_CRITICAL();
            if (ElrsLinkBatchSessionGen != ElrsLinkSessionGen)
            {
                return;
            }
        }

        if (ElrsLinkDmaProcessTo(pos) == 0u)
        {
            return;
        }
        ElrsLinkBatchCommit();
    }
}

void ElrsLinkStop(void)
{
    ElrsLinkDmaActive = 0u;
    __DMB();
    ElrsLinkDmaRestartReq = 0u;
    ElrsLinkSessionReset();
    ElrsLinkDiag.port_active = 0u;
    taskENTER_CRITICAL();
    ElrsLinkDmaPos = 0u;
    ElrsLinkDmaLastPos = 0u;
    ElrsLinkDmaWrapCnt = 0u;
    ElrsLinkDmaLastWrapCnt = 0u;
    ElrsLinkItRxHead = 0u;
    ElrsLinkItRxTail = 0u;
    ElrsLinkItRxOverflow = 0u;
    taskEXIT_CRITICAL();
    BspAuxLinkRxItStop();
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
}

void ElrsLinkRxStart(void)
{
    ElrsLinkRxStartInternal();
}

void ElrsLinkGetStats(ElrsLinkStats *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = ElrsLinkDiag;
    out->port_active = ElrsLinkDmaActive;
    taskEXIT_CRITICAL();
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
        ElrsLinkTransportEpoch = ElrsLinkSeqNext(ElrsLinkTransportEpoch);
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
        if (ElrsLinkItRxOverflow == 0u)
        {
            ElrsLinkItRxOverflow = 1u;
            ElrsLinkTransportEpoch = ElrsLinkSeqNext(ElrsLinkTransportEpoch);
        }
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

    if (ElrsLinkDmaRestartReq == 0u)
    {
        ElrsLinkDmaRestartReq = 1u;
        ElrsLinkTransportEpoch = ElrsLinkSeqNext(ElrsLinkTransportEpoch);
    }
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
        ElrsLinkDiag.port_active = ElrsLinkDmaActive;
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
    ElrsLinkDiag.port_active = 1u;
}

static void ElrsLinkReset(void)
{
    ElrsCrsfPos = 0u;
    ElrsCrsfExpected = 0u;
}

static void ElrsLinkSessionResetLocked(void)
{
    if (ElrsLinkBatchActive != 0u)
    {
        ElrsLinkBatchForceInvalidate = 1u;
    }
    ElrsLinkReset();
    ElrsLinkPendingRc.valid = 0u;
    ElrsLinkLastStatsTick = 0u;
    ElrsLinkLastStatsSessionGen = 0u;
    ElrsLinkSessionGen = ElrsLinkSeqNext(ElrsLinkSessionGen);
    ElrsLinkDiag.uplink_lq = 0u;
    ElrsLinkDiag.state = ELRS_LINK_STATE_WAIT_STATS;
    ElrsLinkBatchActive = 0u;
}

static void ElrsLinkSessionReset(void)
{
    taskENTER_CRITICAL();
    ElrsLinkSessionResetLocked();
    taskEXIT_CRITICAL();
    __DMB();
}

static void ElrsLinkBatchBegin(uint32_t transport_epoch)
{
    taskENTER_CRITICAL();
    ElrsLinkPendingRc.valid = 0u;
    ElrsLinkBatchSessionGen = ElrsLinkSessionGen;
    ElrsLinkBatchTransportEpoch = transport_epoch;
    ElrsLinkBatchForceInvalidate = 0u;
    ElrsLinkBatchActive = 1u;
    taskEXIT_CRITICAL();
}

static uint8_t ElrsLinkBatchContextCurrent(uint32_t session_gen)
{
    uint8_t current;

    taskENTER_CRITICAL();
    current = (uint8_t)(ElrsLinkDmaActive != 0u &&
                        ElrsLinkBatchActive != 0u &&
                        session_gen == ElrsLinkSessionGen &&
                        ElrsLinkBatchSessionGen == session_gen &&
                        ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch);
    taskEXIT_CRITICAL();
    if (current != 0u && BspAuxLinkGetBaudrate() != ELRS_LINK_BAUD)
    {
        current = 0u;
    }
    return current;
}

static uint8_t ElrsLinkManualCommitGuard(void *context)
{
    const ElrsLinkManualCommitContext *expected =
        (const ElrsLinkManualCommitContext *)context;
    uint8_t current = 0u;

    if (expected == NULL)
    {
        return 0u;
    }
    taskENTER_CRITICAL();
    if (ElrsLinkDmaActive != 0u &&
        ElrsLinkBatchActive != 0u &&
        ElrsLinkSessionGen == expected->session_gen &&
        ElrsLinkBatchSessionGen == expected->session_gen &&
        ElrsLinkBatchTransportEpoch == expected->transport_epoch &&
        ElrsLinkTransportEpoch == expected->transport_epoch)
    {
        current = 1u;
    }
    taskEXIT_CRITICAL();
    return current;
}

static uint8_t ElrsLinkAbortTransportIfChanged(uint32_t session_gen)
{
    uint8_t aborted = 0u;
    uint8_t report_overflow = 0u;

    taskENTER_CRITICAL();
    if (ElrsLinkDmaActive != 0u &&
        ElrsLinkBatchActive != 0u &&
        session_gen == ElrsLinkSessionGen &&
        ElrsLinkBatchSessionGen == session_gen &&
        ElrsLinkBatchTransportEpoch != ElrsLinkTransportEpoch)
    {
        if (ElrsLinkDmaRestartReq == 0u)
        {
            ElrsLinkDiag.overflow_count++;
            report_overflow = 1u;
        }
        ElrsLinkDmaLastWrapCnt = ElrsLinkDmaWrapCnt;
        ElrsLinkDmaLastPos = ElrsLinkDmaPos;
        ElrsLinkItRxTail = ElrsLinkItRxHead;
        ElrsLinkItRxOverflow = 0u;
        ElrsLinkSessionResetLocked();
        aborted = 1u;
    }
    taskEXIT_CRITICAL();
    if (aborted != 0u)
    {
        __DMB();
        if (report_overflow != 0u)
        {
            WatchTaskTimeout(WATCH_TASK_ELRS);
        }
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
    }
    return aborted;
}

static void ElrsLinkBatchCommit(void)
{
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t batch_session_gen = ElrsLinkBatchSessionGen;
    uint32_t session_gen;
    uint8_t publish_result;
    uint8_t final_current = 0u;
    uint8_t invalidate = 0u;

    ElrsLinkExpireStats();
    if (ElrsLinkBatchContextCurrent(batch_session_gen) == 0u)
    {
        (void)ElrsLinkAbortTransportIfChanged(batch_session_gen);
        return;
    }
    if (ElrsLinkBatchForceInvalidate != 0u)
    {
        taskENTER_CRITICAL();
        if (ElrsLinkBatchActive != 0u &&
            ElrsLinkBatchSessionGen == ElrsLinkSessionGen &&
            ElrsLinkBatchSessionGen == batch_session_gen &&
            ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch)
        {
            ElrsLinkPendingRc.valid = 0u;
            ElrsLinkBatchActive = 0u;
            invalidate = 1u;
        }
        taskEXIT_CRITICAL();
        if (invalidate != 0u)
        {
            ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        }
        else
        {
            (void)ElrsLinkAbortTransportIfChanged(batch_session_gen);
        }
        return;
    }
    if (ElrsLinkPendingRc.valid == 0u)
    {
        taskENTER_CRITICAL();
        if (ElrsLinkBatchActive != 0u &&
            ElrsLinkBatchSessionGen == ElrsLinkSessionGen &&
            ElrsLinkBatchSessionGen == batch_session_gen &&
            ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch)
        {
            ElrsLinkBatchActive = 0u;
        }
        taskEXIT_CRITICAL();
        if (ElrsLinkBatchActive != 0u)
        {
            (void)ElrsLinkAbortTransportIfChanged(batch_session_gen);
        }
        return;
    }
    session_gen = ElrsLinkPendingRc.session_gen;
    if (session_gen != batch_session_gen ||
        ElrsLinkStatsFresh(now_tick) == 0u)
    {
        taskENTER_CRITICAL();
        if (ElrsLinkBatchActive != 0u &&
            ElrsLinkBatchSessionGen == ElrsLinkSessionGen &&
            ElrsLinkBatchSessionGen == batch_session_gen &&
            ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch)
        {
            ElrsLinkDiag.health_reject_count++;
            ElrsLinkPendingRc.valid = 0u;
            ElrsLinkBatchActive = 0u;
        }
        taskEXIT_CRITICAL();
        if (ElrsLinkBatchActive != 0u)
        {
            (void)ElrsLinkAbortTransportIfChanged(batch_session_gen);
        }
        ElrsLinkExpireStats();
        return;
    }

    publish_result = ElrsLinkPublishRc(ElrsLinkPendingRc.channel, session_gen);
    const uint8_t baud_ok = (BspAuxLinkGetBaudrate() == ELRS_LINK_BAUD) ? 1u : 0u;
    taskENTER_CRITICAL();
    if (baud_ok != 0u &&
        ElrsLinkDmaActive != 0u &&
        ElrsLinkBatchActive != 0u &&
        ElrsLinkBatchSessionGen == session_gen &&
        ElrsLinkSessionGen == session_gen &&
        ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch)
    {
        ElrsLinkPendingRc.valid = 0u;
        ElrsLinkBatchActive = 0u;
        if (publish_result == ELRS_LINK_PUBLISH_OK)
        {
            ElrsLinkDiag.state = ELRS_LINK_STATE_UP;
            ElrsLinkDiag.rc_publish_count++;
            final_current = 1u;
        }
        else if (publish_result == ELRS_LINK_PUBLISH_CHANNEL_INVALID)
        {
            ElrsLinkDiag.channel_reject_count++;
            ElrsLinkDiag.state = ELRS_LINK_STATE_WAIT_RC;
            final_current = 1u;
            invalidate = 1u;
        }
    }
    taskEXIT_CRITICAL();

    if (final_current == 0u)
    {
        const uint8_t transport_aborted = ElrsLinkAbortTransportIfChanged(session_gen);
        if (transport_aborted == 0u)
        {
            taskENTER_CRITICAL();
            if (ElrsLinkBatchActive != 0u &&
                ElrsLinkBatchSessionGen == session_gen &&
                ElrsLinkSessionGen == session_gen)
            {
                ElrsLinkPendingRc.valid = 0u;
                ElrsLinkBatchActive = 0u;
            }
            taskEXIT_CRITICAL();
            ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        }
    }
    else if (invalidate != 0u)
    {
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
    }
}

static void ElrsLinkMarkDown(uint8_t stats_timeout)
{
    if (ElrsLinkDiag.state != ELRS_LINK_STATE_DOWN)
    {
        ElrsLinkDiag.link_down_count++;
        if (stats_timeout != 0u)
        {
            ElrsLinkDiag.stats_timeout_count++;
        }
    }
    ElrsLinkPendingRc.valid = 0u;
    if (ElrsLinkBatchActive != 0u)
    {
        ElrsLinkBatchForceInvalidate = 1u;
    }
    ElrsLinkDiag.state = ELRS_LINK_STATE_DOWN;
    if (ElrsLinkBatchActive == 0u)
    {
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
    }
}

static void ElrsLinkExpireStats(void)
{
    const TickType_t now_tick = xTaskGetTickCount();

    if ((ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_RC ||
         ElrsLinkDiag.state == ELRS_LINK_STATE_UP) &&
        ElrsLinkStatsFresh(now_tick) == 0u)
    {
        ElrsLinkMarkDown(1u);
    }
}

static void ElrsLinkTaskWait(uint32_t *bits)
{
    const TickType_t wait_ticks = ElrsLinkWaitTicks(xTaskGetTickCount());

    WatchTaskWait(WATCH_TASK_ELRS);
    (void)xTaskNotifyWait(0u, 0xFFFFFFFFu, bits, wait_ticks);
    WatchTaskBeat(WATCH_TASK_ELRS);
}

static uint8_t ElrsLinkDmaProcessTo(uint16_t pos)
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
        return 1u;
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

    taskENTER_CRITICAL();
    if (ElrsLinkBatchSessionGen != ElrsLinkSessionGen ||
        ElrsLinkDmaActive == 0u)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }
    ElrsLinkDmaLastPos = (pos == size) ? 0u : pos;
    taskEXIT_CRITICAL();
    return 1u;
}

static void ElrsLinkItDrain(void)
{
    uint16_t target_head;
    uint32_t transport_epoch;
    uint8_t overflow;
    uint8_t more_pending;

    taskENTER_CRITICAL();
    overflow = ElrsLinkItRxOverflow;
    if (overflow != 0u)
    {
        ElrsLinkItRxOverflow = 0u;
        ElrsLinkItRxTail = ElrsLinkItRxHead;
        target_head = ElrsLinkItRxHead;
        transport_epoch = ElrsLinkTransportEpoch;
    }
    else
    {
        target_head = ElrsLinkItRxHead;
        transport_epoch = ElrsLinkTransportEpoch;
    }
    taskEXIT_CRITICAL();
    if (overflow != 0u)
    {
        ElrsLinkDiag.overflow_count++;
        ElrsLinkSessionReset();
        WatchTaskTimeout(WATCH_TASK_ELRS);
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        return;
    }

    ElrsLinkBatchBegin(transport_epoch);
    while (ElrsLinkItRxTail != target_head)
    {
        uint16_t tail;
        uint8_t value;

        taskENTER_CRITICAL();
        if (ElrsLinkBatchSessionGen != ElrsLinkSessionGen ||
            ElrsLinkDmaActive == 0u)
        {
            taskEXIT_CRITICAL();
            return;
        }
        tail = ElrsLinkItRxTail;
        value = ElrsLinkRx.itRing[tail];
        ElrsLinkItRxTail =
            (uint16_t)((tail + 1u) & (uint16_t)(ELRS_LINK_IT_RX_RING_SIZE - 1u));
        taskEXIT_CRITICAL();
        ElrsLinkOnByte(value);
    }
    if (ElrsLinkBatchContextCurrent(ElrsLinkBatchSessionGen) == 0u)
    {
        (void)ElrsLinkAbortTransportIfChanged(ElrsLinkBatchSessionGen);
        return;
    }
    ElrsLinkBatchCommit();

    /* 固定头快照后的新字节属于下一批；与 ISR 的 was_empty 判断合起来避免丢唤醒。 */
    taskENTER_CRITICAL();
    more_pending = (ElrsLinkItRxTail != ElrsLinkItRxHead) ? 1u : 0u;
    if (more_pending != 0u && ElrsLinkTaskHandle == NULL)
    {
        ElrsLinkNotifyPending |= ELRS_LINK_NOTIFY_RX;
    }
    taskEXIT_CRITICAL();
    if (more_pending != 0u && ElrsLinkTaskHandle != NULL)
    {
        (void)xTaskNotify(ElrsLinkTaskHandle, ELRS_LINK_NOTIFY_RX, eSetBits);
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

static uint32_t ElrsLinkTickMs(TickType_t tick)
{
    uint32_t tick_ms = (uint32_t)portTICK_PERIOD_MS;

    if (tick_ms == 0u)
    {
        tick_ms = 1u;
    }
    return (uint32_t)tick * tick_ms;
}

static uint32_t ElrsLinkSeqNext(uint32_t value)
{
    value++;
    return (value == 0u) ? 1u : value;
}

static uint8_t ElrsLinkAddressValid(uint8_t address)
{
    return (uint8_t)(address == CRSF_ADDRESS_BROADCAST ||
                     address == CRSF_ADDRESS_FLIGHT_CONTROLLER ||
                     address == CRSF_ADDRESS_RADIO_TRANSMITTER ||
                     address == CRSF_ADDRESS_CRSF_RECEIVER);
}

static uint8_t ElrsLinkFrameCrcValid(const uint8_t *frame, uint8_t total_len)
{
    uint8_t len;

    if (frame == NULL || total_len < 4u || total_len > CRSF_FRAME_SIZE_MAX ||
        ElrsLinkAddressValid(frame[0]) == 0u)
    {
        return 0u;
    }
    len = frame[1];
    if (len < 2u || len > CRSF_FRAME_LEN_MAX ||
        (uint8_t)(len + 2u) != total_len)
    {
        return 0u;
    }
    return (ElrsLinkCrc8DvbS2(&frame[2], (uint8_t)(len - 1u)) ==
            frame[total_len - 1u]) ? 1u : 0u;
}

static TickType_t ElrsLinkStatsTimeoutTicks(void)
{
    TickType_t timeout_tick = pdMS_TO_TICKS(ELRS_LINK_STATS_TIMEOUT_MS);

    if (timeout_tick == 0u)
    {
        timeout_tick = 1u;
    }
    return timeout_tick;
}

static uint8_t ElrsLinkStatsFresh(TickType_t now_tick)
{
    const TickType_t timeout_tick = ElrsLinkStatsTimeoutTicks();

    if (ElrsLinkDiag.state != ELRS_LINK_STATE_WAIT_RC &&
        ElrsLinkDiag.state != ELRS_LINK_STATE_UP)
    {
        return 0u;
    }
    if (ElrsLinkLastStatsSessionGen != ElrsLinkSessionGen)
    {
        return 0u;
    }
    return ((TickType_t)(now_tick - ElrsLinkLastStatsTick) <= timeout_tick) ? 1u : 0u;
}

static TickType_t ElrsLinkWaitTicks(TickType_t now_tick)
{
    const TickType_t timeout_tick = ElrsLinkStatsTimeoutTicks();

    if (ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_RC ||
        ElrsLinkDiag.state == ELRS_LINK_STATE_UP)
    {
        const TickType_t age = (TickType_t)(now_tick - ElrsLinkLastStatsTick);
        if (age <= timeout_tick)
        {
            // age==timeout 仍有效，因此等待到下一 tick 再撤销。
            return (TickType_t)(timeout_tick - age + 1u);
        }
        return 0u;
    }
    return timeout_tick;
}

static void ElrsLinkHandleStats(const uint8_t *payload, uint8_t payload_len)
{
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t session_gen = ElrsLinkBatchSessionGen;
    uint8_t context_current;
    uint8_t uplink_lq;

    taskENTER_CRITICAL();
    context_current = (uint8_t)(ElrsLinkDmaActive != 0u &&
                                ElrsLinkBatchActive != 0u &&
                                ElrsLinkBatchSessionGen == session_gen &&
                                ElrsLinkSessionGen == session_gen &&
                                ElrsLinkBatchTransportEpoch == ElrsLinkTransportEpoch);
    if (context_current == 0u)
    {
        taskEXIT_CRITICAL();
        (void)ElrsLinkAbortTransportIfChanged(session_gen);
        return;
    }
    if (payload == NULL || payload_len < 10u)
    {
        ElrsLinkDiag.length_error_count++;
        taskEXIT_CRITICAL();
        return;
    }

    uplink_lq = payload[2];
    ElrsLinkLastStatsTick = now_tick;
    ElrsLinkLastStatsSessionGen = session_gen;
    ElrsLinkDiag.last_stats_tick_ms = ElrsLinkTickMs(now_tick);
    ElrsLinkDiag.link_stats_count++;
    ElrsLinkDiag.uplink_rssi_1 = payload[0];
    ElrsLinkDiag.uplink_rssi_2 = payload[1];
    ElrsLinkDiag.uplink_lq = uplink_lq;
    ElrsLinkDiag.uplink_snr = (int8_t)payload[3];

    if (uplink_lq > 100u)
    {
        ElrsLinkDiag.health_reject_count++;
        ElrsLinkMarkDown(0u);
    }
    else if (uplink_lq == 0u)
    {
        ElrsLinkMarkDown(0u);
    }
    else if (ElrsLinkDiag.state != ELRS_LINK_STATE_UP)
    {
        ElrsLinkDiag.state = ELRS_LINK_STATE_WAIT_RC;
    }
    taskEXIT_CRITICAL();
}

static void ElrsLinkDecodeRcChannels(const uint8_t *payload, uint8_t payload_len)
{
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t session_gen = ElrsLinkBatchSessionGen;
    uint16_t *channel = ElrsLinkPendingRc.channel;
    uint32_t bitbuf = 0u;
    uint8_t bitcnt = 0u;
    uint8_t idx = 0u;

    if (payload == NULL || payload_len < CRSF_RC_PAYLOAD_LEN)
    {
        ElrsLinkDiag.length_error_count++;
        return;
    }

    for (uint8_t i = 0u; i < CRSF_RC_PAYLOAD_LEN; i++)
    {
        bitbuf |= ((uint32_t)payload[i]) << bitcnt;
        bitcnt = (uint8_t)(bitcnt + 8u);
        while (bitcnt >= 11u && idx < 16u)
        {
            channel[idx++] = (uint16_t)(bitbuf & 0x07FFu);
            bitbuf >>= 11u;
            bitcnt = (uint8_t)(bitcnt - 11u);
        }
    }

    if (idx < MANUAL_INPUT_CRSF_CHANNEL_COUNT)
    {
        ElrsLinkDiag.length_error_count++;
        return;
    }

    const uint8_t mapped_values_valid =
        ManualInputCrsfMappedValuesValid(channel, &g_config.input);
    taskENTER_CRITICAL();
    if (ElrsLinkDmaActive == 0u ||
        ElrsLinkBatchActive == 0u ||
        ElrsLinkBatchSessionGen != session_gen ||
        ElrsLinkSessionGen != session_gen ||
        ElrsLinkBatchTransportEpoch != ElrsLinkTransportEpoch)
    {
        taskEXIT_CRITICAL();
        (void)ElrsLinkAbortTransportIfChanged(session_gen);
        return;
    }

    ElrsLinkDiag.last_rc_tick_ms = ElrsLinkTickMs(now_tick);
    ElrsLinkDiag.rc_frame_count++;
    if (mapped_values_valid == 0u)
    {
        ElrsLinkDiag.channel_reject_count++;
        ElrsLinkPendingRc.valid = 0u;
        ElrsLinkBatchForceInvalidate = 1u;
        if (ElrsLinkDiag.state == ELRS_LINK_STATE_UP)
        {
            ElrsLinkDiag.state = ELRS_LINK_STATE_WAIT_RC;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (ElrsLinkStatsFresh(now_tick) == 0u)
    {
        ElrsLinkDiag.health_reject_count++;
        ElrsLinkPendingRc.valid = 0u;
        taskEXIT_CRITICAL();
        ElrsLinkExpireStats();
        return;
    }

    ElrsLinkPendingRc.session_gen = session_gen;
    ElrsLinkPendingRc.valid = 1u;
    taskEXIT_CRITICAL();
}

static uint8_t ElrsLinkPublishRc(
    const uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
    uint32_t session_gen)
{
    int16_t ch_raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT] = {0};
    ManualInputState rc = {0};
    sdlog_rc_crsf_t pkt = {0};
    const input_config_t input_config = g_config.input;
    ElrsLinkManualCommitContext commit_context;

    if (channel == NULL)
    {
        return ELRS_LINK_PUBLISH_CHANNEL_INVALID;
    }
    for (uint8_t i = 0u; i < MANUAL_INPUT_CRSF_CHANNEL_COUNT; i++)
    {
        ch_raw[i] = (int16_t)channel[i];
        pkt.ch_raw[i] = channel[i];
    }

    if (ManualInputCrsfDecode(channel, &input_config, &rc) == 0u)
    {
        return ELRS_LINK_PUBLISH_CHANNEL_INVALID;
    }
    __DMB();
    if (ElrsLinkBatchContextCurrent(session_gen) == 0u)
    {
        (void)ElrsLinkAbortTransportIfChanged(session_gen);
        return ELRS_LINK_PUBLISH_STALE;
    }

    commit_context.session_gen = session_gen;
    commit_context.transport_epoch = ElrsLinkBatchTransportEpoch;
    /* guard 与来源入库共用 ManualInput 临界区，ISR 不能夹在最终核对与写入之间。 */
    if (ManualInputUpdateElrsChannelsGuarded(&rc,
                                             channel,
                                             ElrsLinkManualCommitGuard,
                                             &commit_context) == 0u)
    {
        (void)ElrsLinkAbortTransportIfChanged(session_gen);
        return ELRS_LINK_PUBLISH_STALE;
    }
    __DMB();
    if (ElrsLinkBatchContextCurrent(session_gen) == 0u)
    {
        if (ElrsLinkAbortTransportIfChanged(session_gen) == 0u)
        {
            ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        }
        return ELRS_LINK_PUBLISH_STALE;
    }

    ManualInputLogRawSource(MANUAL_INPUT_SRC_ELRS,
                            SDLOG_MANUAL_INPUT_PROTO_CRSF,
                            SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                            MANUAL_INPUT_CRSF_CHANNEL_COUNT,
                            ch_raw,
                            NULL,
                            &rc);

    const uint32_t rc_len = (uint32_t)sizeof(ManualInputState);
    const uint32_t copy_len = (rc_len < (uint32_t)sizeof(pkt.rc_ctrl)) ? rc_len : (uint32_t)sizeof(pkt.rc_ctrl);
    const uint8_t *rc_bytes = (const uint8_t *)&rc;
    for (uint32_t i = 0u; i < copy_len; i++)
    {
        pkt.rc_ctrl[i] = rc_bytes[i];
    }
    SdLogWrite(SDLOG_TAG_RC_CRSF, &pkt, (uint16_t)sizeof(pkt));
    __DMB();
    if (ElrsLinkBatchContextCurrent(session_gen) == 0u)
    {
        if (ElrsLinkAbortTransportIfChanged(session_gen) == 0u)
        {
            ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        }
        return ELRS_LINK_PUBLISH_STALE;
    }
    return ELRS_LINK_PUBLISH_OK;
}

static void ElrsLinkHandleFrame(const uint8_t *frame, uint8_t total_len)
{
    uint8_t len;
    uint8_t type;

    if (frame == NULL || total_len < 4u)
    {
        return;
    }
    len = frame[1];
    type = frame[2];
    ElrsLinkDiag.valid_frame_count++;
    if (frame[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        return;
    }
    if (type == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        if (len < CRSF_LINK_STATS_FRAME_LEN)
        {
            ElrsLinkDiag.length_error_count++;
            return;
        }
        ElrsLinkHandleStats(&frame[3], (uint8_t)(len - 2u));
        return;
    }
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        if (len < CRSF_RC_FRAME_LEN)
        {
            ElrsLinkDiag.length_error_count++;
            return;
        }
        ElrsLinkDecodeRcChannels(&frame[3], CRSF_RC_PAYLOAD_LEN);
    }
}

static void ElrsLinkResync(uint8_t used)
{
    uint8_t partial = 0xFFu;

    if (used > CRSF_FRAME_SIZE_MAX)
    {
        used = CRSF_FRAME_SIZE_MAX;
    }

    /* 先找恰好落在当前缓冲尾部的完整合法帧，避免长度位损坏吞掉下一帧。 */
    for (uint8_t i = 1u; i + 3u < used; i++)
    {
        uint8_t candidate_len;
        uint8_t candidate_total;

        if (ElrsLinkAddressValid(ElrsCrsfBuf[i]) == 0u)
        {
            continue;
        }
        candidate_len = ElrsCrsfBuf[i + 1u];
        if (candidate_len < 2u || candidate_len > CRSF_FRAME_LEN_MAX)
        {
            continue;
        }
        candidate_total = (uint8_t)(candidate_len + 2u);
        if ((uint8_t)(used - i) == candidate_total &&
            ElrsLinkFrameCrcValid(&ElrsCrsfBuf[i], candidate_total) != 0u)
        {
            ElrsLinkDiag.sync_drop_count += i;
            ElrsLinkHandleFrame(&ElrsCrsfBuf[i], candidate_total);
            ElrsLinkReset();
            return;
        }
    }

    /* 没有完整帧时保留最早的合法不完整后缀，下一批字节从这里继续。 */
    for (uint8_t i = 1u; i < used; i++)
    {
        uint8_t available;

        if (ElrsLinkAddressValid(ElrsCrsfBuf[i]) == 0u)
        {
            continue;
        }
        available = (uint8_t)(used - i);
        if (available == 1u)
        {
            partial = i;
            break;
        }
        if (ElrsCrsfBuf[i + 1u] >= 2u &&
            ElrsCrsfBuf[i + 1u] <= CRSF_FRAME_LEN_MAX &&
            available < (uint8_t)(ElrsCrsfBuf[i + 1u] + 2u))
        {
            partial = i;
            break;
        }
    }

    if (partial == 0xFFu)
    {
        ElrsLinkDiag.sync_drop_count += used;
        ElrsLinkReset();
        return;
    }

    ElrsLinkDiag.sync_drop_count += partial;
    ElrsCrsfPos = (uint8_t)(used - partial);
    memmove(ElrsCrsfBuf, &ElrsCrsfBuf[partial], ElrsCrsfPos);
    ElrsCrsfExpected = (ElrsCrsfPos >= 2u) ?
                           (uint8_t)(ElrsCrsfBuf[1] + 2u) :
                           0u;
}

static void ElrsLinkOnByte(uint8_t b)
{
    if (ElrsLinkDmaActive == 0u ||
        ElrsLinkBatchSessionGen != ElrsLinkSessionGen)
    {
        return;
    }
    if (ElrsCrsfPos == 0u)
    {
        if (ElrsLinkAddressValid(b) == 0u)
        {
            ElrsLinkDiag.sync_drop_count++;
            return;
        }
        ElrsCrsfBuf[0] = b;
        ElrsCrsfPos = 1u;
        return;
    }

    if (ElrsCrsfPos == 1u)
    {
        ElrsCrsfBuf[1] = b;
        ElrsCrsfPos = 2u;
        if (b < 2u || b > CRSF_FRAME_LEN_MAX)
        {
            ElrsLinkDiag.length_error_count++;
            ElrsLinkResync(ElrsCrsfPos);
            return;
        }

        ElrsCrsfExpected = (uint8_t)(b + 2u);
        return;
    }

    if (ElrsCrsfExpected < 4u || ElrsCrsfExpected > CRSF_FRAME_SIZE_MAX)
    {
        ElrsLinkDiag.length_error_count++;
        ElrsLinkResync(ElrsCrsfPos);
        return;
    }

    ElrsCrsfBuf[ElrsCrsfPos++] = b;
    if (ElrsCrsfPos >= ElrsCrsfExpected)
    {
        const uint8_t used = ElrsCrsfExpected;
        if (ElrsLinkFrameCrcValid(ElrsCrsfBuf, used) != 0u)
        {
            ElrsLinkHandleFrame(ElrsCrsfBuf, used);
            ElrsLinkReset();
        }
        else
        {
            ElrsLinkDiag.crc_error_count++;
            ElrsLinkResync(used);
        }
    }
}
