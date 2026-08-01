/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BspCan.h"
#include "BspCanZephyr.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>

#define BSP_CAN_BUS_MAX                       3u
#define BSP_CAN_RX_RING_SIZE                  128u
#define BSP_CAN_TX_SLOT_COUNT                 32u
#define BSP_CAN_TX_COMPLETION_RING_SIZE       128u
#define BSP_CAN_STD_ID_DIAG_COUNT             16u

#define BSP_CAN_STATUS_OK                     0u
#define BSP_CAN_STATUS_ERROR                  1u
#define BSP_CAN_STATUS_BUSY                   2u
#define BSP_CAN_STATUS_TIMEOUT                3u

#define BSP_CAN_SLOT_FREE                     0u
#define BSP_CAN_SLOT_PENDING                  1u
#define BSP_CAN_SLOT_COMPLETION_DEFERRED       2u

#define BSP_CAN_INIT_IDLE                     0
#define BSP_CAN_INIT_RUNNING                  1
#define BSP_CAN_INIT_DONE                     2

BUILD_ASSERT((BSP_CAN_RX_RING_SIZE & (BSP_CAN_RX_RING_SIZE - 1u)) == 0u);
BUILD_ASSERT((BSP_CAN_TX_COMPLETION_RING_SIZE &
              (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u)) == 0u);
BUILD_ASSERT(BSP_CAN_TX_COMPLETION_RING_SIZE >
             (BSP_CAN_BUS_MAX * BSP_CAN_TX_SLOT_COUNT));
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_ALIAS(can_primary), okay),
             "ARBATOS requires the can-primary devicetree alias");
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_ALIAS(can_secondary), okay),
             "ARBATOS requires the can-secondary devicetree alias");

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define ARB_CAN_CPU_DATA __dtcm_data_section
#define ARB_CAN_CPU_BSS  __dtcm_bss_section
#else
#define ARB_CAN_CPU_DATA
#define ARB_CAN_CPU_BSS
#endif

typedef struct BspCanZephyrBus BspCanZephyrBus;

typedef struct
{
    BspCanZephyrBus *bus;
    BspCanTxTicket ticket;
    uint16_t stdId;
    uint8_t state;
    uint8_t tracked;
    uint8_t terminalResult;
    uint8_t faultInvalidated;
} BspCanZephyrTxSlot;

struct BspCanZephyrBus
{
    const struct device *dev;
    BspCanFrame rxRing[BSP_CAN_RX_RING_SIZE];
    BspCanZephyrTxSlot txSlots[BSP_CAN_TX_SLOT_COUNT];
    uint32_t rxStdIdCount[BSP_CAN_STD_ID_DIAG_COUNT];
    uint32_t txStdIdCount[BSP_CAN_STD_ID_DIAG_COUNT];
    uint32_t terminalCount[5u];
    uint32_t rxCount;
    uint32_t rxDropCount;
    uint32_t txCount;
    uint32_t txFailCount;
    uint32_t completionBackpressureCount;
    uint32_t lastError;
    uint16_t rxHead;
    uint16_t rxTail;
    uint16_t lastRxStdId;
    uint16_t lastTxStdId;
    uint8_t bus;
    uint8_t lastRxDlc;
    uint8_t lastTxDlc;
    uint8_t lastTxStatus;
    uint8_t reconfiguring;
    uint8_t ready;
    enum can_state state;
    struct can_bus_err_cnt errCount;
    int rxFilterId;
};

static BspCanZephyrBus CanBuses[] ARB_CAN_CPU_DATA = {
    {
        .dev = DEVICE_DT_GET(DT_ALIAS(can_primary)),
        .bus = 1u,
        .state = CAN_STATE_STOPPED,
        .rxFilterId = -1,
    },
    {
        .dev = DEVICE_DT_GET(DT_ALIAS(can_secondary)),
        .bus = 2u,
        .state = CAN_STATE_STOPPED,
        .rxFilterId = -1,
    },
#if DT_NODE_HAS_STATUS(DT_ALIAS(can_tertiary), okay)
    {
        .dev = DEVICE_DT_GET(DT_ALIAS(can_tertiary)),
        .bus = 3u,
        .state = CAN_STATE_STOPPED,
        .rxFilterId = -1,
    },
#endif
};

static BspCanTxCompletion
    CanCompletionRing[BSP_CAN_TX_COMPLETION_RING_SIZE] ARB_CAN_CPU_BSS;
static uint16_t CanCompletionHead;
static uint16_t CanCompletionTail;
static uint8_t CanRxNextBus = 1u;
static TaskHandle_t CanRxTaskHandle;
static atomic_t CanFaultLocked;
static atomic_t CanInitState;
K_MUTEX_DEFINE(CanTxSubmitLock);

static BspCanZephyrBus *BspCanBusGet(uint8_t bus)
{
    if (bus == 0u || bus > ARRAY_SIZE(CanBuses))
    {
        return NULL;
    }
    return &CanBuses[bus - 1u];
}

static uint8_t BspCanTicketValid(const BspCanTxTicket *ticket)
{
    return (uint8_t)(ticket != NULL &&
                     (ticket->epoch != 0u || ticket->seq != 0u));
}

static uint32_t BspCanErrEncode(int error)
{
    return (error < 0) ? (uint32_t)(-error) : (uint32_t)error;
}

static uint8_t BspCanStatusFromErr(int error)
{
    if (error == 0)
    {
        return BSP_CAN_STATUS_OK;
    }
    if (error == -EAGAIN || error == -EBUSY)
    {
        return BSP_CAN_STATUS_BUSY;
    }
    if (error == -ETIMEDOUT)
    {
        return BSP_CAN_STATUS_TIMEOUT;
    }
    return BSP_CAN_STATUS_ERROR;
}

static uint8_t BspCanResultFromErr(int error)
{
    switch (error)
    {
    case 0:
        return (uint8_t)BspCanTxResultComplete;
    case -ECANCELED:
    case -ENETDOWN:
        return (uint8_t)BspCanTxResultAborted;
    case -EIO:
    case -EBUSY:
    case -EAGAIN:
    case -ENETUNREACH:
        return (uint8_t)BspCanTxResultFailed;
    default:
        return (uint8_t)BspCanTxResultUnknown;
    }
}

static uint8_t BspCanCompletionPushLocked(const BspCanZephyrTxSlot *slot,
                                          uint8_t result)
{
    const uint16_t next =
        (uint16_t)((CanCompletionHead + 1u) &
                   (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u));

    if (slot == NULL || slot->bus == NULL ||
        result <= (uint8_t)BspCanTxResultNone ||
        result > (uint8_t)BspCanTxResultUnknown ||
        next == CanCompletionTail)
    {
        return 0u;
    }

    BspCanTxCompletion *completion = &CanCompletionRing[CanCompletionHead];
    completion->ticket = slot->ticket;
    completion->stdId = slot->stdId;
    completion->bus = slot->bus->bus;
    completion->result = result;
    barrier_dmem_fence_full();
    CanCompletionHead = next;
    return 1u;
}

static void BspCanCompletionFlushLocked(void)
{
    for (size_t busIndex = 0u; busIndex < ARRAY_SIZE(CanBuses); busIndex++)
    {
        BspCanZephyrBus *bus = &CanBuses[busIndex];

        for (size_t slotIndex = 0u;
             slotIndex < ARRAY_SIZE(bus->txSlots);
             slotIndex++)
        {
            BspCanZephyrTxSlot *slot = &bus->txSlots[slotIndex];

            if (slot->state != BSP_CAN_SLOT_COMPLETION_DEFERRED)
            {
                continue;
            }
            if (BspCanCompletionPushLocked(slot, slot->terminalResult) == 0u)
            {
                return;
            }
            (void)memset(slot, 0, sizeof(*slot));
        }
    }
}

static BspCanZephyrTxSlot *BspCanTxSlotReserveLocked(BspCanZephyrBus *bus,
                                                     uint16_t stdId,
                                                     const BspCanTxTicket *ticket,
                                                     uint8_t tracked)
{
    for (size_t index = 0u; index < ARRAY_SIZE(bus->txSlots); index++)
    {
        BspCanZephyrTxSlot *slot = &bus->txSlots[index];

        if (slot->state != BSP_CAN_SLOT_FREE)
        {
            continue;
        }

        slot->bus = bus;
        slot->stdId = stdId;
        slot->tracked = tracked;
        slot->terminalResult = (uint8_t)BspCanTxResultNone;
        slot->faultInvalidated = 0u;
        if (ticket != NULL)
        {
            slot->ticket = *ticket;
        }
        else
        {
            slot->ticket.epoch = 0u;
            slot->ticket.seq = 0u;
        }
        barrier_dmem_fence_full();
        slot->state = BSP_CAN_SLOT_PENDING;
        return slot;
    }
    return NULL;
}

static void BspCanTxCallback(const struct device *dev, int error, void *userData)
{
    BspCanZephyrTxSlot *slot = (BspCanZephyrTxSlot *)userData;
    unsigned int key = irq_lock();

    if (slot == NULL || slot->bus == NULL || slot->bus->dev != dev ||
        slot->state != BSP_CAN_SLOT_PENDING)
    {
        irq_unlock(key);
        return;
    }

    BspCanZephyrBus *bus = slot->bus;
    uint8_t result = (slot->faultInvalidated != 0u)
                         ? (uint8_t)BspCanTxResultUnknown
                         : BspCanResultFromErr(error);

    if (error != 0)
    {
        bus->lastError = BspCanErrEncode(error);
    }

    if (slot->tracked == 0u)
    {
        (void)memset(slot, 0, sizeof(*slot));
        irq_unlock(key);
        return;
    }

    bus->terminalCount[result]++;
    if (BspCanCompletionPushLocked(slot, result) != 0u)
    {
        (void)memset(slot, 0, sizeof(*slot));
    }
    else
    {
        slot->terminalResult = result;
        slot->state = BSP_CAN_SLOT_COMPLETION_DEFERRED;
        bus->completionBackpressureCount++;
    }
    irq_unlock(key);
}

static void BspCanRxNotifyFromIsr(void)
{
    TaskHandle_t task = CanRxTaskHandle;

    if (task == NULL)
    {
        return;
    }

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static void BspCanRxCallback(const struct device *dev,
                             struct can_frame *frame,
                             void *userData)
{
    BspCanZephyrBus *bus = (BspCanZephyrBus *)userData;

    if (bus == NULL || bus->dev != dev || frame == NULL ||
        (frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0u)
    {
        return;
    }

    const uint8_t length = can_dlc_to_bytes(frame->dlc);
    if (length > sizeof(((BspCanFrame *)0)->data) ||
        frame->id > CAN_STD_ID_MASK)
    {
        return;
    }

    unsigned int key = irq_lock();
    bus->rxCount++;
    bus->lastRxStdId = (uint16_t)frame->id;
    bus->lastRxDlc = length;
    if (frame->id < BSP_CAN_STD_ID_DIAG_COUNT)
    {
        bus->rxStdIdCount[frame->id]++;
    }

    const uint16_t next =
        (uint16_t)((bus->rxHead + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
    if (next == bus->rxTail)
    {
        bus->rxDropCount++;
        irq_unlock(key);
        return;
    }

    BspCanFrame *out = &bus->rxRing[bus->rxHead];
    out->bus = bus->bus;
    out->std_id = (uint16_t)frame->id;
    out->dlc = length;
    out->flags = 0u;
    if ((frame->flags & CAN_FRAME_FDF) != 0u)
    {
        out->flags |= BSP_CAN_FLAG_FD;
    }
    if ((frame->flags & CAN_FRAME_BRS) != 0u)
    {
        out->flags |= BSP_CAN_FLAG_BRS;
    }
    (void)memset(out->data, 0, sizeof(out->data));
    (void)memcpy(out->data, frame->data, length);
    barrier_dmem_fence_full();
    bus->rxHead = next;
    irq_unlock(key);

    BspCanRxNotifyFromIsr();
}

static void BspCanStateCallback(const struct device *dev,
                                enum can_state state,
                                struct can_bus_err_cnt errCount,
                                void *userData)
{
    BspCanZephyrBus *bus = (BspCanZephyrBus *)userData;

    if (bus == NULL || bus->dev != dev)
    {
        return;
    }

    unsigned int key = irq_lock();
    bus->state = state;
    bus->errCount = errCount;
    if (state == CAN_STATE_BUS_OFF)
    {
        bus->lastError = ENETUNREACH;
    }
    irq_unlock(key);
}

static void BspCanBusRefreshState(BspCanZephyrBus *bus)
{
    enum can_state state;
    struct can_bus_err_cnt errCount;

    if (bus == NULL || bus->ready == 0u || k_is_in_isr())
    {
        return;
    }

    int ret = can_get_state(bus->dev, &state, &errCount);
    unsigned int key = irq_lock();
    if (ret == 0)
    {
        bus->state = state;
        bus->errCount = errCount;
    }
    else
    {
        bus->lastError = BspCanErrEncode(ret);
    }
    irq_unlock(key);
}

static int BspCanBusStart(BspCanZephyrBus *bus)
{
    static const struct can_filter filter = {
        .id = 0u,
        .mask = 0u,
        .flags = 0u,
    };
    can_mode_t capabilities = CAN_MODE_NORMAL;
    can_mode_t mode = CAN_MODE_NORMAL;

    if (!device_is_ready(bus->dev))
    {
        return -ENODEV;
    }

    int ret = can_get_capabilities(bus->dev, &capabilities);
    if (ret != 0)
    {
        return ret;
    }
#if defined(CONFIG_CAN_FD_MODE)
    if ((capabilities & CAN_MODE_FD) != 0u)
    {
        mode |= CAN_MODE_FD;
    }
#endif

    ret = can_set_mode(bus->dev, mode);
    if (ret != 0 && ret != -EBUSY)
    {
        return ret;
    }

    bus->rxFilterId = can_add_rx_filter(bus->dev,
                                        BspCanRxCallback,
                                        bus,
                                        &filter);
    if (bus->rxFilterId < 0)
    {
        return bus->rxFilterId;
    }

    can_set_state_change_callback(bus->dev, BspCanStateCallback, bus);
    ret = can_start(bus->dev);
    if (ret != 0 && ret != -EALREADY)
    {
        can_remove_rx_filter(bus->dev, bus->rxFilterId);
        bus->rxFilterId = -1;
        return ret;
    }
    return 0;
}

void can_filter_init(void)
{
    if (!atomic_cas(&CanInitState, BSP_CAN_INIT_IDLE, BSP_CAN_INIT_RUNNING))
    {
        return;
    }

    atomic_clear(&CanFaultLocked);
    CanCompletionHead = 0u;
    CanCompletionTail = 0u;
    CanRxNextBus = 1u;
    CanRxTaskHandle = NULL;
    (void)memset(CanCompletionRing, 0, sizeof(CanCompletionRing));

    for (size_t index = 0u; index < ARRAY_SIZE(CanBuses); index++)
    {
        BspCanZephyrBus *bus = &CanBuses[index];
        const struct device *dev = bus->dev;
        const uint8_t busNumber = bus->bus;

        (void)memset(bus, 0, sizeof(*bus));
        bus->dev = dev;
        bus->bus = busNumber;
        bus->state = CAN_STATE_STOPPED;
        bus->rxFilterId = -1;

        int ret = BspCanBusStart(bus);
        bus->lastError = BspCanErrEncode(ret);
        bus->ready = (ret == 0) ? 1u : 0u;
    }

    atomic_set(&CanInitState, BSP_CAN_INIT_DONE);
}

uint8_t BspCanZephyrReady(void)
{
    if (atomic_get(&CanInitState) != BSP_CAN_INIT_DONE)
    {
        return 0u;
    }

    for (size_t index = 0u; index < ARRAY_SIZE(CanBuses); index++)
    {
        if (CanBuses[index].ready == 0u)
        {
            return 0u;
        }
    }
    return 1u;
}

void BspCanRxAttachTask(TaskHandle_t task)
{
    unsigned int key = irq_lock();
    CanRxTaskHandle = task;
    irq_unlock(key);
}

static int BspCanRxTryPopBus(BspCanZephyrBus *bus, BspCanFrame *out)
{
    int result = 0;
    unsigned int key = irq_lock();

    if (bus->rxHead != bus->rxTail)
    {
        const uint16_t tail = bus->rxTail;
        *out = bus->rxRing[tail];
        bus->rxTail =
            (uint16_t)((tail + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
        result = 1;
    }
    irq_unlock(key);
    return result;
}

int BspCanRxPop(BspCanFrame *out)
{
    if (out == NULL)
    {
        return 0;
    }

    uint8_t busNumber = CanRxNextBus;
    for (size_t checked = 0u; checked < ARRAY_SIZE(CanBuses); checked++)
    {
        BspCanZephyrBus *bus = BspCanBusGet(busNumber);
        if (bus != NULL && BspCanRxTryPopBus(bus, out) != 0)
        {
            busNumber++;
            CanRxNextBus =
                (busNumber > ARRAY_SIZE(CanBuses)) ? 1u : busNumber;
            return 1;
        }
        busNumber++;
        if (busNumber > ARRAY_SIZE(CanBuses))
        {
            busNumber = 1u;
        }
    }
    return 0;
}

uint32_t BspCanRxPending(void)
{
    uint32_t pending = 0u;
    unsigned int key = irq_lock();

    for (size_t index = 0u; index < ARRAY_SIZE(CanBuses); index++)
    {
        const BspCanZephyrBus *bus = &CanBuses[index];
        pending +=
            (uint32_t)((bus->rxHead - bus->rxTail) &
                       (BSP_CAN_RX_RING_SIZE - 1u));
    }
    irq_unlock(key);
    return pending;
}

uint32_t BspCanRxGetCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->rxCount;
}

uint32_t BspCanRxGetDropCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->rxDropCount;
}

uint32_t BspCanRxGetStdIdCount(uint8_t bus, uint16_t stdId)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL || stdId >= BSP_CAN_STD_ID_DIAG_COUNT)
    {
        return 0u;
    }
    return state->rxStdIdCount[stdId];
}

uint16_t BspCanRxGetLastStdId(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->lastRxStdId;
}

uint8_t BspCanRxGetLastDlc(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->lastRxDlc;
}

static int BspCanTxFlagsInternal(uint8_t busNumber,
                                 uint16_t stdId,
                                 const uint8_t data[8],
                                 uint8_t dlc,
                                 uint8_t flags,
                                 const BspCanTxTicket *ticket,
                                 uint8_t *tracked)
{
    const uint8_t wantsTracking = (uint8_t)(ticket != NULL || tracked != NULL);
    BspCanZephyrBus *bus = BspCanBusGet(busNumber);
    struct can_frame frame = {
        .id = stdId,
        .dlc = dlc,
        .flags = 0u,
    };

    if (tracked != NULL)
    {
        *tracked = 0u;
    }
    if (bus == NULL || data == NULL || dlc > 8u ||
        stdId > CAN_STD_ID_MASK ||
        (flags & (uint8_t)~(BSP_CAN_FLAG_FD | BSP_CAN_FLAG_BRS)) != 0u ||
        ((flags & BSP_CAN_FLAG_BRS) != 0u &&
         (flags & BSP_CAN_FLAG_FD) == 0u) ||
        (wantsTracking != 0u &&
         (tracked == NULL || BspCanTicketValid(ticket) == 0u)))
    {
        return BSP_CAN_STATUS_ERROR;
    }

    if ((flags & BSP_CAN_FLAG_FD) != 0u)
    {
#if defined(CONFIG_CAN_FD_MODE)
        frame.flags |= CAN_FRAME_FDF;
        if ((flags & BSP_CAN_FLAG_BRS) != 0u)
        {
            frame.flags |= CAN_FRAME_BRS;
        }
#else
        return BSP_CAN_STATUS_ERROR;
#endif
    }
    (void)memcpy(frame.data, data, dlc);

    /*
     * 与 BspCanFaultLock 共用提交互斥量。这样故障锁取得该锁后，任何普通帧
     * 都不可能再从“已检查”跨到 can_send() 的硬件提交步骤。
     */
    if (k_is_in_isr() || k_is_pre_kernel() ||
        k_mutex_lock(&CanTxSubmitLock, K_FOREVER) != 0)
    {
        return BSP_CAN_STATUS_ERROR;
    }

    unsigned int key = irq_lock();
    bus->txCount++;
    bus->lastTxStdId = stdId;
    bus->lastTxDlc = dlc;
    if (stdId < BSP_CAN_STD_ID_DIAG_COUNT)
    {
        bus->txStdIdCount[stdId]++;
    }

    if (atomic_get(&CanFaultLocked) != 0 ||
        bus->ready == 0u || bus->reconfiguring != 0u)
    {
        bus->lastTxStatus = BSP_CAN_STATUS_ERROR;
        bus->txFailCount++;
        irq_unlock(key);
        k_mutex_unlock(&CanTxSubmitLock);
        return BSP_CAN_STATUS_ERROR;
    }

    BspCanCompletionFlushLocked();
    BspCanZephyrTxSlot *slot =
        BspCanTxSlotReserveLocked(bus,
                                  stdId,
                                  ticket,
                                  wantsTracking);
    if (slot == NULL)
    {
        bus->lastTxStatus = BSP_CAN_STATUS_BUSY;
        bus->txFailCount++;
        irq_unlock(key);
        k_mutex_unlock(&CanTxSubmitLock);
        return BSP_CAN_STATUS_BUSY;
    }

    bus->lastTxStatus = BSP_CAN_STATUS_OK;
    irq_unlock(key);

    /*
     * K_NO_WAIT 表示邮箱不可用时立即返回；can_send 仍会短暂取得驱动内部互斥量，
     * 所以这里不能保持中断关闭。调用发生在当前任务中，没有被延后到工作队列。
     */
    int ret = can_send(bus->dev,
                       &frame,
                       K_NO_WAIT,
                       BspCanTxCallback,
                       slot);
    key = irq_lock();
    if (ret != 0)
    {
        /*
         * Zephyr 约定提交失败时不会调用完成回调，因此此处可以回收刚才保留的槽。
         */
        if (slot->state == BSP_CAN_SLOT_PENDING)
        {
            (void)memset(slot, 0, sizeof(*slot));
        }
        bus->lastError = BspCanErrEncode(ret);
        bus->lastTxStatus = BspCanStatusFromErr(ret);
        bus->txFailCount++;
        irq_unlock(key);
        k_mutex_unlock(&CanTxSubmitLock);
        return bus->lastTxStatus;
    }

    if (tracked != NULL)
    {
        *tracked = 1u;
    }
    irq_unlock(key);
    k_mutex_unlock(&CanTxSubmitLock);
    return BSP_CAN_STATUS_OK;
}

int BspCanTx(uint8_t bus,
             uint16_t stdId,
             const uint8_t data[8],
             uint8_t dlc)
{
    return BspCanTxFlags(bus, stdId, data, dlc, 0u);
}

int BspCanTxFlags(uint8_t bus,
                  uint16_t stdId,
                  const uint8_t data[8],
                  uint8_t dlc,
                  uint8_t flags)
{
    return BspCanTxFlagsInternal(bus,
                                 stdId,
                                 data,
                                 dlc,
                                 flags,
                                 NULL,
                                 NULL);
}

int BspCanTxTracked(uint8_t bus,
                    uint16_t stdId,
                    const uint8_t data[8],
                    uint8_t dlc,
                    const BspCanTxTicket *ticket,
                    uint8_t *tracked)
{
    return BspCanTxFlagsTracked(bus,
                                stdId,
                                data,
                                dlc,
                                0u,
                                ticket,
                                tracked);
}

int BspCanTxFlagsTracked(uint8_t bus,
                         uint16_t stdId,
                         const uint8_t data[8],
                         uint8_t dlc,
                         uint8_t flags,
                         const BspCanTxTicket *ticket,
                         uint8_t *tracked)
{
    return BspCanTxFlagsInternal(bus,
                                 stdId,
                                 data,
                                 dlc,
                                 flags,
                                 ticket,
                                 tracked);
}

void BspCanTxCompletionPoll(void)
{
    unsigned int key = irq_lock();
    BspCanCompletionFlushLocked();
    irq_unlock(key);
}

int BspCanTxCompletionPop(BspCanTxCompletion *out)
{
    if (out == NULL)
    {
        return 0;
    }

    unsigned int key = irq_lock();
    if (CanCompletionTail == CanCompletionHead)
    {
        irq_unlock(key);
        return 0;
    }

    const uint16_t tail = CanCompletionTail;
    *out = CanCompletionRing[tail];
    CanCompletionTail =
        (uint16_t)((tail + 1u) &
                   (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u));
    BspCanCompletionFlushLocked();
    irq_unlock(key);
    return 1;
}

uint32_t BspCanGetTxCompletionBackpressureCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->completionBackpressureCount;
}

uint32_t BspCanGetTxTerminalCount(uint8_t bus, BspCanTxResult result)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL || result <= BspCanTxResultNone ||
        result > BspCanTxResultUnknown)
    {
        return 0u;
    }
    return state->terminalCount[(uint8_t)result];
}

static void BspCanFaultInvalidatePending(void)
{
    unsigned int key = irq_lock();
    if (atomic_cas(&CanFaultLocked, 0, 1))
    {
        for (size_t busIndex = 0u;
             busIndex < ARRAY_SIZE(CanBuses);
             busIndex++)
        {
            BspCanZephyrBus *bus = &CanBuses[busIndex];
            for (size_t slotIndex = 0u;
                 slotIndex < ARRAY_SIZE(bus->txSlots);
                 slotIndex++)
            {
                BspCanZephyrTxSlot *slot = &bus->txSlots[slotIndex];
                if (slot->state == BSP_CAN_SLOT_PENDING)
                {
                    /*
                     * 公共 API 无法撤销指定邮箱。这一帧此后即使回调为成功，
                     * 对故障边界而言也不能证明它在锁定前还是锁定后上总线。
                     */
                    slot->faultInvalidated = 1u;
                }
            }
        }
    }
    irq_unlock(key);
}

void BspCanFaultLock(void)
{
    /*
     * 异常/中断上下文不能等待可能被当前异常打断的发送任务。此时立即原子
     * 锁定并标记已预约帧，随后上层直接复位。正常任务上下文才使用提交互斥量，
     * 保证本函数返回后没有普通帧越过最终 can_send() 边界。
     */
    if (k_is_in_isr() || k_is_pre_kernel())
    {
        BspCanFaultInvalidatePending();
        return;
    }
    if (k_mutex_lock(&CanTxSubmitLock, K_FOREVER) != 0)
    {
        atomic_set(&CanFaultLocked, 1);
        return;
    }
    BspCanFaultInvalidatePending();
    k_mutex_unlock(&CanTxSubmitLock);
}

uint8_t BspCanFaultLocked(void)
{
    return (atomic_get(&CanFaultLocked) != 0) ? 1u : 0u;
}

int BspCanFaultTx(uint8_t bus,
                  uint16_t stdId,
                  const uint8_t data[8],
                  uint8_t dlc)
{
    ARG_UNUSED(bus);
    ARG_UNUSED(stdId);
    ARG_UNUSED(data);
    ARG_UNUSED(dlc);

    /*
     * fail-closed：公共 can_send() 依赖驱动锁和中断回调，无法满足致命异常环境
     * 的“原始寄存器直发且有界确认”约束。不能把入队成功伪装成安全帧发送成功。
     */
    return BSP_CAN_STATUS_ERROR;
}

void BspCanFaultWaitTxIdle(void)
{
    /*
     * 公共 API 没有查询所有 TX 邮箱空闲或等待硬件完成的无锁接口。这里不阻塞，
     * 调用者必须通过 BspCanZephyrFaultTxSupported() 预先选择外部断能路径。
     */
}

uint8_t BspCanZephyrFaultTxSupported(void)
{
    return 0u;
}

const char *BspCanZephyrFaultTxReason(void)
{
    return "Zephyr CAN public API cannot guarantee raw fault-context TX";
}

int BspCanFdSetDataBitrate(uint8_t busNumber, uint32_t dataBitrate)
{
#if !defined(CONFIG_CAN_FD_MODE)
    ARG_UNUSED(busNumber);
    ARG_UNUSED(dataBitrate);
    return BSP_CAN_STATUS_ERROR;
#else
    BspCanZephyrBus *bus = BspCanBusGet(busNumber);
    can_mode_t capabilities = CAN_MODE_NORMAL;
    if (bus == NULL || dataBitrate == 0u ||
        atomic_get(&CanFaultLocked) != 0 ||
        can_get_capabilities(bus->dev, &capabilities) != 0 ||
        (capabilities & CAN_MODE_FD) == 0u)
    {
        return BSP_CAN_STATUS_ERROR;
    }

    unsigned int key = irq_lock();
    if (bus->reconfiguring != 0u)
    {
        irq_unlock(key);
        return BSP_CAN_STATUS_BUSY;
    }
    bus->reconfiguring = 1u;
    irq_unlock(key);

    int ret = can_stop(bus->dev);
    if (ret == -EALREADY)
    {
        ret = 0;
    }
    if (ret == 0)
    {
        ret = can_set_bitrate_data(bus->dev, dataBitrate);
    }
    if (ret == 0)
    {
        ret = can_set_mode(bus->dev, CAN_MODE_FD);
    }
    if (can_start(bus->dev) != 0 && ret == 0)
    {
        ret = -EIO;
    }

    key = irq_lock();
    bus->reconfiguring = 0u;
    bus->ready = (ret == 0) ? 1u : 0u;
    bus->lastError = BspCanErrEncode(ret);
    irq_unlock(key);
    return BspCanStatusFromErr(ret);
#endif
}

uint32_t BspCanGetLastError(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->lastError;
}

uint8_t BspCanGetLastTxStatus(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? BSP_CAN_STATUS_ERROR : state->lastTxStatus;
}

uint16_t BspCanGetLastTxStdId(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->lastTxStdId;
}

uint8_t BspCanGetLastTxDlc(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->lastTxDlc;
}

uint32_t BspCanGetTxCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->txCount;
}

uint32_t BspCanGetTxFailCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    return (state == NULL) ? 0u : state->txFailCount;
}

uint32_t BspCanGetTxStdIdCount(uint8_t bus, uint16_t stdId)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL || stdId >= BSP_CAN_STD_ID_DIAG_COUNT)
    {
        return 0u;
    }
    return state->txStdIdCount[stdId];
}

uint8_t BspCanGetProtocolLastErrorCode(uint8_t bus)
{
    ARG_UNUSED(bus);
    return 0u;
}

uint8_t BspCanGetProtocolDataLastErrorCode(uint8_t bus)
{
    ARG_UNUSED(bus);
    return 0u;
}

uint8_t BspCanGetProtocolActivity(uint8_t bus)
{
    ARG_UNUSED(bus);
    return 0u;
}

uint8_t BspCanGetProtocolErrorPassive(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL)
    {
        return 0u;
    }
    BspCanBusRefreshState(state);
    return (state->state == CAN_STATE_ERROR_PASSIVE) ? 1u : 0u;
}

uint8_t BspCanGetProtocolWarning(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL)
    {
        return 0u;
    }
    BspCanBusRefreshState(state);
    return (state->state == CAN_STATE_ERROR_WARNING) ? 1u : 0u;
}

uint8_t BspCanGetProtocolBusOff(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL)
    {
        return 0u;
    }
    BspCanBusRefreshState(state);
    return (state->state == CAN_STATE_BUS_OFF) ? 1u : 0u;
}

uint8_t BspCanGetTxErrorCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL)
    {
        return 0u;
    }
    BspCanBusRefreshState(state);
    return state->errCount.tx_err_cnt;
}

uint8_t BspCanGetRxErrorCount(uint8_t bus)
{
    BspCanZephyrBus *state = BspCanBusGet(bus);
    if (state == NULL)
    {
        return 0u;
    }
    BspCanBusRefreshState(state);
    return state->errCount.rx_err_cnt;
}

uint8_t BspCanGetErrorLoggingCount(uint8_t bus)
{
    ARG_UNUSED(bus);
    return 0u;
}
