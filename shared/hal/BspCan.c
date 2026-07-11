/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "BspCan.h"
#include "main.h"

#include <string.h>

#if defined(HAL_FDCAN_MODULE_ENABLED)
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
#define BSP_CAN_ERR_NONE HAL_FDCAN_ERROR_NONE
#elif defined(HAL_CAN_MODULE_ENABLED)
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define BSP_CAN_ERR_NONE HAL_CAN_ERROR_NONE
#else
#error "BspCan requires HAL_CAN_MODULE_ENABLED or HAL_FDCAN_MODULE_ENABLED"
#endif

// ===== RX ring buffers =====
#define BSP_CAN_RX_RING_SIZE 128u
#define BSP_CAN_STD_ID_DIAG_COUNT 16u
#if defined(HAL_FDCAN_MODULE_ENABLED)
#define BSP_CAN_BUS_COUNT 3u
#else
#define BSP_CAN_BUS_COUNT 2u
#endif
#define BSP_CAN_FAULT_ABORT_SPIN_LIMIT 100000u
#define BSP_CAN_FAULT_TX_SPIN_LIMIT 400000u
#define BSP_CAN_FAULT_FLUSH_SPIN_LIMIT 2000000u
#define BSP_CAN_TX_COMPLETION_RING_SIZE 128u
#define BSP_CAN_TX_TRACK_TIMEOUT_MS 50u
#define BSP_CAN_TX_ABORT_GRACE_MS 10u
#if defined(HAL_FDCAN_MODULE_ENABLED)
#define BSP_CAN_TX_SLOT_COUNT 32u
#else
#define BSP_CAN_TX_SLOT_COUNT 3u
#endif
typedef char _check_can_rx_ring_pow2[(BSP_CAN_RX_RING_SIZE & (BSP_CAN_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];
typedef char _check_can_tx_completion_ring_pow2[
    (BSP_CAN_TX_COMPLETION_RING_SIZE & (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u)) == 0u ? 1 : -1];

typedef struct
{
    BspCanTxTicket ticket;
    uint32_t queuedTick;
    uint32_t abortTick;
    uint16_t stdId;
    uint8_t active;
    uint8_t terminalResult;
    uint8_t abortRequested;
    uint8_t reserved0;
} BspCanTxTrackSlot;

static volatile uint16_t can1_rx_head = 0u;
static volatile uint16_t can1_rx_tail = 0u;
static BspCanFrame can1_rx_ring[BSP_CAN_RX_RING_SIZE];

static volatile uint16_t can2_rx_head = 0u;
static volatile uint16_t can2_rx_tail = 0u;
static BspCanFrame can2_rx_ring[BSP_CAN_RX_RING_SIZE];
static uint8_t can_rx_next_bus = 1u;

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint16_t can3_rx_head = 0u;
static volatile uint16_t can3_rx_tail = 0u;
static BspCanFrame can3_rx_ring[BSP_CAN_RX_RING_SIZE];
#endif

static volatile uint32_t can1_rx_drop = 0u;
static volatile uint32_t can2_rx_drop = 0u;
static volatile uint32_t can1_rx_count = 0u;
static volatile uint32_t can2_rx_count = 0u;
static volatile uint16_t can1_rx_last_std_id = 0u;
static volatile uint16_t can2_rx_last_std_id = 0u;
static volatile uint8_t can1_rx_last_dlc = 0u;
static volatile uint8_t can2_rx_last_dlc = 0u;
static volatile uint32_t can1_tx_count = 0u;
static volatile uint32_t can2_tx_count = 0u;
static volatile uint32_t can1_tx_fail = 0u;
static volatile uint32_t can2_tx_fail = 0u;
static volatile uint16_t can1_tx_last_std_id = 0u;
static volatile uint16_t can2_tx_last_std_id = 0u;
static volatile uint8_t can1_tx_last_dlc = 0u;
static volatile uint8_t can2_tx_last_dlc = 0u;
static volatile uint32_t can1_rx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];
static volatile uint32_t can2_rx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];
static volatile uint32_t can1_tx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];
static volatile uint32_t can2_tx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint32_t can3_rx_drop = 0u;
static volatile uint32_t can3_rx_count = 0u;
static volatile uint16_t can3_rx_last_std_id = 0u;
static volatile uint8_t can3_rx_last_dlc = 0u;
static volatile uint32_t can3_tx_count = 0u;
static volatile uint32_t can3_tx_fail = 0u;
static volatile uint16_t can3_tx_last_std_id = 0u;
static volatile uint8_t can3_tx_last_dlc = 0u;
static volatile uint32_t can3_rx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];
static volatile uint32_t can3_tx_std_id_count[BSP_CAN_STD_ID_DIAG_COUNT];
#endif

static TaskHandle_t CanRxTask_handle = NULL;
static volatile uint8_t can_fault_locked = 0u;

static volatile uint16_t can_tx_completion_head = 0u;
static volatile uint16_t can_tx_completion_tail = 0u;
static BspCanTxCompletion can_tx_completion_ring[BSP_CAN_TX_COMPLETION_RING_SIZE];
static BspCanTxTrackSlot can_tx_track[BSP_CAN_BUS_COUNT][BSP_CAN_TX_SLOT_COUNT];
static volatile uint32_t can_tx_completion_backpressure[BSP_CAN_BUS_COUNT];
static volatile uint32_t can_tx_terminal_count[BSP_CAN_BUS_COUNT][5u];
static volatile uint8_t can_tx_reconfiguring[BSP_CAN_BUS_COUNT];

static volatile uint32_t can1_last_error = BSP_CAN_ERR_NONE;
static volatile uint32_t can2_last_error = BSP_CAN_ERR_NONE;
static volatile uint8_t can1_last_tx_status = 0u;
static volatile uint8_t can2_last_tx_status = 0u;

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint32_t can3_last_error = BSP_CAN_ERR_NONE;
static volatile uint8_t can3_last_tx_status = 0u;
#endif

static void BspCanTxPollHardwareLocked(void);
static void BspCanTxPollBusHardwareLocked(uint8_t bus);
static void BspCanTxRefreshNotificationLocked(uint8_t bus);

static uint32_t BspCanIrqEnter(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

static void BspCanIrqExit(uint32_t primask)
{
    __DMB();
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static uint8_t BspCanBusValid(uint8_t bus)
{
    return (uint8_t)(bus >= 1u && bus <= (uint8_t)BSP_CAN_BUS_COUNT);
}

static uint8_t BspCanTxTicketValid(const BspCanTxTicket *ticket)
{
    return (uint8_t)(ticket != NULL &&
                     (ticket->epoch != 0u || ticket->seq != 0u));
}

static uint8_t BspCanMaskIndex(uint32_t mask, uint8_t limit, uint8_t *out)
{
    if (out == NULL || mask == 0u || (mask & (mask - 1u)) != 0u)
    {
        return 0u;
    }
    for (uint8_t index = 0u; index < limit; index++)
    {
        if ((mask & ((uint32_t)1u << index)) != 0u)
        {
            *out = index;
            return 1u;
        }
    }
    return 0u;
}

static uint8_t BspCanTxCompletionPushLocked(uint8_t bus,
                                            const BspCanTxTrackSlot *slot,
                                            uint8_t result)
{
    const uint16_t head = can_tx_completion_head;
    const uint16_t next = (uint16_t)((head + 1u) &
                                     (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u));
    BspCanTxCompletion *completion;

    if (slot == NULL || BspCanBusValid(bus) == 0u ||
        result == (uint8_t)BspCanTxResultNone ||
        next == can_tx_completion_tail)
    {
        return 0u;
    }

    completion = &can_tx_completion_ring[head];
    completion->ticket = slot->ticket;
    completion->stdId = slot->stdId;
    completion->bus = bus;
    completion->result = result;
    __DMB();
    can_tx_completion_head = next;
    return 1u;
}

static void BspCanTxTrackFinishLocked(uint8_t bus,
                                      uint8_t slot_index,
                                      uint8_t result)
{
    BspCanTxTrackSlot *slot;

    if (BspCanBusValid(bus) == 0u || slot_index >= BSP_CAN_TX_SLOT_COUNT ||
        result == (uint8_t)BspCanTxResultNone ||
        result > (uint8_t)BspCanTxResultUnknown)
    {
        return;
    }

    slot = &can_tx_track[bus - 1u][slot_index];
    if (slot->active == 0u || slot->terminalResult != (uint8_t)BspCanTxResultNone)
    {
        return;
    }

    can_tx_terminal_count[bus - 1u][result]++;

    if (BspCanTxCompletionPushLocked(bus, slot, result) != 0u)
    {
        slot->active = 0u;
    }
    else
    {
        slot->terminalResult = result;
        can_tx_completion_backpressure[bus - 1u]++;
    }
    BspCanTxRefreshNotificationLocked(bus);
}

static void BspCanTxFlushDeferredBusLocked(uint8_t bus)
{
    if (BspCanBusValid(bus) == 0u)
    {
        return;
    }
    for (uint8_t slot_index = 0u; slot_index < (uint8_t)BSP_CAN_TX_SLOT_COUNT; slot_index++)
    {
        BspCanTxTrackSlot *slot = &can_tx_track[bus - 1u][slot_index];

        if (slot->active == 0u ||
            slot->terminalResult == (uint8_t)BspCanTxResultNone)
        {
            continue;
        }
        if (BspCanTxCompletionPushLocked(bus,
                                         slot,
                                         slot->terminalResult) == 0u)
        {
            return;
        }
        slot->active = 0u;
        slot->terminalResult = (uint8_t)BspCanTxResultNone;
        BspCanTxRefreshNotificationLocked(bus);
    }
}

static void BspCanTxFlushDeferredLocked(void)
{
    for (uint8_t bus = 1u; bus <= (uint8_t)BSP_CAN_BUS_COUNT; bus++)
    {
        BspCanTxFlushDeferredBusLocked(bus);
    }
}

static uint8_t BspCanTxBusDeferredLocked(uint8_t bus)
{
    if (BspCanBusValid(bus) == 0u)
    {
        return 1u;
    }
    for (uint8_t slot_index = 0u; slot_index < (uint8_t)BSP_CAN_TX_SLOT_COUNT; slot_index++)
    {
        const BspCanTxTrackSlot *slot = &can_tx_track[bus - 1u][slot_index];

        if (slot->active != 0u &&
            slot->terminalResult != (uint8_t)BspCanTxResultNone)
        {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t BspCanTxTrackInstallLocked(uint8_t bus,
                                          uint8_t slot_index,
                                          uint16_t std_id,
                                          const BspCanTxTicket *ticket)
{
    BspCanTxTrackSlot *slot;

    if (BspCanBusValid(bus) == 0u || slot_index >= BSP_CAN_TX_SLOT_COUNT ||
        BspCanTxTicketValid(ticket) == 0u)
    {
        return 0u;
    }

    slot = &can_tx_track[bus - 1u][slot_index];
    if (slot->active != 0u)
    {
        return 0u;
    }
    slot->ticket = *ticket;
    slot->queuedTick = HAL_GetTick();
    slot->abortTick = 0u;
    slot->stdId = std_id;
    slot->terminalResult = (uint8_t)BspCanTxResultNone;
    slot->abortRequested = 0u;
    __DMB();
    slot->active = 1u;
    return 1u;
}

static void BspCanTxInvalidateBusLocked(uint8_t bus, uint8_t result)
{
    if (BspCanBusValid(bus) == 0u)
    {
        return;
    }
    for (uint8_t slot_index = 0u; slot_index < (uint8_t)BSP_CAN_TX_SLOT_COUNT; slot_index++)
    {
        BspCanTxTrackFinishLocked(bus, slot_index, result);
    }
}

static void BspCanResetState(void)
{
    can_fault_locked = 0u;
    can_tx_completion_head = 0u;
    can_tx_completion_tail = 0u;
    (void)memset(can_tx_completion_ring, 0, sizeof(can_tx_completion_ring));
    (void)memset(can_tx_track, 0, sizeof(can_tx_track));
    (void)memset((void *)can_tx_completion_backpressure,
                 0,
                 sizeof(can_tx_completion_backpressure));
    (void)memset((void *)can_tx_terminal_count,
                 0,
                 sizeof(can_tx_terminal_count));
    (void)memset((void *)can_tx_reconfiguring,
                 0,
                 sizeof(can_tx_reconfiguring));
    can1_rx_head = 0u;
    can1_rx_tail = 0u;
    can2_rx_head = 0u;
    can2_rx_tail = 0u;
    can_rx_next_bus = 1u;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    can3_rx_head = 0u;
    can3_rx_tail = 0u;
#endif

    can1_rx_drop = 0u;
    can2_rx_drop = 0u;
    can1_rx_count = 0u;
    can2_rx_count = 0u;
    can1_rx_last_std_id = 0u;
    can2_rx_last_std_id = 0u;
    can1_rx_last_dlc = 0u;
    can2_rx_last_dlc = 0u;
    can1_tx_count = 0u;
    can2_tx_count = 0u;
    can1_tx_fail = 0u;
    can2_tx_fail = 0u;
    can1_tx_last_std_id = 0u;
    can2_tx_last_std_id = 0u;
    can1_tx_last_dlc = 0u;
    can2_tx_last_dlc = 0u;
    for (uint8_t i = 0u; i < BSP_CAN_STD_ID_DIAG_COUNT; i++)
    {
        can1_rx_std_id_count[i] = 0u;
        can2_rx_std_id_count[i] = 0u;
        can1_tx_std_id_count[i] = 0u;
        can2_tx_std_id_count[i] = 0u;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    can3_rx_drop = 0u;
    can3_rx_count = 0u;
    can3_rx_last_std_id = 0u;
    can3_rx_last_dlc = 0u;
    can3_tx_count = 0u;
    can3_tx_fail = 0u;
    can3_tx_last_std_id = 0u;
    can3_tx_last_dlc = 0u;
    for (uint8_t i = 0u; i < BSP_CAN_STD_ID_DIAG_COUNT; i++)
    {
        can3_rx_std_id_count[i] = 0u;
        can3_tx_std_id_count[i] = 0u;
    }
#endif

    can1_last_error = BSP_CAN_ERR_NONE;
    can2_last_error = BSP_CAN_ERR_NONE;
    can1_last_tx_status = 0u;
    can2_last_tx_status = 0u;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    can3_last_error = BSP_CAN_ERR_NONE;
    can3_last_tx_status = 0u;
#endif
}

static void BspCanRxPushCommon(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8], uint8_t flags)
{
    volatile uint16_t *head = NULL;
    volatile uint16_t *tail = NULL;
    BspCanFrame *ring = NULL;
    volatile uint32_t *drop = NULL;
    volatile uint32_t *count = NULL;
    volatile uint32_t *id_count = NULL;
    volatile uint16_t *last_std_id = NULL;
    volatile uint8_t *last_dlc = NULL;

    if (data == NULL || dlc > 8u)
    {
        return;
    }

    if (bus == 1u)
    {
        head = &can1_rx_head;
        tail = &can1_rx_tail;
        ring = can1_rx_ring;
        drop = &can1_rx_drop;
        count = &can1_rx_count;
        id_count = can1_rx_std_id_count;
        last_std_id = &can1_rx_last_std_id;
        last_dlc = &can1_rx_last_dlc;
    }
    else if (bus == 2u)
    {
        head = &can2_rx_head;
        tail = &can2_rx_tail;
        ring = can2_rx_ring;
        drop = &can2_rx_drop;
        count = &can2_rx_count;
        id_count = can2_rx_std_id_count;
        last_std_id = &can2_rx_last_std_id;
        last_dlc = &can2_rx_last_dlc;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else if (bus == 3u)
    {
        head = &can3_rx_head;
        tail = &can3_rx_tail;
        ring = can3_rx_ring;
        drop = &can3_rx_drop;
        count = &can3_rx_count;
        id_count = can3_rx_std_id_count;
        last_std_id = &can3_rx_last_std_id;
        last_dlc = &can3_rx_last_dlc;
    }
#endif
    else
    {
        return;
    }

    (*count)++;
    if (std_id < BSP_CAN_STD_ID_DIAG_COUNT && id_count != NULL)
    {
        id_count[std_id]++;
    }
    *last_std_id = std_id;
    *last_dlc = dlc;

    const uint16_t h = *head;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
    if (next == *tail)
    {
        (*drop)++;
        return;
    }

    BspCanFrame *dst = &ring[h];
    dst->bus = bus;
    dst->dlc = dlc;
    dst->flags = flags;
    dst->std_id = std_id;
    for (uint8_t i = 0u; i < (uint8_t)sizeof(dst->data); i++)
    {
        dst->data[i] = data[i];
    }

    *head = next;
}

static void BspCanRxNotifyFromIsr(void)
{
    if (CanRxTask_handle == NULL)
    {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(CanRxTask_handle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

#if defined(HAL_FDCAN_MODULE_ENABLED)
static uint32_t BspCanFdcanEncodeDlc(uint8_t dlc)
{
    switch (dlc)
    {
    case 0u:
        return FDCAN_DLC_BYTES_0;
    case 1u:
        return FDCAN_DLC_BYTES_1;
    case 2u:
        return FDCAN_DLC_BYTES_2;
    case 3u:
        return FDCAN_DLC_BYTES_3;
    case 4u:
        return FDCAN_DLC_BYTES_4;
    case 5u:
        return FDCAN_DLC_BYTES_5;
    case 6u:
        return FDCAN_DLC_BYTES_6;
    case 7u:
        return FDCAN_DLC_BYTES_7;
    case 8u:
    default:
        return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t BspCanFdcanDecodeDlc(uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0:
        return 0u;
    case FDCAN_DLC_BYTES_1:
        return 1u;
    case FDCAN_DLC_BYTES_2:
        return 2u;
    case FDCAN_DLC_BYTES_3:
        return 3u;
    case FDCAN_DLC_BYTES_4:
        return 4u;
    case FDCAN_DLC_BYTES_5:
        return 5u;
    case FDCAN_DLC_BYTES_6:
        return 6u;
    case FDCAN_DLC_BYTES_7:
        return 7u;
    case FDCAN_DLC_BYTES_8:
        return 8u;
    case FDCAN_DLC_BYTES_12:
        return 12u;
    case FDCAN_DLC_BYTES_16:
        return 16u;
    case FDCAN_DLC_BYTES_20:
        return 20u;
    case FDCAN_DLC_BYTES_24:
        return 24u;
    case FDCAN_DLC_BYTES_32:
        return 32u;
    case FDCAN_DLC_BYTES_48:
        return 48u;
    case FDCAN_DLC_BYTES_64:
        return 64u;
    default:
        return 8u;
    }
}

static FDCAN_HandleTypeDef *BspCanFdcanHandle(uint8_t bus)
{
    if (bus == 1u)
    {
        return &hfdcan1;
    }
    if (bus == 2u)
    {
        return &hfdcan2;
    }
    if (bus == 3u)
    {
        return &hfdcan3;
    }
    return NULL;
}

static volatile uint32_t *BspCanFdcanLastError(uint8_t bus)
{
    if (bus == 1u)
    {
        return &can1_last_error;
    }
    if (bus == 2u)
    {
        return &can2_last_error;
    }
    if (bus == 3u)
    {
        return &can3_last_error;
    }
    return NULL;
}

static uint8_t BspCanFdcanBus(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
    {
        return 1u;
    }
    if (hfdcan == &hfdcan2)
    {
        return 2u;
    }
    if (hfdcan == &hfdcan3)
    {
        return 3u;
    }
    return 0u;
}

static void BspCanTxRefreshNotificationLocked(uint8_t bus)
{
    (void)bus;
    /* FDCAN 在初始化时监视全部 32 个槽，软件 active 表过滤普通帧。 */
}

static void BspCanFdcanFinishMaskLocked(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t buffer_mask)
{
    const uint8_t bus = BspCanFdcanBus(hfdcan);
    uint32_t success_mask;
    uint32_t failed_mask;

    if (bus == 0u || buffer_mask == 0u)
    {
        return;
    }

    /* 取消过晚时 TXBTO/TXBCF 可同时置位，真实成功必须优先。 */
    success_mask = buffer_mask & hfdcan->Instance->TXBTO;
    failed_mask = buffer_mask & hfdcan->Instance->TXBCF & ~success_mask;
    for (uint8_t slot_index = 0u; slot_index < 32u; slot_index++)
    {
        const uint32_t mask = (uint32_t)1u << slot_index;

        if ((success_mask & mask) != 0u)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultComplete);
        }
        else if ((failed_mask & mask) != 0u)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultFailed);
        }
    }
}

static void BspCanTxPollHardwareLocked(void)
{
    for (uint8_t bus = 1u; bus <= (uint8_t)BSP_CAN_BUS_COUNT; bus++)
    {
        BspCanTxPollBusHardwareLocked(bus);
    }
}

static void BspCanTxPollBusHardwareLocked(uint8_t bus)
{
    FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
    uint32_t now;

    if (hfdcan == NULL)
    {
        return;
    }
    {
        const uint32_t terminal_mask = hfdcan->Instance->TXBTO |
                                       hfdcan->Instance->TXBCF;

        BspCanFdcanFinishMaskLocked(hfdcan, terminal_mask);
    }
    now = HAL_GetTick();
    for (uint8_t slot_index = 0u; slot_index < 32u; slot_index++)
    {
        BspCanTxTrackSlot *slot = &can_tx_track[bus - 1u][slot_index];
        const uint32_t mask = (uint32_t)1u << slot_index;

        if (slot->active == 0u ||
            slot->terminalResult != (uint8_t)BspCanTxResultNone ||
            (uint32_t)(now - slot->queuedTick) <= BSP_CAN_TX_TRACK_TIMEOUT_MS)
        {
            continue;
        }
        if ((hfdcan->Instance->TXBRP & mask) == 0u)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultUnknown);
        }
        else if (slot->abortRequested == 0u)
        {
            slot->abortRequested = 1u;
            slot->abortTick = now;
            (void)HAL_FDCAN_AbortTxRequest(hfdcan, mask);
        }
        else if ((uint32_t)(now - slot->abortTick) > BSP_CAN_TX_ABORT_GRACE_MS)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultUnknown);
        }
    }
}

static uint8_t BspCanFdcanSetTiming(FDCAN_HandleTypeDef *hfdcan, uint32_t data_bitrate)
{
    uint32_t prescaler = 1u;
    uint32_t tseg1 = 18u;
    uint32_t tseg2 = 5u;

    if (hfdcan == NULL)
    {
        return 0u;
    }

    switch (data_bitrate)
    {
    case 5000000u:
        prescaler = 1u;
        tseg1 = 13u;
        tseg2 = 2u;
        break;
    case 4000000u:
        prescaler = 1u;
        tseg1 = 14u;
        tseg2 = 5u;
        break;
    case 3200000u:
        prescaler = 1u;
        tseg1 = 19u;
        tseg2 = 5u;
        break;
    case 2500000u:
        prescaler = 1u;
        tseg1 = 25u;
        tseg2 = 6u;
        break;
    case 2000000u:
        prescaler = 1u;
        tseg1 = 29u;
        tseg2 = 10u;
        break;
    case 1000000u:
        prescaler = 2u;
        tseg1 = 29u;
        tseg2 = 10u;
        break;
    default:
        return 0u;
    }

    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.DataPrescaler = prescaler;
    hfdcan->Init.DataSyncJumpWidth = tseg2;
    hfdcan->Init.DataTimeSeg1 = tseg1;
    hfdcan->Init.DataTimeSeg2 = tseg2;
    return 1u;
}

static void BspCanFdcanInitBus(FDCAN_HandleTypeDef *hfdcan, volatile uint32_t *last_error)
{
    FDCAN_FilterTypeDef cfg = {0};

    if (hfdcan == NULL)
    {
        return;
    }

    cfg.IdType = FDCAN_STANDARD_ID;
    cfg.FilterIndex = 0u;
    cfg.FilterType = FDCAN_FILTER_MASK;
    cfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    cfg.FilterID1 = 0u;
    cfg.FilterID2 = 0u;
    cfg.RxBufferIndex = 0u;
    cfg.IsCalibrationMsg = 0u;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &cfg) != HAL_OK && last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK &&
        last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_TX_COMPLETE |
                                           FDCAN_IT_TX_ABORT_COMPLETE,
                                       0xFFFFFFFFu) != HAL_OK &&
        last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK && last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
}

static HAL_StatusTypeDef BspCanTxWithRetry(FDCAN_HandleTypeDef *hfdcan,
                                           FDCAN_TxHeaderTypeDef *header,
                                           const uint8_t data[8],
                                           uint32_t *buffer_mask)
{
    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, (uint8_t *)data);

    if (buffer_mask != NULL)
    {
        *buffer_mask = 0u;
    }
    if (ret == HAL_BUSY && HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u)
    {
        ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, (uint8_t *)data);
    }
    if (ret == HAL_OK && buffer_mask != NULL)
    {
        *buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
    }
    return ret;
}
#elif defined(HAL_CAN_MODULE_ENABLED)
static CAN_HandleTypeDef *BspCanBxHandle(uint8_t bus)
{
    if (bus == 1u)
    {
        return &hcan1;
    }
    if (bus == 2u)
    {
        return &hcan2;
    }
    return NULL;
}

static uint8_t BspCanBxBus(const CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        return 1u;
    }
    if (hcan == &hcan2)
    {
        return 2u;
    }
    return 0u;
}

static uint32_t BspCanBxRqcpMask(uint8_t slot_index)
{
    if (slot_index == 0u)
    {
        return CAN_TSR_RQCP0;
    }
    if (slot_index == 1u)
    {
        return CAN_TSR_RQCP1;
    }
    if (slot_index == 2u)
    {
        return CAN_TSR_RQCP2;
    }
    return 0u;
}

static uint32_t BspCanBxTmeMask(uint8_t slot_index)
{
    if (slot_index == 0u)
    {
        return CAN_TSR_TME0;
    }
    if (slot_index == 1u)
    {
        return CAN_TSR_TME1;
    }
    if (slot_index == 2u)
    {
        return CAN_TSR_TME2;
    }
    return 0u;
}

static uint32_t BspCanBxMailboxMask(uint8_t slot_index)
{
    if (slot_index == 0u)
    {
        return CAN_TX_MAILBOX0;
    }
    if (slot_index == 1u)
    {
        return CAN_TX_MAILBOX1;
    }
    if (slot_index == 2u)
    {
        return CAN_TX_MAILBOX2;
    }
    return 0u;
}

static void BspCanBxClearRqcp(CAN_HandleTypeDef *hcan, uint8_t slot_index)
{
    if (hcan == NULL)
    {
        return;
    }
    if (slot_index == 0u)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);
    }
    else if (slot_index == 1u)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP1);
    }
    else if (slot_index == 2u)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP2);
    }
}

static void BspCanTxRefreshNotificationLocked(uint8_t bus)
{
    CAN_HandleTypeDef *hcan = BspCanBxHandle(bus);
    uint8_t active = 0u;

    if (hcan == NULL)
    {
        return;
    }
    for (uint8_t slot_index = 0u; slot_index < 3u; slot_index++)
    {
        if (can_tx_track[bus - 1u][slot_index].active != 0u)
        {
            active = 1u;
            break;
        }
    }
    if (active != 0u)
    {
        SET_BIT(hcan->Instance->IER, CAN_IT_TX_MAILBOX_EMPTY);
    }
    else
    {
        CLEAR_BIT(hcan->Instance->IER, CAN_IT_TX_MAILBOX_EMPTY);
    }
}

static void BspCanBxDrainHandleLocked(CAN_HandleTypeDef *hcan)
{
    const uint8_t bus = BspCanBxBus(hcan);
    const uint32_t tsr = (hcan != NULL) ? hcan->Instance->TSR : 0u;

    if (bus == 0u)
    {
        return;
    }

    for (uint8_t slot_index = 0u; slot_index < 3u; slot_index++)
    {
        uint32_t rqcp = CAN_TSR_RQCP0;
        uint32_t txok = CAN_TSR_TXOK0;
        uint32_t alst = CAN_TSR_ALST0;
        uint32_t terr = CAN_TSR_TERR0;
        uint32_t clear_flag = CAN_FLAG_RQCP0;
        uint32_t tx_error = 0u;
        uint8_t result;

        if (slot_index == 1u)
        {
            rqcp = CAN_TSR_RQCP1;
            txok = CAN_TSR_TXOK1;
            alst = CAN_TSR_ALST1;
            terr = CAN_TSR_TERR1;
            clear_flag = CAN_FLAG_RQCP1;
        }
        else if (slot_index == 2u)
        {
            rqcp = CAN_TSR_RQCP2;
            txok = CAN_TSR_TXOK2;
            alst = CAN_TSR_ALST2;
            terr = CAN_TSR_TERR2;
            clear_flag = CAN_FLAG_RQCP2;
        }
        if ((tsr & rqcp) == 0u)
        {
            continue;
        }

        if ((tsr & txok) != 0u)
        {
            result = (uint8_t)BspCanTxResultComplete;
        }
        else if ((tsr & alst) != 0u)
        {
            result = (uint8_t)BspCanTxResultFailed;
            tx_error = (slot_index == 0u) ? HAL_CAN_ERROR_TX_ALST0 :
                       ((slot_index == 1u) ? HAL_CAN_ERROR_TX_ALST1 :
                                             HAL_CAN_ERROR_TX_ALST2);
        }
        else if ((tsr & terr) != 0u)
        {
            result = (uint8_t)BspCanTxResultFailed;
            tx_error = (slot_index == 0u) ? HAL_CAN_ERROR_TX_TERR0 :
                       ((slot_index == 1u) ? HAL_CAN_ERROR_TX_TERR1 :
                                             HAL_CAN_ERROR_TX_TERR2);
        }
        else
        {
            result = (uint8_t)BspCanTxResultAborted;
        }
        if (tx_error != 0u)
        {
            if (bus == 1u)
            {
                can1_last_error |= tx_error;
            }
            else
            {
                can2_last_error |= tx_error;
            }
        }
        __HAL_CAN_CLEAR_FLAG(hcan, clear_flag);
        BspCanTxTrackFinishLocked(bus, slot_index, result);
    }
    BspCanTxRefreshNotificationLocked(bus);
}

static void BspCanTxPollHardwareLocked(void)
{
    BspCanTxPollBusHardwareLocked(1u);
    BspCanTxPollBusHardwareLocked(2u);
}

static void BspCanTxPollBusHardwareLocked(uint8_t bus)
{
    CAN_HandleTypeDef *hcan = BspCanBxHandle(bus);
    uint32_t now;

    if (hcan == NULL)
    {
        return;
    }
    BspCanBxDrainHandleLocked(hcan);
    now = HAL_GetTick();
    for (uint8_t slot_index = 0u; slot_index < 3u; slot_index++)
    {
        BspCanTxTrackSlot *slot = &can_tx_track[bus - 1u][slot_index];

        if (slot->active == 0u ||
            slot->terminalResult != (uint8_t)BspCanTxResultNone ||
            (uint32_t)(now - slot->queuedTick) <= BSP_CAN_TX_TRACK_TIMEOUT_MS)
        {
            continue;
        }
        if ((hcan->Instance->TSR & BspCanBxTmeMask(slot_index)) != 0u)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultUnknown);
        }
        else if (slot->abortRequested == 0u)
        {
            slot->abortRequested = 1u;
            slot->abortTick = now;
            (void)HAL_CAN_AbortTxRequest(hcan,
                                         BspCanBxMailboxMask(slot_index));
        }
        else if ((uint32_t)(now - slot->abortTick) > BSP_CAN_TX_ABORT_GRACE_MS)
        {
            BspCanTxTrackFinishLocked(bus,
                                      slot_index,
                                      (uint8_t)BspCanTxResultUnknown);
        }
    }
}

static void BspCanInitBus(CAN_HandleTypeDef *hcan, uint32_t filter_bank, uint32_t slave_start_bank,
                              volatile uint32_t *last_error)
{
    CAN_FilterTypeDef cfg = {0};

    if (hcan == NULL)
    {
        return;
    }

    cfg.FilterActivation = ENABLE;
    cfg.FilterMode = CAN_FILTERMODE_IDMASK;
    cfg.FilterScale = CAN_FILTERSCALE_32BIT;
    cfg.FilterIdHigh = 0x0000;
    cfg.FilterIdLow = 0x0000;
    cfg.FilterMaskIdHigh = 0x0000;
    cfg.FilterMaskIdLow = 0x0000;
    cfg.FilterBank = filter_bank;
    cfg.FilterFIFOAssignment = CAN_RX_FIFO0;
    cfg.SlaveStartFilterBank = slave_start_bank;

    if (HAL_CAN_ConfigFilter(hcan, &cfg) != HAL_OK && last_error != NULL)
    {
        *last_error = HAL_CAN_GetError(hcan);
    }
    if (HAL_CAN_Start(hcan) != HAL_OK && last_error != NULL)
    {
        *last_error = HAL_CAN_GetError(hcan);
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK &&
        last_error != NULL)
    {
        *last_error = HAL_CAN_GetError(hcan);
    }
}

static HAL_StatusTypeDef BspCanTxWithRetry(CAN_HandleTypeDef *hcan,
                                           CAN_TxHeaderTypeDef *header,
                                           const uint8_t data[8],
                                           uint32_t *mailbox)
{
    uint32_t local_mailbox = 0u;
    HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(hcan,
                                                 header,
                                                 (uint8_t *)data,
                                                 &local_mailbox);

    if (mailbox != NULL)
    {
        *mailbox = 0u;
    }
    if (ret == HAL_BUSY && HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u)
    {
        ret = HAL_CAN_AddTxMessage(hcan,
                                  header,
                                  (uint8_t *)data,
                                  &local_mailbox);
    }
    if (ret == HAL_OK && mailbox != NULL)
    {
        *mailbox = local_mailbox;
    }
    return ret;
}
#endif

void can_filter_init(void)
{
    BspCanResetState();

#if defined(HAL_FDCAN_MODULE_ENABLED)
    BspCanFdcanInitBus(&hfdcan1, &can1_last_error);
    BspCanFdcanInitBus(&hfdcan2, &can2_last_error);
    BspCanFdcanInitBus(&hfdcan3, &can3_last_error);
#elif defined(HAL_CAN_MODULE_ENABLED)
    BspCanInitBus(&hcan1, 0u, 14u, &can1_last_error);
    BspCanInitBus(&hcan2, 14u, 14u, &can2_last_error);
#endif
}

#if defined(HAL_FDCAN_MODULE_ENABLED)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[64] = {0};
    uint8_t dlc = 0u;
    uint8_t flags = 0u;
    uint8_t bus = 0u;

    if (hfdcan == NULL || (rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
    {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }
    if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }
    dlc = BspCanFdcanDecodeDlc(rx_header.DataLength);
    if (dlc > 8u)
    {
        return;
    }
    if (rx_header.FDFormat == FDCAN_FD_CAN)
    {
        flags |= BSP_CAN_FLAG_FD;
    }
    if (rx_header.BitRateSwitch == FDCAN_BRS_ON)
    {
        flags |= BSP_CAN_FLAG_BRS;
    }

    if (hfdcan == &hfdcan1)
    {
        bus = 1u;
        can1_last_error = HAL_FDCAN_GetError(&hfdcan1);
    }
    else if (hfdcan == &hfdcan2)
    {
        bus = 2u;
        can2_last_error = HAL_FDCAN_GetError(&hfdcan2);
    }
    else if (hfdcan == &hfdcan3)
    {
        bus = 3u;
        can3_last_error = HAL_FDCAN_GetError(&hfdcan3);
    }
    else
    {
        return;
    }

    BspCanRxPushCommon(bus, (uint16_t)rx_header.Identifier, dlc, rx_data, flags);
    BspCanRxNotifyFromIsr();
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t buffer_indexes)
{
    const uint32_t primask = BspCanIrqEnter();

    BspCanFdcanFinishMaskLocked(hfdcan, buffer_indexes);
    BspCanIrqExit(primask);
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan,
                                     uint32_t buffer_indexes)
{
    const uint32_t primask = BspCanIrqEnter();

    BspCanFdcanFinishMaskLocked(hfdcan, buffer_indexes);
    BspCanIrqExit(primask);
}
#elif defined(HAL_CAN_MODULE_ENABLED)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};
    uint8_t bus = 0u;

    if (hcan == NULL)
    {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    if (hcan == &hcan1)
    {
        bus = 1u;
        can1_last_error = HAL_CAN_GetError(&hcan1);
    }
    else if (hcan == &hcan2)
    {
        bus = 2u;
        can2_last_error = HAL_CAN_GetError(&hcan2);
    }
    else
    {
        return;
    }

    BspCanRxPushCommon(bus, (uint16_t)rx_header.StdId, (uint8_t)rx_header.DLC, rx_data, 0u);
    BspCanRxNotifyFromIsr();
}

static void BspCanBxFinishCallback(CAN_HandleTypeDef *hcan,
                                   uint8_t slot_index,
                                   uint8_t result)
{
    const uint8_t bus = BspCanBxBus(hcan);
    const uint32_t primask = BspCanIrqEnter();

    BspCanTxTrackFinishLocked(bus, slot_index, result);
    BspCanIrqExit(primask);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 0u, (uint8_t)BspCanTxResultComplete);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 1u, (uint8_t)BspCanTxResultComplete);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 2u, (uint8_t)BspCanTxResultComplete);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 0u, (uint8_t)BspCanTxResultAborted);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 1u, (uint8_t)BspCanTxResultAborted);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    BspCanBxFinishCallback(hcan, 2u, (uint8_t)BspCanTxResultAborted);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    const uint32_t tx_error_mask = HAL_CAN_ERROR_TX_ALST0 |
                                   HAL_CAN_ERROR_TX_TERR0 |
                                   HAL_CAN_ERROR_TX_ALST1 |
                                   HAL_CAN_ERROR_TX_TERR1 |
                                   HAL_CAN_ERROR_TX_ALST2 |
                                   HAL_CAN_ERROR_TX_TERR2;
    const uint32_t errors = (hcan != NULL) ? hcan->ErrorCode : 0u;
    const uint8_t bus = BspCanBxBus(hcan);
    const uint32_t primask = BspCanIrqEnter();

    if (bus == 1u)
    {
        can1_last_error = errors;
    }
    else if (bus == 2u)
    {
        can2_last_error = errors;
    }
    if ((errors & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0)) != 0u)
    {
        BspCanTxTrackFinishLocked(bus, 0u, (uint8_t)BspCanTxResultFailed);
    }
    if ((errors & (HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1)) != 0u)
    {
        BspCanTxTrackFinishLocked(bus, 1u, (uint8_t)BspCanTxResultFailed);
    }
    if ((errors & (HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2)) != 0u)
    {
        BspCanTxTrackFinishLocked(bus, 2u, (uint8_t)BspCanTxResultFailed);
    }
    if (hcan != NULL)
    {
        hcan->ErrorCode &= ~tx_error_mask;
    }
    BspCanIrqExit(primask);
}
#endif

void BspCanRxAttachTask(TaskHandle_t task)
{
    CanRxTask_handle = task;
}

static int BspCanRxTryPopBus(uint8_t bus, BspCanFrame *out)
{
    volatile uint16_t *head = NULL;
    volatile uint16_t *tail = NULL;
    BspCanFrame *ring = NULL;

    if (bus == 1u)
    {
        head = &can1_rx_head;
        tail = &can1_rx_tail;
        ring = can1_rx_ring;
    }
    else if (bus == 2u)
    {
        head = &can2_rx_head;
        tail = &can2_rx_tail;
        ring = can2_rx_ring;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else if (bus == 3u)
    {
        head = &can3_rx_head;
        tail = &can3_rx_tail;
        ring = can3_rx_ring;
    }
#endif
    else
    {
        return 0;
    }

    if (*head != *tail)
    {
        const uint16_t current_tail = *tail;
        *out = ring[current_tail];
        *tail = (uint16_t)((current_tail + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
        return 1;
    }

    return 0;
}

int BspCanRxPop(BspCanFrame *out)
{
    uint8_t bus;

    if (out == NULL)
    {
        return 0;
    }

    bus = can_rx_next_bus;
    for (uint8_t checked = 0u; checked < BSP_CAN_BUS_COUNT; checked++)
    {
        if (BspCanRxTryPopBus(bus, out) != 0)
        {
            bus++;
            can_rx_next_bus = (bus > BSP_CAN_BUS_COUNT) ? 1u : bus;
            return 1;
        }

        bus++;
        if (bus > BSP_CAN_BUS_COUNT)
        {
            bus = 1u;
        }
    }

    return 0;
}

uint32_t BspCanRxPending(void)
{
    const uint16_t can1_head = can1_rx_head;
    const uint16_t can1_tail = can1_rx_tail;
    const uint16_t can2_head = can2_rx_head;
    const uint16_t can2_tail = can2_rx_tail;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    const uint16_t can3_head = can3_rx_head;
    const uint16_t can3_tail = can3_rx_tail;
#endif

    const uint32_t can1_used = (uint32_t)((can1_head - can1_tail) & (BSP_CAN_RX_RING_SIZE - 1u));
    const uint32_t can2_used = (uint32_t)((can2_head - can2_tail) & (BSP_CAN_RX_RING_SIZE - 1u));
#if defined(HAL_FDCAN_MODULE_ENABLED)
    const uint32_t can3_used = (uint32_t)((can3_head - can3_tail) & (BSP_CAN_RX_RING_SIZE - 1u));
    return can1_used + can2_used + can3_used;
#else
    return can1_used + can2_used;
#endif
}

uint32_t BspCanRxGetDropCount(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_rx_drop;
    case 2u:
        return can2_rx_drop;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_rx_drop;
#endif
    default:
        return 0u;
    }
}

uint32_t BspCanRxGetCount(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_rx_count;
    case 2u:
        return can2_rx_count;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_rx_count;
#endif
    default:
        return 0u;
    }
}

uint32_t BspCanRxGetStdIdCount(uint8_t bus, uint16_t std_id)
{
    if (std_id >= BSP_CAN_STD_ID_DIAG_COUNT)
    {
        return 0u;
    }
    switch (bus)
    {
    case 1u:
        return can1_rx_std_id_count[std_id];
    case 2u:
        return can2_rx_std_id_count[std_id];
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_rx_std_id_count[std_id];
#endif
    default:
        return 0u;
    }
}

uint16_t BspCanRxGetLastStdId(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_rx_last_std_id;
    case 2u:
        return can2_rx_last_std_id;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_rx_last_std_id;
#endif
    default:
        return 0u;
    }
}

uint8_t BspCanRxGetLastDlc(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_rx_last_dlc;
    case 2u:
        return can2_rx_last_dlc;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_rx_last_dlc;
#endif
    default:
        return 0u;
    }
}

void BspCanTxCompletionPoll(void)
{
    const uint32_t primask = BspCanIrqEnter();

    BspCanTxFlushDeferredLocked();
    BspCanTxPollHardwareLocked();
    BspCanTxFlushDeferredLocked();
    BspCanIrqExit(primask);
}

int BspCanTxCompletionPop(BspCanTxCompletion *out)
{
    uint16_t tail;
    uint32_t primask;

    if (out == NULL)
    {
        return 0;
    }

    primask = BspCanIrqEnter();
    tail = can_tx_completion_tail;
    if (tail == can_tx_completion_head)
    {
        BspCanIrqExit(primask);
        return 0;
    }

    *out = can_tx_completion_ring[tail];
    can_tx_completion_tail = (uint16_t)((tail + 1u) &
                                        (BSP_CAN_TX_COMPLETION_RING_SIZE - 1u));
    __DMB();
    BspCanIrqExit(primask);
    return 1;
}

uint32_t BspCanGetTxCompletionBackpressureCount(uint8_t bus)
{
    if (BspCanBusValid(bus) == 0u)
    {
        return 0u;
    }
    return can_tx_completion_backpressure[bus - 1u];
}

uint32_t BspCanGetTxTerminalCount(uint8_t bus, BspCanTxResult result)
{
    if (BspCanBusValid(bus) == 0u || result <= BspCanTxResultNone ||
        result > BspCanTxResultUnknown)
    {
        return 0u;
    }
    return can_tx_terminal_count[bus - 1u][(uint8_t)result];
}

static int BspCanTxFlagsInternal(uint8_t bus,
                                 uint16_t std_id,
                                 const uint8_t data[8],
                                 uint8_t dlc,
                                 uint8_t flags,
                                 const BspCanTxTicket *ticket,
                                 uint8_t *tracked)
{
    volatile uint8_t *last_status = NULL;
    volatile uint32_t *last_error = NULL;
    volatile uint32_t *tx_count = NULL;
    volatile uint32_t *tx_fail = NULL;
    volatile uint32_t *id_count = NULL;
    volatile uint16_t *last_tx_std_id = NULL;
    volatile uint8_t *last_tx_dlc = NULL;
    HAL_StatusTypeDef ret = HAL_ERROR;
    const uint8_t want_tracked = (uint8_t)(ticket != NULL || tracked != NULL);
    uint32_t primask;

    if (tracked != NULL)
    {
        *tracked = 0u;
    }

    if (can_fault_locked != 0u || data == NULL || dlc > 8u ||
        (want_tracked != 0u &&
         (tracked == NULL || BspCanTxTicketValid(ticket) == 0u)))
    {
        return (int)HAL_ERROR;
    }

    if (bus == 1u)
    {
        last_status = &can1_last_tx_status;
        last_error = &can1_last_error;
        tx_count = &can1_tx_count;
        tx_fail = &can1_tx_fail;
        id_count = can1_tx_std_id_count;
        last_tx_std_id = &can1_tx_last_std_id;
        last_tx_dlc = &can1_tx_last_dlc;
    }
    else if (bus == 2u)
    {
        last_status = &can2_last_tx_status;
        last_error = &can2_last_error;
        tx_count = &can2_tx_count;
        tx_fail = &can2_tx_fail;
        id_count = can2_tx_std_id_count;
        last_tx_std_id = &can2_tx_last_std_id;
        last_tx_dlc = &can2_tx_last_dlc;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else if (bus == 3u)
    {
        last_status = &can3_last_tx_status;
        last_error = &can3_last_error;
        tx_count = &can3_tx_count;
        tx_fail = &can3_tx_fail;
        id_count = can3_tx_std_id_count;
        last_tx_std_id = &can3_tx_last_std_id;
        last_tx_dlc = &can3_tx_last_dlc;
    }
#endif
    else
    {
        return (int)HAL_ERROR;
    }

    if (tx_count != NULL)
    {
        (*tx_count)++;
    }
    if (std_id < BSP_CAN_STD_ID_DIAG_COUNT && id_count != NULL)
    {
        id_count[std_id]++;
    }
    if (last_tx_std_id != NULL)
    {
        *last_tx_std_id = std_id;
    }
    if (last_tx_dlc != NULL)
    {
        *last_tx_dlc = dlc;
    }

    primask = BspCanIrqEnter();
    BspCanTxFlushDeferredBusLocked(bus);
    BspCanTxPollBusHardwareLocked(bus);
    if (can_fault_locked != 0u || can_tx_reconfiguring[bus - 1u] != 0u ||
        BspCanTxBusDeferredLocked(bus) != 0u)
    {
        ret = HAL_ERROR;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else
    {
        FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
        FDCAN_TxHeaderTypeDef hdr = {0};
        uint32_t buffer_mask = 0u;
        uint8_t slot_index = 0u;
        const uint8_t put_index = (uint8_t)((hfdcan->Instance->TXFQS &
                                             FDCAN_TXFQS_TFQPI) >>
                                            FDCAN_TXFQS_TFQPI_Pos);

        hdr.Identifier = std_id;
        hdr.IdType = FDCAN_STANDARD_ID;
        hdr.TxFrameType = FDCAN_DATA_FRAME;
        hdr.DataLength = BspCanFdcanEncodeDlc(dlc);
        hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        hdr.BitRateSwitch = ((flags & BSP_CAN_FLAG_BRS) != 0u) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
        hdr.FDFormat = ((flags & BSP_CAN_FLAG_FD) != 0u) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
        hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        hdr.MessageMarker = 0u;

        if (put_index >= BSP_CAN_TX_SLOT_COUNT ||
            can_tx_track[bus - 1u][put_index].active != 0u)
        {
            ret = HAL_BUSY;
        }
        else
        {
            ret = BspCanTxWithRetry(hfdcan, &hdr, data, &buffer_mask);
            if (ret == HAL_OK && want_tracked != 0u &&
                BspCanMaskIndex(buffer_mask, 32u, &slot_index) != 0u &&
                BspCanTxTrackInstallLocked(bus,
                                           slot_index,
                                           std_id,
                                           ticket) != 0u)
            {
                *tracked = 1u;
            }
        }
        if (last_error != NULL)
        {
            *last_error = HAL_FDCAN_GetError(hfdcan);
        }
    }
#elif defined(HAL_CAN_MODULE_ENABLED)
    else if ((flags & (BSP_CAN_FLAG_FD | BSP_CAN_FLAG_BRS)) != 0u)
    {
        ret = HAL_ERROR;
    }
    else
    {
        CAN_HandleTypeDef *hcan = BspCanBxHandle(bus);
        CAN_TxHeaderTypeDef hdr = {0};
        uint32_t mailbox = 0u;
        uint8_t slot_index = 0u;

        hdr.StdId = std_id;
        hdr.IDE = CAN_ID_STD;
        hdr.RTR = CAN_RTR_DATA;
        hdr.DLC = dlc;

        ret = BspCanTxWithRetry(hcan, &hdr, data, &mailbox);
        if (ret == HAL_OK &&
            BspCanMaskIndex(mailbox, 3u, &slot_index) != 0u)
        {
            const uint32_t rqcp = BspCanBxRqcpMask(slot_index);
            const uint8_t old_tracked = can_tx_track[bus - 1u][slot_index].active;

            /*
             * 普通旧帧可在预轮询之后刚好完成，HAL 随即复用该邮箱，而旧 RQCP
             * 仍在。此时无法把终态可靠归到新旧哪一帧，只能拒绝给新帧凭据；
             * 若旧帧受跟踪，也以 Unknown 终结，绝不把新 TXOK 冒充旧成功。
             */
            if (old_tracked != 0u)
            {
                BspCanTxTrackFinishLocked(bus,
                                          slot_index,
                                          (uint8_t)BspCanTxResultUnknown);
            }
            if ((hcan->Instance->TSR & rqcp) != 0u)
            {
                BspCanBxClearRqcp(hcan, slot_index);
            }
            else if (old_tracked == 0u && want_tracked != 0u &&
                     BspCanTxTrackInstallLocked(bus,
                                                slot_index,
                                                std_id,
                                                ticket) != 0u)
            {
                *tracked = 1u;
                BspCanTxRefreshNotificationLocked(bus);
                BspCanBxDrainHandleLocked(hcan);
            }
        }
        if (last_error != NULL)
        {
            const uint32_t hal_error = HAL_CAN_GetError(hcan);

            if (hal_error != HAL_CAN_ERROR_NONE)
            {
                *last_error = hal_error;
            }
        }
    }
#endif
    BspCanIrqExit(primask);

    if (last_status != NULL)
    {
        *last_status = (uint8_t)ret;
    }
    if (ret != HAL_OK && tx_fail != NULL)
    {
        (*tx_fail)++;
    }

    return (int)ret;
}

int BspCanTxFlags(uint8_t bus,
                  uint16_t std_id,
                  const uint8_t data[8],
                  uint8_t dlc,
                  uint8_t flags)
{
    return BspCanTxFlagsInternal(bus,
                                 std_id,
                                 data,
                                 dlc,
                                 flags,
                                 NULL,
                                 NULL);
}

int BspCanTxFlagsTracked(uint8_t bus,
                         uint16_t std_id,
                         const uint8_t data[8],
                         uint8_t dlc,
                         uint8_t flags,
                         const BspCanTxTicket *ticket,
                         uint8_t *tracked)
{
    return BspCanTxFlagsInternal(bus,
                                 std_id,
                                 data,
                                 dlc,
                                 flags,
                                 ticket,
                                 tracked);
}

int BspCanTx(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc)
{
    return BspCanTxFlags(bus, std_id, data, dlc, 0u);
}

int BspCanTxTracked(uint8_t bus,
                    uint16_t std_id,
                    const uint8_t data[8],
                    uint8_t dlc,
                    const BspCanTxTicket *ticket,
                    uint8_t *tracked)
{
    return BspCanTxFlagsTracked(bus,
                                std_id,
                                data,
                                dlc,
                                0u,
                                ticket,
                                tracked);
}

static uint8_t BspCanFaultBusIdle(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);

    if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_BUSY ||
        hfdcan->Init.TxFifoQueueElmtsNbr == 0u)
    {
        return 1u;
    }
    return (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) >= hfdcan->Init.TxFifoQueueElmtsNbr) ? 1u : 0u;
#elif defined(HAL_CAN_MODULE_ENABLED)
    CAN_HandleTypeDef *hcan;

    if (bus == 1u)
    {
        hcan = &hcan1;
    }
    else if (bus == 2u)
    {
        hcan = &hcan2;
    }
    else
    {
        return 1u;
    }
    if (hcan->State != HAL_CAN_STATE_READY && hcan->State != HAL_CAN_STATE_LISTENING)
    {
        return 1u;
    }
    return (HAL_CAN_GetTxMailboxesFreeLevel(hcan) >= 3u) ? 1u : 0u;
#endif
}

static uint8_t BspCanFaultAllBusesIdle(void)
{
    if (BspCanFaultBusIdle(1u) == 0u || BspCanFaultBusIdle(2u) == 0u)
    {
        return 0u;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    if (BspCanFaultBusIdle(3u) == 0u)
    {
        return 0u;
    }
#endif
    return 1u;
}

static void BspCanFaultWaitTxIdleBounded(uint32_t spin_limit)
{
    while (spin_limit > 0u && BspCanFaultAllBusesIdle() == 0u)
    {
        spin_limit--;
        __NOP();
    }
}

static void BspCanFaultAbortPending(void)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, 0xFFFFFFFFu);
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, 0xFFFFFFFFu);
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan3, 0xFFFFFFFFu);
#elif defined(HAL_CAN_MODULE_ENABLED)
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;

    (void)HAL_CAN_AbortTxRequest(&hcan1, mailboxes);
    (void)HAL_CAN_AbortTxRequest(&hcan2, mailboxes);
#endif
}

void BspCanFaultLock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (can_fault_locked == 0u)
    {
        can_fault_locked = 1u;
        __DMB();
        for (uint8_t bus = 1u; bus <= (uint8_t)BSP_CAN_BUS_COUNT; bus++)
        {
            BspCanTxInvalidateBusLocked(bus,
                                        (uint8_t)BspCanTxResultAborted);
        }
        BspCanFaultAbortPending();
        BspCanFaultWaitTxIdleBounded(BSP_CAN_FAULT_ABORT_SPIN_LIMIT);
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
}

uint8_t BspCanFaultLocked(void)
{
    return can_fault_locked;
}

int BspCanFaultTx(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc)
{
    uint32_t spin = BSP_CAN_FAULT_TX_SPIN_LIMIT;

    if (can_fault_locked == 0u || data == NULL || dlc > 8u || std_id > 0x7FFu)
    {
        return (int)HAL_ERROR;
    }

#if defined(HAL_FDCAN_MODULE_ENABLED)
    {
        FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
        FDCAN_TxHeaderTypeDef header = {0};

        if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_BUSY)
        {
            return (int)HAL_ERROR;
        }
        header.Identifier = std_id;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = BspCanFdcanEncodeDlc(dlc);
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0u;

        while (spin > 0u)
        {
            if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u)
            {
                return (int)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
            }
            spin--;
            __NOP();
        }
    }
#elif defined(HAL_CAN_MODULE_ENABLED)
    {
        CAN_HandleTypeDef *hcan;
        CAN_TxHeaderTypeDef header = {0};
        uint32_t mailbox = 0u;

        if (bus == 1u)
        {
            hcan = &hcan1;
        }
        else if (bus == 2u)
        {
            hcan = &hcan2;
        }
        else
        {
            return (int)HAL_ERROR;
        }
        if (hcan->State != HAL_CAN_STATE_READY && hcan->State != HAL_CAN_STATE_LISTENING)
        {
            return (int)HAL_ERROR;
        }
        header.StdId = std_id;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = dlc;

        while (spin > 0u)
        {
            if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u)
            {
                return (int)HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
            }
            spin--;
            __NOP();
        }
    }
#endif

    return (int)HAL_TIMEOUT;
}

void BspCanFaultWaitTxIdle(void)
{
    BspCanFaultWaitTxIdleBounded(BSP_CAN_FAULT_FLUSH_SPIN_LIMIT);
}

int BspCanFdSetDataBitrate(uint8_t bus, uint32_t data_bitrate)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
    volatile uint32_t *last_error = BspCanFdcanLastError(bus);
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint32_t primask;

    if (hfdcan == NULL || can_fault_locked != 0u)
    {
        return (int)HAL_ERROR;
    }

    primask = BspCanIrqEnter();
    can_tx_reconfiguring[bus - 1u] = 1u;
    BspCanTxInvalidateBusLocked(bus, (uint8_t)BspCanTxResultUnknown);
    BspCanIrqExit(primask);

    (void)HAL_FDCAN_Stop(hfdcan);
    (void)HAL_FDCAN_DeInit(hfdcan);
    if (BspCanFdcanSetTiming(hfdcan, data_bitrate) != 0u)
    {
        ret = HAL_FDCAN_Init(hfdcan);
        if (ret == HAL_OK)
        {
            BspCanFdcanInitBus(hfdcan, last_error);
        }
    }
    if (last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    primask = BspCanIrqEnter();
    can_tx_reconfiguring[bus - 1u] = 0u;
    BspCanIrqExit(primask);
    return (int)ret;
#else
    (void)bus;
    (void)data_bitrate;
    return (int)HAL_ERROR;
#endif
}

uint32_t BspCanGetLastError(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_last_error;
    case 2u:
        return can2_last_error;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_last_error;
#endif
    default:
        return BSP_CAN_ERR_NONE;
    }
}

uint8_t BspCanGetLastTxStatus(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_last_tx_status;
    case 2u:
        return can2_last_tx_status;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_last_tx_status;
#endif
    default:
        return 0u;
    }
}

uint16_t BspCanGetLastTxStdId(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_tx_last_std_id;
    case 2u:
        return can2_tx_last_std_id;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_tx_last_std_id;
#endif
    default:
        return 0u;
    }
}

uint8_t BspCanGetLastTxDlc(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_tx_last_dlc;
    case 2u:
        return can2_tx_last_dlc;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_tx_last_dlc;
#endif
    default:
        return 0u;
    }
}

uint32_t BspCanGetTxCount(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_tx_count;
    case 2u:
        return can2_tx_count;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_tx_count;
#endif
    default:
        return 0u;
    }
}

#if defined(HAL_FDCAN_MODULE_ENABLED)
static uint8_t BspCanGetFdcanProtocolU8(uint8_t bus, uint8_t field)
{
    FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
    FDCAN_ProtocolStatusTypeDef status = {0};

    if (hfdcan == NULL || HAL_FDCAN_GetProtocolStatus(hfdcan, &status) != HAL_OK)
    {
        return 0u;
    }

    switch (field)
    {
    case 0u:
        return (uint8_t)status.LastErrorCode;
    case 1u:
        return (uint8_t)status.DataLastErrorCode;
    case 2u:
        return (uint8_t)status.Activity;
    case 3u:
        return (uint8_t)status.ErrorPassive;
    case 4u:
        return (uint8_t)status.Warning;
    case 5u:
        return (uint8_t)status.BusOff;
    default:
        return 0u;
    }
}

static uint8_t BspCanGetFdcanErrorCounterU8(uint8_t bus, uint8_t field)
{
    FDCAN_HandleTypeDef *hfdcan = BspCanFdcanHandle(bus);
    FDCAN_ErrorCountersTypeDef counters = {0};

    if (hfdcan == NULL || HAL_FDCAN_GetErrorCounters(hfdcan, &counters) != HAL_OK)
    {
        return 0u;
    }

    switch (field)
    {
    case 0u:
        return (uint8_t)counters.TxErrorCnt;
    case 1u:
        return (uint8_t)counters.RxErrorCnt;
    case 2u:
        return (uint8_t)counters.ErrorLogging;
    default:
        return 0u;
    }
}
#endif

uint8_t BspCanGetProtocolLastErrorCode(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 0u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetProtocolDataLastErrorCode(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 1u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetProtocolActivity(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 2u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetProtocolErrorPassive(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 3u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetProtocolWarning(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 4u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetProtocolBusOff(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanProtocolU8(bus, 5u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetTxErrorCount(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanErrorCounterU8(bus, 0u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetRxErrorCount(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanErrorCounterU8(bus, 1u);
#else
    (void)bus;
    return 0u;
#endif
}

uint8_t BspCanGetErrorLoggingCount(uint8_t bus)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    return BspCanGetFdcanErrorCounterU8(bus, 2u);
#else
    (void)bus;
    return 0u;
#endif
}

uint32_t BspCanGetTxFailCount(uint8_t bus)
{
    switch (bus)
    {
    case 1u:
        return can1_tx_fail;
    case 2u:
        return can2_tx_fail;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_tx_fail;
#endif
    default:
        return 0u;
    }
}

uint32_t BspCanGetTxStdIdCount(uint8_t bus, uint16_t std_id)
{
    if (std_id >= BSP_CAN_STD_ID_DIAG_COUNT)
    {
        return 0u;
    }
    switch (bus)
    {
    case 1u:
        return can1_tx_std_id_count[std_id];
    case 2u:
        return can2_tx_std_id_count[std_id];
#if defined(HAL_FDCAN_MODULE_ENABLED)
    case 3u:
        return can3_tx_std_id_count[std_id];
#endif
    default:
        return 0u;
    }
}
