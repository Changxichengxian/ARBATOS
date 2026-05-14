/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_can.h"
#include "main.h"

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
#error "bsp_can requires HAL_CAN_MODULE_ENABLED or HAL_FDCAN_MODULE_ENABLED"
#endif

// ===== RX ring buffers =====
#define BSP_CAN_RX_RING_SIZE 128u
typedef char _check_can_rx_ring_pow2[(BSP_CAN_RX_RING_SIZE & (BSP_CAN_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];

static volatile uint16_t can1_rx_head = 0u;
static volatile uint16_t can1_rx_tail = 0u;
static bsp_can_frame_t can1_rx_ring[BSP_CAN_RX_RING_SIZE];

static volatile uint16_t can2_rx_head = 0u;
static volatile uint16_t can2_rx_tail = 0u;
static bsp_can_frame_t can2_rx_ring[BSP_CAN_RX_RING_SIZE];

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint16_t can3_rx_head = 0u;
static volatile uint16_t can3_rx_tail = 0u;
static bsp_can_frame_t can3_rx_ring[BSP_CAN_RX_RING_SIZE];
#endif

static volatile uint32_t can1_rx_drop = 0u;
static volatile uint32_t can2_rx_drop = 0u;
static volatile uint32_t can1_tx_count = 0u;
static volatile uint32_t can2_tx_count = 0u;
static volatile uint32_t can1_tx_fail = 0u;
static volatile uint32_t can2_tx_fail = 0u;

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint32_t can3_rx_drop = 0u;
static volatile uint32_t can3_tx_count = 0u;
static volatile uint32_t can3_tx_fail = 0u;
#endif

static TaskHandle_t can_feedback_rx_task_handle = NULL;

static volatile uint32_t can1_last_error = BSP_CAN_ERR_NONE;
static volatile uint32_t can2_last_error = BSP_CAN_ERR_NONE;
static volatile uint8_t can1_last_tx_status = 0u;
static volatile uint8_t can2_last_tx_status = 0u;

#if defined(HAL_FDCAN_MODULE_ENABLED)
static volatile uint32_t can3_last_error = BSP_CAN_ERR_NONE;
static volatile uint8_t can3_last_tx_status = 0u;
#endif

static void bsp_can_reset_state(void)
{
    can1_rx_head = 0u;
    can1_rx_tail = 0u;
    can2_rx_head = 0u;
    can2_rx_tail = 0u;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    can3_rx_head = 0u;
    can3_rx_tail = 0u;
#endif

    can1_rx_drop = 0u;
    can2_rx_drop = 0u;
    can1_tx_count = 0u;
    can2_tx_count = 0u;
    can1_tx_fail = 0u;
    can2_tx_fail = 0u;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    can3_rx_drop = 0u;
    can3_tx_count = 0u;
    can3_tx_fail = 0u;
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

static void bsp_can_rx_push_common(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8], uint8_t flags)
{
    volatile uint16_t *head = NULL;
    volatile uint16_t *tail = NULL;
    bsp_can_frame_t *ring = NULL;
    volatile uint32_t *drop = NULL;

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
    }
    else if (bus == 2u)
    {
        head = &can2_rx_head;
        tail = &can2_rx_tail;
        ring = can2_rx_ring;
        drop = &can2_rx_drop;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else if (bus == 3u)
    {
        head = &can3_rx_head;
        tail = &can3_rx_tail;
        ring = can3_rx_ring;
        drop = &can3_rx_drop;
    }
#endif
    else
    {
        return;
    }

    const uint16_t h = *head;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
    if (next == *tail)
    {
        (*drop)++;
        return;
    }

    bsp_can_frame_t *dst = &ring[h];
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

static void bsp_can_rx_notify_from_isr(void)
{
    if (can_feedback_rx_task_handle == NULL)
    {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(can_feedback_rx_task_handle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

#if defined(HAL_FDCAN_MODULE_ENABLED)
static uint32_t bsp_can_fdcan_encode_dlc(uint8_t dlc)
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

static uint8_t bsp_can_fdcan_decode_dlc(uint32_t dlc)
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

static FDCAN_HandleTypeDef *bsp_can_fdcan_handle(uint8_t bus)
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

static volatile uint32_t *bsp_can_fdcan_last_error(uint8_t bus)
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

static uint8_t bsp_can_fdcan_set_timing(FDCAN_HandleTypeDef *hfdcan, uint32_t data_bitrate)
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
        tseg1 = 18u;
        tseg2 = 5u;
        break;
    case 4000000u:
        prescaler = 1u;
        tseg1 = 23u;
        tseg2 = 6u;
        break;
    case 3200000u:
        prescaler = 1u;
        tseg1 = 29u;
        tseg2 = 8u;
        break;
    case 2500000u:
        prescaler = 2u;
        tseg1 = 18u;
        tseg2 = 5u;
        break;
    case 2000000u:
        prescaler = 2u;
        tseg1 = 23u;
        tseg2 = 6u;
        break;
    case 1000000u:
        prescaler = 4u;
        tseg1 = 23u;
        tseg2 = 6u;
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

static void bsp_can_fdcan_init_bus(FDCAN_HandleTypeDef *hfdcan, volatile uint32_t *last_error)
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
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK && last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) != HAL_OK &&
        last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
}

static HAL_StatusTypeDef bsp_can_tx_with_retry(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, const uint8_t data[8])
{
    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, (uint8_t *)data);
    if (ret == HAL_BUSY && HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u)
    {
        ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, (uint8_t *)data);
    }
    return ret;
}
#elif defined(HAL_CAN_MODULE_ENABLED)
static void bsp_can_init_bus(CAN_HandleTypeDef *hcan, uint32_t filter_bank, uint32_t slave_start_bank,
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

static HAL_StatusTypeDef bsp_can_tx_with_retry(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header, const uint8_t data[8])
{
    uint32_t mailbox = 0u;
    HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(hcan, header, (uint8_t *)data, &mailbox);
    if (ret == HAL_BUSY)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u)
        {
            HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        }
        ret = HAL_CAN_AddTxMessage(hcan, header, (uint8_t *)data, &mailbox);
    }
    return ret;
}
#endif

void can_filter_init(void)
{
    bsp_can_reset_state();

#if defined(HAL_FDCAN_MODULE_ENABLED)
    bsp_can_fdcan_init_bus(&hfdcan1, &can1_last_error);
    bsp_can_fdcan_init_bus(&hfdcan2, &can2_last_error);
    bsp_can_fdcan_init_bus(&hfdcan3, &can3_last_error);
#elif defined(HAL_CAN_MODULE_ENABLED)
    bsp_can_init_bus(&hcan1, 0u, 14u, &can1_last_error);
    bsp_can_init_bus(&hcan2, 14u, 14u, &can2_last_error);
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
    dlc = bsp_can_fdcan_decode_dlc(rx_header.DataLength);
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

    bsp_can_rx_push_common(bus, (uint16_t)rx_header.Identifier, dlc, rx_data, flags);
    bsp_can_rx_notify_from_isr();
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

    bsp_can_rx_push_common(bus, (uint16_t)rx_header.StdId, (uint8_t)rx_header.DLC, rx_data, 0u);
    bsp_can_rx_notify_from_isr();
}
#endif

void bsp_can_rx_attach_task(TaskHandle_t task)
{
    can_feedback_rx_task_handle = task;
}

int bsp_can_rx_pop(bsp_can_frame_t *out)
{
    if (out == NULL)
    {
        return 0;
    }

    if (can1_rx_head != can1_rx_tail)
    {
        const uint16_t t = can1_rx_tail;
        *out = can1_rx_ring[t];
        can1_rx_tail = (uint16_t)((t + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
        return 1;
    }
    if (can2_rx_head != can2_rx_tail)
    {
        const uint16_t t = can2_rx_tail;
        *out = can2_rx_ring[t];
        can2_rx_tail = (uint16_t)((t + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
        return 1;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    if (can3_rx_head != can3_rx_tail)
    {
        const uint16_t t = can3_rx_tail;
        *out = can3_rx_ring[t];
        can3_rx_tail = (uint16_t)((t + 1u) & (BSP_CAN_RX_RING_SIZE - 1u));
        return 1;
    }
#endif
    return 0;
}

uint32_t bsp_can_rx_pending(void)
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

uint32_t bsp_can_rx_get_drop_count(uint8_t bus)
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

int bsp_can_tx_flags(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint8_t flags)
{
    volatile uint8_t *last_status = NULL;
    volatile uint32_t *last_error = NULL;
    volatile uint32_t *tx_count = NULL;
    volatile uint32_t *tx_fail = NULL;
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (data == NULL || dlc > 8u)
    {
        return (int)HAL_ERROR;
    }

    if (bus == 1u)
    {
        last_status = &can1_last_tx_status;
        last_error = &can1_last_error;
        tx_count = &can1_tx_count;
        tx_fail = &can1_tx_fail;
    }
    else if (bus == 2u)
    {
        last_status = &can2_last_tx_status;
        last_error = &can2_last_error;
        tx_count = &can2_tx_count;
        tx_fail = &can2_tx_fail;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    else if (bus == 3u)
    {
        last_status = &can3_last_tx_status;
        last_error = &can3_last_error;
        tx_count = &can3_tx_count;
        tx_fail = &can3_tx_fail;
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

#if defined(HAL_FDCAN_MODULE_ENABLED)
    FDCAN_HandleTypeDef *hfdcan = &hfdcan1;
    FDCAN_TxHeaderTypeDef hdr = {0};
    if (bus == 2u)
    {
        hfdcan = &hfdcan2;
    }
    else if (bus == 3u)
    {
        hfdcan = &hfdcan3;
    }
    hdr.Identifier = std_id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = bsp_can_fdcan_encode_dlc(dlc);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = ((flags & BSP_CAN_FLAG_BRS) != 0u) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    hdr.FDFormat = ((flags & BSP_CAN_FLAG_FD) != 0u) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0u;

    ret = bsp_can_tx_with_retry(hfdcan, &hdr, data);
    if (last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
#elif defined(HAL_CAN_MODULE_ENABLED)
    CAN_HandleTypeDef *hcan = (bus == 1u) ? &hcan1 : &hcan2;
    CAN_TxHeaderTypeDef hdr = {0};
    if ((flags & (BSP_CAN_FLAG_FD | BSP_CAN_FLAG_BRS)) != 0u)
    {
        ret = HAL_ERROR;
    }
    else
    {
        hdr.StdId = std_id;
        hdr.IDE = CAN_ID_STD;
        hdr.RTR = CAN_RTR_DATA;
        hdr.DLC = dlc;

        ret = bsp_can_tx_with_retry(hcan, &hdr, data);
        if (last_error != NULL)
        {
            *last_error = HAL_CAN_GetError(hcan);
        }
    }
#endif

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

int bsp_can_tx(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc)
{
    return bsp_can_tx_flags(bus, std_id, data, dlc, 0u);
}

int bsp_can_fd_set_data_bitrate(uint8_t bus, uint32_t data_bitrate)
{
#if defined(HAL_FDCAN_MODULE_ENABLED)
    FDCAN_HandleTypeDef *hfdcan = bsp_can_fdcan_handle(bus);
    volatile uint32_t *last_error = bsp_can_fdcan_last_error(bus);
    HAL_StatusTypeDef ret;

    if (hfdcan == NULL)
    {
        return (int)HAL_ERROR;
    }

    (void)HAL_FDCAN_Stop(hfdcan);
    (void)HAL_FDCAN_DeInit(hfdcan);
    if (bsp_can_fdcan_set_timing(hfdcan, data_bitrate) == 0u)
    {
        return (int)HAL_ERROR;
    }
    ret = HAL_FDCAN_Init(hfdcan);
    if (ret == HAL_OK)
    {
        bsp_can_fdcan_init_bus(hfdcan, last_error);
    }
    if (last_error != NULL)
    {
        *last_error = HAL_FDCAN_GetError(hfdcan);
    }
    return (int)ret;
#else
    (void)bus;
    (void)data_bitrate;
    return (int)HAL_ERROR;
#endif
}

uint32_t bsp_can_get_last_error(uint8_t bus)
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

uint8_t bsp_can_get_last_tx_status(uint8_t bus)
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

uint32_t bsp_can_get_tx_count(uint8_t bus)
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

uint32_t bsp_can_get_tx_fail_count(uint8_t bus)
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
