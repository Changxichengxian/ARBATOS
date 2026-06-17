/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "types.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    uint8_t bus; // 1: CAN1, 2: CAN2, 3: CAN3 on FDCAN boards
    uint8_t dlc;
    uint8_t flags;
    uint16_t std_id;
    uint8_t data[8];
} BspCanFrame;

#define BSP_CAN_FLAG_FD  0x01u
#define BSP_CAN_FLAG_BRS 0x02u

extern void can_filter_init(void);

// ===== RX (ISR -> task) =====
void BspCanRxAttachTask(TaskHandle_t task);
int BspCanRxPop(BspCanFrame *out);
uint32_t BspCanRxPending(void);
uint32_t BspCanRxGetCount(uint8_t bus);
uint32_t BspCanRxGetDropCount(uint8_t bus);
uint32_t BspCanRxGetStdIdCount(uint8_t bus, uint16_t std_id);
uint16_t BspCanRxGetLastStdId(uint8_t bus);
uint8_t BspCanRxGetLastDlc(uint8_t bus);

// ===== TX =====
// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int BspCanTx(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc);
int BspCanTxFlags(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint8_t flags);
int BspCanFdSetDataBitrate(uint8_t bus, uint32_t data_bitrate);

uint32_t BspCanGetLastError(uint8_t bus);
uint8_t BspCanGetLastTxStatus(uint8_t bus);
uint16_t BspCanGetLastTxStdId(uint8_t bus);
uint8_t BspCanGetLastTxDlc(uint8_t bus);
uint32_t BspCanGetTxCount(uint8_t bus);
uint32_t BspCanGetTxFailCount(uint8_t bus);
uint32_t BspCanGetTxStdIdCount(uint8_t bus, uint16_t std_id);
uint8_t BspCanGetProtocolLastErrorCode(uint8_t bus);
uint8_t BspCanGetProtocolDataLastErrorCode(uint8_t bus);
uint8_t BspCanGetProtocolActivity(uint8_t bus);
uint8_t BspCanGetProtocolErrorPassive(uint8_t bus);
uint8_t BspCanGetProtocolWarning(uint8_t bus);
uint8_t BspCanGetProtocolBusOff(uint8_t bus);
uint8_t BspCanGetTxErrorCount(uint8_t bus);
uint8_t BspCanGetRxErrorCount(uint8_t bus);
uint8_t BspCanGetErrorLoggingCount(uint8_t bus);

#endif
