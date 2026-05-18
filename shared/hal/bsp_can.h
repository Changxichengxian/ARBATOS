/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
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
} bsp_can_frame_t;

#define BSP_CAN_FLAG_FD  0x01u
#define BSP_CAN_FLAG_BRS 0x02u

extern void can_filter_init(void);

// ===== RX (ISR -> task) =====
void bsp_can_rx_attach_task(TaskHandle_t task);
int bsp_can_rx_pop(bsp_can_frame_t *out);
uint32_t bsp_can_rx_pending(void);
uint32_t bsp_can_rx_get_count(uint8_t bus);
uint32_t bsp_can_rx_get_drop_count(uint8_t bus);
uint32_t bsp_can_rx_get_std_id_count(uint8_t bus, uint16_t std_id);
uint16_t bsp_can_rx_get_last_std_id(uint8_t bus);
uint8_t bsp_can_rx_get_last_dlc(uint8_t bus);

// ===== TX =====
// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int bsp_can_tx(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc);
int bsp_can_tx_flags(uint8_t bus, uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint8_t flags);
int bsp_can_fd_set_data_bitrate(uint8_t bus, uint32_t data_bitrate);

uint32_t bsp_can_get_last_error(uint8_t bus);
uint8_t bsp_can_get_last_tx_status(uint8_t bus);
uint16_t bsp_can_get_last_tx_std_id(uint8_t bus);
uint8_t bsp_can_get_last_tx_dlc(uint8_t bus);
uint32_t bsp_can_get_tx_count(uint8_t bus);
uint32_t bsp_can_get_tx_fail_count(uint8_t bus);
uint32_t bsp_can_get_tx_std_id_count(uint8_t bus, uint16_t std_id);
uint8_t bsp_can_get_protocol_last_error_code(uint8_t bus);
uint8_t bsp_can_get_protocol_data_last_error_code(uint8_t bus);
uint8_t bsp_can_get_protocol_activity(uint8_t bus);
uint8_t bsp_can_get_protocol_error_passive(uint8_t bus);
uint8_t bsp_can_get_protocol_warning(uint8_t bus);
uint8_t bsp_can_get_protocol_bus_off(uint8_t bus);
uint8_t bsp_can_get_tx_error_count(uint8_t bus);
uint8_t bsp_can_get_rx_error_count(uint8_t bus);
uint8_t bsp_can_get_error_logging_count(uint8_t bus);

#endif
