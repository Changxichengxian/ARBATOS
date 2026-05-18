/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef ELRS_TASK_H
#define ELRS_TASK_H

#include "bsp_usart.h"

// ELRS/CRSF baud rate on the aux link port.
#define ELRS_LINK_BAUD 420000u

// FreeRTOS task entry (created in freertos.c).
void elrs_link_task(void const *argument);

// Start/stop ELRS RX on the aux link port.
void elrs_link_rx_start(void);
void elrs_link_stop(void);

// Callbacks from the aux link ISR context.
void elrs_link_on_rx_event(uint16_t size, bsp_aux_link_rx_event_e evt);
void elrs_link_on_it_byte(uint8_t b);
bool_t elrs_link_on_uart_error(void);

#endif
