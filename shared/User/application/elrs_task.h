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

// ELRS/CRSF default UART baud rate.
#define UART1_ELRS_BAUD 420000u

// FreeRTOS task entry (created in freertos.c).
void uart1_elrs_task(void const *argument);

// Start/stop UART1 ELRS RX.
void uart1_elrs_rx_start(void);
void uart1_elrs_stop(void);

// Hooks for UART1 callbacks (called from ISR context).
void uart1_elrs_on_rx_event(uint16_t size, bsp_uart1_rx_event_e evt);
void uart1_elrs_on_it_byte(uint8_t b);
bool_t uart1_elrs_on_uart_error(void);

#endif
