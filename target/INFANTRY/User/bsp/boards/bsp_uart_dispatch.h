/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_UART_DISPATCH_H
#define BSP_UART_DISPATCH_H

#include "usart.h"

#include <stdint.h>

typedef void (*bsp_uart_rx_event_cb_t)(UART_HandleTypeDef *huart, uint16_t size);
typedef void (*bsp_uart_error_cb_t)(UART_HandleTypeDef *huart);

// Register callbacks for one UART handle. Passing NULL callbacks is allowed.
// Return: 0 on success, -1 on invalid args, -2 if table is full.
int bsp_uart_dispatch_register(UART_HandleTypeDef *huart, bsp_uart_rx_event_cb_t rx_event_cb, bsp_uart_error_cb_t error_cb);

#endif

