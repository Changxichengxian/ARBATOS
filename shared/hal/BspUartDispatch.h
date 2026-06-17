/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_UART_DISPATCH_H
#define BSP_UART_DISPATCH_H

#include "usart.h"

#include <stdint.h>

typedef void (*BspUartRxEventCb)(UART_HandleTypeDef *huart, uint16_t size);
typedef void (*BspUartErrorCb)(UART_HandleTypeDef *huart);

// Register callbacks for one UART handle. Passing NULL callbacks is allowed.
// Return: 0 on success, -1 on invalid args, -2 if table is full.
int BspUartDispatchRegister(UART_HandleTypeDef *huart, BspUartRxEventCb rx_event_cb, BspUartErrorCb error_cb);

#endif

