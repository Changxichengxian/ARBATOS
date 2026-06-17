/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "BspUartDispatch.h"

#include <stddef.h>

#define BSP_UART_DISPATCH_MAX 8u

typedef struct
{
    UART_HandleTypeDef *huart;
    BspUartRxEventCb rx_event_cb;
    BspUartErrorCb error_cb;
} BspUartDispatchEntry;

static BspUartDispatchEntry g_uart_dispatch[BSP_UART_DISPATCH_MAX];

static BspUartDispatchEntry *BspUartDispatchFind(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0u; i < BSP_UART_DISPATCH_MAX; i++)
    {
        if (g_uart_dispatch[i].huart == huart)
        {
            return &g_uart_dispatch[i];
        }
    }
    return NULL;
}

int BspUartDispatchRegister(UART_HandleTypeDef *huart, BspUartRxEventCb rx_event_cb, BspUartErrorCb error_cb)
{
    if (huart == NULL)
    {
        return -1;
    }

    BspUartDispatchEntry *existing = BspUartDispatchFind(huart);
    if (existing != NULL)
    {
        existing->rx_event_cb = rx_event_cb;
        existing->error_cb = error_cb;
        return 0;
    }

    for (uint8_t i = 0u; i < BSP_UART_DISPATCH_MAX; i++)
    {
        if (g_uart_dispatch[i].huart == NULL)
        {
            g_uart_dispatch[i].huart = huart;
            g_uart_dispatch[i].rx_event_cb = rx_event_cb;
            g_uart_dispatch[i].error_cb = error_cb;
            return 0;
        }
    }

    return -2;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BspUartDispatchEntry *e = BspUartDispatchFind(huart);
    if (e == NULL || e->rx_event_cb == NULL)
    {
        return;
    }

    e->rx_event_cb(huart, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
    }

    BspUartDispatchEntry *e = BspUartDispatchFind(huart);
    if (e == NULL || e->error_cb == NULL)
    {
        return;
    }

    e->error_cb(huart);
}
