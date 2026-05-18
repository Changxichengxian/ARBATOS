/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_uart_dispatch.h"

#include <stddef.h>

#define BSP_UART_DISPATCH_MAX 8u

typedef struct
{
    UART_HandleTypeDef *huart;
    bsp_uart_rx_event_cb_t rx_event_cb;
    bsp_uart_error_cb_t error_cb;
} bsp_uart_dispatch_entry_t;

static bsp_uart_dispatch_entry_t g_uart_dispatch[BSP_UART_DISPATCH_MAX];

static bsp_uart_dispatch_entry_t *bsp_uart_dispatch_find(UART_HandleTypeDef *huart)
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

int bsp_uart_dispatch_register(UART_HandleTypeDef *huart, bsp_uart_rx_event_cb_t rx_event_cb, bsp_uart_error_cb_t error_cb)
{
    if (huart == NULL)
    {
        return -1;
    }

    bsp_uart_dispatch_entry_t *existing = bsp_uart_dispatch_find(huart);
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
    bsp_uart_dispatch_entry_t *e = bsp_uart_dispatch_find(huart);
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

    bsp_uart_dispatch_entry_t *e = bsp_uart_dispatch_find(huart);
    if (e == NULL || e->error_cb == NULL)
    {
        return;
    }

    e->error_cb(huart);
}
