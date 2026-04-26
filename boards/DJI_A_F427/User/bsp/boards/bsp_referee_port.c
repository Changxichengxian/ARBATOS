/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_usart.h"
#include "bsp_board_ports.h"
#include "usart.h"

#define BSP_REFEREE_UART_HANDLE BSP_BOARD_REFEREE_UART_HANDLE

int bsp_referee_tx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_REFEREE_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_REFEREE_UART_HANDLE, (uint8_t *)data, len, 10u);
}

uint8_t bsp_referee_tx_ready(void)
{
    return (BSP_REFEREE_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}
