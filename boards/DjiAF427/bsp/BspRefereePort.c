/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "BspUsart.h"
#include "BspBoardPorts.h"
#include "usart.h"

#define BSP_REFEREE_UART_HANDLE BSP_BOARD_REFEREE_UART_HANDLE

int BspRefereeTx(const uint8_t *data, uint16_t len)
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

uint8_t BspRefereeTxReady(void)
{
    return (BSP_REFEREE_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}
