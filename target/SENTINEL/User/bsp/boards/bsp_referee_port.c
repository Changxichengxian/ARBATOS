/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_usart.h"
#include "usart.h"

int bsp_referee_tx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&huart6, (uint8_t *)data, len, 10u);
}

uint8_t bsp_referee_tx_ready(void)
{
    return (huart6.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}
