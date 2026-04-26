/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "bsp_rc.h"
#include "bsp_rc_cfg.h"

#include "bsp_uart_dispatch.h"
#include "usart.h"

static uint8_t sbus_rx_dma_buf[BSP_RC_SBUS_FRAME_LENGTH];

static void bsp_rc_uart1_rx_event(UART_HandleTypeDef *huart, uint16_t size);
static void bsp_rc_uart1_error(UART_HandleTypeDef *huart);

void bsp_rc_port_init(void)
{
    (void)bsp_uart_dispatch_register(&BSP_RC_UART_HANDLE, bsp_rc_uart1_rx_event, bsp_rc_uart1_error);
    RC_Init(NULL, NULL, 0u);
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    (void)rx1_buf;
    (void)rx2_buf;
    (void)dma_buf_num;

    (void)HAL_UART_DMAStop(&BSP_RC_UART_HANDLE);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&BSP_RC_UART_HANDLE, sbus_rx_dma_buf, (uint16_t)sizeof(sbus_rx_dma_buf));
    if (BSP_RC_UART_HANDLE.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(BSP_RC_UART_HANDLE.hdmarx, DMA_IT_HT);
    }
}

void RC_unable(void)
{
    (void)HAL_UART_DMAStop(&BSP_RC_UART_HANDLE);
    __HAL_UART_DISABLE(&BSP_RC_UART_HANDLE);
}

void RC_restart(uint16_t dma_buf_num)
{
    (void)dma_buf_num;

    __HAL_UART_ENABLE(&BSP_RC_UART_HANDLE);
    (void)HAL_UART_DMAStop(&BSP_RC_UART_HANDLE);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&BSP_RC_UART_HANDLE, sbus_rx_dma_buf, (uint16_t)sizeof(sbus_rx_dma_buf));
    if (BSP_RC_UART_HANDLE.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(BSP_RC_UART_HANDLE.hdmarx, DMA_IT_HT);
    }
}

static void bsp_rc_uart1_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_RC_UART_HANDLE)
    {
        return;
    }

    bsp_rc_sbus_on_frame_isr(sbus_rx_dma_buf, size);

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&BSP_RC_UART_HANDLE, sbus_rx_dma_buf, (uint16_t)sizeof(sbus_rx_dma_buf));
    if (BSP_RC_UART_HANDLE.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(BSP_RC_UART_HANDLE.hdmarx, DMA_IT_HT);
    }
}

static void bsp_rc_uart1_error(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_RC_UART_HANDLE)
    {
        return;
    }

    RC_restart(0u);
}
