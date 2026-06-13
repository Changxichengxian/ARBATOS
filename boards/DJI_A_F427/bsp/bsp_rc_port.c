/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * @brief      BSP: SBUS port (UART1 + ReceiveToIdle DMA)
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_rc.h"
#include "bsp_rc_cfg.h"

#include "bsp_uart_dispatch.h"
#include "usart.h"

static uint8_t sbus_rx_dma_buf[BSP_RC_SBUS_FRAME_LENGTH];
static volatile bsp_rc_diag_t rc_diag;

static void bsp_rc_uart1_rx_event(UART_HandleTypeDef *huart, uint16_t size);
static void bsp_rc_uart1_error(UART_HandleTypeDef *huart);
static uint16_t bsp_rc_uart_event_code(UART_HandleTypeDef *huart);

void bsp_rc_port_init(void)
{
    rc_diag.rx_event_cnt = 0u;
    rc_diag.rx_bad_size_cnt = 0u;
    rc_diag.uart_error_cnt = 0u;
    rc_diag.uart_last_error = 0u;
    rc_diag.restart_cnt = 0u;
    rc_diag.drop_cnt = 0u;
    rc_diag.uart_sr = 0u;
    rc_diag.uart_cr1 = 0u;
    rc_diag.dma_ndtr = 0u;
    rc_diag.dma_cr = 0u;
    rc_diag.rx_last_size = 0u;
    rc_diag.rx_last_event = 0u;

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

    rc_diag.restart_cnt++;

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

    rc_diag.rx_event_cnt++;
    rc_diag.rx_last_size = size;
    rc_diag.rx_last_event = bsp_rc_uart_event_code(huart);
    if (size != BSP_RC_SBUS_FRAME_LENGTH)
    {
        rc_diag.rx_bad_size_cnt++;
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

    rc_diag.uart_error_cnt++;
    rc_diag.uart_last_error = huart->ErrorCode;
    RC_restart(0u);
}

static uint16_t bsp_rc_uart_event_code(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return 0u;
    }

    const HAL_UART_RxEventTypeTypeDef evt = HAL_UARTEx_GetRxEventType(huart);
    if (evt == HAL_UART_RXEVENT_IDLE)
    {
        return 1u;
    }
    if (evt == HAL_UART_RXEVENT_HT)
    {
        return 2u;
    }
    if (evt == HAL_UART_RXEVENT_TC)
    {
        return 3u;
    }
    return 0u;
}

void bsp_rc_get_diag(bsp_rc_diag_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->rx_event_cnt = rc_diag.rx_event_cnt;
    out->rx_bad_size_cnt = rc_diag.rx_bad_size_cnt;
    out->uart_error_cnt = rc_diag.uart_error_cnt;
    out->uart_last_error = rc_diag.uart_last_error;
    out->restart_cnt = rc_diag.restart_cnt;
    out->drop_cnt = bsp_rc_sbus_rx_get_drop_count();
    out->uart_sr = BSP_RC_UART_HANDLE.Instance->SR;
    out->uart_cr1 = BSP_RC_UART_HANDLE.Instance->CR1;
    out->dma_ndtr = (BSP_RC_UART_HANDLE.hdmarx != NULL) ? BSP_RC_UART_HANDLE.hdmarx->Instance->NDTR : 0u;
    out->dma_cr = (BSP_RC_UART_HANDLE.hdmarx != NULL) ? BSP_RC_UART_HANDLE.hdmarx->Instance->CR : 0u;
    out->rx_last_size = rc_diag.rx_last_size;
    out->rx_last_event = rc_diag.rx_last_event;
}
