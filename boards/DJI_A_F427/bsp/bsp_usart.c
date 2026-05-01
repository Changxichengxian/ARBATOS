/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_usart.h"
#include "bsp_board_ports.h"

#include "bsp_uart_dispatch.h"
#include "main.h"
#include "usart.h"

/*
 * A-board keeps RC on USART1.
 * The shared aux-link role is wired to UART8 on this board.
 */
#define BSP_AUX_UART_HANDLE     BSP_BOARD_AUX_UART_HANDLE
#define BSP_REFEREE_UART_HANDLE BSP_BOARD_REFEREE_UART_HANDLE

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart6;

static bsp_aux_link_rx_event_cb_t g_aux_rx_event_cb = NULL;
static bsp_aux_link_rx_byte_cb_t g_aux_rx_byte_cb = NULL;
static bsp_aux_link_error_cb_t g_aux_error_cb = NULL;
static volatile uint8_t g_aux_it_rx_active = 0u;
static uint8_t g_aux_it_rx_byte = 0u;
static uint8_t g_aux_dispatch_registered = 0u;

static void bsp_aux_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);
static void bsp_aux_uart_error(UART_HandleTypeDef *huart);
static void bsp_aux_uart_register_dispatch_once(void);

static void bsp_aux_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_AUX_UART_HANDLE)
    {
        return;
    }

    bsp_aux_link_rx_event_e evt = BSP_AUX_LINK_RXEVENT_UNKNOWN;
    const HAL_UART_RxEventTypeTypeDef hal_evt = HAL_UARTEx_GetRxEventType(huart);
    if (hal_evt == HAL_UART_RXEVENT_IDLE)
    {
        evt = BSP_AUX_LINK_RXEVENT_IDLE;
    }
    else if (hal_evt == HAL_UART_RXEVENT_HT)
    {
        evt = BSP_AUX_LINK_RXEVENT_HT;
    }
    else if (hal_evt == HAL_UART_RXEVENT_TC)
    {
        evt = BSP_AUX_LINK_RXEVENT_TC;
    }

    if (g_aux_rx_event_cb != NULL)
    {
        g_aux_rx_event_cb(size, evt);
    }
}

static void bsp_aux_uart_error(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_AUX_UART_HANDLE)
    {
        return;
    }

    uint8_t handled = 0u;
    if (g_aux_error_cb != NULL)
    {
        handled = g_aux_error_cb();
    }

    if (!handled && g_aux_it_rx_active)
    {
        (void)HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    }
}

static void bsp_aux_uart_register_dispatch_once(void)
{
    if (g_aux_dispatch_registered != 0u)
    {
        return;
    }

    if (bsp_uart_dispatch_register(&BSP_AUX_UART_HANDLE, bsp_aux_uart_rx_event, bsp_aux_uart_error) == 0)
    {
        g_aux_dispatch_registered = 1u;
    }
}

void bsp_usart6_referee_init(void)
{
}

void bsp_usart6_rx_attach_task(TaskHandle_t task)
{
    (void)task;
}

int bsp_usart6_rx_pop(uint8_t *out, uint16_t *out_len)
{
    (void)out;
    (void)out_len;
    return 0;
}

uint32_t bsp_usart6_rx_get_drop_count(void)
{
    return 0u;
}

void bsp_referee_uart_init(void)
{
    bsp_usart6_referee_init();
}

void bsp_referee_rx_attach_task(TaskHandle_t task)
{
    (void)task;
}

int bsp_referee_rx_pop(uint8_t *out, uint16_t *out_len)
{
    (void)out;
    (void)out_len;
    return 0;
}

uint32_t bsp_referee_rx_get_drop_count(void)
{
    return 0u;
}

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

void bsp_aux_link_set_rx_event_cb(bsp_aux_link_rx_event_cb_t cb)
{
    bsp_aux_uart_register_dispatch_once();
    g_aux_rx_event_cb = cb;
}

void bsp_aux_link_set_rx_byte_cb(bsp_aux_link_rx_byte_cb_t cb)
{
    g_aux_rx_byte_cb = cb;
}

void bsp_aux_link_set_error_cb(bsp_aux_link_error_cb_t cb)
{
    bsp_aux_uart_register_dispatch_once();
    g_aux_error_cb = cb;
}

uint32_t bsp_aux_link_get_baudrate(void)
{
    return BSP_AUX_UART_HANDLE.Init.BaudRate;
}

int bsp_aux_link_set_baudrate(uint32_t baudrate)
{
    if (baudrate == 0u)
    {
        return (int)HAL_ERROR;
    }
    if (BSP_AUX_UART_HANDLE.Init.BaudRate == baudrate)
    {
        return (int)HAL_OK;
    }

    g_aux_it_rx_active = 0u;
    (void)HAL_UART_Abort(&BSP_AUX_UART_HANDLE);

    BSP_AUX_UART_HANDLE.Init.BaudRate = baudrate;
    return (int)HAL_UART_Init(&BSP_AUX_UART_HANDLE);
}

int bsp_aux_link_tx_dma(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_AUX_UART_HANDLE.hdmatx != NULL)
    {
        return (int)HAL_UART_Transmit_DMA(&BSP_AUX_UART_HANDLE, (uint8_t *)data, len);
    }

    return (int)HAL_UART_Transmit(&BSP_AUX_UART_HANDLE, (uint8_t *)data, len, 10u);
}

uint8_t bsp_aux_link_tx_ready(void)
{
    return (BSP_AUX_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

int bsp_aux_link_rx_it_start(void)
{
    bsp_aux_uart_register_dispatch_once();
    g_aux_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        g_aux_it_rx_active = 0u;
    }
    return (int)ret;
}

void bsp_aux_link_rx_it_stop(void)
{
    g_aux_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
}

uint8_t bsp_aux_link_rx_has_dma(void)
{
    return (BSP_AUX_UART_HANDLE.hdmarx != NULL) ? 1u : 0u;
}

int bsp_aux_link_rx_to_idle_dma_start(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    bsp_aux_uart_register_dispatch_once();
    if (BSP_AUX_UART_HANDLE.hdmarx == NULL)
    {
        return (int)HAL_ERROR;
    }

    g_aux_it_rx_active = 0u;

    if (BSP_AUX_UART_HANDLE.hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        BSP_AUX_UART_HANDLE.hdmarx->Init.Mode = DMA_CIRCULAR;
        (void)HAL_DMA_DeInit(BSP_AUX_UART_HANDLE.hdmarx);
        const HAL_StatusTypeDef dma_ret = HAL_DMA_Init(BSP_AUX_UART_HANDLE.hdmarx);
        if (dma_ret != HAL_OK)
        {
            return (int)dma_ret;
        }
        BSP_AUX_UART_HANDLE.hdmarx->Parent = &BSP_AUX_UART_HANDLE;
    }

    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&BSP_AUX_UART_HANDLE, buf, len);
    if (ret != HAL_OK)
    {
        return (int)ret;
    }

    __HAL_DMA_ENABLE_IT(BSP_AUX_UART_HANDLE.hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(BSP_AUX_UART_HANDLE.hdmarx, DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(BSP_AUX_UART_HANDLE.hdmarx, DMA_IT_TE);
    return (int)ret;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_AUX_UART_HANDLE)
    {
        return;
    }

    const uint8_t b = g_aux_it_rx_byte;
    if (g_aux_rx_byte_cb != NULL)
    {
        g_aux_rx_byte_cb(b);
    }

    if (g_aux_it_rx_active)
    {
        (void)HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    }
}
