/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "BspUsart.h"
#include "BspBoardPorts.h"

#include "BspUartDispatch.h"
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

static BspAuxLinkRxEventCb g_aux_rx_event_cb = NULL;
static BspAuxLinkRxByteCb g_aux_rx_byte_cb = NULL;
static BspAuxLinkErrorCb g_aux_error_cb = NULL;
static volatile uint8_t g_aux_it_rx_active = 0u;
static uint8_t g_aux_it_rx_byte = 0u;
static uint8_t g_aux_dispatch_registered = 0u;

static void BspAuxUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
static void BspAuxUartError(UART_HandleTypeDef *huart);
static void BspAuxUartRegisterDispatchOnce(void);

static void BspAuxUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_AUX_UART_HANDLE)
    {
        return;
    }

    BspAuxLinkRxEvent evt = BSP_AUX_LINK_RXEVENT_UNKNOWN;
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

static void BspAuxUartError(UART_HandleTypeDef *huart)
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

static void BspAuxUartRegisterDispatchOnce(void)
{
    if (g_aux_dispatch_registered != 0u)
    {
        return;
    }

    if (BspUartDispatchRegister(&BSP_AUX_UART_HANDLE, BspAuxUartRxEvent, BspAuxUartError) == 0)
    {
        g_aux_dispatch_registered = 1u;
    }
}

void BspUsart6RefereeInit(void)
{
}

void BspUsart6RxAttachTask(TaskHandle_t task)
{
    (void)task;
}

int BspUsart6RxPop(uint8_t *out, uint16_t *out_len)
{
    (void)out;
    (void)out_len;
    return 0;
}

uint32_t BspUsart6RxGetDropCount(void)
{
    return 0u;
}

void BspRefereeUartInit(void)
{
    BspUsart6RefereeInit();
}

void BspRefereeRxAttachTask(TaskHandle_t task)
{
    (void)task;
}

int BspRefereeRxPop(uint8_t *out, uint16_t *out_len)
{
    (void)out;
    (void)out_len;
    return 0;
}

uint32_t BspRefereeRxGetDropCount(void)
{
    return 0u;
}

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

void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb cb)
{
    BspAuxUartRegisterDispatchOnce();
    g_aux_rx_event_cb = cb;
}

void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb cb)
{
    g_aux_rx_byte_cb = cb;
}

void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb cb)
{
    BspAuxUartRegisterDispatchOnce();
    g_aux_error_cb = cb;
}

uint32_t BspAuxLinkGetBaudrate(void)
{
    return BSP_AUX_UART_HANDLE.Init.BaudRate;
}

int BspAuxLinkSetBaudrate(uint32_t baudrate)
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

int BspAuxLinkTxDma(const uint8_t *data, uint16_t len)
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

uint8_t BspAuxLinkTxReady(void)
{
    return (BSP_AUX_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

int BspAuxLinkRxItStart(void)
{
    BspAuxUartRegisterDispatchOnce();
    g_aux_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        g_aux_it_rx_active = 0u;
    }
    return (int)ret;
}

void BspAuxLinkRxItStop(void)
{
    g_aux_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
}

uint8_t BspAuxLinkRxHasDma(void)
{
    return (BSP_AUX_UART_HANDLE.hdmarx != NULL) ? 1u : 0u;
}

int BspAuxLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    BspAuxUartRegisterDispatchOnce();
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
