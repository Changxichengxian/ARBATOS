/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_usart.h"
#include "main.h"

#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

static uint8_t usart6_rx_dma_buf[2][BSP_USART6_RX_BUF_LENGTH];

#define BSP_USART6_RX_RING_SIZE 4u

typedef struct
{
    uint16_t len;
    uint8_t data[BSP_USART6_RX_BUF_LENGTH];
} bsp_usart6_rx_chunk_t;

static bsp_usart6_rx_chunk_t usart6_rx_ring[BSP_USART6_RX_RING_SIZE];
static volatile uint16_t usart6_rx_head = 0u;
static volatile uint16_t usart6_rx_tail = 0u;
static volatile uint32_t usart6_rx_drop = 0u;
static TaskHandle_t usart6_rx_task_handle = NULL;
static uint8_t referee_tx_dma_buf[BSP_USART6_RX_BUF_LENGTH];

// ===== UART1 (USART1) tuning/ELRS port =====
static bsp_uart1_rx_event_cb_t usart1_rx_event_cb = NULL;
static bsp_uart1_rx_byte_cb_t usart1_rx_byte_cb = NULL;
static bsp_uart1_error_cb_t usart1_error_cb = NULL;
static volatile uint8_t usart1_it_rx_active = 0u;
static uint8_t usart1_it_rx_byte = 0u;

static uint8_t bsp_usart6_push_from_isr(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        usart6_rx_drop++;
        return 0u;
    }

    const uint16_t h = usart6_rx_head;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_USART6_RX_RING_SIZE - 1u));
    if (next == usart6_rx_tail)
    {
        usart6_rx_drop++;
        return 0u;
    }

    usart6_rx_ring[h].len = len;
    memcpy(usart6_rx_ring[h].data, data, len);
    usart6_rx_head = next;
    return 1u;
}

static void bsp_usart6_notify_from_isr(void)
{
    if (usart6_rx_task_handle == NULL)
    {
        return;
    }
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(usart6_rx_task_handle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void bsp_usart6_referee_init(void)
{
    usart6_rx_head = 0u;
    usart6_rx_tail = 0u;
    usart6_rx_drop = 0u;
    usart6_init(usart6_rx_dma_buf[0], usart6_rx_dma_buf[1], BSP_USART6_RX_BUF_LENGTH);
}

void bsp_usart6_rx_attach_task(TaskHandle_t task)
{
    usart6_rx_task_handle = task;
}

int bsp_usart6_rx_pop(uint8_t *out, uint16_t *out_len)
{
    if (out == NULL || out_len == NULL)
    {
        return 0;
    }

    const uint16_t t = usart6_rx_tail;
    if (t == usart6_rx_head)
    {
        return 0;
    }

    const uint16_t len = usart6_rx_ring[t].len;
    if (len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        usart6_rx_tail = (uint16_t)((t + 1u) & (BSP_USART6_RX_RING_SIZE - 1u));
        usart6_rx_drop++;
        return 0;
    }

    memcpy(out, usart6_rx_ring[t].data, len);
    *out_len = len;
    usart6_rx_tail = (uint16_t)((t + 1u) & (BSP_USART6_RX_RING_SIZE - 1u));
    return 1;
}

uint32_t bsp_usart6_rx_get_drop_count(void)
{
    return usart6_rx_drop;
}

void bsp_referee_uart_init(void)
{
    bsp_usart6_referee_init();
}

void bsp_referee_rx_attach_task(TaskHandle_t task)
{
    bsp_usart6_rx_attach_task(task);
}

int bsp_referee_rx_pop(uint8_t *out, uint16_t *out_len)
{
    return bsp_usart6_rx_pop(out, out_len);
}

uint32_t bsp_referee_rx_get_drop_count(void)
{
    return bsp_usart6_rx_get_drop_count();
}

int bsp_referee_tx(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (data == NULL || len == 0u || len > (uint16_t)sizeof(referee_tx_dma_buf))
    {
        return (int)HAL_ERROR;
    }

    taskENTER_CRITICAL();
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        taskEXIT_CRITICAL();
        return (int)HAL_BUSY;
    }

    memcpy(referee_tx_dma_buf, data, len);
    ret = HAL_UART_Transmit_DMA(&huart6, referee_tx_dma_buf, len);
    taskEXIT_CRITICAL();
    return (int)ret;
}

uint8_t bsp_referee_tx_ready(void)
{
    return (huart6.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

void bsp_usart1_set_rx_event_cb(bsp_uart1_rx_event_cb_t cb)
{
    usart1_rx_event_cb = cb;
}

void bsp_usart1_set_rx_byte_cb(bsp_uart1_rx_byte_cb_t cb)
{
    usart1_rx_byte_cb = cb;
}

void bsp_usart1_set_error_cb(bsp_uart1_error_cb_t cb)
{
    usart1_error_cb = cb;
}

uint32_t bsp_usart1_get_baudrate(void)
{
    return huart1.Init.BaudRate;
}

int bsp_usart1_set_baudrate(uint32_t baudrate)
{
    if (baudrate == 0u)
    {
        return (int)HAL_ERROR;
    }
    if (huart1.Init.BaudRate == baudrate)
    {
        return (int)HAL_OK;
    }

    usart1_it_rx_active = 0u;
    (void)HAL_UART_Abort(&huart1);

    huart1.Init.BaudRate = baudrate;
    return (int)HAL_UART_Init(&huart1);
}

int bsp_usart1_tx_dma(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }
    return (int)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, len);
}

uint8_t bsp_usart1_tx_ready(void)
{
    return (huart1.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

int bsp_usart1_rx_it_start(void)
{
    usart1_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&huart1);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart1, &usart1_it_rx_byte, 1);
    if (ret != HAL_OK)
    {
        usart1_it_rx_active = 0u;
    }
    return (int)ret;
}

void bsp_usart1_rx_it_stop(void)
{
    usart1_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&huart1);
}

uint8_t bsp_usart1_rx_has_dma(void)
{
    return (huart1.hdmarx != NULL) ? 1u : 0u;
}

int bsp_usart1_rx_to_idle_dma_start(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }
    if (huart1.hdmarx == NULL)
    {
        return (int)HAL_ERROR;
    }

    usart1_it_rx_active = 0u;

    // Ensure USART1 RX DMA runs in circular mode (CubeMX default is NORMAL).
    if (huart1.hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        huart1.hdmarx->Init.Mode = DMA_CIRCULAR;
        (void)HAL_DMA_DeInit(huart1.hdmarx);
        const HAL_StatusTypeDef dma_ret = HAL_DMA_Init(huart1.hdmarx);
        if (dma_ret != HAL_OK)
        {
            return (int)dma_ret;
        }
        huart1.hdmarx->Parent = &huart1;
    }

    (void)HAL_UART_AbortReceive(&huart1);
    const HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buf, len);
    if (ret != HAL_OK)
    {
        return (int)ret;
    }

    // Use IDLE/HT/TC events to drive an RX task (no polling).
    __HAL_DMA_ENABLE_IT(huart1.hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(huart1.hdmarx, DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(huart1.hdmarx, DMA_IT_TE);
    return (int)ret;
}

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

void USART6_IRQHandler(void)
{
    if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0u;
        uint8_t pushed = 0u;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = (uint16_t)(BSP_USART6_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, BSP_USART6_RX_BUF_LENGTH);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);

            pushed = bsp_usart6_push_from_isr(usart6_rx_dma_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = (uint16_t)(BSP_USART6_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, BSP_USART6_RX_BUF_LENGTH);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);

            pushed = bsp_usart6_push_from_isr(usart6_rx_dma_buf[1], this_time_rx_len);
        }

        if (pushed)
        {
            bsp_usart6_notify_from_isr();
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart1)
    {
        return;
    }

    bsp_uart1_rx_event_e evt = BSP_UART1_RXEVENT_UNKNOWN;
    const HAL_UART_RxEventTypeTypeDef hal_evt = HAL_UARTEx_GetRxEventType(huart);
    if (hal_evt == HAL_UART_RXEVENT_IDLE)
    {
        evt = BSP_UART1_RXEVENT_IDLE;
    }
    else if (hal_evt == HAL_UART_RXEVENT_HT)
    {
        evt = BSP_UART1_RXEVENT_HT;
    }
    else if (hal_evt == HAL_UART_RXEVENT_TC)
    {
        evt = BSP_UART1_RXEVENT_TC;
    }

    if (usart1_rx_event_cb != NULL)
    {
        usart1_rx_event_cb(Size, evt);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    const uint8_t b = usart1_it_rx_byte;
    if (usart1_rx_byte_cb != NULL)
    {
        usart1_rx_byte_cb(b);
    }

    if (usart1_it_rx_active)
    {
        (void)HAL_UART_Receive_IT(&huart1, &usart1_it_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);

    uint8_t handled = 0u;
    if (usart1_error_cb != NULL)
    {
        handled = usart1_error_cb();
    }

    if (!handled && usart1_it_rx_active)
    {
        (void)HAL_UART_Receive_IT(&huart1, &usart1_it_rx_byte, 1);
    }
}
