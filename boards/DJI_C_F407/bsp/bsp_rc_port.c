/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "bsp_rc.h"
#include "bsp_rc_cfg.h"
#include "usart.h"

extern DMA_HandleTypeDef BSP_RC_DMA_HANDLE;

static uint8_t sbus_rx_dma_buf[2][BSP_RC_SBUS_RX_BUF_NUM];
static volatile bsp_rc_diag_t rc_diag;

static void bsp_rc_record_rx(uint16_t size);

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

    HAL_NVIC_SetPriority(BSP_RC_UART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    RC_Init(sbus_rx_dma_buf[0], sbus_rx_dma_buf[1], BSP_RC_SBUS_RX_BUF_NUM);
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    SET_BIT(BSP_RC_UART_HANDLE.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&BSP_RC_UART_HANDLE, UART_IT_IDLE);

    __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);
    while (BSP_RC_DMA_HANDLE.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);
    }

    BSP_RC_DMA_HANDLE.Instance->PAR = (uint32_t)&(BSP_RC_UART_HANDLE.Instance->DR);
    BSP_RC_DMA_HANDLE.Instance->M0AR = (uint32_t)(rx1_buf);
    BSP_RC_DMA_HANDLE.Instance->M1AR = (uint32_t)(rx2_buf);
    BSP_RC_DMA_HANDLE.Instance->NDTR = dma_buf_num;
    SET_BIT(BSP_RC_DMA_HANDLE.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&BSP_RC_UART_HANDLE);
}

void RC_restart(uint16_t dma_buf_num)
{
    rc_diag.restart_cnt++;

    __HAL_UART_DISABLE(&BSP_RC_UART_HANDLE);
    __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);

    BSP_RC_DMA_HANDLE.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);
    __HAL_UART_ENABLE(&BSP_RC_UART_HANDLE);
}

static void bsp_rc_record_rx(uint16_t size)
{
    rc_diag.rx_event_cnt++;
    rc_diag.rx_last_size = size;
    rc_diag.rx_last_event = 1u;
    if (size != BSP_RC_SBUS_FRAME_LENGTH)
    {
        rc_diag.rx_bad_size_cnt++;
    }
}

void USART3_IRQHandler(void)
{
    rc_diag.uart_sr = BSP_RC_UART_HANDLE.Instance->SR;
    rc_diag.uart_cr1 = BSP_RC_UART_HANDLE.Instance->CR1;
    rc_diag.dma_ndtr = BSP_RC_DMA_HANDLE.Instance->NDTR;
    rc_diag.dma_cr = BSP_RC_DMA_HANDLE.Instance->CR;

    if (rc_diag.uart_sr & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&BSP_RC_UART_HANDLE);
        return;
    }

    if (rc_diag.uart_sr & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0u;

        __HAL_UART_CLEAR_PEFLAG(&BSP_RC_UART_HANDLE);

        if ((BSP_RC_DMA_HANDLE.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);
            this_time_rx_len = (uint16_t)(BSP_RC_SBUS_RX_BUF_NUM - BSP_RC_DMA_HANDLE.Instance->NDTR);
            BSP_RC_DMA_HANDLE.Instance->NDTR = BSP_RC_SBUS_RX_BUF_NUM;
            BSP_RC_DMA_HANDLE.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);

            bsp_rc_record_rx(this_time_rx_len);
            bsp_rc_sbus_on_frame_isr(sbus_rx_dma_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);
            this_time_rx_len = (uint16_t)(BSP_RC_SBUS_RX_BUF_NUM - BSP_RC_DMA_HANDLE.Instance->NDTR);
            BSP_RC_DMA_HANDLE.Instance->NDTR = BSP_RC_SBUS_RX_BUF_NUM;
            BSP_RC_DMA_HANDLE.Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);

            bsp_rc_record_rx(this_time_rx_len);
            bsp_rc_sbus_on_frame_isr(sbus_rx_dma_buf[1], this_time_rx_len);
        }
    }
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
    out->dma_ndtr = BSP_RC_DMA_HANDLE.Instance->NDTR;
    out->dma_cr = BSP_RC_DMA_HANDLE.Instance->CR;
    out->rx_last_size = rc_diag.rx_last_size;
    out->rx_last_event = rc_diag.rx_last_event;
}
