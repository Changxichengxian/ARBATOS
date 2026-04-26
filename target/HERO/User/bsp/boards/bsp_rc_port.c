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
#include "usart.h"

extern DMA_HandleTypeDef BSP_RC_DMA_HANDLE;

static uint8_t sbus_rx_dma_buf[2][BSP_RC_SBUS_RX_BUF_NUM];

void bsp_rc_port_init(void)
{
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
    __HAL_UART_DISABLE(&BSP_RC_UART_HANDLE);
    __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);

    BSP_RC_DMA_HANDLE.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);
    __HAL_UART_ENABLE(&BSP_RC_UART_HANDLE);
}

void USART3_IRQHandler(void)
{
    if (BSP_RC_UART_HANDLE.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&BSP_RC_UART_HANDLE);
        return;
    }

    if (BSP_RC_UART_HANDLE.Instance->SR & UART_FLAG_IDLE)
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

            bsp_rc_sbus_on_frame_isr(sbus_rx_dma_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&BSP_RC_DMA_HANDLE);
            this_time_rx_len = (uint16_t)(BSP_RC_SBUS_RX_BUF_NUM - BSP_RC_DMA_HANDLE.Instance->NDTR);
            BSP_RC_DMA_HANDLE.Instance->NDTR = BSP_RC_SBUS_RX_BUF_NUM;
            BSP_RC_DMA_HANDLE.Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&BSP_RC_DMA_HANDLE);

            bsp_rc_sbus_on_frame_isr(sbus_rx_dma_buf[1], this_time_rx_len);
        }
    }
}
