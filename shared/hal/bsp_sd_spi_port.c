/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_sd_spi_port.h"

#include "main.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t sd_spi_dma_sem = NULL;
static StaticSemaphore_t sd_spi_dma_sem_buf;

static void sd_spi_port_dma_sem_init(void)
{
    taskENTER_CRITICAL();
    if (sd_spi_dma_sem == NULL)
    {
        sd_spi_dma_sem = xSemaphoreCreateBinaryStatic(&sd_spi_dma_sem_buf);
    }
    taskEXIT_CRITICAL();
}

void sd_spi_port_cs_high(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void sd_spi_port_cs_low(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

uint8_t sd_spi_port_txrx(uint8_t data)
{
    uint8_t rx = 0xFFu;
    (void)HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 1000);
    return rx;
}

uint32_t sd_spi_port_tick_ms(void)
{
    return HAL_GetTick();
}

int sd_spi_port_txrx_dma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    if (tx == NULL || rx == NULL || len == 0u)
    {
        return -1;
    }

    if (hspi2.hdmatx == NULL || hspi2.hdmarx == NULL)
    {
        return -2;
    }

    sd_spi_port_dma_sem_init();
    if (sd_spi_dma_sem == NULL)
    {
        return -3;
    }

    (void)xSemaphoreTake(sd_spi_dma_sem, 0);

    if (HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)tx, rx, len) != HAL_OK)
    {
        return -4;
    }

    if (xSemaphoreTake(sd_spi_dma_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        (void)HAL_SPI_Abort(&hspi2);
        return -5;
    }

    return (hspi2.ErrorCode == HAL_SPI_ERROR_NONE) ? 0 : -6;
}

int sd_spi_port_receive(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0u)
    {
        return -1;
    }
    return (HAL_SPI_Receive(&hspi2, buf, len, timeout_ms) == HAL_OK) ? 0 : -2;
}

int sd_spi_port_transmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0u)
    {
        return -1;
    }
    return (HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, timeout_ms) == HAL_OK) ? 0 : -2;
}

void sd_spi_port_set_speed(sd_spi_port_speed_e speed)
{
    uint32_t prescaler = SPI_BAUDRATEPRESCALER_256;
    if (speed == SD_SPI_PORT_SPEED_FAST)
    {
        prescaler = SPI_BAUDRATEPRESCALER_8;
    }

    if (hspi2.Init.BaudRatePrescaler == prescaler)
    {
        return;
    }

    (void)HAL_SPI_DeInit(&hspi2);
    hspi2.Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL || hspi->Instance != SPI2 || sd_spi_dma_sem == NULL)
    {
        return;
    }
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(sd_spi_dma_sem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxCpltCallback(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxCpltCallback(hspi);
}

