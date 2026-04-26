/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_SD_SPI_PORT_H
#define BSP_SD_SPI_PORT_H

#include <stdint.h>

typedef enum
{
    SD_SPI_PORT_SPEED_INIT = 0,
    SD_SPI_PORT_SPEED_FAST,
} sd_spi_port_speed_e;

void sd_spi_port_cs_high(void);
void sd_spi_port_cs_low(void);
uint8_t sd_spi_port_txrx(uint8_t data);
uint32_t sd_spi_port_tick_ms(void);

int sd_spi_port_txrx_dma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms);
int sd_spi_port_receive(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
int sd_spi_port_transmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

void sd_spi_port_set_speed(sd_spi_port_speed_e speed);

#endif
