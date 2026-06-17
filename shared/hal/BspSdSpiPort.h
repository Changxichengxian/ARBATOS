/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
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
} SdSpiPortSpeed;

void SdSpiPortCsHigh(void);
void SdSpiPortCsLow(void);
uint8_t SdSpiPortTxrx(uint8_t data);
uint32_t SdSpiPortTickMs(void);

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms);
int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

void SdSpiPortSetSpeed(SdSpiPortSpeed speed);

#endif
