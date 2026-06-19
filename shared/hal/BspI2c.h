/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "Types.h"
#include "main.h"

#define I2C_ACK 1
#define I2C_NO_ACK  0


extern void BspI2cReset(I2C_TypeDef *I2C);
extern void BspI2cMasterTransmit(I2C_TypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len);
extern bool_t BspI2cCheckAck(I2C_TypeDef *I2C, uint16_t I2C_address);

#endif
