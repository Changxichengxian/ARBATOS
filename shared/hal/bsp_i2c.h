/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "types.h"
#include "main.h"

#define I2C_ACK 1
#define I2C_NO_ACK  0


extern void bsp_I2C_reset(I2C_TypeDef *I2C);
extern void bsp_I2C_master_transmit(I2C_TypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len);
extern bool_t bsp_I2C_check_ack(I2C_TypeDef *I2C, uint16_t I2C_address);

#endif
