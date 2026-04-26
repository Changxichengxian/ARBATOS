/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#ifndef BSP_IST8310_PORT_H
#define BSP_IST8310_PORT_H

#include "struct_typedef.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310的IIC地址
#define IST8310_IIC_READ_MSB (0x80) //IST8310的SPI读取发送第一个bit为1

extern void ist8310_GPIO_init(void); //ist8310的io初始化
extern void ist8310_com_init(void);  //ist8310的通讯初始化
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //复位IO 置高
extern void ist8310_RST_L(void); //复位IO 置地 置地会引起ist8310重启

#endif
