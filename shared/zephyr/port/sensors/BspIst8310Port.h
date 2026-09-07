/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_IST8310_PORT_H
#define BSP_IST8310_PORT_H

#include <stdint.h>
#define IST8310_IIC_ADDRESS (0x0E << 1)
#define IST8310_IIC_READ_MSB 0x80
void ist8310_GPIO_init(void);
void ist8310_com_init(void);
uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
void ist8310_delay_ms(uint16_t ms);
void ist8310_delay_us(uint16_t us);
void ist8310_RST_H(void);
void ist8310_RST_L(void);

#endif
