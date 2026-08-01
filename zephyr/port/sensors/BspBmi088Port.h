/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_BMI088_PORT_H
#define BSP_BMI088_PORT_H

#include <stdint.h>

#define BMI088_USE_SPI

void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
uint8_t BMI088_read_write_byte(uint8_t txdata);

#endif
