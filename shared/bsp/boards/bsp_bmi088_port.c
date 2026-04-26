/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_bmi088_port.h"

#include "cmsis_os2.h"
#include "bsp_delay.h"
#include <bsp_bmi088_cfg.h>
#include "spi.h"

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{

    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(BSP_BMI088_ACCEL_CS_GPIO_Port, BSP_BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(BSP_BMI088_ACCEL_CS_GPIO_Port, BSP_BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(BSP_BMI088_GYRO_CS_GPIO_Port, BSP_BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(BSP_BMI088_GYRO_CS_GPIO_Port, BSP_BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BSP_BMI088_SPI_HANDLE, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
