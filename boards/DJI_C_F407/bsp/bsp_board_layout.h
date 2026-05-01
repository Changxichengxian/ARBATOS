/*
 * SPDX-FileCopyrightText: 2026 Chen Yi <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Yi <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-11
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_BOARD_LAYOUT_H
#define BSP_BOARD_LAYOUT_H

/*
 * DJI_C_F407 fixed board resources.
 * Serial port reference is kept in the target-local bsp_board_port_usage.h
 * file so daily port changes only need one entry point.
 */

#define BSP_BOARD_NAME "DJI_C_F407"

#define BSP_BOARD_KEY_GPIO_Port              GPIOG
#define BSP_BOARD_KEY_Pin                    GPIO_PIN_0
#define BSP_BOARD_KEY_ACTIVE_LOW             1u
#define BSP_BOARD_KEY_DEBOUNCE_MS            30u

#define BSP_BOARD_BMI088_SPI_HANDLE          hspi1
#define BSP_BOARD_BMI088_ACCEL_CS_GPIO_Port  CS1_ACCEL_GPIO_Port
#define BSP_BOARD_BMI088_ACCEL_CS_Pin        CS1_ACCEL_Pin
#define BSP_BOARD_BMI088_GYRO_CS_GPIO_Port   CS1_GYRO_GPIO_Port
#define BSP_BOARD_BMI088_GYRO_CS_Pin         CS1_GYRO_Pin

#define BSP_BOARD_BUZZER_TIM_HANDLE          htim4
#define BSP_BOARD_BUZZER_TIM_CHANNEL         TIM_CHANNEL_3
#define BSP_BOARD_BUZZER_HAS_PCM             1
#define BSP_BOARD_BUZZER_PCM_USE_DMA         1
#define BSP_BOARD_BUZZER_DMA_ID              TIM_DMA_ID_CC3

#define BSP_BOARD_IMU_PWM_TIM_HANDLE         htim10
#define BSP_BOARD_IMU_PWM_TIM_CHANNEL        TIM_CHANNEL_1

#endif
