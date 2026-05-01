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
 * DM_MC02_H7 fixed board resources.
 * Serial port reference is kept in the target-local bsp_board_port_usage.h
 * file so daily port changes only need one entry point.
 */

#define BSP_BOARD_NAME "DM_MC02_H7"

#define BSP_BOARD_KEY_GPIO_Port              GPIOA
#define BSP_BOARD_KEY_Pin                    GPIO_PIN_15
#define BSP_BOARD_KEY_ACTIVE_LOW             1u
#define BSP_BOARD_KEY_DEBOUNCE_MS            30u

#define BSP_BOARD_BMI088_SPI_HANDLE          hspi2
#define BSP_BOARD_BMI088_ACCEL_CS_GPIO_Port  ACC_CS_GPIO_Port
#define BSP_BOARD_BMI088_ACCEL_CS_Pin        ACC_CS_Pin
#define BSP_BOARD_BMI088_GYRO_CS_GPIO_Port   GYRO_CS_GPIO_Port
#define BSP_BOARD_BMI088_GYRO_CS_Pin         GYRO_CS_Pin

#define BSP_BOARD_BUZZER_TIM_HANDLE          htim12
#define BSP_BOARD_BUZZER_TIM_CHANNEL         TIM_CHANNEL_2
#define BSP_BOARD_BUZZER_HAS_PCM             0
#define BSP_BOARD_BUZZER_PCM_USE_DMA         0

#define BSP_BOARD_IMU_PWM_TIM_HANDLE         htim3
#define BSP_BOARD_IMU_PWM_TIM_CHANNEL        TIM_CHANNEL_4

#endif
