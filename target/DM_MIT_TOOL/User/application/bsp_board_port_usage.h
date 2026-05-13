/*
 * SPDX-FileCopyrightText: 2026 Chen Yi <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Yi <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-11
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_BOARD_PORT_USAGE_H
#define BSP_BOARD_PORT_USAGE_H

#include "bsp_board_layout.h"

/*
 * DM MIT TOOL port table on DJI_C_F407.
 *
 * The notes below are reference only. Editing the note block does not change
 * code. The macros under "active assignment" are the real binding.
 *
 * Board facts:
 * - USART3 DBUS: TX PC10, RX PC11. Hardware-inverted receiver port with RX DMA.
 *   In practice this is the dedicated DJI receiver port.
 * - USART1 AUX: TX PA9, RX PB7. General serial port with RX/TX DMA.
 * - USART6 REFEREE: TX PG14, RX PG9. General serial port with RX/TX DMA.
 * - KEY: PG0, active low. Current BSP path uses EXTI0 / PG0.
 * - BUTTON_TRIG: PI7.
 *
 * Active assignment:
 * - RC receiver: USART3 DBUS
 * - AUX / tuning / ELRS / video link: USART1 AUX
 * - Referee: USART6 REFEREE
 */

#define BSP_BOARD_RC_UART_HANDLE        huart3
#define BSP_BOARD_RC_UART_IRQn          USART3_IRQn
#define BSP_BOARD_RC_DMA_HANDLE         hdma_usart3_rx
#define BSP_BOARD_AUX_UART_HANDLE       huart1
#define BSP_BOARD_REFEREE_UART_HANDLE   huart6

#endif
