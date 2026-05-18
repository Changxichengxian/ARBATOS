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
 * MINIWHEELEG-M port table on DM_MC02_H7.
 *
 * The notes below are reference only. Editing the note block does not change
 * code. The macros under "active assignment" are the real binding.
 *
 * Board facts:
 * - UART5 DBUS: TX PC12, RX PD2. Receiver-facing serial port.
 * - USART10: TX PE3, RX PE2. General serial port.
 * - USART1 AUX: TX PA9, RX PA10. General serial port.
 * - UART7 REFEREE: TX PE8, RX PE7. Common referee or spare serial port.
 * - USART2 RS485_0: TX PD5, RX PD6, DE PD4. On-board RS485 transceiver.
 * - USART3 RS485_1: TX PD8, RX PD9, DE PB14. On-board RS485 transceiver.
 * - KEY: PA15, active low.
 *
 * Active assignment:
 * - RC receiver: UART5 DBUS
 * - AUX / tuning / ELRS / video link: USART1 AUX
 * - Referee: UART7 REFEREE
 * - RS485 port 0: USART2 RS485_0
 * - RS485 port 1: USART3 RS485_1
 */

#define BSP_BOARD_RC_UART_HANDLE            huart5
#define BSP_BOARD_RC_UART_IRQn              UART5_IRQn
#define BSP_BOARD_AUX_UART_HANDLE           huart1
#define BSP_BOARD_REFEREE_UART_HANDLE       huart7
#define BSP_BOARD_RS485_PORT0_UART_HANDLE   huart2
#define BSP_BOARD_RS485_PORT1_UART_HANDLE   huart3

#endif
