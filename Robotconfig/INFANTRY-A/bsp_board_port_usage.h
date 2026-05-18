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
 * INFANTRY-A port table on DJI_A_F427.
 *
 * The notes below are reference only. Editing the note block does not change
 * code. The macros under "active assignment" are the real binding.
 *
 * Board facts:
 * - USART1 DBUS: TX PB6, RX PB7. Hardware-inverted receiver port with RX DMA.
 *   In practice this is the dedicated DJI receiver port.
 * - UART8 AUX: TX PE1, RX PE0. General serial port with RX/TX DMA.
 * - USART6 REFEREE: TX PG14, RX PG9. Common referee port.
 * - UART7 GENERAL: TX PE8, RX PE7. Spare general serial port.
 * - USART3 GENERAL: TX PD8, RX PD9. Spare general serial port.
 * - KEY: PB2.
 *
 * Active assignment:
 * - RC receiver: USART1 DBUS
 * - AUX / tuning / ELRS / video link: UART8 AUX
 * - Referee: USART6 REFEREE
 * - Spare serial port: UART7 GENERAL
 * - Spare serial port: USART3 GENERAL
 */

#define BSP_BOARD_RC_UART_HANDLE        huart1
#define BSP_BOARD_RC_UART_IRQn          USART1_IRQn
#define BSP_BOARD_RC_DMA_HANDLE         hdma_usart1_rx
#define BSP_BOARD_AUX_UART_HANDLE       huart8
#define BSP_BOARD_REFEREE_UART_HANDLE   huart6

#endif
