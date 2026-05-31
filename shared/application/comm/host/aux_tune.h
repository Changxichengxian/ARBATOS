/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xuan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_TUNE_H
#define AUX_TUNE_H

#include <stdint.h>
#include "types.h"

#define AUX_TUNE_RX_LINE_MAX 96u

void aux_tune_rx_start(void);
void aux_tune_reset_rx(void);
void aux_tune_on_byte(uint8_t b);
uint8_t aux_tune_on_uart_error(void);
void aux_tune_poll(void);
void aux_tune_try_send_telem(void);
uint32_t aux_tune_get_cmd_seq(void);

#endif
