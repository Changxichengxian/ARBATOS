/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xuan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_PORT_H
#define AUX_PORT_H

#include <stdint.h>
#include "types.h"

#define AUX_TUNE_BAUD 230400u

void aux_port_init(void);
void aux_port_poll(void);
void aux_port_stop(void);
bool_t aux_port_apply_baud(uint32_t baud);
uint8_t aux_port_is_elrs_mode(uint32_t baud);
uint8_t aux_port_is_image_mode(uint32_t baud);
uint8_t aux_port_is_tune_mode(uint32_t baud);

#endif
