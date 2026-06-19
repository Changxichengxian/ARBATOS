/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_PORT_H
#define AUX_PORT_H

#include <stdint.h>
#include "Types.h"

#define AUX_TUNE_BAUD 230400u

void AuxPortInit(void);
void AuxPortPoll(void);
void AuxPortStop(void);
bool_t AuxPortApplyBaud(uint32_t baud);
uint8_t AuxPortIsElrsMode(uint32_t baud);
uint8_t AuxPortIsImageMode(uint32_t baud);
uint8_t AuxPortIsTuneMode(uint32_t baud);

#endif
