/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_TUNE_H
#define AUX_TUNE_H

#include <stdint.h>
#include "Types.h"

#define AUX_TUNE_RX_LINE_MAX 96u

void AuxTuneRxStart(void);
void AuxTuneResetRx(void);
void AuxTuneOnByte(uint8_t b);
uint8_t AuxTuneOnUartError(void);
void AuxTunePoll(void);
void AuxTuneTrySendTelem(void);
uint32_t AuxTuneGetCmdSeq(void);

#endif
