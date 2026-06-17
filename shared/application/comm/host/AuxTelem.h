/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_TELEM_H
#define AUX_TELEM_H

#include "types.h"

void AuxTelemPrepareMotorIds(void);
void AuxTelemSetInsSources(const fp32 *quat, const fp32 *angle, const fp32 *gyro, const fp32 *accel);
void AuxTelemReset(void);
void AuxTelemTrySendFrame(void);

#endif
