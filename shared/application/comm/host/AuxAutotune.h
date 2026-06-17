/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_AUTOTUNE_H
#define AUX_AUTOTUNE_H

#include "types.h"

typedef enum
{
    AUX_AUTOTUNE_TARGET_NONE = 0u,
    AUX_AUTOTUNE_TARGET_PITCH_SPEED,
    AUX_AUTOTUNE_TARGET_PITCH_ANGLE,
    AUX_AUTOTUNE_TARGET_YAW_SPEED,
    AUX_AUTOTUNE_TARGET_YAW_ANGLE,
    AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW,
    AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED,
} AuxAutotuneTarget;

void AuxAutotuneResetTiming(void);
void AuxAutotuneSetPeriodMs(uint32_t period_ms);
void AuxAutotuneStop(void);
bool_t AuxAutotuneStart(AuxAutotuneTarget target);
bool_t AuxAutotuneParseTarget(const char *s, AuxAutotuneTarget *out);
bool_t AuxAutotuneTargetIsActive(AuxAutotuneTarget target);
bool_t AuxAutotuneTrySendFrame(void);

#endif
