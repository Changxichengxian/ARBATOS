/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xuan <2811158416@qq.com>
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
} aux_autotune_target_e;

void aux_autotune_reset_timing(void);
void aux_autotune_set_period_ms(uint32_t period_ms);
void aux_autotune_stop(void);
bool_t aux_autotune_start(aux_autotune_target_e target);
bool_t aux_autotune_parse_target(const char *s, aux_autotune_target_e *out);
bool_t aux_autotune_target_is_active(aux_autotune_target_e target);
bool_t aux_autotune_try_send_frame(void);

#endif
