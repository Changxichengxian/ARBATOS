/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_math.h"
#include "bsp_buzzer.h"

#include <math.h>

__weak float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

__weak float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

__weak uint16_t buzzer_legacy_pwm_half(void)
{
    return 10000u;
}
