/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_math.h"
#include "BspBuzzer.h"

#include <math.h>

__weak float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

__weak float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

__weak uint16_t BuzzerLegacyPwmHalf(void)
{
    return 10000u;
}
