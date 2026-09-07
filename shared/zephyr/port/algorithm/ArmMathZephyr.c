/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/* 提供当前固件使用的弱符号数学函数。 */
#include "arm_math.h"

#include <math.h>

__attribute__((weak)) float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

__attribute__((weak)) float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}
