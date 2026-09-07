/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ARM_MATH_H
#define _ARM_MATH_H

#include <math.h>

/*
 * ARBATOS 当前七个目标只调用 CMSIS-DSP 的两项浮点三角函数。Zephyr 迁移期
 * 使用这个窄接口，避免继续携带旧版 CMSIS-DSP 的整份大头文件。
 *
 * 新代码需要其他 arm_* 接口时，应显式扩展实现或接入 Zephyr 的 CMSIS-DSP
 * 模块，不能只补一个没有实现的声明。
 */
typedef float float32_t;

#ifdef __cplusplus
extern "C" {
#endif

float32_t arm_sin_f32(float32_t x);
float32_t arm_cos_f32(float32_t x);

#ifdef __cplusplus
}
#endif

#endif
