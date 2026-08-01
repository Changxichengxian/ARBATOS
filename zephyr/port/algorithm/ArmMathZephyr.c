/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * 复用 GCC 构建已有的弱符号实现。H723 目标的 Mc02Compat.c 也提供弱符号，
 * 两者语义相同；以后接入完整 CMSIS-DSP 时，强符号可以自然覆盖这里。
 */
#include "../../../tools/build/gcc_support/algorithm/arm_math_compat.c"
