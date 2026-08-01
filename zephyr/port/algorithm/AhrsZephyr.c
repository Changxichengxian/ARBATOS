/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * GCC 工程和 Zephyr 共用一份 AHRS 源实现，避免两个移植版本逐渐产生行为差异。
 * CMake 只编译本包装文件，不要再同时编译 AHRS_gcc.c。
 */
#include "../../../tools/build/gcc_support/algorithm/AHRS_gcc.c"
