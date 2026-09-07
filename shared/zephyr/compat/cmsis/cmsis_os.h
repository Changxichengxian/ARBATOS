/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_CMSIS_OS_H
#define ARBATOS_ZEPHYR_CMSIS_OS_H

/*
 * ARBATOS 的 cmsis_os.h 使用点只依赖 osDelay 和 CMSIS-RTOS2 类型。
 * 统一导向 v2，避免同时启用 Zephyr 的 v1/v2 实现产生重复符号。
 */
#include "cmsis_os2.h"

#endif
