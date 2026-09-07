/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_STM32_MAIN_H
#define ARBATOS_ZEPHYR_STM32_MAIN_H

/*
 * 旧 CubeMX 工程把 CMSIS 内核定义和 HAL 时基声明集中放在 main.h。
 * Zephyr 接管启动与外设初始化后，共享业务代码仍只需要这两个稳定边界。
 * 不在这里重新暴露任何 CubeMX 外设句柄。
 */
#include <stdint.h>

#include <cmsis_core.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay);

#ifdef __cplusplus
}
#endif

#endif
