/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_CAN_ZEPHYR_H
#define BSP_CAN_ZEPHYR_H

#include <stdint.h>

/*
 * Zephyr 的公共 CAN 接口不能在致命异常环境中绕过内核锁、撤销指定邮箱并确认
 * 安全帧已经上总线。上层可显式查询能力；返回 0 时必须采用其他断能措施。
 */
uint8_t BspCanZephyrFaultTxSupported(void);
const char *BspCanZephyrFaultTxReason(void);

/* 所有配置的 CAN 控制器、接收过滤器和设备启动均已成功时返回 1。 */
uint8_t BspCanZephyrReady(void);

#endif
