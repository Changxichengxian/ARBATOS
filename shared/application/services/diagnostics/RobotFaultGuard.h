/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_FAULT_GUARD_H
#define ROBOT_FAULT_GUARD_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "RobotFaultTypes.h"

/*
 * 这里仅处理已经无法可靠继续调度的系统致命故障。普通设备掉线和控制域故障
 * 必须走 FaultMgr 的局部策略，不能调用本文件的全局安全帧和复位入口。
 * DEVICE_DOMAIN_FAULTS_USE_FAULT_MGR：这个标记由架构检查固定上述边界。
 */

/* Reset_Handler 在 SystemInit 和 C 运行库初始化前调用，只写本模块固定状态。 */
void RobotFaultEarlyInit(void);
void RobotFaultTaskAndReset(uint32_t reason,
                            uint32_t arg0,
                            uint32_t arg1,
                            TaskHandle_t task,
                            const char *task_name);
void RobotFaultResetFromException(uint32_t reason,
                                  uint32_t arg0,
                                  uint32_t arg1);
void RobotFaultHardFaultEntry(uint32_t *stack, uint32_t exc_return);
void RobotFaultDefaultHandler(void);
void RobotFaultRecordAndReset(uint32_t reason,
                              uint32_t arg0,
                              uint32_t arg1);
void RobotFaultAssert(const char *file, uint32_t line);

#endif
