/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_RUNTIME_H
#define ARBATOS_ZEPHYR_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ARBATOS_RUNTIME_OK = 0,
    ARBATOS_RUNTIME_ALREADY_STARTED,
    ARBATOS_RUNTIME_PLATFORM_INIT_FAILED,
    ARBATOS_RUNTIME_DEFAULT_TASK_FAILED,
    ARBATOS_RUNTIME_MODULE_TASK_FAILED,
} ArbatosRuntimeStatus;

/*
 * 由各 Zephyr 板级实现完成时钟、引脚、总线和设备初始化。
 * 返回 0 表示成功；非 0 表示运行层不得继续创建控制任务。
 */
int ArbatosPlatformInit(void);

/* 初始化公共控制配置与输入，再创建默认监控任务和配置启用的任务。 */
ArbatosRuntimeStatus ArbatosRuntimeStart(void);

const char *ArbatosRuntimeStatusName(ArbatosRuntimeStatus status);

#ifdef __cplusplus
}
#endif

#endif
