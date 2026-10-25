/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * 日常固件不保留完整 Watch 镜像。故障证据本体由 RobotFaultGuard 和
 * BspResetEvidence 保存；这里仅维持异常路径需要的启动阶段接口，避免为了
 * 调试器观察而常驻整块状态镜像。
 */

#include "Watch.h"

static volatile uint32_t s_watch_lite_boot_stage;
static volatile uint32_t s_watch_lite_error_arg0;
static volatile uint32_t s_watch_lite_error_arg1;
static volatile uint32_t s_watch_lite_fatal_reason;

void WatchInit(void)
{
    s_watch_lite_error_arg0 = 0u;
    s_watch_lite_error_arg1 = 0u;
    s_watch_lite_fatal_reason = 0u;
}

void WatchUpdate(void)
{
}

void WatchDiagSetBootStage(WatchBootStage stage)
{
    s_watch_lite_boot_stage = (uint32_t)stage;
}

uint32_t WatchDiagBootStageGet(void)
{
    return s_watch_lite_boot_stage;
}

void WatchDiagMarkErrorHandler(uint32_t tick_ms, uint32_t ipsr)
{
    (void)tick_ms;
    (void)ipsr;
}

void WatchDiagSetErrorArgs(uint32_t arg0, uint32_t arg1)
{
    s_watch_lite_error_arg0 = arg0;
    s_watch_lite_error_arg1 = arg1;
}

void WatchDiagMarkFatal(uint32_t reason, uint32_t task_handle, const char *task_name)
{
    (void)task_handle;
    (void)task_name;
    s_watch_lite_fatal_reason = reason;
}

void WatchTaskModuleCreateReset(void)
{
}

void WatchTaskModuleCreateResult(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state)
{
    (void)module;
    (void)name;
    (void)thread_handle;
    (void)state;
}

void WatchTaskBeat(WatchTaskId task_id)
{
    (void)task_id;
}

void WatchTaskWait(WatchTaskId task_id)
{
    (void)task_id;
}

void WatchTaskTimeout(WatchTaskId task_id)
{
    (void)task_id;
}

void WatchTaskError(WatchTaskId task_id)
{
    (void)task_id;
}

void WatchImuSetStage(WatchImuStage stage)
{
    (void)stage;
}

void WatchIrqHit(WatchIrqId irq_id)
{
    (void)irq_id;
}

const WatchBlockDesc *WatchGetBlockTable(uint32_t *count)
{
    if (count != NULL)
    {
        *count = 0u;
    }
    return NULL;
}

const WatchBlockDesc *WatchFindBlock(WatchBlockId id)
{
    (void)id;
    return NULL;
}

uint8_t WatchBlockIsActive(WatchBlockId id)
{
    (void)id;
    return 0u;
}
