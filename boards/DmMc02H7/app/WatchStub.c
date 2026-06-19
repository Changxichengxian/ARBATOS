/*
 * SPDX-FileCopyrightText: 2026 Chen Yi <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-11
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "Watch.h"

/*
 * This carrier build does not link the full watch/diagnostic pipeline.
 * Keep shared task hooks linkable with local no-op implementations.
 */

void WatchInit(void) {}
void WatchUpdate(void) {}
void WatchDiagSetBootStage(WatchBootStage stage) { (void)stage; }
void WatchDiagMarkErrorHandler(uint32_t tick_ms, uint32_t ipsr)
{
    (void)tick_ms;
    (void)ipsr;
}

void WatchDiagSetErrorArgs(uint32_t arg0, uint32_t arg1)
{
    (void)arg0;
    (void)arg1;
}

void WatchDiagMarkFatal(uint32_t reason, uint32_t task_handle, const char *task_name)
{
    (void)reason;
    (void)task_handle;
    (void)task_name;
}

void WatchTaskModuleCreateReset(void) {}
void WatchTaskModuleCreateResult(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state)
{
    (void)module;
    (void)name;
    (void)thread_handle;
    (void)state;
}

void WatchTaskBeat(WatchTaskId task_id) { (void)task_id; }
void WatchTaskWait(WatchTaskId task_id) { (void)task_id; }
void WatchTaskTimeout(WatchTaskId task_id) { (void)task_id; }
void WatchTaskError(WatchTaskId task_id) { (void)task_id; }
void WatchIrqHit(WatchIrqId irq_id) { (void)irq_id; }

const WatchBlockDesc *WatchGetBlockTable(uint32_t *count)
{
    if (count != 0)
    {
        *count = 0u;
    }
    return 0;
}

const WatchBlockDesc *WatchFindBlock(WatchBlockId id)
{
    (void)id;
    return 0;
}

uint8_t WatchBlockIsActive(WatchBlockId id)
{
    (void)id;
    return 0u;
}
