/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CpuUsage.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>

static uint64_t cpu_last_total_cycles;
static uint64_t cpu_last_busy_cycles;
static uint16_t cpu_last_permille;

void CpuUsageInit(void)
{
    k_thread_runtime_stats_t stats;
    if (k_thread_runtime_stats_cpu_get(0, &stats) == 0)
    {
        cpu_last_total_cycles = stats.execution_cycles;
        cpu_last_busy_cycles = stats.total_cycles;
    }
    cpu_last_permille = 0u;
}

uint16_t CpuUsageGetPermille(void)
{
    k_thread_runtime_stats_t stats;
    uint64_t total, busy;
    if (k_thread_runtime_stats_cpu_get(0, &stats) != 0) return cpu_last_permille;
    total = stats.execution_cycles - cpu_last_total_cycles;
    busy = stats.total_cycles - cpu_last_busy_cycles;
    cpu_last_total_cycles = stats.execution_cycles;
    cpu_last_busy_cycles = stats.total_cycles;
    if (total != 0u)
    {
        if (busy > total) busy = total;
        cpu_last_permille = (uint16_t)((busy * 1000u) / total);
    }
    return cpu_last_permille;
}
#else
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t cpu_total_ticks = 0u;
static volatile uint32_t cpu_idle_ticks = 0u;
static TaskHandle_t idle_task_handle = NULL;

static uint32_t cpu_last_total_ticks = 0u;
static uint32_t cpu_last_idle_ticks = 0u;
static uint16_t cpu_last_permille = 0u;

void CpuUsageInit(void)
{
    idle_task_handle = xTaskGetIdleTaskHandle();
    taskENTER_CRITICAL();
    cpu_last_total_ticks = cpu_total_ticks;
    cpu_last_idle_ticks = cpu_idle_ticks;
    taskEXIT_CRITICAL();
    cpu_last_permille = 0u;
}

void vApplicationTickHook(void)
{
    cpu_total_ticks++;

    TaskHandle_t idle = idle_task_handle;
    if (idle == NULL)
    {
        idle = xTaskGetIdleTaskHandle();
        idle_task_handle = idle;
    }

    if (xTaskGetCurrentTaskHandle() == idle)
    {
        cpu_idle_ticks++;
    }
}

uint16_t CpuUsageGetPermille(void)
{
    uint32_t total = 0u;
    uint32_t idle = 0u;
    taskENTER_CRITICAL();
    total = cpu_total_ticks;
    idle = cpu_idle_ticks;
    taskEXIT_CRITICAL();

    const uint32_t total_delta = total - cpu_last_total_ticks;
    const uint32_t idle_delta = idle - cpu_last_idle_ticks;

    cpu_last_total_ticks = total;
    cpu_last_idle_ticks = idle;

    if (total_delta == 0u)
    {
        return cpu_last_permille;
    }

    uint32_t busy_delta = total_delta;
    if (idle_delta < total_delta)
    {
        busy_delta = total_delta - idle_delta;
    }
    else
    {
        busy_delta = 0u;
    }

    uint32_t permille = (busy_delta * 1000u) / total_delta;
    if (permille > 1000u)
    {
        permille = 1000u;
    }

    cpu_last_permille = (uint16_t)permille;
    return cpu_last_permille;
}
#endif
