/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef RT_PROFILER_H
#define RT_PROFILER_H

#include <stdint.h>

#include "bsp_dwt.h"

#ifndef RT_PROFILER_ENABLE
#define RT_PROFILER_ENABLE 1u
#endif

typedef enum
{
    RT_PROFILER_GIMBAL_CONTROL_LOOP = 0,
    RT_PROFILER_CHASSIS_CONTROL_LOOP,
    RT_PROFILER_CAN_COMMAND_TX_LOOP,
    RT_PROFILER_CAN_FEEDBACK_RX_WAKE,
    RT_PROFILER_SDLOG_WRITE,
    RT_PROFILER_WATCH_TASK_BEAT,
    RT_PROFILER_COUNT
} rt_profiler_id_e;

typedef struct
{
    uint32_t count;
    uint32_t last_us;
    uint32_t max_us;
    uint32_t avg_us;
    uint32_t budget_us;
    uint32_t overrun_count;
} rt_profiler_stats_t;

static inline uint64_t rt_profiler_begin(void)
{
#if RT_PROFILER_ENABLE
    return BSP_DWT_GetUs();
#else
    return 0u;
#endif
}

void rt_profiler_record(rt_profiler_id_e id, uint32_t elapsed_us);
void rt_profiler_end(rt_profiler_id_e id, uint64_t start_us);
void rt_profiler_reset(rt_profiler_id_e id);
void rt_profiler_reset_all(void);
void rt_profiler_set_budget_us(rt_profiler_id_e id, uint32_t budget_us);
void rt_profiler_get(rt_profiler_id_e id, rt_profiler_stats_t *out);

#endif
