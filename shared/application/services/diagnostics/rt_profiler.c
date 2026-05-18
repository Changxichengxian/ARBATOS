/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "rt_profiler.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "robot_task_profile.h"

#if RT_PROFILER_ENABLE

static rt_profiler_stats_t s_rt_profiler[RT_PROFILER_COUNT] = {
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US, 0u},
};

static uint8_t rt_profiler_id_valid(rt_profiler_id_e id)
{
    return ((uint32_t)id < (uint32_t)RT_PROFILER_COUNT) ? 1u : 0u;
}

#endif

void rt_profiler_record(rt_profiler_id_e id, uint32_t elapsed_us)
{
#if RT_PROFILER_ENABLE
    if (!rt_profiler_id_valid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    rt_profiler_stats_t *s = &s_rt_profiler[id];
    s->count++;
    s->last_us = elapsed_us;
    if (elapsed_us > s->max_us)
    {
        s->max_us = elapsed_us;
    }
    if (s->count == 1u)
    {
        s->avg_us = elapsed_us;
    }
    else if (elapsed_us >= s->avg_us)
    {
        s->avg_us += (elapsed_us - s->avg_us) >> 4;
    }
    else
    {
        s->avg_us -= (s->avg_us - elapsed_us) >> 4;
    }
    if (s->budget_us != 0u && elapsed_us > s->budget_us)
    {
        s->overrun_count++;
    }
    taskEXIT_CRITICAL();
#else
    (void)id;
    (void)elapsed_us;
#endif
}

void rt_profiler_end(rt_profiler_id_e id, uint64_t start_us)
{
#if RT_PROFILER_ENABLE
    const uint64_t now_us = BSP_DWT_GetUs();
    const uint64_t elapsed_us = now_us - start_us;
    const uint32_t elapsed_clamped = (elapsed_us > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)elapsed_us;
    rt_profiler_record(id, elapsed_clamped);
#else
    (void)id;
    (void)start_us;
#endif
}

void rt_profiler_reset(rt_profiler_id_e id)
{
#if RT_PROFILER_ENABLE
    if (!rt_profiler_id_valid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    const uint32_t budget_us = s_rt_profiler[id].budget_us;
    memset(&s_rt_profiler[id], 0, sizeof(s_rt_profiler[id]));
    s_rt_profiler[id].budget_us = budget_us;
    taskEXIT_CRITICAL();
#else
    (void)id;
#endif
}

void rt_profiler_reset_all(void)
{
#if RT_PROFILER_ENABLE
    for (uint32_t i = 0u; i < (uint32_t)RT_PROFILER_COUNT; i++)
    {
        rt_profiler_reset((rt_profiler_id_e)i);
    }
#endif
}

void rt_profiler_set_budget_us(rt_profiler_id_e id, uint32_t budget_us)
{
#if RT_PROFILER_ENABLE
    if (!rt_profiler_id_valid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    s_rt_profiler[id].budget_us = budget_us;
    taskEXIT_CRITICAL();
#else
    (void)id;
    (void)budget_us;
#endif
}

void rt_profiler_get(rt_profiler_id_e id, rt_profiler_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

#if RT_PROFILER_ENABLE
    if (!rt_profiler_id_valid(id))
    {
        memset(out, 0, sizeof(*out));
        return;
    }

    taskENTER_CRITICAL();
    *out = s_rt_profiler[id];
    taskEXIT_CRITICAL();
#else
    (void)id;
    memset(out, 0, sizeof(*out));
#endif
}
