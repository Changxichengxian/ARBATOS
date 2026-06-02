/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "rt_profiler.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "robot_task_profile.h"

static const rt_profiler_desc_t s_rt_profiler_desc[RT_PROFILER_COUNT] = {
    {RT_PROFILER_GIMBAL_CONTROL_LOOP,
     (uint8_t)ROBOT_TASK_MODULE_SINGLE_GIMBAL,
     (uint8_t)ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL,
     (uint8_t)RT_PROFILER_KIND_LOOP,
     (uint8_t)RT_PROFILER_FLAG_FAST_PATH,
     "prof.gimbal_control_loop",
     ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US},
    {RT_PROFILER_CHASSIS_CONTROL_LOOP,
     (uint8_t)ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_LOOP,
     (uint8_t)RT_PROFILER_FLAG_FAST_PATH,
     "prof.chassis_control_loop",
     ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US},
    {RT_PROFILER_CAN_COMMAND_TX_LOOP,
     (uint8_t)ROBOT_TASK_MODULE_CAN_COMMAND_TX,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_LOOP,
     (uint8_t)RT_PROFILER_FLAG_FAST_PATH,
     "prof.can_command_tx_loop",
     ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US},
    {RT_PROFILER_CAN_FEEDBACK_RX_WAKE,
     (uint8_t)ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_WAKE,
     (uint8_t)(RT_PROFILER_FLAG_FAST_PATH | RT_PROFILER_FLAG_EVENT_DRIVEN),
     "prof.can_feedback_rx_wake",
     ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US},
    {RT_PROFILER_SDLOG_WRITE,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_IO,
     0u,
     "prof.sdlog_write",
     ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US},
    {RT_PROFILER_SDLOG_COMPRESS,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_IO,
     0u,
     "prof.sdlog_compress",
     ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US},
    {RT_PROFILER_SDLOG_BLOCK_WRITE,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_IO,
     0u,
     "prof.sdlog_block_write",
     ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US},
    {RT_PROFILER_SDLOG_SYNC,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_IO,
     0u,
     "prof.sdlog_sync",
     ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US},
    {RT_PROFILER_WATCH_TASK_BEAT,
     (uint8_t)ROBOT_TASK_MODULE_HEALTH_MONITOR,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RT_PROFILER_KIND_SERVICE,
     0u,
     "prof.watch_task_beat",
     ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US},
};

static uint8_t rt_profiler_id_valid(rt_profiler_id_e id)
{
    return ((uint32_t)id < (uint32_t)RT_PROFILER_COUNT) ? 1u : 0u;
}

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

#endif

const rt_profiler_desc_t *rt_profiler_descriptors(uint8_t *count)
{
    if (count != NULL)
    {
        *count = (uint8_t)RT_PROFILER_COUNT;
    }

    return s_rt_profiler_desc;
}

const rt_profiler_desc_t *rt_profiler_descriptor(rt_profiler_id_e id)
{
    if (rt_profiler_id_valid(id) == 0u)
    {
        return NULL;
    }

    return &s_rt_profiler_desc[id];
}

uint32_t rt_profiler_period_ms(rt_profiler_id_e id)
{
    switch (id)
    {
    case RT_PROFILER_GIMBAL_CONTROL_LOOP:
        return (uint32_t)robot_profile_gimbal_control_period_ms();
    case RT_PROFILER_CHASSIS_CONTROL_LOOP:
        return (uint32_t)robot_profile_chassis_control_period_ms();
    case RT_PROFILER_CAN_COMMAND_TX_LOOP:
        return (uint32_t)robot_profile_can_command_tx_period_ms();
    case RT_PROFILER_WATCH_TASK_BEAT:
        return (uint32_t)ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS;
    default:
        return 0u;
    }
}

uint32_t rt_profiler_budget_us(rt_profiler_id_e id)
{
#if RT_PROFILER_ENABLE
    uint32_t budget_us;

    if (rt_profiler_id_valid(id) == 0u)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    budget_us = s_rt_profiler[id].budget_us;
    taskEXIT_CRITICAL();
    return budget_us;
#else
    const rt_profiler_desc_t *desc = rt_profiler_descriptor(id);

    return (desc == NULL) ? 0u : desc->default_budget_us;
#endif
}

uint8_t rt_profiler_over_budget(rt_profiler_id_e id)
{
    rt_profiler_stats_t stats;

    rt_profiler_get(id, &stats);
    return (uint8_t)(stats.budget_us != 0u && stats.last_us > stats.budget_us);
}

uint8_t rt_profiler_active(rt_profiler_id_e id)
{
    const rt_profiler_desc_t *desc = rt_profiler_descriptor(id);

    if (desc == NULL)
    {
        return 0u;
    }
    if (desc->module == (uint8_t)ROBOT_TASK_MODULE_NONE &&
        desc->module_alt == (uint8_t)ROBOT_TASK_MODULE_NONE)
    {
        return 1u;
    }

    return (uint8_t)(robot_profile_module_id_enabled(desc->module) ||
                     robot_profile_module_id_enabled(desc->module_alt));
}

void rt_profiler_get_summary(rt_profiler_summary_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->total_count = (uint8_t)RT_PROFILER_COUNT;

    for (uint8_t i = 0u; i < (uint8_t)RT_PROFILER_COUNT; i++)
    {
        rt_profiler_stats_t stats;
        const rt_profiler_id_e id = (rt_profiler_id_e)i;
        uint32_t over_budget_us = 0u;

        if (rt_profiler_active(id) == 0u)
        {
            continue;
        }

        out->active_count++;
        rt_profiler_get(id, &stats);
        out->total_overrun_count += stats.overrun_count;
        if (stats.budget_us != 0u && stats.last_us > stats.budget_us)
        {
            out->over_budget_count++;
            over_budget_us = stats.last_us - stats.budget_us;
        }
        if (stats.last_us > out->max_last_us)
        {
            out->max_last_us = stats.last_us;
            out->max_budget_us = stats.budget_us;
        }
        if (over_budget_us > out->max_over_budget_us)
        {
            out->max_over_budget_us = over_budget_us;
        }
    }
}

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
