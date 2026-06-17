/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RtProf.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "RobotTaskProfile.h"

static const RtProfDesc sRtProfDesc[RtProfCount] = {
    {RtProfGimbalLoop,
     (uint8_t)ROBOT_TASK_MODULE_SINGLE_GIMBAL,
     (uint8_t)ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL,
     (uint8_t)RtProfKindLoop,
     (uint8_t)RT_PROF_FAST_PATH,
     "prof.GimbalControlLoop",
     ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US},
    {RtProfChassisLoop,
     (uint8_t)ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindLoop,
     (uint8_t)RT_PROF_FAST_PATH,
     "prof.ChassisControlLoop",
     ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US},
    {RtProfWheellegMitLoop,
     (uint8_t)ROBOT_TASK_MODULE_WHEELLEG_MIT,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindLoop,
     (uint8_t)RT_PROF_FAST_PATH,
     "prof.WheelLegMitControlLoop",
     ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US},
    {RtProfCanTxLoop,
     (uint8_t)ROBOT_TASK_MODULE_CAN_COMMAND_TX,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindLoop,
     (uint8_t)RT_PROF_FAST_PATH,
     "prof.can_command_tx_loop",
     ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US},
    {RtProfCanRxWake,
     (uint8_t)ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindWake,
     (uint8_t)(RT_PROF_FAST_PATH | RT_PROF_EVENT_DRIVEN),
     "prof.can_feedback_rx_wake",
     ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US},
    {RtProfSdLogWrite,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindIo,
     0u,
     "prof.SdLogWrite",
     ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US},
    {RtProfSdLogCompress,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindIo,
     0u,
     "prof.sdlog_compress",
     ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US},
    {RtProfSdLogBlockWrite,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindIo,
     0u,
     "prof.sdlog_block_write",
     ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US},
    {RtProfSdLogSync,
     (uint8_t)ROBOT_TASK_MODULE_SDLOG,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindIo,
     0u,
     "prof.sdlog_sync",
     ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US},
    {RtProfWatchBeat,
     (uint8_t)ROBOT_TASK_MODULE_HEALTH_MONITOR,
     (uint8_t)ROBOT_TASK_MODULE_NONE,
     (uint8_t)RtProfKindService,
     0u,
     "prof.WatchTaskBeat",
     ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US},
};

static uint8_t RtProfIdValid(RtProfId id)
{
    return ((uint32_t)id < (uint32_t)RtProfCount) ? 1u : 0u;
}

#if RT_PROFILER_ENABLE

static RtProfStats sRtProf[RtProfCount] = {
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US, 0u},
    {0u, 0u, 0u, 0u, ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US, 0u},
};

#endif

const RtProfDesc *RtProfDescs(uint8_t *count)
{
    if (count != NULL)
    {
        *count = (uint8_t)RtProfCount;
    }

    return sRtProfDesc;
}

const RtProfDesc *RtProfDescGet(RtProfId id)
{
    if (RtProfIdValid(id) == 0u)
    {
        return NULL;
    }

    return &sRtProfDesc[id];
}

uint32_t RtProfPeriodMs(RtProfId id)
{
    switch (id)
    {
    case RtProfGimbalLoop:
        return (uint32_t)RobotProfileGimbalControlPeriodMs();
    case RtProfChassisLoop:
        return (uint32_t)RobotProfileChassisControlPeriodMs();
    case RtProfWheellegMitLoop:
        return (uint32_t)g_config.WheelLegMit.control_period_ms;
    case RtProfCanTxLoop:
        return (uint32_t)RobotProfileCanCommandTxPeriodMs();
    case RtProfWatchBeat:
        return (uint32_t)ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS;
    default:
        return 0u;
    }
}

uint32_t RtProfBudgetUs(RtProfId id)
{
#if RT_PROFILER_ENABLE
    uint32_t budget_us;

    if (RtProfIdValid(id) == 0u)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    budget_us = sRtProf[id].budget_us;
    taskEXIT_CRITICAL();
    return budget_us;
#else
    const RtProfDesc *desc = RtProfDescGet(id);

    return (desc == NULL) ? 0u : desc->default_budget_us;
#endif
}

uint8_t RtProfOverBudget(RtProfId id)
{
    RtProfStats stats;

    RtProfGet(id, &stats);
    return (uint8_t)(stats.budget_us != 0u && stats.last_us > stats.budget_us);
}

uint8_t RtProfActive(RtProfId id)
{
    const RtProfDesc *desc = RtProfDescGet(id);

    if (desc == NULL)
    {
        return 0u;
    }
    if (desc->module == (uint8_t)ROBOT_TASK_MODULE_NONE &&
        desc->module_alt == (uint8_t)ROBOT_TASK_MODULE_NONE)
    {
        return 1u;
    }

    return (uint8_t)(RobotProfileModuleIdEnabled(desc->module) ||
                     RobotProfileModuleIdEnabled(desc->module_alt));
}

void RtProfGetSummary(RtProfSummary *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->total_count = (uint8_t)RtProfCount;

    for (uint8_t i = 0u; i < (uint8_t)RtProfCount; i++)
    {
        RtProfStats stats;
        const RtProfId id = (RtProfId)i;
        uint32_t over_budget_us = 0u;

        if (RtProfActive(id) == 0u)
        {
            continue;
        }

        out->active_count++;
        RtProfGet(id, &stats);
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

void RtProfRecord(RtProfId id, uint32_t elapsed_us)
{
#if RT_PROFILER_ENABLE
    if (!RtProfIdValid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    RtProfStats *s = &sRtProf[id];
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

void RtProfEnd(RtProfId id, uint64_t start_us)
{
#if RT_PROFILER_ENABLE
    const uint64_t now_us = BSP_DWT_GetUs();
    const uint64_t elapsed_us = now_us - start_us;
    const uint32_t elapsed_clamped = (elapsed_us > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)elapsed_us;
    RtProfRecord(id, elapsed_clamped);
#else
    (void)id;
    (void)start_us;
#endif
}

void RtProfReset(RtProfId id)
{
#if RT_PROFILER_ENABLE
    if (!RtProfIdValid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    const uint32_t budget_us = sRtProf[id].budget_us;
    memset(&sRtProf[id], 0, sizeof(sRtProf[id]));
    sRtProf[id].budget_us = budget_us;
    taskEXIT_CRITICAL();
#else
    (void)id;
#endif
}

void RtProfResetAll(void)
{
#if RT_PROFILER_ENABLE
    for (uint32_t i = 0u; i < (uint32_t)RtProfCount; i++)
    {
        RtProfReset((RtProfId)i);
    }
#endif
}

void RtProfSetBudgetUs(RtProfId id, uint32_t budget_us)
{
#if RT_PROFILER_ENABLE
    if (!RtProfIdValid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    sRtProf[id].budget_us = budget_us;
    taskEXIT_CRITICAL();
#else
    (void)id;
    (void)budget_us;
#endif
}

void RtProfGet(RtProfId id, RtProfStats *out)
{
    if (out == NULL)
    {
        return;
    }

#if RT_PROFILER_ENABLE
    if (!RtProfIdValid(id))
    {
        memset(out, 0, sizeof(*out));
        return;
    }

    taskENTER_CRITICAL();
    *out = sRtProf[id];
    taskEXIT_CRITICAL();
#else
    (void)id;
    memset(out, 0, sizeof(*out));
#endif
}
