/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef RT_PROF_H
#define RT_PROF_H

#include <stdint.h>

#include "BspDwt.h"

#ifndef RT_PROFILER_ENABLE
#define RT_PROFILER_ENABLE 1u
#endif

typedef enum
{
    RtProfGimbalLoop = 0,
    RtProfChassisLoop,
    RtProfWheellegMitLoop,
    RtProfCanTxLoop,
    RtProfCanRxWake,
    RtProfSdLogWrite,
    RtProfSdLogCompress,
    RtProfSdLogBlockWrite,
    RtProfSdLogSync,
    RtProfWatchBeat,
    RtProfCount
} RtProfId;

typedef struct
{
    uint32_t count;
    uint32_t last_us;
    uint32_t max_us;
    uint32_t avg_us;
    uint32_t budget_us;
    uint32_t overrun_count;
} RtProfStats;

typedef enum
{
    RtProfKindLoop = 0u,
    RtProfKindWake,
    RtProfKindIo,
    RtProfKindService,
} RtProfKind;

#define RT_PROF_FAST_PATH (1u << 0)
#define RT_PROF_EVENT_DRIVEN (1u << 1)

typedef struct
{
    RtProfId id;
    uint8_t module;
    uint8_t module_alt;
    uint8_t kind;
    uint8_t flags;
    const char *name;
    uint32_t default_budget_us;
} RtProfDesc;

typedef struct
{
    uint8_t total_count;
    uint8_t active_count;
    uint8_t over_budget_count;
    uint8_t reserved0;
    uint32_t total_overrun_count;
    uint32_t max_last_us;
    uint32_t max_budget_us;
    uint32_t max_over_budget_us;
} RtProfSummary;

static inline uint64_t RtProfBegin(void)
{
#if RT_PROFILER_ENABLE
    return BSP_DWT_GetUs();
#else
    return 0u;
#endif
}

void RtProfRecord(RtProfId id, uint32_t elapsed_us);
void RtProfEnd(RtProfId id, uint64_t start_us);
void RtProfReset(RtProfId id);
void RtProfResetAll(void);
void RtProfSetBudgetUs(RtProfId id, uint32_t budget_us);
void RtProfGet(RtProfId id, RtProfStats *out);
const RtProfDesc *RtProfDescs(uint8_t *count);
const RtProfDesc *RtProfDescGet(RtProfId id);
uint32_t RtProfPeriodMs(RtProfId id);
uint32_t RtProfBudgetUs(RtProfId id);
uint8_t RtProfOverBudget(RtProfId id);
uint8_t RtProfActive(RtProfId id);
void RtProfGetSummary(RtProfSummary *out);

#endif
