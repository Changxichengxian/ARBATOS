/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "DetectTask.h"
#include "DetectCommon.h"

#include "RobotConfig.h"
#include "Watch.h"

#include "cmsis_os.h"
#include "main.h"

#include "SdLog.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

// Minimal offline-detect implementation for the A-board port.
// Keeps the public API used by HERO modules (DetectHook/DetectIsError).

#define WATCH_UPDATE_PERIOD_MS 1000u

typedef struct
{
    uint8_t fromIsr;
    UBaseType_t savedMask;
} DetectCriticalState;

static DetectRuntime g_detect;

static DetectCriticalState DetectEnterCritical(void)
{
    DetectCriticalState state;

    state.fromIsr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.savedMask = 0u;
    if (state.fromIsr != 0u)
    {
        state.savedMask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void DetectExitCritical(DetectCriticalState state)
{
    if (state.fromIsr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.savedMask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

static void DetectRefresh(uint32_t nowMs)
{
    for (uint8_t toe = 0u; toe < (uint8_t)DETECT_ERROR_COUNT; toe++)
    {
        DetectReceiptFact fact;
        DetectCriticalState critical = DetectEnterCritical();
        DetectCommonTakeFact(g_detect.receipt, &fact, (uint8_t)DETECT_ERROR_COUNT, toe);
        DetectExitCritical(critical);
        DetectCommonRefreshOne(&g_detect.working[toe], &fact, nowMs);
    }
}

static void DetectPublish(uint32_t nowMs)
{
    const uint8_t nextIndex = (uint8_t)(g_detect.activeIndex ^ 1u);
    uint32_t nextSeq = g_detect.publishSeq + 1u;
    if (nextSeq == 0u)
    {
        nextSeq = 1u;
    }
    DetectCommonPublish(&g_detect.snapshot[nextIndex],
                        g_detect.working,
                        (uint8_t)DETECT_ERROR_COUNT,
                        nowMs,
                        nextSeq);
    g_detect.publishSeq = nextSeq;
    DetectCriticalState critical = DetectEnterCritical();
    g_detect.activeIndex = nextIndex;
    DetectExitCritical(critical);
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t DetectTaskStack;
#endif

static void DetectInitOnce(void)
{
    if (g_detect.writerInitialized != 0u)
    {
        return;
    }
    DetectCommonInitFromConfig(g_detect.working,
                               (uint8_t)DETECT_ERROR_COUNT,
                               &g_config.detect,
                               HAL_GetTick());
    g_detect.writerInitialized = 1u;
    DetectPublish(HAL_GetTick());
}

void HealthMonitorTask(void const *pvParameters)
{
    DetectTask(pvParameters);
}

void DetectHook(uint8_t toe)
{
    if (toe >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return;
    }

    DetectCriticalState critical = DetectEnterCritical();
    DetectCommonHook(g_detect.receipt, (uint8_t)DETECT_ERROR_COUNT, toe, HAL_GetTick());
    DetectExitCritical(critical);
}

bool_t DetectIsError(uint8_t err)
{
    bool_t result;

    if (err >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return 1u;
    }

    DetectCriticalState critical = DetectEnterCritical();
    result = DetectCommonSnapshotIsError(&g_detect.snapshot[g_detect.activeIndex],
                                         (uint8_t)DETECT_ERROR_COUNT,
                                         err);
    DetectExitCritical(critical);
    return result;
}

uint8_t DetectSnapshotRead(DetectSnapshot *out)
{
    uint8_t valid;
    DetectCriticalState critical = DetectEnterCritical();
    valid = DetectCommonSnapshotRead(&g_detect.snapshot[g_detect.activeIndex], out);
    DetectExitCritical(critical);
    return valid;
}

uint8_t DetectSummaryRead(DetectSummary *out)
{
    uint8_t valid;
    DetectCriticalState critical = DetectEnterCritical();
    valid = DetectCommonSummaryRead(&g_detect.snapshot[g_detect.activeIndex], out);
    DetectExitCritical(critical);
    return valid;
}

void DetectTask(void const *pvParameters)
{
    (void)pvParameters;

    DetectInitOnce();
    WatchInit();
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_RUN);
    osDelay(DETECT_TASK_INIT_TIME);

    static uint8_t ConfigBuf[sizeof(sdlog_config_header_t) + sizeof(g_config)];
    uint8_t ConfigLogged = 0u;
    uint32_t last_watch_snapshot_ms = HAL_GetTick();

    for (;;)
    {
        WatchTaskBeat(WATCH_TASK_DETECT);
        const uint32_t now_ms = HAL_GetTick();
        DetectRefresh(now_ms);
        DetectPublish(now_ms);

        // Log configuration snapshot once after boot (when SD log is active).
        if (!ConfigLogged && SdLogIsActive())
        {
            const uint16_t cfg_size = (uint16_t)sizeof(ConfigBuf);
            taskENTER_CRITICAL();
            sdlog_config_header_t *cfg_hdr = (sdlog_config_header_t *)ConfigBuf;
            cfg_hdr->version = SDLOG_CONFIG_VERSION;
            cfg_hdr->header_size = (uint16_t)sizeof(*cfg_hdr);
            cfg_hdr->ConfigSize = (uint16_t)sizeof(g_config);
            cfg_hdr->flags = 0u;
            memcpy(ConfigBuf + sizeof(*cfg_hdr), &g_config, sizeof(g_config));
            taskEXIT_CRITICAL();
            SdLogWrite(SDLOG_TAG_CONFIG, ConfigBuf, cfg_size);
            ConfigLogged = 1u;
        }

        if ((uint32_t)(now_ms - last_watch_snapshot_ms) >= WATCH_UPDATE_PERIOD_MS)
        {
            last_watch_snapshot_ms = now_ms;
            WatchUpdate();
        }

#if INCLUDE_uxTaskGetStackHighWaterMark
        DetectTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
        osDelay(DETECT_COMMON_RUNTIME_POLL_MS);
    }
}
