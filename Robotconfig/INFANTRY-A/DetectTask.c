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
// Keeps the public API used by HERO modules (DetectHook/toe_is_error).

#define WATCH_UPDATE_PERIOD_MS 1000u

static DetectError g_error_list[DETECT_ERROR_COUNT];
static uint32_t g_last_tick_ms[DETECT_ERROR_COUNT];
static uint8_t g_detect_inited = 0u;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t DetectTaskStack;
#endif

static void DetectInitOnce(void)
{
    if (g_detect_inited != 0u)
    {
        return;
    }
    g_detect_inited = 1u;

    DetectCommonInitFromConfig(g_error_list,
                                   g_last_tick_ms,
                                   (uint8_t)DETECT_ERROR_COUNT,
                                   &g_config.detect,
                                   HAL_GetTick());
}

void HealthMonitorTask(void const *pvParameters)
{
    DetectTask(pvParameters);
}

void DetectHook(uint8_t toe)
{
    DetectInitOnce();

    if (toe >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return;
    }

    DetectCommonHook(g_error_list, g_last_tick_ms, (uint8_t)DETECT_ERROR_COUNT, toe, HAL_GetTick());
}

bool_t toe_is_error(uint8_t err)
{
    DetectInitOnce();

    if (err >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return 1u;
    }

    return DetectCommonIsError(g_error_list,
                                  g_last_tick_ms,
                                  (uint8_t)DETECT_ERROR_COUNT,
                                  err,
                                  HAL_GetTick());
}

const DetectError *get_error_list_point(void)
{
    DetectInitOnce();
    return g_error_list;
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
        DetectCommonRefreshAll(g_error_list, g_last_tick_ms, (uint8_t)DETECT_ERROR_COUNT, now_ms);

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
