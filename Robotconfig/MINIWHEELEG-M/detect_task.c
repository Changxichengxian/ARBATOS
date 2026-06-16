/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "detect_task.h"
#include "detect_common.h"

#include "config.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#include "SdLog.h"
#include "task.h"

#include <string.h>

// Minimal offline-detect implementation for the A-board port.
// Keeps the public API used by HERO modules (detect_hook/toe_is_error).

static detect_error_t g_error_list[DETECT_ERROR_COUNT];
static uint32_t g_last_tick_ms[DETECT_ERROR_COUNT];
static uint8_t g_detect_inited = 0u;

static void detect_init_once(void)
{
    if (g_detect_inited != 0u)
    {
        return;
    }
    g_detect_inited = 1u;

    detect_common_init_from_config(g_error_list,
                                   g_last_tick_ms,
                                   (uint8_t)DETECT_ERROR_COUNT,
                                   &g_config.detect,
                                   HAL_GetTick());
}

void health_monitor_task(void const *pvParameters)
{
    detect_task(pvParameters);
}

void detect_hook(uint8_t toe)
{
    detect_init_once();

    if (toe >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return;
    }

    detect_common_hook(g_error_list, g_last_tick_ms, (uint8_t)DETECT_ERROR_COUNT, toe, HAL_GetTick());
}

bool_t toe_is_error(uint8_t err)
{
    detect_init_once();

    if (err >= (uint8_t)DETECT_ERROR_COUNT)
    {
        return 1u;
    }

    return detect_common_is_error(g_error_list,
                                  g_last_tick_ms,
                                  (uint8_t)DETECT_ERROR_COUNT,
                                  err,
                                  HAL_GetTick());
}

const detect_error_t *get_error_list_point(void)
{
    detect_init_once();
    return g_error_list;
}

void detect_task(void const *pvParameters)
{
    (void)pvParameters;

    detect_init_once();
    osDelay(DETECT_TASK_INIT_TIME);

    static uint8_t config_buf[sizeof(sdlog_config_header_t) + sizeof(g_config)];
    uint8_t config_logged = 0u;

    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();
        detect_common_refresh_all(g_error_list, g_last_tick_ms, (uint8_t)DETECT_ERROR_COUNT, now_ms);

        // Log configuration snapshot once after boot (when SD log is active).
        if (!config_logged && SdLogIsActive())
        {
            const uint16_t cfg_size = (uint16_t)sizeof(config_buf);
            taskENTER_CRITICAL();
            sdlog_config_header_t *cfg_hdr = (sdlog_config_header_t *)config_buf;
            cfg_hdr->version = SDLOG_CONFIG_VERSION;
            cfg_hdr->header_size = (uint16_t)sizeof(*cfg_hdr);
            cfg_hdr->config_size = (uint16_t)sizeof(g_config);
            cfg_hdr->flags = 0u;
            memcpy(config_buf + sizeof(*cfg_hdr), &g_config, sizeof(g_config));
            taskEXIT_CRITICAL();
            SdLogWrite(SDLOG_TAG_CONFIG, config_buf, cfg_size);
            config_logged = 1u;
        }

        osDelay(DETECT_COMMON_RUNTIME_POLL_MS);
    }
}
