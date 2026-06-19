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
#include "MotorConfig.h"
#include "RobotTaskProfile.h"
#include "Watch.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#include "SdLog.h"
#include "task.h"

#include <string.h>

// Minimal offline-detect implementation for the A-board port.
// Keeps the public API used by HERO modules (DetectHook/toe_is_error).
#define WATCH_UPDATE_PERIOD_MS 250u

static DetectError g_error_list[DETECT_ERROR_COUNT];
static uint32_t g_last_tick_ms[DETECT_ERROR_COUNT];
static uint8_t g_detect_inited = 0u;

static uint8_t DetectToeEnabledByProfile(uint8_t toe)
{
    switch (toe)
    {
    case CHASSIS_MOTOR1_TOE:
    case CHASSIS_MOTOR2_TOE:
    case CHASSIS_MOTOR3_TOE:
    case CHASSIS_MOTOR4_TOE:
        return RobotProfileNeedClassicChassisControlTask();
    case YAW_GIMBAL_MOTOR_TOE:
    case PITCH_GIMBAL_MOTOR_TOE:
        return (uint8_t)(RobotProfileNeedSingleGimbalControlTask() ||
                         RobotProfileNeedDualGimbalControlTask());
    case TRIGGER_MOTOR_TOE:
        return (uint8_t)(MotorCfgNodeId(&g_config.motor.trigger) != 0u);
    default:
        return 1u;
    }
}

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
    for (uint8_t i = 0u; i < (uint8_t)DETECT_ERROR_COUNT; i++)
    {
        g_error_list[i].enable = (uint8_t)(g_error_list[i].enable && DetectToeEnabledByProfile(i));
        g_error_list[i].error_exist = g_error_list[i].enable;
        g_error_list[i].is_lost = g_error_list[i].enable;
        g_error_list[i].data_is_error = g_error_list[i].enable;
    }
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
    osDelay(DETECT_TASK_INIT_TIME);

    static uint8_t ConfigBuf[sizeof(sdlog_config_header_t) + sizeof(g_config)];
    uint8_t ConfigLogged = 0u;
    uint32_t last_watch_snapshot_tick = HAL_GetTick();

    for (;;)
    {
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

        if ((uint32_t)(now_ms - last_watch_snapshot_tick) >= WATCH_UPDATE_PERIOD_MS)
        {
            last_watch_snapshot_tick = now_ms;
            WatchUpdate();
        }

        osDelay(DETECT_COMMON_RUNTIME_POLL_MS);
    }
}
