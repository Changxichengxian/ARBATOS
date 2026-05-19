/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "detect_task.h"

#include "config.h"
#include "motor_config.h"
#include "robot_task_profile.h"
#include "watch.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#include "sdlog.h"
#include "task.h"

#include <string.h>

// Minimal offline-detect implementation for the A-board port.
// Keeps the public API used by HERO modules (detect_hook/toe_is_error).
#define WATCH_UPDATE_PERIOD_MS 250u

static error_t g_error_list[ERROR_LIST_LENGHT];
static uint32_t g_last_tick_ms[ERROR_LIST_LENGHT];
static uint8_t g_detect_inited = 0u;

static uint8_t detect_toe_enabled_by_profile(uint8_t toe)
{
    switch (toe)
    {
    case CHASSIS_MOTOR1_TOE:
    case CHASSIS_MOTOR2_TOE:
    case CHASSIS_MOTOR3_TOE:
    case CHASSIS_MOTOR4_TOE:
        return robot_profile_need_classic_chassis_control_task();
    case YAW_GIMBAL_MOTOR_TOE:
    case PITCH_GIMBAL_MOTOR_TOE:
        return (uint8_t)(robot_profile_need_single_gimbal_control_task() ||
                         robot_profile_need_dual_gimbal_control_task());
    case TRIGGER_MOTOR_TOE:
        return (uint8_t)(motor_cfg_node_id(&g_config.motor.trigger) != 0u);
    default:
        return 1u;
    }
}

static void detect_init_once(void)
{
    if (g_detect_inited != 0u)
    {
        return;
    }
    g_detect_inited = 1u;

    memset(g_error_list, 0, sizeof(g_error_list));
    memset(g_last_tick_ms, 0, sizeof(g_last_tick_ms));

    for (uint8_t i = 0u; i < (uint8_t)ERROR_LIST_LENGHT; i++)
    {
        const detect_item_t *item = &g_config.detect.items[i];
        const uint8_t enabled_by_config = (uint8_t)((g_config.detect.enable_mask >> i) & 0x1u);
        const uint8_t enabled = (uint8_t)(enabled_by_config && detect_toe_enabled_by_profile(i));

        g_error_list[i].enable = enabled;
        g_error_list[i].priority = item->priority;
        g_error_list[i].set_online_time = item->online_time_ms;
        g_error_list[i].set_offline_time = item->offline_time_ms;
        g_error_list[i].error_exist = enabled;
        g_error_list[i].is_lost = enabled;
        g_error_list[i].data_is_error = enabled;
    }
}

void health_monitor_task(void const *pvParameters)
{
    detect_task(pvParameters);
}

void detect_hook(uint8_t toe)
{
    detect_init_once();

    if (toe >= (uint8_t)ERROR_LIST_LENGHT)
    {
        return;
    }

    const uint32_t now_ms = HAL_GetTick();
    g_last_tick_ms[toe] = now_ms;
    g_error_list[toe].new_time = now_ms;
    g_error_list[toe].last_time = now_ms;
    g_error_list[toe].error_exist = 0u;
    g_error_list[toe].is_lost = 0u;
    g_error_list[toe].data_is_error = 0u;
}

bool_t toe_is_error(uint8_t err)
{
    detect_init_once();

    if (err >= (uint8_t)ERROR_LIST_LENGHT)
    {
        return 1u;
    }

    const error_t *e = &g_error_list[err];
    if (e->enable == 0u)
    {
        return 0u;
    }

    const uint16_t offline_ms = e->set_offline_time;
    if (offline_ms == 0u)
    {
        return 0u;
    }

    const uint32_t last = g_last_tick_ms[err];
    if (last == 0u)
    {
        return 1u;
    }

    const uint32_t now_ms = HAL_GetTick();
    return ((uint32_t)(now_ms - last) > (uint32_t)offline_ms) ? 1u : 0u;
}

const error_t *get_error_list_point(void)
{
    detect_init_once();
    return g_error_list;
}

void detect_task(void const *pvParameters)
{
    (void)pvParameters;

    detect_init_once();
    watch_init();
    osDelay(DETECT_TASK_INIT_TIME);

    static uint8_t config_buf[sizeof(sdlog_config_header_t) + sizeof(g_config)];
    uint8_t config_logged = 0u;
    uint32_t last_watch_snapshot_tick = HAL_GetTick();

    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();
        for (uint8_t i = 0u; i < (uint8_t)ERROR_LIST_LENGHT; i++)
        {
            error_t *e = &g_error_list[i];
            if (e->enable == 0u || e->set_offline_time == 0u)
            {
                continue;
            }

            const uint32_t last = g_last_tick_ms[i];
            const uint8_t lost = (last == 0u) ? 1u : (((uint32_t)(now_ms - last) > (uint32_t)e->set_offline_time) ? 1u : 0u);
            e->is_lost = lost;
            e->error_exist = lost;
        }

        // Log configuration snapshot once after boot (when SD log is active).
        if (!config_logged && sdlog_is_active())
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
            sdlog_write(SDLOG_TAG_CONFIG, config_buf, cfg_size);
            config_logged = 1u;
        }

        if ((uint32_t)(now_ms - last_watch_snapshot_tick) >= WATCH_UPDATE_PERIOD_MS)
        {
            last_watch_snapshot_tick = now_ms;
            watch_update();
        }

        osDelay(DETECT_CONTROL_TIME);
    }
}
