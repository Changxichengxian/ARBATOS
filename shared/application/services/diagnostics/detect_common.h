/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DETECT_COMMON_H
#define DETECT_COMMON_H

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "detect_task.h"

#ifndef DETECT_COMMON_DEFAULT_OFFLINE_MS
#define DETECT_COMMON_DEFAULT_OFFLINE_MS 200u
#endif

#ifndef DETECT_COMMON_DBUS_OFFLINE_MS
#define DETECT_COMMON_DBUS_OFFLINE_MS 100u
#endif

#ifndef DETECT_COMMON_RUNTIME_POLL_MS
#define DETECT_COMMON_RUNTIME_POLL_MS 100u
#endif

static inline void detect_common_init_from_config(error_t *list,
                                                  uint32_t *last_tick_ms,
                                                  uint8_t count,
                                                  const detect_config_t *cfg,
                                                  uint32_t now_ms)
{
    if (list == NULL || last_tick_ms == NULL)
    {
        return;
    }

    (void)memset(list, 0, (size_t)count * sizeof(list[0]));
    (void)memset(last_tick_ms, 0, (size_t)count * sizeof(last_tick_ms[0]));

    for (uint8_t i = 0u; i < count; i++)
    {
        const uint8_t enabled = (cfg != NULL) ? (uint8_t)((cfg->enable_mask >> i) & 0x1u) : 1u;

        list[i].enable = enabled;
        list[i].priority = (cfg != NULL) ? cfg->items[i].priority : 0u;
        list[i].set_online_time = (cfg != NULL) ? cfg->items[i].online_time_ms : 0u;
        list[i].set_offline_time = (cfg != NULL) ? cfg->items[i].offline_time_ms : DETECT_COMMON_DEFAULT_OFFLINE_MS;
        list[i].error_exist = enabled;
        list[i].is_lost = enabled;
        list[i].data_is_error = enabled;
        list[i].new_time = now_ms;
        list[i].last_time = now_ms;
        list[i].lost_time = now_ms;
        list[i].work_time = now_ms;
    }

    if (cfg == NULL && (uint8_t)DBUS_TOE < count)
    {
        list[DBUS_TOE].set_offline_time = DETECT_COMMON_DBUS_OFFLINE_MS;
    }
}

static inline void detect_common_hook(error_t *list,
                                      uint32_t *last_tick_ms,
                                      uint8_t count,
                                      uint8_t toe,
                                      uint32_t now_ms)
{
    if (list == NULL || last_tick_ms == NULL || toe >= count)
    {
        return;
    }

    const uint8_t was_lost = list[toe].is_lost;

    list[toe].last_time = list[toe].new_time;
    list[toe].new_time = now_ms;
    last_tick_ms[toe] = now_ms;
    list[toe].is_lost = 0u;
    list[toe].error_exist = 0u;

    if (was_lost != 0u)
    {
        list[toe].work_time = now_ms;
    }

    if (list[toe].data_is_error_fun != NULL && list[toe].data_is_error_fun())
    {
        list[toe].error_exist = 1u;
        list[toe].data_is_error = 1u;
        if (list[toe].solve_data_error_fun != NULL)
        {
            list[toe].solve_data_error_fun();
        }
    }
    else
    {
        list[toe].data_is_error = 0u;
    }
}

static inline bool_t detect_common_is_error(error_t *list,
                                            const uint32_t *last_tick_ms,
                                            uint8_t count,
                                            uint8_t toe,
                                            uint32_t now_ms)
{
    error_t *e;
    uint32_t last;

    if (list == NULL || last_tick_ms == NULL || toe >= count)
    {
        return 1u;
    }

    e = &list[toe];
    if (e->enable == 0u || e->set_offline_time == 0u)
    {
        e->is_lost = 0u;
        e->error_exist = 0u;
        return 0u;
    }

    last = last_tick_ms[toe];
    if (last == 0u || (uint32_t)(now_ms - last) > (uint32_t)e->set_offline_time)
    {
        e->is_lost = 1u;
        e->error_exist = 1u;
        e->lost_time = now_ms;
        if (e->solve_lost_fun != NULL)
        {
            e->solve_lost_fun();
        }
        return 1u;
    }

    e->is_lost = 0u;
    if ((uint32_t)(now_ms - e->work_time) < (uint32_t)e->set_online_time)
    {
        e->error_exist = 1u;
        return 1u;
    }

    e->error_exist = e->data_is_error;
    if (e->new_time > e->last_time)
    {
        e->frequency = 1000.0f / (fp32)(e->new_time - e->last_time);
    }
    return (bool_t)e->error_exist;
}

static inline void detect_common_refresh_all(error_t *list,
                                             const uint32_t *last_tick_ms,
                                             uint8_t count,
                                             uint32_t now_ms)
{
    if (list == NULL || last_tick_ms == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        (void)detect_common_is_error(list, last_tick_ms, count, i, now_ms);
    }
}

#endif
