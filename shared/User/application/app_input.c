/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "app_input.h"

static app_input_t g_app_input;

static int16_t app_input_map_axis(const app_axis_map_t *map, const RC_ctrl_t *rc)
{
    if (map == NULL || rc == NULL)
    {
        return 0;
    }
    if (map->rc_ch >= 5u)
    {
        return 0;
    }

    int16_t value = rc->rc.ch[map->rc_ch];
    if (map->invert != 0u)
    {
        value = (int16_t)(-value);
    }
    return value;
}

static uint8_t app_input_map_switch(const app_switch_map_t *map, const RC_ctrl_t *rc)
{
    if (map == NULL || rc == NULL)
    {
        return RC_SW_UP;
    }
    if (map->rc_sw >= 2u)
    {
        return RC_SW_UP;
    }

    uint8_t value = (uint8_t)rc->rc.s[map->rc_sw];
    if (map->invert != 0u)
    {
        if (value == RC_SW_UP)
        {
            return RC_SW_DOWN;
        }
        if (value == RC_SW_DOWN)
        {
            return RC_SW_UP;
        }
    }
    return value;
}

void app_input_update_from_rc(const RC_ctrl_t *rc)
{
    if (rc == NULL)
    {
        for (uint32_t i = 0u; i < (uint32_t)APP_AXIS_COUNT; i++)
        {
            g_app_input.axis[i] = 0;
        }
        for (uint32_t i = 0u; i < (uint32_t)APP_SW_COUNT; i++)
        {
            g_app_input.sw[i] = RC_SW_UP;
        }
        return;
    }

    for (uint32_t i = 0u; i < (uint32_t)APP_AXIS_COUNT; i++)
    {
        g_app_input.axis[i] = app_input_map_axis(&g_app_config.input.axis[i], rc);
    }
    for (uint32_t i = 0u; i < (uint32_t)APP_SW_COUNT; i++)
    {
        g_app_input.sw[i] = app_input_map_switch(&g_app_config.input.sw[i], rc);
    }
}

const app_input_t *app_input_get(void)
{
    return &g_app_input;
}

int16_t app_input_axis(app_axis_e axis)
{
    if ((uint32_t)axis >= (uint32_t)APP_AXIS_COUNT)
    {
        return 0;
    }
    return g_app_input.axis[axis];
}

uint8_t app_input_switch(app_switch_e sw)
{
    if ((uint32_t)sw >= (uint32_t)APP_SW_COUNT)
    {
        return RC_SW_UP;
    }
    return g_app_input.sw[sw];
}
