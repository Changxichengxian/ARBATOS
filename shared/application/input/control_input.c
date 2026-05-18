/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "control_input.h"

static control_input_state_t g_control_input;

static uint8_t input_switch_pos_sanitize(uint8_t pos)
{
    if (pos <= (uint8_t)MANUAL_INPUT_SWITCH_POS_MAX)
    {
        return pos;
    }
    return (uint8_t)MANUAL_INPUT_SWITCH_POS_UP;
}

static int16_t input_map_axis(const input_axis_map_t *map, const manual_input_state_t *rc)
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

static uint8_t input_map_switch(const input_switch_map_t *map, const manual_input_state_t *rc)
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

void control_input_update_from_manual_input(const manual_input_state_t *rc)
{
    if (rc == NULL)
    {
        for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
        {
            g_control_input.axis[i] = 0;
        }
        for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
        {
            g_control_input.sw[i] = RC_SW_UP;
        }
        return;
    }

    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        g_control_input.axis[i] = input_map_axis(&g_config.input.axis[i], rc);
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        g_control_input.sw[i] = input_map_switch(&g_config.input.sw[i], rc);
    }
}

const control_input_state_t *control_input_get_state(void)
{
    return &g_control_input;
}

int16_t control_input_axis(input_axis_e axis)
{
    if ((uint32_t)axis >= (uint32_t)INPUT_AXIS_COUNT)
    {
        return 0;
    }
    return g_control_input.axis[axis];
}

uint8_t control_input_switch(input_switch_e sw)
{
    if ((uint32_t)sw >= (uint32_t)INPUT_SW_COUNT)
    {
        return RC_SW_UP;
    }
    return g_control_input.sw[sw];
}

uint8_t control_input_switch_pos_to_raw(uint8_t pos)
{
    switch (input_switch_pos_sanitize(pos))
    {
    case MANUAL_INPUT_SWITCH_POS_DOWN:
        return RC_SW_DOWN;
    case MANUAL_INPUT_SWITCH_POS_MID:
        return RC_SW_MID;
    case MANUAL_INPUT_SWITCH_POS_UP:
    default:
        return RC_SW_UP;
    }
}

uint8_t control_input_switch_is_pos(uint16_t raw, uint8_t pos)
{
    return (uint8_t)(raw == (uint16_t)control_input_switch_pos_to_raw(pos));
}

void input_update_from_rc(const manual_input_state_t *rc)
{
    control_input_update_from_manual_input(rc);
}

const control_input_state_t *input_get(void)
{
    return control_input_get_state();
}

int16_t input_axis(input_axis_e axis)
{
    return control_input_axis(axis);
}

uint8_t input_switch(input_switch_e sw)
{
    return control_input_switch(sw);
}

uint8_t input_switch_pos_to_raw(uint8_t pos)
{
    return control_input_switch_pos_to_raw(pos);
}

uint8_t input_switch_is_pos(uint16_t raw, uint8_t pos)
{
    return control_input_switch_is_pos(raw, pos);
}
