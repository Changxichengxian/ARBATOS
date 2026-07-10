/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ControlInput.h"

#include <string.h>

static uint8_t ControlInputSanitizeSwitchPos(uint8_t pos)
{
    if (pos <= (uint8_t)MANUAL_INPUT_SWITCH_POS_MAX)
    {
        return pos;
    }
    return (uint8_t)MANUAL_INPUT_SWITCH_POS_UP;
}

static int16_t ControlInputMapAxis(const input_axis_map_t *map, const ManualInputState *rc)
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

static uint8_t ControlInputMapSwitch(const input_switch_map_t *map, const ManualInputState *rc)
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

void ControlInputBuild(const ManualInputState *manual,
                       const input_config_t *config,
                       ControlInputState *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        out->sw[i] = RC_SW_UP;
    }
    if (manual == NULL || config == NULL)
    {
        return;
    }

    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        out->axis[i] = ControlInputMapAxis(&config->axis[i], manual);
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        out->sw[i] = ControlInputMapSwitch(&config->sw[i], manual);
    }
}

uint8_t ControlInputSwitchPosToRaw(uint8_t pos)
{
    switch (ControlInputSanitizeSwitchPos(pos))
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

uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos)
{
    return (uint8_t)(raw == (uint16_t)ControlInputSwitchPosToRaw(pos));
}
