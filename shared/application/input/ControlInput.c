/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ControlInput.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <string.h>

static ControlInputState g_control_input;

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} ControlInputCriticalState;

static ControlInputCriticalState ControlInputEnterCritical(void)
{
    ControlInputCriticalState state;

    state.from_isr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.saved_mask = 0u;
    if (state.from_isr != 0u)
    {
        state.saved_mask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void ControlInputExitCritical(ControlInputCriticalState state)
{
    if (state.from_isr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.saved_mask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

static uint8_t ControlInputSanitizeSwitchPos(uint8_t pos)
{
    if (pos <= (uint8_t)MANUAL_INPUT_SWITCH_POS_MAX)
    {
        return pos;
    }
    return (uint8_t)MANUAL_INPUT_SWITCH_POS_UP;
}

static int16_t input_map_axis(const input_axis_map_t *map, const ManualInputState *rc)
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

void ControlInputUpdateFromManualInput(const ManualInputState *rc)
{
    ControlInputState next;

    memset(&next, 0, sizeof(next));
    if (rc == NULL)
    {
        for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
        {
            next.axis[i] = 0;
        }
        for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
        {
            next.sw[i] = RC_SW_UP;
        }
        ControlInputCriticalState critical = ControlInputEnterCritical();
        g_control_input = next;
        ControlInputExitCritical(critical);
        return;
    }

    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        next.axis[i] = input_map_axis(&g_config.input.axis[i], rc);
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        next.sw[i] = ControlInputMapSwitch(&g_config.input.sw[i], rc);
    }

    ControlInputCriticalState critical = ControlInputEnterCritical();
    g_control_input = next;
    ControlInputExitCritical(critical);
}

const ControlInputState *ControlInputGetState(void)
{
    return &g_control_input;
}

uint8_t ControlInputGetCopy(ControlInputState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    ControlInputCriticalState critical = ControlInputEnterCritical();
    *out = g_control_input;
    ControlInputExitCritical(critical);
    return 1u;
}

int16_t ControlInputAxis(input_axis_e axis)
{
    int16_t value;

    if ((uint32_t)axis >= (uint32_t)INPUT_AXIS_COUNT)
    {
        return 0;
    }

    ControlInputCriticalState critical = ControlInputEnterCritical();
    value = g_control_input.axis[axis];
    ControlInputExitCritical(critical);
    return value;
}

uint8_t ControlInputSwitch(input_switch_e sw)
{
    uint8_t value;

    if ((uint32_t)sw >= (uint32_t)INPUT_SW_COUNT)
    {
        return RC_SW_UP;
    }

    ControlInputCriticalState critical = ControlInputEnterCritical();
    value = g_control_input.sw[sw];
    ControlInputExitCritical(critical);
    return value;
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

void input_update_from_rc(const ManualInputState *rc)
{
    ControlInputUpdateFromManualInput(rc);
}

const ControlInputState *input_get(void)
{
    return ControlInputGetState();
}

uint8_t input_get_copy(ControlInputState *out)
{
    return ControlInputGetCopy(out);
}

int16_t input_axis(input_axis_e axis)
{
    return ControlInputAxis(axis);
}

uint8_t input_switch(input_switch_e sw)
{
    return ControlInputSwitch(sw);
}

uint8_t input_switch_pos_to_raw(uint8_t pos)
{
    return ControlInputSwitchPosToRaw(pos);
}

uint8_t input_switch_is_pos(uint16_t raw, uint8_t pos)
{
    return ControlInputSwitchIsPos(raw, pos);
}
