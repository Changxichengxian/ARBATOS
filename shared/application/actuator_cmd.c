/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "actuator_cmd.h"

static volatile int16_t g_trigger_current_can1 = 0;
static volatile int16_t g_yaw_current_can1 = 0;
static volatile int16_t g_yaw_upper_current_can1 = 0;
static volatile int16_t g_pitch_current_can1 = 0;
static volatile int16_t g_chassis_current_can1[4] = {0};
static volatile int16_t g_friction_current_can2[4] = {0};

void actuator_cmd_set_chassis_current_can1(uint8_t index, int16_t current)
{
    if (index >= 4U)
    {
        return;
    }

    g_chassis_current_can1[index] = current;
}

int16_t actuator_cmd_get_chassis_current_can1(uint8_t index)
{
    if (index >= 4U)
    {
        return 0;
    }

    return g_chassis_current_can1[index];
}

void actuator_cmd_set_yaw_current_can1(int16_t current)
{
    g_yaw_current_can1 = current;
}

int16_t actuator_cmd_get_yaw_current_can1(void)
{
    return g_yaw_current_can1;
}

void actuator_cmd_set_yaw_upper_current_can1(int16_t current)
{
    g_yaw_upper_current_can1 = current;
}

int16_t actuator_cmd_get_yaw_upper_current_can1(void)
{
    return g_yaw_upper_current_can1;
}

void actuator_cmd_set_pitch_current_can1(int16_t current)
{
    g_pitch_current_can1 = current;
}

int16_t actuator_cmd_get_pitch_current_can1(void)
{
    return g_pitch_current_can1;
}

void actuator_cmd_set_trigger_current_can1(int16_t current)
{
    g_trigger_current_can1 = current;
}

int16_t actuator_cmd_get_trigger_current_can1(void)
{
    return g_trigger_current_can1;
}

void actuator_cmd_set_friction_current_can2(uint8_t index, int16_t current)
{
    if (index >= 4U)
    {
        return;
    }

    g_friction_current_can2[index] = current;
}

int16_t actuator_cmd_get_friction_current_can2(uint8_t index)
{
    if (index >= 4U)
    {
        return 0;
    }

    return g_friction_current_can2[index];
}
