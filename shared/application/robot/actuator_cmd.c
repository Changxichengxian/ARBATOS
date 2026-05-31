/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "actuator_cmd.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

static actuator_cmd_t g_actuator_cmd[ACTUATOR_ID__COUNT];
static actuator_feedback_t g_actuator_feedback[ACTUATOR_ID__COUNT];

static uint8_t actuator_id_valid(actuator_id_e id)
{
    return ((uint32_t)id < (uint32_t)ACTUATOR_ID__COUNT) ? 1u : 0u;
}

void actuator_cmd_clear_all(void)
{
    taskENTER_CRITICAL();
    (void)memset(g_actuator_cmd, 0, sizeof(g_actuator_cmd));
    taskEXIT_CRITICAL();
}

void actuator_cmd_clear(actuator_id_e id)
{
    if (actuator_id_valid(id) == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memset(&g_actuator_cmd[id], 0, sizeof(g_actuator_cmd[id]));
    taskEXIT_CRITICAL();
}

void actuator_cmd_set_current(actuator_id_e id, int16_t current)
{
    actuator_cmd_t cmd;

    if (actuator_id_valid(id) == 0u)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
    cmd.current = current;
    taskENTER_CRITICAL();
    g_actuator_cmd[id] = cmd;
    taskEXIT_CRITICAL();
}

int16_t actuator_cmd_get_current(actuator_id_e id)
{
    int16_t current;

    if (actuator_id_valid(id) == 0u)
    {
        return 0;
    }

    taskENTER_CRITICAL();
    current = g_actuator_cmd[id].current;
    taskEXIT_CRITICAL();
    return current;
}

void actuator_cmd_set_state_torque(actuator_id_e id, const actuator_cmd_t *cmd)
{
    actuator_cmd_t tmp;

    if (actuator_id_valid(id) == 0u || cmd == NULL)
    {
        return;
    }

    tmp = *cmd;
    tmp.active = 1u;
    tmp.mode = (uint8_t)ACTUATOR_CMD_MODE_STATE_TORQUE;
    taskENTER_CRITICAL();
    g_actuator_cmd[id] = tmp;
    taskEXIT_CRITICAL();
}

void actuator_cmd_set_speed(actuator_id_e id, fp32 velocity, fp32 kd, fp32 torque)
{
    actuator_cmd_t cmd;

    if (actuator_id_valid(id) == 0u)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_SPEED;
    cmd.velocity = velocity;
    cmd.kd = kd;
    cmd.torque = torque;
    taskENTER_CRITICAL();
    g_actuator_cmd[id] = cmd;
    taskEXIT_CRITICAL();
}

uint8_t actuator_cmd_get_copy(actuator_id_e id, actuator_cmd_t *out)
{
    if (actuator_id_valid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    *out = g_actuator_cmd[id];
    taskEXIT_CRITICAL();
    return 1u;
}

const actuator_cmd_t *actuator_cmd_get_ptr(actuator_id_e id)
{
    if (actuator_id_valid(id) == 0u)
    {
        return NULL;
    }

    return &g_actuator_cmd[id];
}

void actuator_feedback_clear_all(void)
{
    taskENTER_CRITICAL();
    (void)memset(g_actuator_feedback, 0, sizeof(g_actuator_feedback));
    taskEXIT_CRITICAL();
}

void actuator_feedback_update(actuator_id_e id, const actuator_feedback_t *feedback)
{
    if (actuator_id_valid(id) == 0u || feedback == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_actuator_feedback[id] = *feedback;
    taskEXIT_CRITICAL();
}

uint8_t actuator_feedback_get_copy(actuator_id_e id, actuator_feedback_t *out)
{
    if (actuator_id_valid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    *out = g_actuator_feedback[id];
    taskEXIT_CRITICAL();
    return 1u;
}

const actuator_feedback_t *actuator_feedback_get_ptr(actuator_id_e id)
{
    if (actuator_id_valid(id) == 0u)
    {
        return NULL;
    }

    return &g_actuator_feedback[id];
}
