/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
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

static uint8_t actuator_cmd_copy_valid(const actuator_cmd_t *cmd)
{
    return (uint8_t)(cmd != NULL && actuator_cmd_mode_known(cmd->mode));
}

static void actuator_cmd_set_many_unchecked(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        g_actuator_cmd[ids[i]] = cmds[i];
    }
    taskEXIT_CRITICAL();
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

uint8_t actuator_cmd_set_many(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count)
{
    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_id_valid(ids[i]) == 0u || actuator_cmd_copy_valid(&cmds[i]) == 0u)
        {
            return 0u;
        }
    }

    actuator_cmd_set_many_unchecked(ids, cmds, count);
    return 1u;
}

uint8_t actuator_cmd_set_copy(actuator_id_e id, const actuator_cmd_t *cmd)
{
    return actuator_cmd_set_many(&id, cmd, 1u);
}

const char *actuator_cmd_mode_name(actuator_cmd_mode_e mode)
{
    switch (mode)
    {
    case ACTUATOR_CMD_MODE_NONE:
        return "none";
    case ACTUATOR_CMD_MODE_CURRENT:
        return "current";
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
        return "state_torque";
    case ACTUATOR_CMD_MODE_POS_VEL:
        return "pos_vel";
    case ACTUATOR_CMD_MODE_SPEED:
        return "speed";
    case ACTUATOR_CMD_MODE_FORCE_POS:
        return "force_pos";
    default:
        return "unknown";
    }
}

void actuator_cmd_set_current(actuator_id_e id, int16_t current)
{
    actuator_cmd_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
    cmd.current = current;
    (void)actuator_cmd_set_copy(id, &cmd);
}

uint8_t actuator_cmd_set_current_many(const actuator_id_e *ids, const int16_t *currents, uint8_t count)
{
    actuator_cmd_t cmds[ACTUATOR_ID__COUNT];

    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || currents == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_id_valid(ids[i]) == 0u)
        {
            return 0u;
        }

        (void)memset(&cmds[i], 0, sizeof(cmds[i]));
        cmds[i].active = 1u;
        cmds[i].mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
        cmds[i].current = currents[i];
    }

    actuator_cmd_set_many_unchecked(ids, cmds, count);
    return 1u;
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

uint8_t actuator_cmd_get_current_many(const actuator_id_e *ids, int16_t *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_id_valid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = g_actuator_cmd[ids[i]].current;
    }
    taskEXIT_CRITICAL();
    return 1u;
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
    (void)actuator_cmd_set_copy(id, &tmp);
}

void actuator_cmd_set_speed(actuator_id_e id, fp32 velocity, fp32 kd, fp32 torque)
{
    actuator_cmd_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_SPEED;
    cmd.velocity = velocity;
    cmd.kd = kd;
    cmd.torque = torque;
    (void)actuator_cmd_set_copy(id, &cmd);
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

uint8_t actuator_cmd_get_copy_many(const actuator_id_e *ids, actuator_cmd_t *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_id_valid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = g_actuator_cmd[ids[i]];
    }
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

uint8_t actuator_feedback_get_copy_many(const actuator_id_e *ids, actuator_feedback_t *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_id_valid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = g_actuator_feedback[ids[i]];
    }
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
