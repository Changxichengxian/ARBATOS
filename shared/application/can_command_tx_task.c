/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "can_command_tx_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "can_mit_motor_driver.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

typedef struct
{
    actuator_id_e actuator_id;
    const motor_node_param_t *node;
    int16_t current;
} can_tx_item_t;

__weak uint8_t can_tx_allow_can1_yaw_override(void);
__weak uint8_t can_tx_process_extra_item(uint8_t bus, const motor_node_param_t *node, int16_t current);

static bool_t can_tx_allow_chassis(void)
{
    const test_mode_e mode = (test_mode_e)g_config.test.mode;
    return (mode == TEST_MODE_NONE) || (mode == TEST_MODE_ENTERTAIN) || (mode == TEST_MODE_CHASSIS_ONLY);
}

static uint8_t can_tx_log_due(void)
{
    static uint16_t skip = 0u;
    const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
    const uint16_t log_period_ms = robot_profile_can_command_tx_log_period_ms();
    const uint16_t divisor = (log_period_ms + period_ms - 1u) / period_ms;

    if (skip != 0u)
    {
        skip--;
        return 0u;
    }

    skip = (divisor > 1u) ? (uint16_t)(divisor - 1u) : 0u;
    return 1u;
}

static fp32 can_tx_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    if (x > x_max)
    {
        return x_max;
    }
    return x;
}

static uint8_t can_tx_actuator_id_valid(actuator_id_e id)
{
    return ((uint32_t)id < (uint32_t)ACTUATOR_ID__COUNT) ? 1u : 0u;
}

static uint8_t can_tx_cmd_nonzero(const actuator_cmd_t *cmd)
{
    if (cmd == NULL || cmd->active == 0u)
    {
        return 0u;
    }
    if (cmd->current != 0)
    {
        return 1u;
    }
    if (cmd->position != 0.0f || cmd->velocity != 0.0f ||
        cmd->kp != 0.0f || cmd->kd != 0.0f || cmd->torque != 0.0f)
    {
        return 1u;
    }
    return 0u;
}

static int16_t can_tx_fp32_to_i16_saturated(fp32 x)
{
    if (x > 32767.0f)
    {
        return 32767;
    }
    if (x < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)x;
}

static uint8_t can_tx_node_bus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    if (node != NULL && (node->can_bus == 1u || node->can_bus == 2u))
    {
        return node->can_bus;
    }
    return fallback_bus;
}

static uint8_t can_tx_cmd_is_state_mode(const actuator_cmd_t *cmd)
{
    if (cmd == NULL || cmd->active == 0u)
    {
        return 0u;
    }

    switch ((actuator_cmd_mode_e)cmd->mode)
    {
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
    case ACTUATOR_CMD_MODE_POS_VEL:
    case ACTUATOR_CMD_MODE_SPEED:
    case ACTUATOR_CMD_MODE_FORCE_POS:
        return 1u;
    default:
        return 0u;
    }
}

static int16_t can_tx_build_rm_current_from_actuator(const can_tx_item_t *item)
{
    actuator_cmd_t cmd;
    actuator_feedback_t fb;
    fp32 current;

    if (item == NULL || item->node == NULL)
    {
        return 0;
    }

    if (actuator_cmd_get_copy(item->actuator_id, &cmd) == 0u ||
        can_tx_cmd_is_state_mode(&cmd) == 0u ||
        actuator_feedback_get_copy(item->actuator_id, &fb) == 0u)
    {
        return motor_cfg_limit_current_node(item->node, item->current);
    }

    // RM motors receive current. For MIT-style commands, kp/kd/torque are current-like terms here.
    switch ((actuator_cmd_mode_e)cmd.mode)
    {
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
    case ACTUATOR_CMD_MODE_POS_VEL:
    case ACTUATOR_CMD_MODE_FORCE_POS:
        current = cmd.kp * (cmd.position - fb.position) +
                  cmd.kd * (cmd.velocity - fb.velocity) +
                  cmd.torque;
        break;
    case ACTUATOR_CMD_MODE_SPEED:
        current = cmd.kd * (cmd.velocity - fb.velocity) + cmd.torque;
        break;
    default:
        current = (fp32)item->current;
        break;
    }

    return motor_cfg_limit_current_node(item->node, can_tx_fp32_to_i16_saturated(current));
}

static fp32 can_tx_current_to_mit_torque(const motor_node_param_t *node,
                                         int16_t current,
                                         const can_mit_motor_limits_t *limits)
{
    const motor_model_db_entry_t *entry;
    int16_t range_abs = 0;
    fp32 torque;

    if (node == NULL || limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0.0f;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry != NULL)
    {
        range_abs = entry->cmd_current_range_abs;
    }
    if (range_abs <= 0)
    {
        range_abs = 32767;
    }

    torque = ((fp32)current) * limits->torque_max / (fp32)range_abs;
    return can_tx_clamp_fp32(torque, -limits->torque_max, limits->torque_max);
}

static void can_tx_build_mit_cmd_from_actuator(const motor_node_param_t *node,
                                               const actuator_cmd_t *src,
                                               int16_t current,
                                               const can_mit_motor_limits_t *limits,
                                               mit_motor_cmd_t *out)
{
    uint8_t mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (src != NULL && src->active != 0u && src->mode != (uint8_t)ACTUATOR_CMD_MODE_NONE)
    {
        mode = src->mode;
    }

    switch ((actuator_cmd_mode_e)mode)
    {
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
    case ACTUATOR_CMD_MODE_POS_VEL:
    case ACTUATOR_CMD_MODE_FORCE_POS:
        out->position = (src != NULL) ? src->position : 0.0f;
        out->velocity = (src != NULL) ? src->velocity : 0.0f;
        out->kp = (src != NULL) ? src->kp : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->torque : 0.0f;
        break;
    case ACTUATOR_CMD_MODE_SPEED:
        out->velocity = (src != NULL) ? src->velocity : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->torque : 0.0f;
        break;
    case ACTUATOR_CMD_MODE_CURRENT:
    default:
        out->torque = can_tx_current_to_mit_torque(node, current, limits);
        break;
    }
}

static uint8_t can_tx_process_can_mit_item(uint8_t bus,
                                           actuator_id_e actuator_id,
                                           const motor_node_param_t *node,
                                           int16_t current)
{
    static uint8_t mit_enabled[ACTUATOR_ID__COUNT];
    const can_mit_motor_limits_t *limits;
    actuator_cmd_t cmd;
    mit_motor_cmd_t mit_cmd;
    uint8_t have_cmd;
    uint16_t std_id;

    if (node == NULL)
    {
        return 0u;
    }

    limits = motor_cfg_mit_limits(node);
    if (limits == NULL)
    {
        return 0u;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    have_cmd = actuator_cmd_get_copy(actuator_id, &cmd);
    if (have_cmd == 0u || cmd.active == 0u)
    {
        cmd.active = 1u;
        cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
        cmd.current = current;
    }

    std_id = motor_cfg_can_id(node);
    if (std_id == 0u)
    {
        return 0u;
    }

    if (can_tx_actuator_id_valid(actuator_id) != 0u &&
        mit_enabled[actuator_id] == 0u &&
        can_tx_cmd_nonzero(&cmd) != 0u)
    {
        can_mit_motor_send_enable(bus, std_id);
        mit_enabled[actuator_id] = 1u;
    }

    if (can_tx_actuator_id_valid(actuator_id) != 0u &&
        mit_enabled[actuator_id] == 0u &&
        can_tx_cmd_nonzero(&cmd) == 0u)
    {
        return 1u;
    }

    can_tx_build_mit_cmd_from_actuator(node, &cmd, current, limits, &mit_cmd);
    can_mit_motor_send_cmd(bus, std_id, limits, &mit_cmd);
    return 1u;
}

static uint8_t can_tx_process_non_rm_item(uint8_t bus, const can_tx_item_t *item)
{
    uint8_t node_bus;

    if (item == NULL || item->node == NULL)
    {
        return 0u;
    }

    node_bus = can_tx_node_bus(bus, item->node);
    if (can_tx_process_can_mit_item(node_bus, item->actuator_id, item->node, item->current) != 0u)
    {
        return 1u;
    }

    return can_tx_process_extra_item(node_bus, item->node, item->current);
}

static void can_tx_log_actuator_current(const sdlog_actuator_current_t *log)
{
    if (log != NULL && can_tx_log_due() != 0u)
    {
        sdlog_write(SDLOG_TAG_ACTUATOR_CURRENT, log, (uint16_t)sizeof(*log));
    }
}

static void can_tx_limit_friction_currents(int16_t fric[4])
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.shoot.fric_motor_dir[i] == 0)
        {
            fric[i] = 0;
        }
    }
}

static void can_tx_pack_rm_frames(uint8_t bus,
                                  const can_tx_item_t *items,
                                  uint32_t count,
                                  int16_t out_200[4],
                                  int16_t out_1ff[4])
{
    (void)memset(out_200, 0, sizeof(int16_t) * 4u);
    (void)memset(out_1ff, 0, sizeof(int16_t) * 4u);

    if (items == NULL)
    {
        return;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (motor_cfg_is_rm_group_protocol(items[i].node) == 0u)
        {
            if (can_tx_process_non_rm_item(bus, &items[i]) == 0u)
            {
                watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
            }
            continue;
        }
        const uint16_t can_id = motor_cfg_can_id(items[i].node);
        const int16_t current = can_tx_build_rm_current_from_actuator(&items[i]);
        if (can_id >= 0x201u && can_id <= 0x204u)
        {
            out_200[can_id - 0x201u] = current;
        }
        else if (can_id >= 0x205u && can_id <= 0x208u)
        {
            out_1ff[can_id - 0x205u] = current;
        }
    }
}

__weak uint8_t can_tx_allow_can1_yaw_override(void)
{
    return 0u;
}

__weak uint8_t can_tx_process_extra_item(uint8_t bus, const motor_node_param_t *node, int16_t current)
{
    (void)bus;
    (void)node;
    (void)current;
    return 0u;
}

void can_command_tx_task(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        watch_task_beat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
        const bool_t dbus_offline = toe_is_error(DBUS_TOE);
        const test_mode_e mode = (test_mode_e)g_config.test.mode;
        const bool_t allow_friction_offline = (mode == TEST_MODE_ENTERTAIN);

        if (dbus_offline)
        {
            int16_t fric[4] = {0};
            int16_t yaw = 0;
            if (allow_friction_offline)
            {
                fric[0] = actuator_cmd_get_friction_current(0);
                fric[1] = actuator_cmd_get_friction_current(1);
                fric[2] = actuator_cmd_get_friction_current(2);
                fric[3] = actuator_cmd_get_friction_current(3);
            }
            if (can_tx_allow_can1_yaw_override() != 0u)
            {
                yaw = actuator_cmd_get_yaw_current();
            }

            can_tx_limit_friction_currents(fric);

            sdlog_actuator_current_t log = {0};
            log.yaw = yaw;
            log.friction[0] = fric[0];
            log.friction[1] = fric[1];
            log.friction[2] = fric[2];
            log.friction[3] = fric[3];
            can_tx_log_actuator_current(&log);

            CAN_cmd_rm_group(1u, (uint16_t)CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
            {
                const can_tx_item_t can1_items[] = {
                    {ACTUATOR_ID_YAW, &g_config.motor.yaw, yaw},
                };
                int16_t can1_200[4] = {0};
                int16_t can1_1ff[4] = {0};

                can_tx_pack_rm_frames(1u,
                                      can1_items,
                                      (uint32_t)(sizeof(can1_items) / sizeof(can1_items[0])),
                                      can1_200,
                                      can1_1ff);
                CAN_cmd_rm_group(1u, (uint16_t)CAN_GIMBAL_ALL_ID, can1_1ff[0], can1_1ff[1], can1_1ff[2], can1_1ff[3]);
            }
            CAN_cmd_rm_group(2u, (uint16_t)CAN_CHASSIS_ALL_ID, fric[0], fric[1], fric[2], fric[3]);
            CAN_cmd_rm_group(2u, (uint16_t)CAN_GIMBAL_ALL_ID, 0, 0, 0, 0);
        }
        else
        {
            const bool_t allow_chassis = can_tx_allow_chassis();

            int16_t chassis[4] = {0};
            int16_t yaw = 0;
            int16_t yaw_upper = 0;
            int16_t pitch = 0;
            int16_t trigger = 0;
            int16_t fric[4] = {0};

            chassis[0] = allow_chassis ? actuator_cmd_get_chassis_current(0) : 0;
            chassis[1] = allow_chassis ? actuator_cmd_get_chassis_current(1) : 0;
            chassis[2] = allow_chassis ? actuator_cmd_get_chassis_current(2) : 0;
            chassis[3] = allow_chassis ? actuator_cmd_get_chassis_current(3) : 0;
            yaw = actuator_cmd_get_yaw_current();
            yaw_upper = actuator_cmd_get_yaw_upper_current();
            pitch = actuator_cmd_get_pitch_current();
            trigger = actuator_cmd_get_trigger_current();
            fric[0] = actuator_cmd_get_friction_current(0);
            fric[1] = actuator_cmd_get_friction_current(1);
            fric[2] = actuator_cmd_get_friction_current(2);
            fric[3] = actuator_cmd_get_friction_current(3);

            can_tx_limit_friction_currents(fric);

            sdlog_actuator_current_t log = {0};
            log.chassis[0] = chassis[0];
            log.chassis[1] = chassis[1];
            log.chassis[2] = chassis[2];
            log.chassis[3] = chassis[3];
            log.yaw = yaw;
            log.pitch = pitch;
            log.trigger = trigger;
            log.friction[0] = fric[0];
            log.friction[1] = fric[1];
            log.friction[2] = fric[2];
            log.friction[3] = fric[3];
            can_tx_log_actuator_current(&log);

            {
                const can_tx_item_t can1_items[] = {
                    {ACTUATOR_ID_CHASSIS0, &g_config.motor.chassis[0], chassis[0]},
                    {ACTUATOR_ID_CHASSIS1, &g_config.motor.chassis[1], chassis[1]},
                    {ACTUATOR_ID_CHASSIS2, &g_config.motor.chassis[2], chassis[2]},
                    {ACTUATOR_ID_CHASSIS3, &g_config.motor.chassis[3], chassis[3]},
                    {ACTUATOR_ID_YAW, &g_config.motor.yaw, yaw},
                    {ACTUATOR_ID_YAW_UPPER, &g_config.motor.yaw_upper, yaw_upper},
                    {ACTUATOR_ID_PITCH, &g_config.motor.pitch, pitch},
                    {ACTUATOR_ID_TRIGGER, &g_config.motor.trigger, trigger},
                };
                const can_tx_item_t can2_items[] = {
                    {ACTUATOR_ID_FRICTION0, &g_config.motor.friction[0], fric[0]},
                    {ACTUATOR_ID_FRICTION1, &g_config.motor.friction[1], fric[1]},
                    {ACTUATOR_ID_FRICTION2, &g_config.motor.friction[2], fric[2]},
                    {ACTUATOR_ID_FRICTION3, &g_config.motor.friction[3], fric[3]},
                };

                int16_t can1_200[4] = {0};
                int16_t can1_1ff[4] = {0};
                int16_t can2_200[4] = {0};
                int16_t can2_1ff[4] = {0};

                can_tx_pack_rm_frames(1u,
                                      can1_items,
                                      (uint32_t)(sizeof(can1_items) / sizeof(can1_items[0])),
                                      can1_200,
                                      can1_1ff);
                can_tx_pack_rm_frames(2u,
                                      can2_items,
                                      (uint32_t)(sizeof(can2_items) / sizeof(can2_items[0])),
                                      can2_200,
                                      can2_1ff);

                CAN_cmd_rm_group(1u, (uint16_t)CAN_CHASSIS_ALL_ID, can1_200[0], can1_200[1], can1_200[2], can1_200[3]);
                CAN_cmd_rm_group(1u, (uint16_t)CAN_GIMBAL_ALL_ID, can1_1ff[0], can1_1ff[1], can1_1ff[2], can1_1ff[3]);
                CAN_cmd_rm_group(2u, (uint16_t)CAN_CHASSIS_ALL_ID, can2_200[0], can2_200[1], can2_200[2], can2_200[3]);
                CAN_cmd_rm_group(2u, (uint16_t)CAN_GIMBAL_ALL_ID, can2_1ff[0], can2_1ff[1], can2_1ff[2], can2_1ff[3]);
            }
        }

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
