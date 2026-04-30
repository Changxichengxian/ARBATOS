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
#include <can_command_axis_bindings.h>
#include "can_mit_motor_driver.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

__weak uint8_t can_tx_allow_can1_yaw_override(void);
__weak uint8_t can_tx_process_extra_item(uint8_t bus, const motor_node_param_t *node, int16_t current);

static int16_t s_can_tx_chassis_cmd[4];
static int16_t s_can_tx_friction_cmd[4];
static int16_t s_can_tx_yaw_cmd;
static int16_t s_can_tx_yaw_upper_cmd;
static int16_t s_can_tx_pitch_cmd;
static int16_t s_can_tx_trigger_cmd;

static int16_t s_can_tx_can1_200[4];
static int16_t s_can_tx_can1_1ff[4];
static int16_t s_can_tx_can2_200[4];
static int16_t s_can_tx_can2_1ff[4];

static inline bool_t can_tx_allow_chassis(void)
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

static inline fp32 can_tx_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
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

static inline uint8_t can_tx_actuator_id_valid(actuator_id_e id)
{
    return ((uint32_t)id < (uint32_t)ACTUATOR_ID__COUNT) ? 1u : 0u;
}

static inline uint8_t can_tx_cmd_nonzero(const actuator_cmd_t *cmd)
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

static inline uint8_t can_tx_node_bus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    if (node != NULL && (node->can_bus == 1u || node->can_bus == 2u))
    {
        return node->can_bus;
    }
    return fallback_bus;
}

static inline fp32 can_tx_current_to_mit_torque(const motor_node_param_t *node,
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

static inline void can_tx_build_mit_cmd_from_actuator(const motor_node_param_t *node,
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

static inline uint8_t can_tx_process_can_mit_item(uint8_t bus,
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

static void can_tx_log_actuator_current(const sdlog_actuator_current_t *log)
{
    if (log != NULL && can_tx_log_due() != 0u)
    {
        sdlog_write(SDLOG_TAG_ACTUATOR_CURRENT, log, (uint16_t)sizeof(*log));
    }
}

static void can_tx_clear_cmd_cache(void)
{
    (void)memset(s_can_tx_chassis_cmd, 0, sizeof(s_can_tx_chassis_cmd));
    (void)memset(s_can_tx_friction_cmd, 0, sizeof(s_can_tx_friction_cmd));
    s_can_tx_yaw_cmd = 0;
    s_can_tx_yaw_upper_cmd = 0;
    s_can_tx_pitch_cmd = 0;
    s_can_tx_trigger_cmd = 0;
}

static void can_tx_limit_friction_cmds(void)
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.shoot.fric_motor_dir[i] == 0)
        {
            s_can_tx_friction_cmd[i] = 0;
        }
    }
}

static void can_tx_collect_offline_cmds(void)
{
    const test_mode_e mode = (test_mode_e)g_config.test.mode;
    const bool_t allow_friction_offline = (mode == TEST_MODE_ENTERTAIN);

    can_tx_clear_cmd_cache();

    if (allow_friction_offline)
    {
        s_can_tx_friction_cmd[0] = actuator_cmd_get_friction_current(0);
        s_can_tx_friction_cmd[1] = actuator_cmd_get_friction_current(1);
        s_can_tx_friction_cmd[2] = actuator_cmd_get_friction_current(2);
        s_can_tx_friction_cmd[3] = actuator_cmd_get_friction_current(3);
    }
    if (can_tx_allow_can1_yaw_override() != 0u)
    {
        s_can_tx_yaw_cmd = actuator_cmd_get_yaw_current();
    }

    can_tx_limit_friction_cmds();
}

static void can_tx_collect_online_cmds(void)
{
    const bool_t allow_chassis = can_tx_allow_chassis();

    can_tx_clear_cmd_cache();

    s_can_tx_chassis_cmd[0] = allow_chassis ? actuator_cmd_get_chassis_current(0) : 0;
    s_can_tx_chassis_cmd[1] = allow_chassis ? actuator_cmd_get_chassis_current(1) : 0;
    s_can_tx_chassis_cmd[2] = allow_chassis ? actuator_cmd_get_chassis_current(2) : 0;
    s_can_tx_chassis_cmd[3] = allow_chassis ? actuator_cmd_get_chassis_current(3) : 0;
    s_can_tx_yaw_cmd = actuator_cmd_get_yaw_current();
    s_can_tx_yaw_upper_cmd = actuator_cmd_get_yaw_upper_current();
    s_can_tx_pitch_cmd = actuator_cmd_get_pitch_current();
    s_can_tx_trigger_cmd = actuator_cmd_get_trigger_current();
    s_can_tx_friction_cmd[0] = actuator_cmd_get_friction_current(0);
    s_can_tx_friction_cmd[1] = actuator_cmd_get_friction_current(1);
    s_can_tx_friction_cmd[2] = actuator_cmd_get_friction_current(2);
    s_can_tx_friction_cmd[3] = actuator_cmd_get_friction_current(3);

    can_tx_limit_friction_cmds();
}

static void can_tx_log_offline_cmds(void)
{
    sdlog_actuator_current_t log = {0};

    log.yaw = s_can_tx_yaw_cmd;
    log.friction[0] = s_can_tx_friction_cmd[0];
    log.friction[1] = s_can_tx_friction_cmd[1];
    log.friction[2] = s_can_tx_friction_cmd[2];
    log.friction[3] = s_can_tx_friction_cmd[3];
    can_tx_log_actuator_current(&log);
}

static void can_tx_log_online_cmds(void)
{
    sdlog_actuator_current_t log = {0};

    log.chassis[0] = s_can_tx_chassis_cmd[0];
    log.chassis[1] = s_can_tx_chassis_cmd[1];
    log.chassis[2] = s_can_tx_chassis_cmd[2];
    log.chassis[3] = s_can_tx_chassis_cmd[3];
    log.yaw = s_can_tx_yaw_cmd;
    log.pitch = s_can_tx_pitch_cmd;
    log.trigger = s_can_tx_trigger_cmd;
    log.friction[0] = s_can_tx_friction_cmd[0];
    log.friction[1] = s_can_tx_friction_cmd[1];
    log.friction[2] = s_can_tx_friction_cmd[2];
    log.friction[3] = s_can_tx_friction_cmd[3];
    can_tx_log_actuator_current(&log);
}

static void can_tx_clear_rm_frames(void)
{
    (void)memset(s_can_tx_can1_200, 0, sizeof(s_can_tx_can1_200));
    (void)memset(s_can_tx_can1_1ff, 0, sizeof(s_can_tx_can1_1ff));
    (void)memset(s_can_tx_can2_200, 0, sizeof(s_can_tx_can2_200));
    (void)memset(s_can_tx_can2_1ff, 0, sizeof(s_can_tx_can2_1ff));
}

// Axis functions stay parameterless; these macros expand the common send body.
#define CAN_TX_STORE_RM_CURRENT(fallback_bus_, can_id_expr_, current_expr_)        \
    do                                                                            \
    {                                                                             \
        const uint8_t bus__ = (uint8_t)(fallback_bus_);                            \
        const int16_t current__ = (int16_t)(current_expr_);                        \
        const uint16_t can_id__ = (uint16_t)(can_id_expr_);                        \
        int16_t *frame_200__ = (bus__ == 1u) ? s_can_tx_can1_200 : s_can_tx_can2_200; \
        int16_t *frame_1ff__ = (bus__ == 1u) ? s_can_tx_can1_1ff : s_can_tx_can2_1ff; \
        if (can_id__ >= 0x201u && can_id__ <= 0x204u)                              \
        {                                                                         \
            frame_200__[can_id__ - 0x201u] = current__;                           \
        }                                                                         \
        else if (can_id__ >= 0x205u && can_id__ <= 0x208u)                         \
        {                                                                         \
            frame_1ff__[can_id__ - 0x205u] = current__;                           \
        }                                                                         \
    } while (0)

#define CAN_TX_EXEC_AXIS(fallback_bus_, actuator_id_, node_expr_, is_rm_expr_, can_id_expr_, limited_current_expr_, current_expr_) \
    do                                                                            \
    {                                                                             \
        const uint8_t fallback_bus__ = (uint8_t)(fallback_bus_);                   \
        const motor_node_param_t *node__ = (node_expr_);                           \
        const int16_t current__ = (int16_t)(current_expr_);                        \
        if ((is_rm_expr_) == 0u)                                                   \
        {                                                                         \
            const uint8_t node_bus__ = can_tx_node_bus(fallback_bus__, node__);    \
            if (can_tx_process_can_mit_item(node_bus__,                            \
                                            (actuator_id_),                        \
                                            node__,                                \
                                            current__) == 0u &&                    \
                can_tx_process_extra_item(node_bus__, node__, current__) == 0u)    \
            {                                                                     \
                watch_task_error(WATCH_TASK_CAN_COMMAND_TX);                      \
            }                                                                     \
        }                                                                         \
        else                                                                      \
        {                                                                         \
            CAN_TX_STORE_RM_CURRENT(fallback_bus__,                                \
                                    (can_id_expr_),                                \
                                    (limited_current_expr_));                      \
        }                                                                         \
    } while (0)

static inline void can_tx_exec_chassis0(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS0_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS0_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS0_NODE(),
                     CAN_TX_AXIS_CHASSIS0_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS0_CAN_ID(),
                     CAN_TX_AXIS_CHASSIS0_LIMIT_CURRENT(s_can_tx_chassis_cmd[0]),
                     s_can_tx_chassis_cmd[0]);
}

static inline void can_tx_exec_chassis1(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS1_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS1_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS1_NODE(),
                     CAN_TX_AXIS_CHASSIS1_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS1_CAN_ID(),
                     CAN_TX_AXIS_CHASSIS1_LIMIT_CURRENT(s_can_tx_chassis_cmd[1]),
                     s_can_tx_chassis_cmd[1]);
}

static inline void can_tx_exec_chassis2(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS2_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS2_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS2_NODE(),
                     CAN_TX_AXIS_CHASSIS2_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS2_CAN_ID(),
                     CAN_TX_AXIS_CHASSIS2_LIMIT_CURRENT(s_can_tx_chassis_cmd[2]),
                     s_can_tx_chassis_cmd[2]);
}

static inline void can_tx_exec_chassis3(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS3_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS3_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS3_NODE(),
                     CAN_TX_AXIS_CHASSIS3_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS3_CAN_ID(),
                     CAN_TX_AXIS_CHASSIS3_LIMIT_CURRENT(s_can_tx_chassis_cmd[3]),
                     s_can_tx_chassis_cmd[3]);
}

static inline void can_tx_exec_yaw(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_YAW_FALLBACK_BUS,
                     CAN_TX_AXIS_YAW_ACTUATOR_ID,
                     CAN_TX_AXIS_YAW_NODE(),
                     CAN_TX_AXIS_YAW_IS_RM_GROUP(),
                     CAN_TX_AXIS_YAW_CAN_ID(),
                     CAN_TX_AXIS_YAW_LIMIT_CURRENT(s_can_tx_yaw_cmd),
                     s_can_tx_yaw_cmd);
}

static inline void can_tx_exec_yaw_upper(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_YAW_UPPER_FALLBACK_BUS,
                     CAN_TX_AXIS_YAW_UPPER_ACTUATOR_ID,
                     CAN_TX_AXIS_YAW_UPPER_NODE(),
                     CAN_TX_AXIS_YAW_UPPER_IS_RM_GROUP(),
                     CAN_TX_AXIS_YAW_UPPER_CAN_ID(),
                     CAN_TX_AXIS_YAW_UPPER_LIMIT_CURRENT(s_can_tx_yaw_upper_cmd),
                     s_can_tx_yaw_upper_cmd);
}

static inline void can_tx_exec_pitch(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_PITCH_FALLBACK_BUS,
                     CAN_TX_AXIS_PITCH_ACTUATOR_ID,
                     CAN_TX_AXIS_PITCH_NODE(),
                     CAN_TX_AXIS_PITCH_IS_RM_GROUP(),
                     CAN_TX_AXIS_PITCH_CAN_ID(),
                     CAN_TX_AXIS_PITCH_LIMIT_CURRENT(s_can_tx_pitch_cmd),
                     s_can_tx_pitch_cmd);
}

static inline void can_tx_exec_trigger(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_TRIGGER_FALLBACK_BUS,
                     CAN_TX_AXIS_TRIGGER_ACTUATOR_ID,
                     CAN_TX_AXIS_TRIGGER_NODE(),
                     CAN_TX_AXIS_TRIGGER_IS_RM_GROUP(),
                     CAN_TX_AXIS_TRIGGER_CAN_ID(),
                     CAN_TX_AXIS_TRIGGER_LIMIT_CURRENT(s_can_tx_trigger_cmd),
                     s_can_tx_trigger_cmd);
}

static inline void can_tx_exec_friction0(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION0_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION0_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION0_NODE(),
                     CAN_TX_AXIS_FRICTION0_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION0_CAN_ID(),
                     CAN_TX_AXIS_FRICTION0_LIMIT_CURRENT(s_can_tx_friction_cmd[0]),
                     s_can_tx_friction_cmd[0]);
}

static inline void can_tx_exec_friction1(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION1_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION1_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION1_NODE(),
                     CAN_TX_AXIS_FRICTION1_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION1_CAN_ID(),
                     CAN_TX_AXIS_FRICTION1_LIMIT_CURRENT(s_can_tx_friction_cmd[1]),
                     s_can_tx_friction_cmd[1]);
}

static inline void can_tx_exec_friction2(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION2_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION2_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION2_NODE(),
                     CAN_TX_AXIS_FRICTION2_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION2_CAN_ID(),
                     CAN_TX_AXIS_FRICTION2_LIMIT_CURRENT(s_can_tx_friction_cmd[2]),
                     s_can_tx_friction_cmd[2]);
}

static inline void can_tx_exec_friction3(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION3_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION3_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION3_NODE(),
                     CAN_TX_AXIS_FRICTION3_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION3_CAN_ID(),
                     CAN_TX_AXIS_FRICTION3_LIMIT_CURRENT(s_can_tx_friction_cmd[3]),
                     s_can_tx_friction_cmd[3]);
}

static void can_tx_exec_offline_axes(void)
{
    can_tx_clear_rm_frames();
    can_tx_exec_yaw();
    can_tx_exec_friction0();
    can_tx_exec_friction1();
    can_tx_exec_friction2();
    can_tx_exec_friction3();
}

static void can_tx_exec_online_axes(void)
{
    can_tx_clear_rm_frames();
    can_tx_exec_chassis0();
    can_tx_exec_chassis1();
    can_tx_exec_chassis2();
    can_tx_exec_chassis3();
    can_tx_exec_yaw();
    can_tx_exec_yaw_upper();
    can_tx_exec_pitch();
    can_tx_exec_trigger();
    can_tx_exec_friction0();
    can_tx_exec_friction1();
    can_tx_exec_friction2();
    can_tx_exec_friction3();
}

static void can_tx_emit_rm_frames(void)
{
    CAN_cmd_rm_group(1u,
                     (uint16_t)CAN_CHASSIS_ALL_ID,
                     s_can_tx_can1_200[0],
                     s_can_tx_can1_200[1],
                     s_can_tx_can1_200[2],
                     s_can_tx_can1_200[3]);
    CAN_cmd_rm_group(1u,
                     (uint16_t)CAN_GIMBAL_ALL_ID,
                     s_can_tx_can1_1ff[0],
                     s_can_tx_can1_1ff[1],
                     s_can_tx_can1_1ff[2],
                     s_can_tx_can1_1ff[3]);
    CAN_cmd_rm_group(2u,
                     (uint16_t)CAN_CHASSIS_ALL_ID,
                     s_can_tx_can2_200[0],
                     s_can_tx_can2_200[1],
                     s_can_tx_can2_200[2],
                     s_can_tx_can2_200[3]);
    CAN_cmd_rm_group(2u,
                     (uint16_t)CAN_GIMBAL_ALL_ID,
                     s_can_tx_can2_1ff[0],
                     s_can_tx_can2_1ff[1],
                     s_can_tx_can2_1ff[2],
                     s_can_tx_can2_1ff[3]);
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

        if (dbus_offline)
        {
            can_tx_collect_offline_cmds();
            can_tx_log_offline_cmds();
            can_tx_exec_offline_axes();
            can_tx_emit_rm_frames();
        }
        else
        {
            can_tx_collect_online_cmds();
            can_tx_log_online_cmds();
            can_tx_exec_online_axes();
            can_tx_emit_rm_frames();
        }

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
