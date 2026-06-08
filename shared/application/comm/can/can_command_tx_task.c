/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：命令限幅、RM/MIT 命令换算、单轴协议处理。
 * - 中段：在线/离线收集各轴电流命令，并记录 CAN 电流日志。
 * - 后段：按轴装配表执行发送，RM 组帧缓存最后统一发出。
 * - 入口：can_command_tx_task() 每周期收集 LowCmd，再按电机配置发到 CAN/RS485。
 */

#include "can_command_tx_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "CAN_receive.h"
#include "LowCmd.h"
#include "can_mit_motor_driver.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "motor_instance.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"
#include "robot_mode.h"
#include "robot_safety.h"

#include <string.h>

#define CAN_TX_MIT_WHEEL_CMD_PERIOD_MS 2u
#define CAN_TX_MIT_JOINT_CMD_PERIOD_MS 5u
#define CAN_TX_MIT_MAX_FRAMES_PER_MS 3u
#define CAN_TX_MIT_STATE_DISABLED 0u
#define CAN_TX_MIT_STATE_ENABLED 1u
#define CAN_TX_MIT_STATE_FAULT_MIN 8u

__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         MotorId actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current);

static MotorCmd s_can_tx_cmd_cache[MotorCount];
static uint8_t s_can_tx_cmd_cache_valid[MotorCount];
static uint8_t s_can_tx_cmd_expired[MotorCount];

static int16_t s_can_tx_can1_200[4];
static int16_t s_can_tx_can1_1ff[4];
static int16_t s_can_tx_can2_200[4];
static int16_t s_can_tx_can2_1ff[4];
static uint32_t s_can_tx_mit_budget_tick_ms;
static uint8_t s_can_tx_mit_budget_used;
static uint8_t s_can_tx_rm_group_configured[2][2];

static inline bool_t can_tx_allow_chassis(void)
{
    return (robot_mode_allow_chassis() != 0u) ? 1 : 0;
}

// 按任务周期换算日志间隔，避免每个 CAN 周期都写 SD 卡。
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

static inline uint8_t can_tx_actuator_id_valid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static uint32_t can_tx_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint8_t can_tx_cmd_expired(const MotorCmd *cmd, uint32_t now_ms)
{
    if (cmd == NULL || cmd->active == 0u || cmd->timeoutMs == 0u || cmd->tick == 0u)
    {
        return 0u;
    }

    return ((uint32_t)(now_ms - cmd->tick) > (uint32_t)cmd->timeoutMs) ? 1u : 0u;
}

static MotorDriveState can_tx_drive_state_from_feedback(MotorId id)
{
    MotorState fb;

    if (LowStateGetMotor(id, &fb) == 0u || fb.online == 0u)
    {
        return MotorDriveStateOffline;
    }
    if (fb.driveState != (uint8_t)MotorDriveStateUnknown)
    {
        return (MotorDriveState)fb.driveState;
    }
    return MotorDriveStateEnabled;
}

static uint8_t can_tx_get_cmd_copy(MotorId id, MotorCmd *out)
{
    if (out == NULL || can_tx_actuator_id_valid(id) == 0u)
    {
        return 0u;
    }

    if (s_can_tx_cmd_cache_valid[id] != 0u)
    {
        *out = s_can_tx_cmd_cache[id];
        if (can_tx_cmd_expired(out, can_tx_now_ms()) != 0u)
        {
            (void)memset(out, 0, sizeof(*out));
            out->mode = (uint8_t)MotorModeDisable;
            s_can_tx_cmd_expired[id] = 1u;
        }
        return 1u;
    }

    if (LowCmdGetMotor(id, out) == 0u)
    {
        return 0u;
    }
    if (can_tx_cmd_expired(out, can_tx_now_ms()) != 0u)
    {
        (void)memset(out, 0, sizeof(*out));
        out->mode = (uint8_t)MotorModeDisable;
        s_can_tx_cmd_expired[id] = 1u;
    }
    return 1u;
}

static void can_tx_clear_cmd_cache(void)
{
    (void)memset(s_can_tx_cmd_cache_valid, 0, sizeof(s_can_tx_cmd_cache_valid));
    (void)memset(s_can_tx_cmd_expired, 0, sizeof(s_can_tx_cmd_expired));
}

static void can_tx_cache_lowcmd(void)
{
    LowCmd lowcmd;

    can_tx_clear_cmd_cache();
    if (LowCmdGet(&lowcmd) == 0u)
    {
        return;
    }

    for (uint8_t i = 0u; i < (uint8_t)MotorCount; i++)
    {
        s_can_tx_cmd_cache[i] = lowcmd.motorCmd[i];
        s_can_tx_cmd_cache_valid[i] = 1u;
    }
}

static void can_tx_force_disabled_cmd(MotorId id)
{
    if (can_tx_actuator_id_valid(id) == 0u)
    {
        return;
    }

    (void)memset(&s_can_tx_cmd_cache[id], 0, sizeof(s_can_tx_cmd_cache[id]));
    s_can_tx_cmd_cache[id].active = 1u;
    s_can_tx_cmd_cache[id].mode = (uint8_t)MotorModeDisable;
    s_can_tx_cmd_cache[id].tick = can_tx_now_ms();
    s_can_tx_cmd_cache_valid[id] = 1u;
}

static uint8_t can_tx_bus_index(uint8_t bus, uint8_t *out)
{
    if (out == NULL || (bus != 1u && bus != 2u))
    {
        return 0u;
    }

    *out = (uint8_t)(bus - 1u);
    return 1u;
}

static uint8_t can_tx_rm_group_index(uint16_t group_id, uint8_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    if (group_id == (uint16_t)CAN_RM_GROUP_0X200_ID)
    {
        *out = 0u;
        return 1u;
    }
    if (group_id == (uint16_t)CAN_RM_GROUP_0X1FF_ID)
    {
        *out = 1u;
        return 1u;
    }
    return 0u;
}

static void can_tx_mark_rm_group_configured(uint8_t bus, uint16_t can_id)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;

    if (can_id >= 0x201u && can_id <= 0x204u)
    {
        group_index = 0u;
    }
    else if (can_id >= 0x205u && can_id <= 0x208u)
    {
        group_index = 1u;
    }
    else
    {
        return;
    }

    if (can_tx_bus_index(bus, &bus_index) == 0u)
    {
        return;
    }

    s_can_tx_rm_group_configured[bus_index][group_index] = 1u;
}

static void can_tx_cache_rm_groups(void)
{
    const uint8_t count = motor_route_count();

    (void)memset(s_can_tx_rm_group_configured, 0, sizeof(s_can_tx_rm_group_configured));

    for (uint8_t i = 0u; i < count; i++)
    {
        const motor_route_t *route = motor_route_get(i);
        const uint16_t can_id = (route != NULL) ? route->canId : 0u;

        if (route == NULL ||
            route->enabled == 0u ||
            route->bus == 0u ||
            route->transport != (uint8_t)MotorTransportCAN ||
            route->isRmGroup == 0u ||
            can_id == 0u)
        {
            continue;
        }

        can_tx_mark_rm_group_configured(route->bus, can_id);
    }
}

static uint8_t can_tx_mit_is_wheelleg_wheel(MotorId actuator_id)
{
    if (robot_profile_is_wheelleg_mit() == 0u)
    {
        return 0u;
    }

    return (uint8_t)((uint8_t)actuator_id == g_config.wheelleg_mit.left_wheel_actuator ||
                     (uint8_t)actuator_id == g_config.wheelleg_mit.right_wheel_actuator);
}

static uint16_t can_tx_mit_cmd_period_ms(MotorId actuator_id)
{
    return (can_tx_mit_is_wheelleg_wheel(actuator_id) != 0u) ?
               CAN_TX_MIT_WHEEL_CMD_PERIOD_MS :
               CAN_TX_MIT_JOINT_CMD_PERIOD_MS;
}

static uint8_t can_tx_mit_take_frame_budget(uint32_t now_ms)
{
    if (s_can_tx_mit_budget_tick_ms != now_ms)
    {
        s_can_tx_mit_budget_tick_ms = now_ms;
        s_can_tx_mit_budget_used = 0u;
    }

    if (s_can_tx_mit_budget_used >= CAN_TX_MIT_MAX_FRAMES_PER_MS)
    {
        return 0u;
    }

    s_can_tx_mit_budget_used++;
    return 1u;
}

static uint8_t can_tx_mit_period_due(uint32_t now_ms, uint32_t last_tick_ms, uint16_t period_ms)
{
    return (uint8_t)(last_tick_ms == 0u ||
                     (uint32_t)(now_ms - last_tick_ms) >= (uint32_t)period_ms);
}

static uint8_t can_tx_mit_get_feedback_state(MotorId actuator_id,
                                             const motor_node_param_t *node,
                                             MotorState *feedback)
{
    const uint16_t expected_rx_id = motor_cfg_feedback_id(node);
    const uint8_t expected_motor_low_id = (uint8_t)(motor_cfg_can_id(node) & 0x0Fu);

    if (feedback == NULL || can_tx_actuator_id_valid(actuator_id) == 0u)
    {
        return 0u;
    }

    if (LowStateGetMotor(actuator_id, feedback) == 0u ||
        feedback->online == 0u ||
        feedback->transport != (uint8_t)MotorTransportCAN ||
        feedback->rxId != expected_rx_id ||
        feedback->rxDlc < 8u)
    {
        return 0u;
    }

    if ((feedback->rxData0 & 0x0Fu) != expected_motor_low_id)
    {
        return 0u;
    }

    return 1u;
}

static void can_tx_mit_sync_mode_state(MotorId actuator_id,
                                       const motor_node_param_t *node,
                                       uint8_t *enabled,
                                       uint8_t *disabled_confirmed)
{
    MotorState feedback;

    if (enabled == NULL || disabled_confirmed == NULL ||
        can_tx_mit_get_feedback_state(actuator_id, node, &feedback) == 0u)
    {
        return;
    }

    if (feedback.state == CAN_TX_MIT_STATE_ENABLED)
    {
        *enabled = 1u;
        *disabled_confirmed = 0u;
    }
    else if (feedback.state == CAN_TX_MIT_STATE_DISABLED ||
             feedback.state >= CAN_TX_MIT_STATE_FAULT_MIN)
    {
        *enabled = 0u;
        *disabled_confirmed = 1u;
    }
    else
    {
        *enabled = 0u;
        *disabled_confirmed = 0u;
    }
}

static inline int16_t can_tx_fp32_to_i16_saturated(fp32 x)
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

static inline uint8_t can_tx_cmd_mode_uses_position_feedback(MotorMode mode)
{
    switch (mode)
    {
    case MotorModeStateTorque:
    case MotorModePosVel:
    case MotorModeForcePos:
        return 1u;
    default:
        return 0u;
    }
}

static inline uint8_t can_tx_cmd_mode_uses_velocity_feedback(MotorMode mode)
{
    if (can_tx_cmd_mode_uses_position_feedback(mode) != 0u)
    {
        return 1u;
    }
    return (uint8_t)((mode == MotorModeSpeed || mode == MotorModeDamping) ? 1u : 0u);
}

// 轴配置里可以指定 CAN 总线；没指定时使用该轴的默认总线。
static inline uint8_t can_tx_node_bus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    return motor_cfg_can_bus(fallback_bus, node);
}

// RM 电机仍优先吃旧双环 PID 给出的电流；只有通用状态命令才临时换算成电流。
static int16_t can_tx_build_rm_current_from_actuator(MotorId actuator_id,
                                                     int16_t current)
{
    MotorCmd cmd;
    MotorState fb;
    MotorMode mode;
    fp32 rm_current;

    if (can_tx_get_cmd_copy(actuator_id, &cmd) == 0u || cmd.active == 0u)
    {
        return current;
    }

    mode = (MotorMode)cmd.mode;
    if (can_tx_cmd_mode_uses_velocity_feedback(mode) == 0u)
    {
        return current;
    }

    if (LowStateGetMotor(actuator_id, &fb) == 0u || fb.online == 0u)
    {
        return current;
    }

    if (mode == MotorModeDisable)
    {
        return 0;
    }

    rm_current = cmd.tau;
    if (can_tx_cmd_mode_uses_position_feedback(mode) != 0u)
    {
        rm_current += cmd.kp * (cmd.q - fb.q);
    }
    if (can_tx_cmd_mode_uses_velocity_feedback(mode) != 0u)
    {
        rm_current += cmd.kd * (cmd.dq - fb.dq);
    }

    return can_tx_fp32_to_i16_saturated(rm_current);
}

// 旧控制链常给“电流”，MIT 电机要力矩，这里按型号量程换成力矩。
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

// 把通用执行器命令翻译成 MIT 命令；没有新命令时退回电流转力矩。
static inline void can_tx_build_mit_cmd_from_actuator(const motor_node_param_t *node,
                                                      const MotorCmd *src,
                                                      int16_t current,
                                                      const can_mit_motor_limits_t *limits,
                                                      mit_motor_cmd_t *out)
{
    uint8_t mode = (uint8_t)MotorModeCurrent;

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (src != NULL &&
        src->active != 0u &&
        src->mode != (uint8_t)MotorModeNone &&
        src->mode != (uint8_t)MotorModeDisable)
    {
        mode = src->mode;
    }

    if (can_tx_cmd_mode_uses_position_feedback((MotorMode)mode) != 0u)
    {
        out->position = (src != NULL) ? src->q : 0.0f;
        out->velocity = (src != NULL) ? src->dq : 0.0f;
        out->kp = (src != NULL) ? src->kp : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->tau : 0.0f;
    }
    else if (can_tx_cmd_mode_uses_velocity_feedback((MotorMode)mode) != 0u)
    {
        out->velocity = (src != NULL) ? src->dq : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->tau : 0.0f;
    }
    else
    {
        out->torque = can_tx_current_to_mit_torque(node, current, limits);
    }
}

// 处理单个 MIT 轴：必要时先使能，再发送本周期命令。
static inline uint8_t can_tx_process_can_mit_item(uint8_t bus,
                                                  MotorId actuator_id,
                                                  const motor_node_param_t *node,
                                                  int16_t current)
{
    static uint8_t mit_enabled[MotorCount];
    static uint8_t mit_disabled_confirmed[MotorCount];
    static uint32_t mit_last_enable_tick[MotorCount];
    static uint32_t mit_last_disable_tick[MotorCount];
    static uint32_t mit_last_cmd_tick[MotorCount];
    const can_mit_motor_limits_t *limits;
    MotorCmd cmd;
    mit_motor_cmd_t mit_cmd;
    uint8_t have_cmd;
    uint8_t active_cmd;
    uint16_t cmd_period_ms;
    uint16_t std_id;
    const uint32_t now = can_tx_now_ms();

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
    have_cmd = can_tx_get_cmd_copy(actuator_id, &cmd);
    active_cmd = (uint8_t)(have_cmd != 0u &&
                           cmd.active != 0u &&
                           cmd.mode != (uint8_t)MotorModeNone &&
                           cmd.mode != (uint8_t)MotorModeDisable);
    if (active_cmd == 0u && current != 0)
    {
        cmd.active = 1u;
        cmd.mode = (uint8_t)MotorModeCurrent;
        cmd.current = current;
        active_cmd = 1u;
    }

    std_id = motor_cfg_can_id(node);
    if (std_id == 0u)
    {
        return 0u;
    }
    cmd_period_ms = can_tx_mit_cmd_period_ms(actuator_id);

    if (can_tx_actuator_id_valid(actuator_id) != 0u)
    {
        can_tx_mit_sync_mode_state(actuator_id,
                                   node,
                                   &mit_enabled[actuator_id],
                                   &mit_disabled_confirmed[actuator_id]);

        if (active_cmd == 0u)
        {
            mit_last_enable_tick[actuator_id] = 0u;
            mit_last_cmd_tick[actuator_id] = 0u;

            if (mit_disabled_confirmed[actuator_id] == 0u &&
                can_tx_mit_period_due(now, mit_last_disable_tick[actuator_id], cmd_period_ms) != 0u &&
                can_tx_mit_take_frame_budget(now) != 0u)
            {
                const int ret = can_mit_motor_send_disable(bus, std_id);
                if (ret == 0)
                {
                    mit_last_disable_tick[actuator_id] = now;
                    mit_enabled[actuator_id] = 0u;
                }
            }
            return 1u;
        }

        mit_disabled_confirmed[actuator_id] = 0u;
        mit_last_disable_tick[actuator_id] = 0u;

        if (mit_enabled[actuator_id] == 0u)
        {
            if (can_tx_mit_period_due(now, mit_last_enable_tick[actuator_id], cmd_period_ms) != 0u &&
                can_tx_mit_take_frame_budget(now) != 0u)
            {
                const int ret = can_mit_motor_send_enable(bus, std_id);
                if (ret == 0)
                {
                    mit_last_enable_tick[actuator_id] = now;
                }
            }
            return 1u;
        }

        if (mit_last_cmd_tick[actuator_id] != 0u &&
            (uint32_t)(now - mit_last_cmd_tick[actuator_id]) < cmd_period_ms)
        {
            return 1u;
        }

        if (can_tx_mit_take_frame_budget(now) == 0u)
        {
            return 1u;
        }

        can_tx_build_mit_cmd_from_actuator(node, &cmd, current, limits, &mit_cmd);
        {
            const int ret = can_mit_motor_send_cmd(bus, std_id, limits, &mit_cmd);
            if (ret == 0)
            {
                mit_last_cmd_tick[actuator_id] = now;
            }
        }
        return 1u;
    }

    if (active_cmd == 0u)
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

static uint8_t can_tx_arm_role_active(void)
{
    return (uint8_t)(robot_profile_need_arm_task() || robot_profile_is_wheelleg_mit());
}

static uint8_t can_tx_route_role_active(const motor_route_t *route)
{
    if (route == NULL)
    {
        return 0u;
    }
    if (route->role == MOTOR_INSTANCE_ROLE_ARM && can_tx_arm_role_active() == 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t can_tx_friction_enabled(const motor_route_t *route)
{
    if (route == NULL || route->role != MOTOR_INSTANCE_ROLE_FRICTION)
    {
        return 1u;
    }
    if (route->roleIndex >= 4u)
    {
        return 0u;
    }
    return (g_config.shoot.fric_motor_dir[route->roleIndex] != 0) ? 1u : 0u;
}

static uint8_t can_tx_route_allowed_online(const motor_route_t *route)
{
    if (route == NULL)
    {
        return 0u;
    }
    if (robot_mode_allow_motor(route->motorId) == 0u)
    {
        return 0u;
    }
    if (route->role == MOTOR_INSTANCE_ROLE_CHASSIS && can_tx_allow_chassis() == 0u)
    {
        return 0u;
    }
    if (can_tx_friction_enabled(route) == 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t can_tx_route_allowed_offline(const motor_route_t *route)
{
    if (route == NULL)
    {
        return 0u;
    }
    if (robot_mode_allow_motor(route->motorId) == 0u)
    {
        return 0u;
    }

    switch (route->role)
    {
    case MOTOR_INSTANCE_ROLE_YAW:
    case MOTOR_INSTANCE_ROLE_ARM:
        return 1u;
    case MOTOR_INSTANCE_ROLE_FRICTION:
        return (uint8_t)((robot_mode_is_entertain() != 0u && can_tx_friction_enabled(route) != 0u) ? 1u : 0u);
    default:
        return 0u;
    }
}

static uint8_t can_tx_route_allowed(const motor_route_t *route, uint8_t online)
{
    return (online != 0u) ? can_tx_route_allowed_online(route) : can_tx_route_allowed_offline(route);
}

static void can_tx_log_motor_cmd(sdlog_actuator_current_t *log,
                                 const motor_route_t *route,
                                 int16_t current)
{
    if (log == NULL || route == NULL)
    {
        return;
    }

    switch (route->role)
    {
    case MOTOR_INSTANCE_ROLE_CHASSIS:
        if (route->roleIndex < 4u)
        {
            log->chassis[route->roleIndex] = current;
        }
        break;
    case MOTOR_INSTANCE_ROLE_YAW:
        log->yaw = current;
        break;
    case MOTOR_INSTANCE_ROLE_PITCH:
        log->pitch = current;
        break;
    case MOTOR_INSTANCE_ROLE_TRIGGER:
        log->trigger = current;
        break;
    case MOTOR_INSTANCE_ROLE_FRICTION:
        if (route->roleIndex < 4u)
        {
            log->friction[route->roleIndex] = current;
        }
        break;
    default:
        break;
    }
}

// 清空大疆组帧缓存，后续按 CAN ID 填进 0x200 或 0x1FF 的四个槽位。
static void can_tx_clear_rm_frames(void)
{
    (void)memset(s_can_tx_can1_200, 0, sizeof(s_can_tx_can1_200));
    (void)memset(s_can_tx_can1_1ff, 0, sizeof(s_can_tx_can1_1ff));
    (void)memset(s_can_tx_can2_200, 0, sizeof(s_can_tx_can2_200));
    (void)memset(s_can_tx_can2_1ff, 0, sizeof(s_can_tx_can2_1ff));
}

static inline void can_tx_store_rm_current(uint8_t bus,
                                           uint16_t can_id,
                                           int16_t current)
{
    int16_t *frame_200 = (bus == 1u) ? s_can_tx_can1_200 : s_can_tx_can2_200;
    int16_t *frame_1ff = (bus == 1u) ? s_can_tx_can1_1ff : s_can_tx_can2_1ff;

    if (can_id >= 0x201u && can_id <= 0x204u)
    {
        frame_200[can_id - 0x201u] = current;
    }
    else if (can_id >= 0x205u && can_id <= 0x208u)
    {
        frame_1ff[can_id - 0x205u] = current;
    }
}

static inline void can_tx_process_rs485_axis(MotorId actuator_id,
                                             const motor_node_param_t *node,
                                             int16_t current)
{
    const uint8_t port = (node != NULL) ? node->rs485_port : 0u;

    if (can_tx_process_extra_item(port, actuator_id, node, current) == 0u)
    {
        watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
    }
}

static inline void can_tx_process_mit_or_extra_axis(uint8_t fallback_bus,
                                                    MotorId actuator_id,
                                                    const motor_node_param_t *node,
                                                    int16_t current)
{
    const uint8_t node_bus = can_tx_node_bus(fallback_bus, node);

    if (can_tx_process_can_mit_item(node_bus, actuator_id, node, current) == 0u &&
        can_tx_process_extra_item(node_bus, actuator_id, node, current) == 0u)
    {
        watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
    }
}

static inline void can_tx_process_rm_axis(uint8_t fallback_bus,
                                          MotorId actuator_id,
                                          const motor_node_param_t *node,
                                          uint16_t can_id,
                                          int16_t current,
                                          uint8_t *flags,
                                          int16_t *out_current)
{
    const uint8_t node_bus = can_tx_node_bus(fallback_bus, node);
    const int16_t rm_current = can_tx_build_rm_current_from_actuator(actuator_id, current);
    const int16_t limited_current = motor_cfg_limit_current_node(node, rm_current);

    if (flags != NULL && limited_current != rm_current)
    {
        *flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    can_tx_store_rm_current(node_bus, can_id, limited_current);
    if (out_current != NULL)
    {
        *out_current = limited_current;
    }
}

static inline int16_t can_tx_process_axis(uint8_t fallback_bus,
                                          MotorId actuator_id,
                                          const motor_node_param_t *node,
                                          uint8_t is_rm_group,
                                          uint16_t can_id,
                                          int16_t current,
                                          uint8_t *flags)
{
    if (motor_cfg_transport(node) == MOTOR_TRANSPORT_RS485)
    {
        can_tx_process_rs485_axis(actuator_id, node, current);
        return current;
    }
    else if (is_rm_group == 0u)
    {
        can_tx_process_mit_or_extra_axis(fallback_bus, actuator_id, node, current);
        return current;
    }
    else
    {
        int16_t limited_current = current;
        can_tx_process_rm_axis(fallback_bus, actuator_id, node, can_id, current, flags, &limited_current);
        return limited_current;
    }
}

static void can_tx_clamp_applied_mit(const motor_route_t *route, MotorApplied *applied)
{
    const can_mit_motor_limits_t *limits = (route != NULL) ? route->mitLimits : NULL;
    fp32 old_value;

    if (route == NULL ||
        applied == NULL ||
        limits == NULL ||
        applied->active == 0u ||
        applied->mode == (uint8_t)MotorModeDisable)
    {
        return;
    }

    old_value = applied->q;
    applied->q = can_tx_clamp_fp32(applied->q, -limits->position_max, limits->position_max);
    if (applied->q != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->dq;
    applied->dq = can_tx_clamp_fp32(applied->dq, -limits->velocity_max, limits->velocity_max);
    if (applied->dq != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->kp;
    applied->kp = can_tx_clamp_fp32(applied->kp, 0.0f, limits->kp_max);
    if (applied->kp != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->kd;
    applied->kd = can_tx_clamp_fp32(applied->kd, 0.0f, limits->kd_max);
    if (applied->kd != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->tau;
    applied->tau = can_tx_clamp_fp32(applied->tau, -limits->torque_max, limits->torque_max);
    if (applied->tau != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
}

static void can_tx_update_applied(const motor_route_t *route,
                                  const MotorCmd *cmd,
                                  int16_t current,
                                  uint8_t flags)
{
    MotorApplied applied;

    if (route == NULL || (uint32_t)route->motorId >= (uint32_t)MotorCount)
    {
        return;
    }

    (void)memset(&applied, 0, sizeof(applied));
    applied.tick = can_tx_now_ms();
    applied.bus = route->bus;
    applied.transport = route->transport;
    applied.protocol = route->protocol;
    applied.txId = route->canId;
    applied.current = current;
    applied.flags = flags;
    applied.driveState = (uint8_t)can_tx_drive_state_from_feedback(route->motorId);

    if ((flags & ((uint8_t)MotorAppliedFlagForceDisabled | (uint8_t)MotorAppliedFlagCmdExpired)) != 0u)
    {
        applied.active = 1u;
        applied.mode = (uint8_t)MotorModeDisable;
        applied.driveState = (uint8_t)MotorDriveStateDisabled;
    }
    else if (cmd != NULL && cmd->active != 0u && cmd->mode != (uint8_t)MotorModeNone)
    {
        applied.active = 1u;
        applied.mode = cmd->mode;
        applied.q = cmd->q;
        applied.dq = cmd->dq;
        applied.kp = cmd->kp;
        applied.kd = cmd->kd;
        applied.tau = cmd->tau;
    }
    else if (current != 0)
    {
        applied.active = 1u;
        applied.mode = (uint8_t)MotorModeCurrent;
    }

    if (route->transport == (uint8_t)MotorTransportCAN &&
        route->isRmGroup == 0u &&
        route->mitLimits != NULL &&
        applied.mode == (uint8_t)MotorModeCurrent)
    {
        applied.tau = can_tx_current_to_mit_torque(route->node, current, route->mitLimits);
    }

    if (route->transport == (uint8_t)MotorTransportCAN && route->isRmGroup == 0u)
    {
        can_tx_clamp_applied_mit(route, &applied);
    }

    LowStateUpdateApplied(route->motorId, &applied);
}

static void can_tx_process_instance(const motor_route_t *route,
                                    uint8_t allowed,
                                    sdlog_actuator_current_t *log)
{
    MotorId actuator_id;
    const motor_node_param_t *node;
    MotorCmd cmd;
    uint8_t have_cmd;
    uint8_t flags = 0u;
    int16_t current = 0;
    int16_t applied_current = 0;

    if (route == NULL || route->enabled == 0u)
    {
        return;
    }

    actuator_id = route->motorId;
    node = route->node;
    if (node == NULL || can_tx_actuator_id_valid(actuator_id) == 0u)
    {
        return;
    }

    if (allowed == 0u)
    {
        can_tx_force_disabled_cmd(actuator_id);
        flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    have_cmd = can_tx_get_cmd_copy(actuator_id, &cmd);
    if (s_can_tx_cmd_expired[actuator_id] != 0u)
    {
        flags |= (uint8_t)MotorAppliedFlagCmdExpired;
    }
    if (allowed != 0u &&
        have_cmd != 0u &&
        cmd.active != 0u &&
        cmd.mode != (uint8_t)MotorModeNone &&
        cmd.mode != (uint8_t)MotorModeDisable)
    {
        current = cmd.current;
    }
    can_tx_log_motor_cmd(log, route, current);

    if (route->transport == (uint8_t)MotorTransportCAN && route->canId == 0u)
    {
        flags |= (uint8_t)MotorAppliedFlagSkipped;
        can_tx_update_applied(route, (have_cmd != 0u) ? &cmd : NULL, 0, flags);
        return;
    }

    applied_current = can_tx_process_axis(route->bus,
                                          actuator_id,
                                          node,
                                          route->isRmGroup,
                                          route->canId,
                                          current,
                                          &flags);
    can_tx_update_applied(route, (have_cmd != 0u) ? &cmd : NULL, applied_current, flags);
}

static void can_tx_exec_instances(uint8_t online, uint8_t output_locked)
{
    sdlog_actuator_current_t log = {0};
    const uint8_t count = motor_route_count();

    can_tx_clear_rm_frames();
    can_tx_cache_lowcmd();

    for (uint8_t i = 0u; i < count; i++)
    {
        const motor_route_t *route = motor_route_get(i);
        uint8_t allowed;

        if (route == NULL || route->enabled == 0u)
        {
            continue;
        }
        if (can_tx_route_role_active(route) == 0u)
        {
            continue;
        }

        allowed = (output_locked != 0u) ? 0u : can_tx_route_allowed(route, online);
        can_tx_process_instance(route, allowed, &log);
    }

    can_tx_log_actuator_current(&log);
}

static uint8_t can_tx_rm_frame_has_output(const int16_t frame[4])
{
    return (uint8_t)(frame[0] != 0 ||
                     frame[1] != 0 ||
                     frame[2] != 0 ||
                     frame[3] != 0);
}

static uint8_t can_tx_rm_group_has_config(uint8_t bus, uint16_t group_id)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;

    if (can_tx_bus_index(bus, &bus_index) == 0u ||
        can_tx_rm_group_index(group_id, &group_index) == 0u)
    {
        return 0u;
    }

    return s_can_tx_rm_group_configured[bus_index][group_index];
}

static void can_tx_emit_rm_group_if_needed(uint8_t bus, uint16_t group_id, const int16_t frame[4])
{
    if (can_tx_rm_frame_has_output(frame) == 0u &&
        can_tx_rm_group_has_config(bus, group_id) == 0u)
    {
        return;
    }

    CAN_cmd_rm_group(bus, group_id, frame[0], frame[1], frame[2], frame[3]);
}

static void can_tx_emit_rm_frames(void)
{
    can_tx_emit_rm_group_if_needed(1u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can1_200);
    can_tx_emit_rm_group_if_needed(1u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can1_1ff);
    can_tx_emit_rm_group_if_needed(2u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can2_200);
    can_tx_emit_rm_group_if_needed(2u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can2_1ff);
}

// 目标工程可在这里接入非大疆、非 MIT 的特殊电机发送逻辑。
__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         MotorId actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current)
{
    (void)bus;
    (void)actuator_id;
    (void)node;
    (void)current;
    return 0u;
}

__weak int can_mit_motor_send_cmd(uint8_t bus,
                                  uint16_t std_id,
                                  const can_mit_motor_limits_t *limits,
                                  const can_mit_motor_cmd_t *cmd)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    (void)cmd;
    return -1;
}

__weak int can_mit_motor_send_enable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int can_mit_motor_send_disable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int can_mit_motor_send_stop(uint8_t bus,
                                   uint16_t std_id,
                                   const can_mit_motor_limits_t *limits)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    return -1;
}

__weak uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                             uint8_t motor_id,
                                             const can_mit_motor_limits_t *limits,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             can_mit_motor_feedback_t *feedback)
{
    (void)std_id;
    (void)motor_id;
    (void)limits;
    (void)dlc;
    (void)data;
    (void)feedback;
    return 0u;
}

// CAN 命令发送任务：收集各轴命令，按轴装配表转换协议并统一发出。
void can_command_tx_task(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    can_tx_cache_rm_groups();

    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        watch_task_beat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
        const bool_t dbus_offline = toe_is_error(DBUS_TOE);
        const uint8_t output_locked = robot_safety_output_locked();

        can_tx_exec_instances(dbus_offline ? 0u : 1u, output_locked);
        can_tx_emit_rm_frames();

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
