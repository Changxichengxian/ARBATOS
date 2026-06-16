/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by can_command_tx_task.c.
 */

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
                                                    int16_t current,
                                                    uint8_t *flags)
{
    const uint8_t node_bus = can_tx_node_bus(fallback_bus, node);

    if (can_tx_process_can_mit_item(node_bus, actuator_id, node, current, flags) == 0u &&
        can_tx_process_extra_item(node_bus, actuator_id, node, current) == 0u)
    {
        if (flags != NULL)
        {
            *flags |= (uint8_t)MotorAppliedFlagSkipped;
        }
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
        can_tx_process_mit_or_extra_axis(fallback_bus, actuator_id, node, current, flags);
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
    uint8_t start_index = 0u;

    can_tx_clear_rm_frames();
    can_tx_cache_lowcmd();

    if (count != 0u)
    {
        start_index = (uint8_t)(s_can_tx_route_start_index % count);
        s_can_tx_route_start_index = (uint8_t)((start_index + 1u) % count);
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        const uint8_t route_index = (uint8_t)((start_index + i) % count);
        const motor_route_t *route = motor_route_get(route_index);
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
