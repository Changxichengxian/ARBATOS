/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by CanTxTask.c.
 */

static void CanTxLogActuatorCurrent(const sdlog_actuator_current_t *log)
{
    if (log != NULL && CanTxLogDue() != 0u)
    {
        SdLogWrite(SDLOG_TAG_ACTUATOR_CURRENT, log, (uint16_t)sizeof(*log));
    }
}

static uint8_t CanTxArmRoleActive(void)
{
    return (uint8_t)(RobotProfileNeedArmTask() || RobotProfileIsWheelLegMit());
}

static uint8_t CanTxRouteRoleActive(const MotorRoute *route)
{
    if (route == NULL)
    {
        return 0u;
    }
    if (route->role == MotorRoleArm && CanTxArmRoleActive() == 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t CanTxFrictionEnabled(const MotorRoute *route)
{
    if (route == NULL || route->role != MotorRoleFriction)
    {
        return 1u;
    }
    if (route->roleIndex >= 4u)
    {
        return 0u;
    }
    return (g_config.shoot.fric_motor_dir[route->roleIndex] != 0) ? 1u : 0u;
}

static uint8_t CanTxRouteAllowed(const MotorRoute *route)
{
    if (route == NULL)
    {
        return 0u;
    }
    if (robot_mode_allow_motor(route->motorId) == 0u)
    {
        return 0u;
    }
    if (route->role == MotorRoleChassis && CanTxAllowChassis() == 0u)
    {
        return 0u;
    }
    if (CanTxFrictionEnabled(route) == 0u)
    {
        return 0u;
    }
    return 1u;
}

static void CanTxLogMotorCmd(sdlog_actuator_current_t *log,
                                 const MotorRoute *route,
                                 int16_t current)
{
    if (log == NULL || route == NULL)
    {
        return;
    }

    switch (route->role)
    {
    case MotorRoleChassis:
        if (route->roleIndex < 4u)
        {
            log->chassis[route->roleIndex] = current;
        }
        break;
    case MotorRoleYaw:
        log->yaw = current;
        break;
    case MotorRolePitch:
        log->pitch = current;
        break;
    case MotorRoleTrigger:
        log->trigger = current;
        break;
    case MotorRoleFriction:
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
static void CanTxClearRmFrames(void)
{
    (void)memset(s_can_tx_can1_200, 0, sizeof(s_can_tx_can1_200));
    (void)memset(s_can_tx_can1_1ff, 0, sizeof(s_can_tx_can1_1ff));
    (void)memset(s_can_tx_can2_200, 0, sizeof(s_can_tx_can2_200));
    (void)memset(s_can_tx_can2_1ff, 0, sizeof(s_can_tx_can2_1ff));
    for (uint8_t bus = 0u; bus < 2u; bus++)
    {
        for (uint8_t group = 0u; group < 2u; group++)
        {
            for (uint8_t slot = 0u; slot < 4u; slot++)
            {
                s_can_tx_rm_motor[bus][group][slot] = MotorCount;
            }
        }
    }
}

static inline void CanTxStoreRmCurrent(uint8_t bus,
                                            uint16_t can_id,
                                            int16_t current,
                                            MotorId actuator_id)
{
    int16_t *frame_200;
    int16_t *frame_1ff;
    uint8_t bus_index;

    if ((bus != 1u && bus != 2u) || CanTxActuatorIdValid(actuator_id) == 0u)
    {
        return;
    }

    bus_index = (uint8_t)(bus - 1u);
    frame_200 = (bus == 1u) ? s_can_tx_can1_200 : s_can_tx_can2_200;
    frame_1ff = (bus == 1u) ? s_can_tx_can1_1ff : s_can_tx_can2_1ff;

    if (can_id >= 0x201u && can_id <= 0x204u)
    {
        const uint8_t slot = (uint8_t)(can_id - 0x201u);
        frame_200[slot] = current;
        s_can_tx_rm_motor[bus_index][0u][slot] = actuator_id;
    }
    else if (can_id >= 0x205u && can_id <= 0x208u)
    {
        const uint8_t slot = (uint8_t)(can_id - 0x205u);
        frame_1ff[slot] = current;
        s_can_tx_rm_motor[bus_index][1u][slot] = actuator_id;
    }
}

static inline uint8_t CanTxProcessRs485Axis(MotorId actuator_id,
                                              const motor_node_param_t *node,
                                              int16_t current,
                                              const MotorCmd *cmd,
                                              uint8_t *flags)
{
    const uint8_t port = (node != NULL) ? node->rs485_port : 0u;

    if (CanTxProcessExtraItem(port, actuator_id, node, current, cmd) == 0u)
    {
        if (flags != NULL)
        {
            *flags |= (uint8_t)MotorAppliedFlagSkipped;
        }
        WatchTaskError(WATCH_TASK_CAN_COMMAND_TX);
        return 0u;
    }
    return 1u;
}

static uint8_t CanTxRs485DriverOwnsApplied(const motor_node_param_t *node)
{
    const motor_protocol_e protocol = MotorCfgProtocol(node);

    return (uint8_t)(protocol == MOTOR_PROTOCOL_UNITREE_RS485 ||
                     protocol == MOTOR_PROTOCOL_N6014B_RS485);
}

static uint8_t CanTxIsUnitreeRoute(const MotorRoute *route)
{
    return (uint8_t)(route != NULL &&
                     route->transport == (uint8_t)MotorTransportRS485 &&
                     MotorCfgProtocol(route->node) == MOTOR_PROTOCOL_UNITREE_RS485);
}

static uint16_t CanTxUnitreePeriodMs(const MotorRoute *route)
{
    if (route != NULL && route->role == MotorRoleArm && route->roleIndex == 0u)
    {
        return (g_config.ArmJ0Unitree.control_period_ms != 0u) ?
            g_config.ArmJ0Unitree.control_period_ms : 5u;
    }
    return RobotProfileCanCommandTxPeriodMs();
}

static void CanTxMergeDriverAppliedFlags(MotorId actuator_id, uint8_t flags)
{
    MotorApplied applied;

    if (flags == 0u || LowStateGetApplied(actuator_id, &applied) == 0u)
    {
        return;
    }

    applied.flags |= flags;
    LowStateUpdateApplied(actuator_id, &applied);
}

static inline void CanTxProcessMitOrExtraAxis(uint8_t fallback_bus,
                                                    MotorId actuator_id,
                                                    const motor_node_param_t *node,
                                                    int16_t current,
                                                    const MotorCmd *cmd,
                                                    uint8_t *flags)
{
    const uint8_t node_bus = CanTxNodeBus(fallback_bus, node);

    if (CanTxProcessCanMitItem(node_bus, actuator_id, node, current, flags) == 0u &&
        CanTxProcessExtraItem(node_bus, actuator_id, node, current, cmd) == 0u)
    {
        if (flags != NULL)
        {
            *flags |= (uint8_t)MotorAppliedFlagSkipped;
        }
        WatchTaskError(WATCH_TASK_CAN_COMMAND_TX);
    }
}

static inline void CanTxProcessRmAxis(uint8_t fallback_bus,
                                          MotorId actuator_id,
                                          const motor_node_param_t *node,
                                          uint16_t can_id,
                                          int16_t current,
                                          uint8_t *flags,
                                          int16_t *out_current)
{
    const uint8_t node_bus = CanTxNodeBus(fallback_bus, node);
    const int16_t rm_current = CanTxBuildRmCurrentFromActuator(actuator_id, current);
    const int16_t limited_current = MotorCfgLimitCurrentNode(node, rm_current);

    if (flags != NULL && limited_current != rm_current)
    {
        *flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    CanTxStoreRmCurrent(node_bus, can_id, limited_current, actuator_id);
    if (out_current != NULL)
    {
        *out_current = limited_current;
    }
}

static inline int16_t CanTxProcessAxis(uint8_t fallback_bus,
                                          MotorId actuator_id,
                                          const motor_node_param_t *node,
                                          uint8_t is_rm_group,
                                          uint16_t can_id,
                                          int16_t current,
                                          const MotorCmd *cmd,
                                          uint8_t *flags,
                                          uint8_t *tx_succeeded)
{
    if (tx_succeeded != NULL)
    {
        *tx_succeeded = 0u;
    }
    if (MotorCfgTransport(node) == MOTOR_TRANSPORT_RS485)
    {
        const uint8_t success = CanTxProcessRs485Axis(actuator_id, node, current, cmd, flags);
        if (tx_succeeded != NULL)
        {
            *tx_succeeded = success;
        }
        return current;
    }
    else if (is_rm_group == 0u)
    {
        CanTxProcessMitOrExtraAxis(fallback_bus, actuator_id, node, current, cmd, flags);
        return current;
    }
    else
    {
        int16_t limited_current = current;
        CanTxProcessRmAxis(fallback_bus, actuator_id, node, can_id, current, flags, &limited_current);
        return limited_current;
    }
}

static void CanTxClampAppliedMit(const MotorRoute *route, MotorApplied *applied)
{
    const CanMitMotorLimits *limits = (route != NULL) ? route->mitLimits : NULL;
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
    applied->q = CanTxClampFp32(applied->q, -limits->position_max, limits->position_max);
    if (applied->q != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->dq;
    applied->dq = CanTxClampFp32(applied->dq, -limits->velocity_max, limits->velocity_max);
    if (applied->dq != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->kp;
    applied->kp = CanTxClampFp32(applied->kp, 0.0f, limits->kp_max);
    if (applied->kp != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->kd;
    applied->kd = CanTxClampFp32(applied->kd, 0.0f, limits->kd_max);
    if (applied->kd != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
    old_value = applied->tau;
    applied->tau = CanTxClampFp32(applied->tau, -limits->torque_max, limits->torque_max);
    if (applied->tau != old_value)
    {
        applied->flags |= (uint8_t)MotorAppliedFlagLimited;
    }
}

static void CanTxUpdateApplied(const MotorRoute *route,
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
    applied.tick = CanTxNowMs();
    applied.bus = route->bus;
    applied.transport = route->transport;
    applied.protocol = route->protocol;
    applied.txId = route->canId;
    applied.current = current;
    applied.flags = flags;
    applied.driveState = (uint8_t)CanTxDriveStateFromFeedback(route->motorId);

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
        applied.tau = CanTxCurrentToMitTorque(route->node, current, route->mitLimits);
    }

    if (route->transport == (uint8_t)MotorTransportCAN && route->isRmGroup == 0u)
    {
        CanTxClampAppliedMit(route, &applied);
    }

    LowStateUpdateApplied(route->motorId, &applied);
}

static void CanTxApplyInhibitGate(MotorId actuator_id,
                                  MotorCmd *cmd,
                                  uint8_t *have_cmd,
                                  uint8_t *flags)
{
    MotorCmd latest;
    uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

    if (cmd == NULL || have_cmd == NULL || flags == NULL ||
        LowCmdGetInhibitWriter(actuator_id, &inhibitWriter) == 0u ||
        inhibitWriter == (uint16_t)LOWCMD_WRITER_NONE)
    {
        return;
    }

    (void)memset(&latest, 0, sizeof(latest));
    if (*have_cmd != 0u &&
        LowCmdGetMotor(actuator_id, &latest) != 0u &&
        CanTxCachedCmdAuthorized(cmd, &latest, inhibitWriter) != 0u)
    {
        return;
    }

    /* 覆盖本轮缓存，让 RM/MIT/扩展驱动都不能继续消费故障前的旧 current。 */
    CanTxForceDisabledCmd(actuator_id);
    *cmd = s_can_tx_cmd_cache[actuator_id];
    *have_cmd = 1u;
    *flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    s_can_tx_authority_reject_count[actuator_id]++;
}

static void CanTxRecheckUnitreeAuthority(MotorId actuator_id,
                                         MotorCmd *cmd,
                                         uint8_t *have_cmd,
                                         uint8_t *flags)
{
    MotorCmd latest;
    uint16_t inhibit_writer = (uint16_t)LOWCMD_WRITER_NONE;

    if (cmd == NULL || have_cmd == NULL || flags == NULL ||
        (*have_cmd != 0u && cmd->mode == (uint8_t)MotorModeDisable))
    {
        return;
    }

    (void)memset(&latest, 0, sizeof(latest));
    if (*have_cmd != 0u &&
        LowCmdGetMotor(actuator_id, &latest) != 0u &&
        LowCmdGetInhibitWriter(actuator_id, &inhibit_writer) != 0u &&
        UnitreeMotorCmdSnapshotAllowed(cmd, &latest, inhibit_writer) != 0u)
    {
        return;
    }

    /* 节流前再判一次，故障后必须先把旧缓存变成会立即发送的 BRAKE。 */
    CanTxForceDisabledCmd(actuator_id);
    *cmd = s_can_tx_cmd_cache[actuator_id];
    *have_cmd = 1u;
    *flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    s_can_tx_authority_reject_count[actuator_id]++;
}

static uint8_t CanTxRecheckCommandAuthority(MotorId actuator_id,
                                            MotorCmd *cmd,
                                            uint8_t *have_cmd,
                                            uint8_t *flags)
{
    MotorCmd latest;
    uint16_t inhibit_writer = (uint16_t)LOWCMD_WRITER_NONE;

    if (cmd == NULL || have_cmd == NULL || flags == NULL)
    {
        return 0u;
    }
    if (*have_cmd == 0u || cmd->active == 0u ||
        cmd->mode == (uint8_t)MotorModeNone ||
        cmd->mode == (uint8_t)MotorModeDisable)
    {
        return 1u;
    }

    (void)memset(&latest, 0, sizeof(latest));
    if (LowCmdGetMotor(actuator_id, &latest) != 0u &&
        LowCmdGetInhibitWriter(actuator_id, &inhibit_writer) != 0u &&
        CanTxCachedCmdAuthorized(cmd, &latest, inhibit_writer) != 0u)
    {
        return 1u;
    }

    /* 扩展驱动调用前最后复核，不允许 N6014b 等协议重用故障前快照。 */
    CanTxForceDisabledCmd(actuator_id);
    *cmd = s_can_tx_cmd_cache[actuator_id];
    *have_cmd = 1u;
    *flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    s_can_tx_authority_reject_count[actuator_id]++;
    return 0u;
}

static void CanTxProcessInstance(const MotorRoute *route,
                                    uint8_t allowed,
                                    sdlog_actuator_current_t *log)
{
    MotorId actuator_id;
    const motor_node_param_t *node;
    MotorCmd cmd;
    uint8_t have_cmd;
    uint8_t flags = 0u;
    uint8_t tx_succeeded = 0u;
    int16_t current = 0;
    int16_t applied_current = 0;

    if (route == NULL || route->enabled == 0u)
    {
        return;
    }

    actuator_id = route->motorId;
    node = route->node;
    if (node == NULL || CanTxActuatorIdValid(actuator_id) == 0u)
    {
        return;
    }

    if (allowed == 0u)
    {
        CanTxForceDisabledCmd(actuator_id);
        flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    have_cmd = CanTxGetCmdCopy(actuator_id, &cmd);
    if (s_can_tx_cmd_expired[actuator_id] != 0u)
    {
        flags |= (uint8_t)MotorAppliedFlagCmdExpired;
    }
    if (allowed != 0u)
    {
        CanTxApplyInhibitGate(actuator_id, &cmd, &have_cmd, &flags);
    }
    if (CanTxIsUnitreeRoute(route) != 0u)
    {
        CanTxRecheckUnitreeAuthority(actuator_id, &cmd, &have_cmd, &flags);
    }
    if (allowed != 0u &&
        have_cmd != 0u &&
        cmd.active != 0u &&
        cmd.mode != (uint8_t)MotorModeNone &&
        cmd.mode != (uint8_t)MotorModeDisable)
    {
        current = cmd.current;
    }
    if (route->transport == (uint8_t)MotorTransportCAN && route->canId == 0u)
    {
        flags |= (uint8_t)MotorAppliedFlagSkipped;
        CanTxLogMotorCmd(log, route, 0);
        CanTxUpdateApplied(route, (have_cmd != 0u) ? &cmd : NULL, 0, flags);
        return;
    }

    if (CanTxIsUnitreeRoute(route) != 0u &&
        UnitreeMotorTxDue(&s_can_tx_unitree_schedule[actuator_id],
                          (have_cmd != 0u) ? &cmd : NULL,
                          CanTxNowMs(),
                          CanTxUnitreePeriodMs(route)) == 0u)
    {
        CanTxMergeDriverAppliedFlags(actuator_id, flags);
        return;
    }

    if (CanTxRecheckCommandAuthority(actuator_id, &cmd, &have_cmd, &flags) == 0u)
    {
        current = 0;
    }
    CanTxLogMotorCmd(log, route, current);

    applied_current = CanTxProcessAxis(route->bus,
                                          actuator_id,
                                          node,
                                          route->isRmGroup,
                                          route->canId,
                                          current,
                                          (have_cmd != 0u) ? &cmd : NULL,
                                          &flags,
                                          &tx_succeeded);
    if (CanTxIsUnitreeRoute(route) != 0u && tx_succeeded != 0u)
    {
        UnitreeMotorTxMark(&s_can_tx_unitree_schedule[actuator_id],
                           (have_cmd != 0u) ? &cmd : NULL,
                           CanTxNowMs());
    }
    if (route->transport == (uint8_t)MotorTransportRS485 &&
        CanTxRs485DriverOwnsApplied(node) != 0u)
    {
        /* RS485 驱动已发布协议限幅和真实发送结果，这里只合并通用门控原因。 */
        CanTxMergeDriverAppliedFlags(actuator_id, flags);
    }
    else
    {
        CanTxUpdateApplied(route, (have_cmd != 0u) ? &cmd : NULL, applied_current, flags);
    }
}

static void CanTxExecInstances(uint8_t output_locked)
{
    sdlog_actuator_current_t log = {0};
    const uint8_t count = MotorRouteCount();
    uint8_t start_index = 0u;

    CanTxClearRmFrames();
    CanTxCacheLowcmd();

    if (count != 0u)
    {
        start_index = (uint8_t)(s_can_tx_route_start_index % count);
        s_can_tx_route_start_index = (uint8_t)((start_index + 1u) % count);
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        const uint8_t route_index = (uint8_t)((start_index + i) % count);
        const MotorRoute *route = MotorRouteGet(route_index);
        uint8_t allowed;

        if (route == NULL || route->enabled == 0u)
        {
            continue;
        }
        if (CanTxRouteRoleActive(route) == 0u)
        {
            continue;
        }

        allowed = (output_locked != 0u) ? 0u : CanTxRouteAllowed(route);
        CanTxProcessInstance(route, allowed, &log);
    }

    CanTxEmitRmFrames(&log);
    CanTxLogActuatorCurrent(&log);
}
