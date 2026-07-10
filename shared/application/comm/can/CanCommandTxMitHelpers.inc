/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by CanTxTask.c.
 */

static uint8_t CanTxMitIsWheelLegWheel(MotorId actuator_id)
{
    if (RobotProfileIsWheelLegMit() == 0u)
    {
        return 0u;
    }

    return (uint8_t)((uint8_t)actuator_id == g_config.WheelLegMit.left_wheel_actuator ||
                     (uint8_t)actuator_id == g_config.WheelLegMit.right_wheel_actuator);
}

static uint16_t CanTxMitCmdPeriodMs(MotorId actuator_id)
{
    return (CanTxMitIsWheelLegWheel(actuator_id) != 0u) ?
               CAN_TX_MIT_WHEEL_CMD_PERIOD_MS :
               CAN_TX_MIT_JOINT_CMD_PERIOD_MS;
}

static uint8_t CanTxMitTakeFrameBudget(uint32_t now_ms)
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

static uint8_t CanTxMitPeriodDue(uint32_t now_ms, uint32_t last_tick_ms, uint16_t period_ms)
{
    return (uint8_t)(last_tick_ms == 0u ||
                     (uint32_t)(now_ms - last_tick_ms) >= (uint32_t)period_ms);
}

static uint8_t CanTxMitGetFeedbackState(MotorId actuator_id,
                                             const motor_node_param_t *node,
                                             MotorState *feedback)
{
    const uint16_t expected_rx_id = MotorCfgFeedbackId(node);
    const uint8_t expected_motor_low_id = (uint8_t)(MotorCfgCanId(node) & 0x0Fu);

    if (feedback == NULL || CanTxActuatorIdValid(actuator_id) == 0u)
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

static uint8_t CanTxMitCommandAuthorizedNow(MotorId actuator_id, const MotorCmd *cached)
{
    MotorCmd latest;
    uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

    (void)memset(&latest, 0, sizeof(latest));
    if (cached == NULL ||
        LowCmdGetMotor(actuator_id, &latest) == 0u ||
        LowCmdGetInhibitWriter(actuator_id, &inhibitWriter) == 0u)
    {
        return 0u;
    }
    return CanTxCachedCmdAuthorized(cached, &latest, inhibitWriter);
}

static uint8_t CanTxMitSendEnableAuthorized(MotorId actuator_id,
                                            const MotorCmd *cached,
                                            uint8_t bus,
                                            uint16_t std_id,
                                            int *out_ret)
{
    uint8_t authorized;
    int ret = -1;

    taskENTER_CRITICAL();
    authorized = (uint8_t)(RobotSafetyOutputLocked() == 0u &&
                           CanTxMitCommandAuthorizedNow(actuator_id, cached) != 0u);
    if (authorized != 0u)
    {
        ret = CanMitMotorSendEnable(bus, std_id);
    }
    taskEXIT_CRITICAL();

    if (out_ret != NULL)
    {
        *out_ret = ret;
    }
    return authorized;
}

static uint8_t CanTxMitSendCmdAuthorized(MotorId actuator_id,
                                         const MotorCmd *cached,
                                         uint8_t bus,
                                         uint16_t std_id,
                                         const CanMitMotorLimits *limits,
                                         const CanMitMotorCmd *mit_cmd,
                                         int *out_ret)
{
    uint8_t authorized;
    int ret = -1;

    taskENTER_CRITICAL();
    authorized = (uint8_t)(RobotSafetyOutputLocked() == 0u &&
                           CanTxMitCommandAuthorizedNow(actuator_id, cached) != 0u);
    if (authorized != 0u)
    {
        /* CAN 入队是非阻塞操作，和最终复核放在同一短临界区。 */
        ret = CanMitMotorSendCmd(bus, std_id, limits, mit_cmd);
    }
    taskEXIT_CRITICAL();

    if (out_ret != NULL)
    {
        *out_ret = ret;
    }
    return authorized;
}

static void CanTxMitMarkAuthorityReject(MotorId actuator_id, uint8_t *flags)
{
    if (flags != NULL)
    {
        *flags |= (uint8_t)MotorAppliedFlagForceDisabled |
                  (uint8_t)MotorAppliedFlagSkipped;
    }
    if (CanTxActuatorIdValid(actuator_id) != 0u)
    {
        s_can_tx_authority_reject_count[actuator_id]++;
    }
}

static void CanTxMitSyncModeState(MotorId actuator_id,
                                       const motor_node_param_t *node,
                                       uint8_t *enabled,
                                       uint8_t *disabled_confirmed)
{
    MotorState feedback;

    if (enabled == NULL || disabled_confirmed == NULL ||
        CanTxMitGetFeedbackState(actuator_id, node, &feedback) == 0u)
    {
        return;
    }

    if (feedback.state == CAN_TX_MIT_STATE_ENABLED)
    {
        *enabled = 1u;
        *disabled_confirmed = 0u;
    }
    else if (feedback.state == 0u && MotorCfgProtocol(node) == MOTOR_PROTOCOL_DM_3MODE)
    {
        *disabled_confirmed = 0u;
    }
    else if (feedback.state >= CAN_TX_MIT_STATE_FAULT_MIN)
    {
        *enabled = 0u;
        *disabled_confirmed = 1u;
    }
    else if (feedback.state == CAN_TX_MIT_STATE_DISABLED)
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

static inline int16_t CanTxFp32ToI16Saturated(fp32 x)
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

static inline uint8_t CanTxCmdModeUsesPositionFeedback(MotorMode mode)
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

static inline uint8_t CanTxCmdModeUsesVelocityFeedback(MotorMode mode)
{
    if (CanTxCmdModeUsesPositionFeedback(mode) != 0u)
    {
        return 1u;
    }
    return (uint8_t)((mode == MotorModeSpeed || mode == MotorModeDamping) ? 1u : 0u);
}

// 轴配置里可以指定 CAN 总线；没指定时使用该轴的默认总线。
static inline uint8_t CanTxNodeBus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    return MotorCfgCanBus(fallback_bus, node);
}

// RM 电机仍优先吃旧双环 PID 给出的电流；只有通用状态命令才临时换算成电流。
static int16_t CanTxBuildRmCurrentFromActuator(MotorId actuator_id,
                                                     int16_t current)
{
    MotorCmd cmd;
    MotorState fb;
    MotorMode mode;
    fp32 rm_current;

    if (CanTxGetCmdCopy(actuator_id, &cmd) == 0u || cmd.active == 0u)
    {
        return current;
    }

    mode = (MotorMode)cmd.mode;
    if (CanTxCmdModeUsesVelocityFeedback(mode) == 0u)
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
    if (CanTxCmdModeUsesPositionFeedback(mode) != 0u)
    {
        rm_current += cmd.kp * (cmd.q - fb.q);
    }
    if (CanTxCmdModeUsesVelocityFeedback(mode) != 0u)
    {
        rm_current += cmd.kd * (cmd.dq - fb.dq);
    }

    return CanTxFp32ToI16Saturated(rm_current);
}

// 旧控制链常给“电流”，MIT 电机要力矩，这里按型号量程换成力矩。
static inline fp32 CanTxCurrentToMitTorque(const motor_node_param_t *node,
                                                int16_t current,
                                                const CanMitMotorLimits *limits)
{
    const MotorModelDbEntry *entry;
    int16_t range_abs = 0;
    fp32 torque;

    if (node == NULL || limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0.0f;
    }

    entry = MotorCfgModelDb(node->model);
    if (entry != NULL)
    {
        range_abs = entry->cmd_current_range_abs;
    }
    if (range_abs <= 0)
    {
        range_abs = 32767;
    }

    torque = ((fp32)current) * limits->torque_max / (fp32)range_abs;
    return CanTxClampFp32(torque, -limits->torque_max, limits->torque_max);
}

// 把通用执行器命令翻译成 MIT 命令；没有新命令时退回电流转力矩。
static inline void CanTxBuildMitCmdFromActuator(const motor_node_param_t *node,
                                                      const MotorCmd *src,
                                                      int16_t current,
                                                      const CanMitMotorLimits *limits,
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

    if (CanTxCmdModeUsesPositionFeedback((MotorMode)mode) != 0u)
    {
        out->position = (src != NULL) ? src->q : 0.0f;
        out->velocity = (src != NULL) ? src->dq : 0.0f;
        out->kp = (src != NULL) ? src->kp : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->tau : 0.0f;
    }
    else if (CanTxCmdModeUsesVelocityFeedback((MotorMode)mode) != 0u)
    {
        out->velocity = (src != NULL) ? src->dq : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->tau : 0.0f;
    }
    else
    {
        out->torque = CanTxCurrentToMitTorque(node, current, limits);
    }
}

// 处理单个 MIT 轴：必要时先使能，再发送本周期命令。
static inline uint8_t CanTxProcessCanMitItem(uint8_t bus,
                                                  MotorId actuator_id,
                                                  const motor_node_param_t *node,
                                                  int16_t current,
                                                  uint8_t *flags)
{
    static uint8_t mit_enabled[MotorCount];
    static uint8_t mit_disabled_confirmed[MotorCount];
    static uint32_t mit_last_enable_tick[MotorCount];
    static uint32_t mit_last_disable_tick[MotorCount];
    static uint32_t mit_last_cmd_tick[MotorCount];
    const CanMitMotorLimits *limits;
    MotorCmd cmd;
    mit_motor_cmd_t mit_cmd;
    uint8_t have_cmd;
    uint8_t active_cmd;
    uint16_t cmd_period_ms;
    uint16_t std_id;
    const uint32_t now = CanTxNowMs();

    if (node == NULL)
    {
        return 0u;
    }

    limits = MotorCfgMitLimits(node);
    if (limits == NULL)
    {
        return 0u;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    have_cmd = CanTxGetCmdCopy(actuator_id, &cmd);
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
    if (active_cmd != 0u && CanTxMitCommandAuthorizedNow(actuator_id, &cmd) == 0u)
    {
        active_cmd = 0u;
        current = 0;
        CanTxMitMarkAuthorityReject(actuator_id, flags);
    }

    std_id = MotorCfgCanId(node);
    if (std_id == 0u)
    {
        return 0u;
    }
    cmd_period_ms = CanTxMitCmdPeriodMs(actuator_id);

    if (CanTxActuatorIdValid(actuator_id) != 0u)
    {
        CanTxMitSyncModeState(actuator_id,
                                   node,
                                   &mit_enabled[actuator_id],
                                   &mit_disabled_confirmed[actuator_id]);

        if (active_cmd == 0u)
        {
            mit_last_enable_tick[actuator_id] = 0u;
            mit_last_cmd_tick[actuator_id] = 0u;

            if (mit_disabled_confirmed[actuator_id] == 0u &&
                CanTxMitPeriodDue(now, mit_last_disable_tick[actuator_id], cmd_period_ms) != 0u)
            {
                if (CanTxMitTakeFrameBudget(now) != 0u)
                {
                    const int ret = CanMitMotorSendDisable(bus, std_id);
                    if (ret == 0)
                    {
                        mit_last_disable_tick[actuator_id] = now;
                        mit_enabled[actuator_id] = 0u;
                    }
                    else if (flags != NULL)
                    {
                        *flags |= (uint8_t)MotorAppliedFlagSkipped;
                    }
                }
                else if (flags != NULL)
                {
                    *flags |= (uint8_t)MotorAppliedFlagSkipped;
                }
            }
            return 1u;
        }

        mit_disabled_confirmed[actuator_id] = 0u;
        mit_last_disable_tick[actuator_id] = 0u;

        if (mit_enabled[actuator_id] == 0u)
        {
            if (CanTxMitPeriodDue(now, mit_last_enable_tick[actuator_id], cmd_period_ms) != 0u)
            {
                if (CanTxMitTakeFrameBudget(now) != 0u)
                {
                    int ret;

                    if (CanTxMitSendEnableAuthorized(actuator_id,
                                                     &cmd,
                                                     bus,
                                                     std_id,
                                                     &ret) == 0u)
                    {
                        CanTxMitMarkAuthorityReject(actuator_id, flags);
                        return 1u;
                    }
                    if (ret == 0)
                    {
                        mit_last_enable_tick[actuator_id] = now;
                        s_can_tx_mit_enable_tx_count[actuator_id]++;
                    }
                    else if (flags != NULL)
                    {
                        *flags |= (uint8_t)MotorAppliedFlagSkipped;
                    }
                }
                else if (flags != NULL)
                {
                    *flags |= (uint8_t)MotorAppliedFlagSkipped;
                }
            }
            return 1u;
        }

        if (mit_last_cmd_tick[actuator_id] != 0u &&
            (uint32_t)(now - mit_last_cmd_tick[actuator_id]) < cmd_period_ms)
        {
            return 1u;
        }

        if (CanTxMitTakeFrameBudget(now) == 0u)
        {
            if (flags != NULL)
            {
                *flags |= (uint8_t)MotorAppliedFlagSkipped;
            }
            return 1u;
        }

        CanTxBuildMitCmdFromActuator(node, &cmd, current, limits, &mit_cmd);
        {
            int ret;

            if (CanTxMitSendCmdAuthorized(actuator_id,
                                          &cmd,
                                          bus,
                                          std_id,
                                          limits,
                                          &mit_cmd,
                                          &ret) == 0u)
            {
                CanTxMitMarkAuthorityReject(actuator_id, flags);
                return 1u;
            }
            if (ret == 0)
            {
                mit_last_cmd_tick[actuator_id] = now;
                s_can_tx_mit_cmd_tx_count[actuator_id]++;
            }
            else if (flags != NULL)
            {
                *flags |= (uint8_t)MotorAppliedFlagSkipped;
            }
        }
        return 1u;
    }

    if (active_cmd == 0u)
    {
        return 1u;
    }

    CanTxBuildMitCmdFromActuator(node, &cmd, current, limits, &mit_cmd);
    {
        int ret;

        if (CanTxMitSendCmdAuthorized(actuator_id,
                                      &cmd,
                                      bus,
                                      std_id,
                                      limits,
                                      &mit_cmd,
                                      &ret) == 0u)
        {
            CanTxMitMarkAuthorityReject(actuator_id, flags);
            return 1u;
        }
        if (ret == 0)
        {
            s_can_tx_mit_cmd_tx_count[actuator_id]++;
        }
        else if (flags != NULL)
        {
            *flags |= (uint8_t)MotorAppliedFlagSkipped;
        }
    }
    return 1u;
}
