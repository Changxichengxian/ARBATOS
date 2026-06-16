/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by can_command_tx_task.c.
 */

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
                                                  int16_t current,
                                                  uint8_t *flags)
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
                can_tx_mit_period_due(now, mit_last_disable_tick[actuator_id], cmd_period_ms) != 0u)
            {
                if (can_tx_mit_take_frame_budget(now) != 0u)
                {
                    const int ret = can_mit_motor_send_disable(bus, std_id);
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
            if (can_tx_mit_period_due(now, mit_last_enable_tick[actuator_id], cmd_period_ms) != 0u)
            {
                if (can_tx_mit_take_frame_budget(now) != 0u)
                {
                    const int ret = can_mit_motor_send_enable(bus, std_id);
                    if (ret == 0)
                    {
                        mit_last_enable_tick[actuator_id] = now;
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

        if (can_tx_mit_take_frame_budget(now) == 0u)
        {
            if (flags != NULL)
            {
                *flags |= (uint8_t)MotorAppliedFlagSkipped;
            }
            return 1u;
        }

        can_tx_build_mit_cmd_from_actuator(node, &cmd, current, limits, &mit_cmd);
        {
            const int ret = can_mit_motor_send_cmd(bus, std_id, limits, &mit_cmd);
            if (ret == 0)
            {
                mit_last_cmd_tick[actuator_id] = now;
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

    can_tx_build_mit_cmd_from_actuator(node, &cmd, current, limits, &mit_cmd);
    if (can_mit_motor_send_cmd(bus, std_id, limits, &mit_cmd) != 0 && flags != NULL)
    {
        *flags |= (uint8_t)MotorAppliedFlagSkipped;
    }
    return 1u;
}
