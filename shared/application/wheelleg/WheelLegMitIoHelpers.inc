/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static uint8_t WheelLegReadFeedback(MotorId id, MotorState *out, uint32_t now_ms)
{
    MotorHealthResult health;

    if (out == NULL)
    {
        return 0u;
    }
    (void)memset(out, 0, sizeof(*out));
    if (MotorHealthRead(id, now_ms, WHEELLEG_FEEDBACK_TIMEOUT_MS, &health) == 0u ||
        health.feedbackValid == 0u)
    {
        return 0u;
    }

    *out = health.feedback;
    out->online = health.healthy;
    return health.healthy;
}

static void WheelLegConfigureActuators(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    uint8_t valid = 1u;
    uint8_t i;

    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].front = WheelLegActuatorFromU8(cfg->right_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].back = WheelLegActuatorFromU8(cfg->right_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].wheel = WheelLegActuatorFromU8(cfg->right_wheel_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].front = WheelLegActuatorFromU8(cfg->left_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].back = WheelLegActuatorFromU8(cfg->left_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].wheel = WheelLegActuatorFromU8(cfg->left_wheel_actuator);

    s_wheelleg.domain_member[WheelLegMemberLeftFront].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].front;
    s_wheelleg.domain_member[WheelLegMemberLeftFront].offline_fault = WHEELLEG_FAULT_LEFT_LEG_OFFLINE;
    s_wheelleg.domain_member[WheelLegMemberLeftBack].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].back;
    s_wheelleg.domain_member[WheelLegMemberLeftBack].offline_fault = WHEELLEG_FAULT_LEFT_LEG_OFFLINE;
    s_wheelleg.domain_member[WheelLegMemberLeftWheel].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].wheel;
    s_wheelleg.domain_member[WheelLegMemberLeftWheel].offline_fault = WHEELLEG_FAULT_LEFT_WHEEL_OFFLINE;
    s_wheelleg.domain_member[WheelLegMemberRightFront].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].front;
    s_wheelleg.domain_member[WheelLegMemberRightFront].offline_fault = WHEELLEG_FAULT_RIGHT_LEG_OFFLINE;
    s_wheelleg.domain_member[WheelLegMemberRightBack].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].back;
    s_wheelleg.domain_member[WheelLegMemberRightBack].offline_fault = WHEELLEG_FAULT_RIGHT_LEG_OFFLINE;
    s_wheelleg.domain_member[WheelLegMemberRightWheel].actuator =
        s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].wheel;
    s_wheelleg.domain_member[WheelLegMemberRightWheel].offline_fault = WHEELLEG_FAULT_RIGHT_WHEEL_OFFLINE;

    for (i = 0u; i < (uint8_t)WheelLegMemberCount; i++)
    {
        const MotorId actuator = s_wheelleg.domain_member[i].actuator;

        if ((uint32_t)actuator >= (uint32_t)MotorCount ||
            MotorInstFindByMotor(actuator) == NULL)
        {
            valid = 0u;
            continue;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (actuator == s_wheelleg.domain_member[j].actuator)
            {
                valid = 0u;
                break;
            }
        }
    }
    s_wheelleg.domain_binding_valid = valid;
}

static uint8_t WheelLegDomainMemberResolve(MotorId actuator, uint8_t *member_out)
{
    uint8_t match_count = 0u;
    uint8_t matched_member = (uint8_t)WheelLegMemberCount;

    if (member_out != NULL)
    {
        *member_out = (uint8_t)WheelLegMemberCount;
    }
    if ((uint32_t)actuator >= (uint32_t)MotorCount ||
        MotorInstFindByMotor(actuator) == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < (uint8_t)WheelLegMemberCount; i++)
    {
        if (s_wheelleg.domain_member[i].actuator == actuator)
        {
            matched_member = i;
            match_count++;
        }
    }
    if (match_count != 1u)
    {
        return 0u;
    }

    if (member_out != NULL)
    {
        *member_out = matched_member;
    }
    return 1u;
}

static uint16_t WheelLegUpdateFeedback(uint32_t now_ms)
{
    uint16_t faults = WHEELLEG_FAULT_NONE;
    uint8_t online_mask = 0u;
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const WheelLegActuatorMap *map = &s_wheelleg.actuator[side];
        const uint8_t front_ok = WheelLegReadFeedback(map->front, &s_wheelleg.front_fb[side], now_ms);
        const uint8_t back_ok = WheelLegReadFeedback(map->back, &s_wheelleg.back_fb[side], now_ms);
        const uint8_t wheel_ok = WheelLegReadFeedback(map->wheel, &s_wheelleg.wheel_fb[side], now_ms);

        if (side == WHEELLEG_SIDE_LEFT)
        {
            if (front_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberLeftFront);
            }
            if (back_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberLeftBack);
            }
            if (wheel_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberLeftWheel);
            }
            if (front_ok == 0u || back_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_LEFT_LEG_OFFLINE;
            }
            if (wheel_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_LEFT_WHEEL_OFFLINE;
            }
        }
        else
        {
            if (front_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberRightFront);
            }
            if (back_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberRightBack);
            }
            if (wheel_ok != 0u)
            {
                online_mask |= (uint8_t)(1u << WheelLegMemberRightWheel);
            }
            if (front_ok == 0u || back_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_RIGHT_LEG_OFFLINE;
            }
            if (wheel_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_RIGHT_WHEEL_OFFLINE;
            }
        }
    }
    s_wheelleg.domain_online_mask = online_mask;
    if (s_wheelleg.domain_binding_valid == 0u)
    {
        faults |= WHEELLEG_FAULT_CONTROLLER;
    }
    return faults;
}

static uint8_t WheelLegCalcKinematics(WheelLegLegCalc *leg,
                                        fp32 front_pos,
                                        fp32 back_pos,
                                        fp32 pitch,
                                        fp32 gyro_y,
                                        uint8_t left_side,
                                        fp32 dt)
{
    const WheelLegCoreGeometry geo = WheelLegCoreGeometryFromConfig();

    return WheelLegCoreCalcKinematics(&geo, leg, front_pos, back_pos, pitch, gyro_y, left_side, dt);
}

static uint8_t WheelLegCalcVmc(WheelLegLegCalc *leg)
{
    return WheelLegCoreCalcVmc(leg);
}

static uint8_t WheelLegSendStateCmd(MotorId id,
                                       fp32 position,
                                       fp32 velocity,
                                       fp32 kp,
                                       fp32 kd,
                                       fp32 torque)
{
    MotorCmd cmd;

    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return 0u;
    }

    control_core_cmd_set_state_torque(&cmd, position, velocity, kp, kd, torque);
    if (MotorInstSetStateTorqueId(id, &cmd) == 0u)
    {
        return 0u;
    }
    s_wheelleg.domain_outputs_active = 1u;
    return 1u;
}

static void WheelLegClearStateCmd(MotorId id)
{
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return;
    }

    (void)MotorInstClearId(id);
}

static void WheelLegClearLegVirtualOutputs(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.leg[side].f0 = 0.0f;
        s_wheelleg.leg[side].tp = 0.0f;
        s_wheelleg.leg[side].joint_torque[0] = 0.0f;
        s_wheelleg.leg[side].joint_torque[1] = 0.0f;
        s_wheelleg.leg[side].contact = 0u;
    }
}

static uint8_t WheelLegDomainIdsCollect(MotorId ids[WHEELLEG_DOMAIN_MEMBER_COUNT],
                                        uint8_t *count_out)
{
    uint8_t count = 0u;
    uint8_t all_members_bound = 1u;
    uint8_t i;

    if (ids == NULL || count_out == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < (uint8_t)WheelLegMemberCount; i++)
    {
        const MotorId actuator = s_wheelleg.domain_member[i].actuator;
        uint8_t duplicate = 0u;

        if ((uint32_t)actuator >= (uint32_t)MotorCount ||
            MotorInstFindByMotor(actuator) == NULL)
        {
            all_members_bound = 0u;
            continue;
        }
        for (uint8_t j = 0u; j < count; j++)
        {
            if (actuator == ids[j])
            {
                duplicate = 1u;
                all_members_bound = 0u;
                break;
            }
        }
        if (duplicate == 0u)
        {
            ids[count] = actuator;
            count++;
        }
    }

    *count_out = count;
    return (uint8_t)(all_members_bound != 0u && count == (uint8_t)WheelLegMemberCount);
}

static uint8_t WheelLegStopDomainOutputsEx(uint8_t force)
{
    MotorId ids[WHEELLEG_DOMAIN_MEMBER_COUNT];
    uint8_t count = 0u;
    uint8_t all_members_bound;

    if (s_wheelleg.domain_outputs_active == 0u && force == 0u)
    {
        return 1u;
    }

    all_members_bound = WheelLegDomainIdsCollect(ids, &count);
    if (count == 0u || MotorInstClearIds(ids, count) == 0u)
    {
        s_wheelleg.domain_stop_fail_count++;
        return 0u;
    }

    s_wheelleg.domain_outputs_active = 0u;
    s_wheelleg.domain_stop_count++;
    s_wheelleg.domain_last_stop_tick_ms = WheelLegTickMs();
    if (all_members_bound == 0u || count != (uint8_t)WheelLegMemberCount)
    {
        s_wheelleg.domain_stop_fail_count++;
        return 0u;
    }
    return 1u;
}

static uint8_t WheelLegStopDomainOutputs(void)
{
    return WheelLegStopDomainOutputsEx(0u);
}

static uint8_t WheelLegForceStopDomainOutputs(void)
{
    return WheelLegStopDomainOutputsEx(1u);
}

static uint8_t WheelLegDomainInhibitAcquire(void)
{
    MotorId ids[WHEELLEG_DOMAIN_MEMBER_COUNT];
    uint8_t count = 0u;

    if (WheelLegDomainIdsCollect(ids, &count) == 0u ||
        MotorInstInhibitIds(ids, count) == 0u)
    {
        s_wheelleg.domain_inhibit_complete = 0u;
        s_wheelleg.domain_inhibit_fail_count++;
        return 0u;
    }

    s_wheelleg.domain_inhibit_complete = 1u;
    return 1u;
}

static uint8_t WheelLegDomainInhibitHold(void)
{
    const uint8_t inhibit_ok = WheelLegDomainInhibitAcquire();

    if (inhibit_ok == 0u)
    {
        /* 配置不完整或更高优先级占用时，仍尽力重发原有协议停机。 */
        (void)WheelLegForceStopDomainOutputs();
        return 0u;
    }
    if (s_wheelleg.domain_stop_clear_complete == 0u)
    {
        if (WheelLegForceStopDomainOutputs() == 0u)
        {
            return 0u;
        }
        s_wheelleg.domain_stop_clear_complete = 1u;
    }
    return 1u;
}

static uint8_t WheelLegDomainInhibitRelease(void)
{
    MotorId ids[WHEELLEG_DOMAIN_MEMBER_COUNT];
    uint8_t count = 0u;

    if (WheelLegDomainInhibitAcquire() == 0u)
    {
        (void)WheelLegForceStopDomainOutputs();
        return 0u;
    }
    if (WheelLegForceStopDomainOutputs() == 0u)
    {
        s_wheelleg.domain_inhibit_release_fail_count++;
        return 0u;
    }
    s_wheelleg.domain_stop_clear_complete = 1u;

    /* 急停仍有效时，即使 SAFETY 级清零成功也继续持有六轴禁写。 */
    if (LowCmdEmergencyActive() != 0u)
    {
        s_wheelleg.domain_inhibit_release_fail_count++;
        return 0u;
    }
    if (WheelLegDomainIdsCollect(ids, &count) == 0u ||
        MotorInstReleaseInhibitIds(ids, count) == 0u)
    {
        s_wheelleg.domain_inhibit_release_fail_count++;
        return 0u;
    }

    s_wheelleg.domain_inhibit_complete = 0u;
    s_wheelleg.domain_stop_clear_complete = 0u;
    return 1u;
}
