/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static uint8_t WheelLegReadFeedback(MotorId id, MotorState *out, uint32_t now_ms)
{
    if ((uint32_t)id >= (uint32_t)MotorCount || out == NULL)
    {
        return 0u;
    }
    if (LowStateGetMotor(id, out) == 0u)
    {
        return 0u;
    }
    if (WheelLegFeedbackFresh(out, now_ms) == 0u)
    {
        out->online = 0u;
        return 0u;
    }
    return 1u;
}

static void WheelLegConfigureActuators(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;

    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].front = WheelLegActuatorFromU8(cfg->right_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].back = WheelLegActuatorFromU8(cfg->right_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].wheel = WheelLegActuatorFromU8(cfg->right_wheel_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].front = WheelLegActuatorFromU8(cfg->left_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].back = WheelLegActuatorFromU8(cfg->left_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].wheel = WheelLegActuatorFromU8(cfg->left_wheel_actuator);
}

static uint16_t WheelLegUpdateFeedback(uint32_t now_ms)
{
    uint16_t faults = WHEELLEG_FAULT_NONE;
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const WheelLegActuatorMap *map = &s_wheelleg.actuator[side];
        const uint8_t front_ok = WheelLegReadFeedback(map->front, &s_wheelleg.front_fb[side], now_ms);
        const uint8_t back_ok = WheelLegReadFeedback(map->back, &s_wheelleg.back_fb[side], now_ms);
        const uint8_t wheel_ok = WheelLegReadFeedback(map->wheel, &s_wheelleg.wheel_fb[side], now_ms);

        if (side == WHEELLEG_SIDE_LEFT)
        {
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
    return MotorInstSetStateTorqueId(id, &cmd);
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

static void WheelLegClearAllControlCmds(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const WheelLegActuatorMap *map = &s_wheelleg.actuator[side];
        WheelLegClearStateCmd(map->front);
        WheelLegClearStateCmd(map->back);
        WheelLegClearStateCmd(map->wheel);
    }
    WheelLegClearStateCmd(WheelLegSingleTestActuator());
}
