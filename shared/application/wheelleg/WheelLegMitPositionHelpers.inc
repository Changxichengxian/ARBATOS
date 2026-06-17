/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static fp32 WheelLegManualYTargetFromAxis(int16_t axis)
{
    fp32 center = g_config.WheelLegMit.default_leg_length_m;
    fp32 min_leg = g_config.WheelLegMit.min_leg_length_m;
    fp32 max_leg = g_config.WheelLegMit.max_leg_length_m;
    fp32 u;

    if (min_leg <= 0.02f)
    {
        min_leg = 0.02f;
    }
    if (max_leg < min_leg)
    {
        max_leg = min_leg;
    }
    center = WheelLegClamp(center, min_leg, max_leg);

    if (WheelLegAxisInDeadband(axis, g_config.WheelLegMit.rc_deadband) != 0u)
    {
        return center;
    }

    u = WheelLegClamp(((fp32)axis) / (fp32)RC_CH_VALUE_ABS_LEGACY, -1.0f, 1.0f);
    if (u >= 0.0f)
    {
        return center + u * (max_leg - center);
    }
    return center + u * (center - min_leg);
}

static fp32 WheelLegManualXTargetFromAxis(int16_t axis)
{
    return WheelLegAxisToFp32(axis,
                                WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                g_config.WheelLegMit.rc_deadband);
}

static uint8_t WheelLegPositionApplyLeg(const WheelLegActuatorMap *map,
                                           uint8_t side,
                                           fp32 front_raw_zero,
                                           fp32 back_raw_zero,
                                           int8_t front_dir,
                                           int8_t back_dir,
                                           fp32 target_foot_x,
                                           fp32 target_foot_y,
                                           fp32 kp,
                                           fp32 kd)
{
    WheelLegFootPoint target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_target;
    fp32 back_target;

    if (map == NULL)
    {
        return 0u;
    }
    if (WheelLegKinematicZero(&front_kin_zero, &back_kin_zero) == 0u ||
        WheelLegFootPointFromXy(target_foot_x, target_foot_y, &target) == 0u ||
        WheelLegInversePoint(&target, front_kin_zero, back_kin_zero, &front_target, &back_target) == 0u)
    {
        return 0u;
    }

    if (side < WHEELLEG_SIDE_COUNT)
    {
        s_wheelleg.foot_test_target[side] = target;
    }

    if (WheelLegSendStateCmd(map->front,
                                WheelLegKinematicToRaw(front_target,
                                                          front_raw_zero,
                                                          front_dir,
                                                          front_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                0.0f) == 0u ||
        WheelLegSendStateCmd(map->back,
                                WheelLegKinematicToRaw(back_target,
                                                          back_raw_zero,
                                                          back_dir,
                                                          back_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                0.0f) == 0u)
    {
        return 0u;
    }

    WheelLegClearStateCmd(map->wheel);
    return 1u;
}

static uint8_t WheelLegManualPositionApply(fp32 target_foot_x, fp32 target_foot_y)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const WheelLegActuatorMap *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const WheelLegActuatorMap *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t ok_left;
    uint8_t ok_right;

    s_wheelleg.foot_test_ik_ok = 0u;

    ok_left = WheelLegPositionApplyLeg(left_map,
                                          WHEELLEG_SIDE_LEFT,
                                          cfg->left_front_zero_rad,
                                          cfg->left_back_zero_rad,
                                          cfg->left_front_dir,
                                          cfg->left_back_dir,
                                          target_foot_x,
                                          target_foot_y,
                                          kp,
                                          kd);
    ok_right = WheelLegPositionApplyLeg(right_map,
                                           WHEELLEG_SIDE_RIGHT,
                                           cfg->right_front_zero_rad,
                                           cfg->right_back_zero_rad,
                                           cfg->right_front_dir,
                                           cfg->right_back_dir,
                                           target_foot_x,
                                           target_foot_y,
                                           kp,
                                           kd);
    if (ok_left == 0u || ok_right == 0u)
    {
        WheelLegClearJointTestCmds();
        return 0u;
    }

    s_wheelleg.foot_test_ik_ok = 1u;
    return 1u;
}

static fp32 WheelLegLeftLegTestTarget(uint32_t elapsed_ms)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const uint32_t zero_ms = cfg->left_test_zero_time_ms;
    uint32_t move_ms = cfg->left_test_move_time_ms;
    uint32_t cycle_ms;
    uint32_t t;
    const fp32 angle = cfg->left_test_angle_rad;

    if (move_ms == 0u)
    {
        move_ms = 1000u;
    }
    cycle_ms = zero_ms + 4u * move_ms;
    if (cycle_ms == 0u)
    {
        return 0.0f;
    }

    t = elapsed_ms % cycle_ms;
    if (t < zero_ms)
    {
        return 0.0f;
    }
    t -= zero_ms;
    if (t < move_ms)
    {
        return WheelLegLerp(0.0f, angle, t, move_ms);
    }
    t -= move_ms;
    if (t < move_ms)
    {
        return WheelLegLerp(angle, 0.0f, t, move_ms);
    }
    t -= move_ms;
    if (t < move_ms)
    {
        return WheelLegLerp(0.0f, -angle, t, move_ms);
    }
    t -= move_ms;
    return WheelLegLerp(-angle, 0.0f, t, move_ms);
}

static uint8_t WheelLegLeftLegTestApply(uint32_t now_ms)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const WheelLegActuatorMap *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const WheelLegActuatorMap *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    fp32 torque_limit;
    fp32 torque;
    fp32 target;
    uint8_t sent = 0u;

    if (s_wheelleg.left_test_active == 0u)
    {
        s_wheelleg.left_test_start_ms = now_ms;
        s_wheelleg.left_test_active = 1u;
    }

    WheelLegClearJointTestCmds();

    torque_limit = WheelLegAbs(cfg->left_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? WheelLegClamp(cfg->left_test_torque_ff_nm, -torque_limit, torque_limit)
                 : 0.0f;
    target = WheelLegLeftLegTestTarget(now_ms - s_wheelleg.left_test_start_ms);

    if (cfg->left_test_front_dir != 0)
    {
        if (WheelLegSendStateCmd(left_map->front,
                                    WheelLegJointToRaw(target,
                                                          cfg->left_front_zero_rad,
                                                          cfg->left_test_front_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->left_test_back_dir != 0)
    {
        if (WheelLegSendStateCmd(left_map->back,
                                    WheelLegJointToRaw(target,
                                                          cfg->left_back_zero_rad,
                                                          cfg->left_test_back_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->right_test_front_dir != 0)
    {
        if (WheelLegSendStateCmd(right_map->front,
                                    WheelLegJointToRaw(target,
                                                          cfg->right_front_zero_rad,
                                                          cfg->right_test_front_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->right_test_back_dir != 0)
    {
        if (WheelLegSendStateCmd(right_map->back,
                                    WheelLegJointToRaw(target,
                                                          cfg->right_back_zero_rad,
                                                          cfg->right_test_back_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }

    return sent;
}
