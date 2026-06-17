/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static void WheelLegClearTestRuntimeFlags(void)
{
    s_wheelleg.left_test_active = 0u;
    s_wheelleg.foot_test_active = 0u;
    s_wheelleg.detached_test_active = 0u;
    s_wheelleg.foot_test_phase = 0u;
    s_wheelleg.foot_test_ik_ok = 0u;
}

static void WheelLegBalanceIdleReset(int16_t vx_axis, int16_t yaw_axis, fp32 yaw)
{
    const uint8_t idle =
        (uint8_t)((WheelLegAxisInDeadband(vx_axis, g_config.WheelLegMit.rc_deadband) != 0u &&
                   WheelLegAxisInDeadband(yaw_axis, g_config.WheelLegMit.rc_deadband) != 0u)
                      ? 1u
                      : 0u);

    if (idle == 0u)
    {
        s_wheelleg.balance_idle_latched = 0u;
        return;
    }

    if (s_wheelleg.balance_idle_latched != 0u)
    {
        return;
    }

    s_wheelleg.x_m = 0.0f;
    s_wheelleg.yaw_set = yaw;
    s_wheelleg.yaw_inited = 1u;
    s_wheelleg.balance_idle_latched = 1u;
}

static MotorId WheelLegCoreActuatorToId(WheelLegCoreActuator actuator)
{
    const WheelLegActuatorMap *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    const WheelLegActuatorMap *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];

    switch (actuator)
    {
    case WHEELLEG_CORE_ACT_RIGHT_FRONT:
        return right_map->front;
    case WHEELLEG_CORE_ACT_RIGHT_BACK:
        return right_map->back;
    case WHEELLEG_CORE_ACT_RIGHT_WHEEL:
        return right_map->wheel;
    case WHEELLEG_CORE_ACT_LEFT_FRONT:
        return left_map->front;
    case WHEELLEG_CORE_ACT_LEFT_BACK:
        return left_map->back;
    case WHEELLEG_CORE_ACT_LEFT_WHEEL:
        return left_map->wheel;
    default:
        return MotorCount;
    }
}

static uint8_t WheelLegSendCoreOutput(const WheelLegCoreOutput *out)
{
    MotorId ids[WHEELLEG_CORE_ACTUATOR_COUNT];
    MotorCmd cmds[WHEELLEG_CORE_ACTUATOR_COUNT];
    uint8_t written = 0u;
    uint8_t i;

    if (out == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < out->actuator_count && i < WHEELLEG_CORE_ACTUATOR_COUNT; i++)
    {
        MotorId id;

        if (out->actuator[i].active == 0u)
        {
            continue;
        }

        id = WheelLegCoreActuatorToId((WheelLegCoreActuator)i);
        if ((uint32_t)id >= (uint32_t)MotorCount)
        {
            return 0u;
        }

        ids[written] = id;
        cmds[written] = out->actuator[i];
        written++;
    }

    if (written == 0u)
    {
        return 0u;
    }

    return MotorInstSetStateTorqueIds(ids, cmds, written);
}

static void WheelLegSendWheelTorques(const fp32 wheel_torque[WHEELLEG_SIDE_COUNT])
{
    WheelLegCoreOutput out;

    if (wheel_torque == NULL)
    {
        return;
    }

    WheelLegCoreOutputClear(&out);
    WheelLegCoreSetWheelTorques(&out, wheel_torque);
    (void)WheelLegSendCoreOutput(&out);
}

static void WheelLegSendVmcJointTorques(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    WheelLegCoreOutput out;

    WheelLegCoreOutputClear(&out);
    WheelLegCoreSetVmcJointTorques(&out,
                                        s_wheelleg.leg,
                                        cfg->right_front_dir,
                                        cfg->right_back_dir,
                                        cfg->left_front_dir,
                                        cfg->left_back_dir);
    (void)WheelLegSendCoreOutput(&out);
}

static void WheelLegClearJointTestCmds(void)
{
    const WheelLegActuatorMap *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const WheelLegActuatorMap *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];

    WheelLegClearStateCmd(left_map->front);
    WheelLegClearStateCmd(left_map->back);
    WheelLegClearStateCmd(left_map->wheel);
    WheelLegClearStateCmd(right_map->front);
    WheelLegClearStateCmd(right_map->back);
    WheelLegClearStateCmd(right_map->wheel);
    WheelLegClearStateCmd(WheelLegSingleTestActuator());
}

static void WheelLegBenchHoldPrepareDefault(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;

    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_LEFT] = cfg->left_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_LEFT] = cfg->left_back_zero_rad;
    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_back_zero_rad;

    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_RIGHT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_RIGHT] = 1u;

    s_wheelleg.bench_hold_target_leg_m = WheelLegBenchDefaultLegM();
    s_wheelleg.bench_hold_target_foot_x_m = WHEELLEG_BENCH_TARGET_FOOT_X_M;
    s_wheelleg.bench_hold_pose_valid = 1u;
}

static uint8_t WheelLegBenchHoldApplyJoints(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t side;
    uint8_t sent = 0u;

    WheelLegBenchHoldPrepareDefault();

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const WheelLegActuatorMap *map = &s_wheelleg.actuator[side];

        if (s_wheelleg.bench_hold_front_valid[side] != 0u)
        {
            sent += WheelLegSendStateCmd(map->front,
                                            s_wheelleg.bench_hold_front_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
        if (s_wheelleg.bench_hold_back_valid[side] != 0u)
        {
            sent += WheelLegSendStateCmd(map->back,
                                            s_wheelleg.bench_hold_back_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
    }

    return (sent > 0u) ? 1u : 0u;
}

static void WheelLegBenchHoldUpdatePoseTarget(void)
{
    WheelLegBenchHoldPrepareDefault();
}

static MotorId WheelLegSingleTestActuator(void)
{
    const MotorId target = robot_mode_target_motor();
    return (target != MotorCount) ? target : (MotorId)g_config.WheelLegMit.single_test_actuator;
}

static uint8_t WheelLegSingleTestTargetIsWheelLeg(MotorId id)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;

    return (uint8_t)((id == (MotorId)cfg->left_front_actuator) ||
                     (id == (MotorId)cfg->left_back_actuator) ||
                     (id == (MotorId)cfg->left_wheel_actuator) ||
                     (id == (MotorId)cfg->right_front_actuator) ||
                     (id == (MotorId)cfg->right_back_actuator) ||
                     (id == (MotorId)cfg->right_wheel_actuator));
}

static uint8_t WheelLegSingleTestEnabled(void)
{
    if (robot_mode_variant() == ROBOT_RUN_VARIANT_WHEELLEG_SINGLE_MOTOR)
    {
        return 1u;
    }

    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_SINGLE_MOTOR &&
                     WheelLegSingleTestTargetIsWheelLeg(robot_mode_target_motor()) != 0u);
}

static uint8_t WheelLegSingleTestApply(int16_t speed_axis)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const MotorId id = WheelLegSingleTestActuator();
    const fp32 velocity = WheelLegAxisToFp32(speed_axis,
                                               cfg->single_test_velocity_radps,
                                               cfg->rc_deadband);
    fp32 torque_limit;
    fp32 torque;

    WheelLegClearJointTestCmds();

    torque_limit = WheelLegAbs(cfg->single_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? WheelLegClamp(cfg->single_test_torque_nm, -torque_limit, torque_limit)
                 : 0.0f;
    return WheelLegSendStateCmd(id,
                                   cfg->single_test_position_rad,
                                   velocity,
                                   cfg->single_test_kp,
                                   cfg->single_test_kd,
                                   torque);
}

static fp32 WheelLegLerp(fp32 start, fp32 end, uint32_t elapsed_ms, uint32_t duration_ms)
{
    return WheelLegCoreLerp(start, end, elapsed_ms, duration_ms);
}

static void WheelLegFootPointLerp(const WheelLegFootPoint *start,
                                     const WheelLegFootPoint *end,
                                     uint32_t elapsed_ms,
                                     uint32_t duration_ms,
                                     WheelLegFootPoint *out)
{
    if (start == NULL || end == NULL || out == NULL)
    {
        return;
    }

    out->x_m = WheelLegLerp(start->x_m, end->x_m, elapsed_ms, duration_ms);
    out->y_m = WheelLegLerp(start->y_m, end->y_m, elapsed_ms, duration_ms);
    out->length_m = sqrtf(out->x_m * out->x_m + out->y_m * out->y_m);
}

static uint8_t WheelLegFootPointFromLengthX(fp32 length_m,
                                                 fp32 x_m,
                                                 WheelLegFootPoint *out)
{
    fp32 max_x;
    fp32 y_sq;

    if (out == NULL || length_m <= 0.02f)
    {
        return 0u;
    }

    max_x = length_m * 0.98f;
    x_m = WheelLegClamp(x_m, -max_x, max_x);
    y_sq = length_m * length_m - x_m * x_m;
    if (y_sq < 0.0f)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = sqrtf(y_sq);
    out->length_m = length_m;
    return 1u;
}

static uint8_t WheelLegFootPointFromXy(fp32 x_m,
                                           fp32 y_m,
                                           WheelLegFootPoint *out)
{
    fp32 length_m = 0.0f;

    if (out == NULL)
    {
        return 0u;
    }

    if (WheelLegLimitFootXy(&x_m, &y_m, &length_m) == 0u)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = y_m;
    out->length_m = length_m;
    return 1u;
}

static uint8_t WheelLegInversePoint(const WheelLegFootPoint *target,
                                      fp32 front_ref,
                                      fp32 back_ref,
                                      fp32 *front_out,
                                      fp32 *back_out)
{
    const WheelLegCoreGeometry geo = WheelLegCoreGeometryFromConfig();
    WheelLegCoreFootPoint core_target;

    if (target == NULL || front_out == NULL || back_out == NULL)
    {
        return 0u;
    }

    WheelLegCorePointFromLocal(target, &core_target);
    return WheelLegCoreInversePoint(&geo, &core_target, front_ref, back_ref, front_out, back_out);
}

static uint8_t WheelLegKinematicZero(fp32 *front_zero, fp32 *back_zero)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    WheelLegFootPoint zero;
    fp32 length = cfg->min_leg_length_m;

    if (front_zero == NULL || back_zero == NULL)
    {
        return 0u;
    }
    if (length <= 0.02f)
    {
        length = cfg->default_leg_length_m;
    }
    if (cfg->max_leg_length_m > 0.02f)
    {
        length = WheelLegClamp(length, 0.02f, cfg->max_leg_length_m);
    }

    zero.x_m = 0.0f;
    zero.y_m = length;
    zero.length_m = length;

    return WheelLegInversePoint(&zero,
                                  -WHEELLEG_PI,
                                  -WHEELLEG_PI,
                                  front_zero,
                                  back_zero);
}

static uint8_t WheelLegFootTestTarget(const WheelLegFootPoint *zero,
                                         uint32_t elapsed_ms,
                                         WheelLegFootPoint *target,
                                         uint8_t *phase)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const fp32 length_m = WheelLegClamp((cfg->foot_test_length_m > 0.02f)
                                             ? cfg->foot_test_length_m
                                             : cfg->default_leg_length_m,
                                         cfg->min_leg_length_m,
                                         cfg->max_leg_length_m);
    const fp32 forward_m = WheelLegAbs(cfg->foot_test_forward_m) *
                           ((cfg->foot_test_forward_dir < 0) ? -1.0f : 1.0f);
    uint32_t zero_ms = cfg->foot_test_zero_hold_time_ms;
    uint32_t extend_ms = cfg->foot_test_extend_time_ms;
    uint32_t swing_ms = cfg->foot_test_swing_time_ms;
    uint32_t return_ms = cfg->foot_test_return_time_ms;
    uint32_t cycle_ms;
    uint32_t t;
    WheelLegFootPoint center;
    WheelLegFootPoint forward;
    WheelLegFootPoint backward;

    if (zero == NULL || target == NULL || phase == NULL)
    {
        return 0u;
    }
    if (extend_ms == 0u)
    {
        extend_ms = 1000u;
    }
    if (swing_ms == 0u)
    {
        swing_ms = 1000u;
    }
    if (return_ms == 0u)
    {
        return_ms = 1000u;
    }

    if (WheelLegFootPointFromLengthX(length_m, 0.0f, &center) == 0u ||
        WheelLegFootPointFromLengthX(length_m, forward_m, &forward) == 0u ||
        WheelLegFootPointFromLengthX(length_m, -forward_m, &backward) == 0u)
    {
        return 0u;
    }

    cycle_ms = zero_ms + extend_ms + 3u * swing_ms + return_ms;
    if (cycle_ms == 0u)
    {
        return 0u;
    }

    t = elapsed_ms % cycle_ms;
    if (t < zero_ms)
    {
        *target = *zero;
        *phase = 0u;
        return 1u;
    }
    t -= zero_ms;
    if (t < extend_ms)
    {
        WheelLegFootPointLerp(zero, &center, t, extend_ms, target);
        *phase = 1u;
        return 1u;
    }
    t -= extend_ms;
    if (t < swing_ms)
    {
        WheelLegFootPointLerp(&center, &forward, t, swing_ms, target);
        *phase = 2u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        WheelLegFootPointLerp(&forward, &backward, t, swing_ms, target);
        *phase = 3u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        WheelLegFootPointLerp(&backward, &center, t, swing_ms, target);
        *phase = 4u;
        return 1u;
    }
    t -= swing_ms;
    WheelLegFootPointLerp(&center, zero, t, return_ms, target);
    *phase = 5u;
    return 1u;
}

static void WheelLegFootTestResetWheelZero(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.foot_test_wheel_zero_rad[side] = 0.0f;
        s_wheelleg.foot_test_wheel_zero_valid[side] = 0u;
        s_wheelleg.foot_test_wheel_dx_m[side] = 0.0f;
        s_wheelleg.foot_test_wheel_comp_rad[side] = 0.0f;
        s_wheelleg.foot_test_wheel_target_rad[side] = 0.0f;
    }
}

static fp32 WheelLegFootTestGroundX(const WheelLegFootPoint *point,
                                        uint8_t side,
                                        fp32 pitch)
{
    const fp32 pitch_side = (side == WHEELLEG_SIDE_LEFT) ? -pitch : pitch;

    if (point == NULL)
    {
        return 0.0f;
    }
    return point->x_m * cosf(pitch_side) - point->y_m * sinf(pitch_side);
}

static uint8_t WheelLegFootTestApplyWheel(const WheelLegActuatorMap *map,
                                              uint8_t side,
                                              const WheelLegFootPoint *zero,
                                              const WheelLegFootPoint *target,
                                              fp32 pitch,
                                              fp32 kp,
                                              fp32 kd)
{
    fp32 radius;
    fp32 dx;
    fp32 wheel_target;

    if (map == NULL || zero == NULL || target == NULL || side >= WHEELLEG_SIDE_COUNT)
    {
        return 0u;
    }

    radius = g_config.WheelLegMit.wheel_radius_m;
    if (radius <= 0.001f)
    {
        return 0u;
    }

    dx = WheelLegFootTestGroundX(target, side, pitch) -
         WheelLegFootTestGroundX(zero, side, pitch);
    s_wheelleg.foot_test_wheel_dx_m[side] = dx;
    s_wheelleg.foot_test_wheel_comp_rad[side] = dx / radius;

    if ((uint32_t)map->wheel >= (uint32_t)MotorCount)
    {
        s_wheelleg.foot_test_wheel_target_rad[side] = s_wheelleg.foot_test_wheel_comp_rad[side];
        return 1u;
    }

    if (s_wheelleg.foot_test_wheel_zero_valid[side] == 0u)
    {
        s_wheelleg.foot_test_wheel_zero_rad[side] =
            (s_wheelleg.wheel_fb[side].online != 0u) ? s_wheelleg.wheel_fb[side].q : 0.0f;
        s_wheelleg.foot_test_wheel_zero_valid[side] = 1u;
    }

    wheel_target = s_wheelleg.foot_test_wheel_zero_rad[side] +
                   s_wheelleg.foot_test_wheel_comp_rad[side];
    s_wheelleg.foot_test_wheel_target_rad[side] = wheel_target;
    return WheelLegSendStateCmd(map->wheel, wheel_target, 0.0f, kp, kd, 0.0f);
}

static uint8_t WheelLegFootTestApplyLeg(const WheelLegActuatorMap *map,
                                            uint8_t side,
                                            fp32 front_raw_zero,
                                            fp32 back_raw_zero,
                                            int8_t front_dir,
                                            int8_t back_dir,
                                            uint32_t elapsed_ms,
                                            fp32 pitch,
                                            fp32 kp,
                                            fp32 kd,
                                            fp32 torque)
{
    WheelLegFootPoint zero;
    WheelLegFootPoint target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_target;
    fp32 back_target;
    uint8_t phase = 0u;

    if (map == NULL)
    {
        return 0u;
    }
    if (WheelLegKinematicZero(&front_kin_zero, &back_kin_zero) == 0u)
    {
        return 0u;
    }

    zero.x_m = 0.0f;
    zero.y_m = g_config.WheelLegMit.min_leg_length_m;
    zero.length_m = zero.y_m;
    if (zero.length_m <= 0.02f)
    {
        zero.y_m = g_config.WheelLegMit.default_leg_length_m;
        zero.length_m = zero.y_m;
    }

    if (WheelLegFootTestTarget(&zero, elapsed_ms, &target, &phase) == 0u ||
        WheelLegInversePoint(&target, front_kin_zero, back_kin_zero, &front_target, &back_target) == 0u)
    {
        return 0u;
    }

    if (side < WHEELLEG_SIDE_COUNT)
    {
        s_wheelleg.foot_test_target[side] = target;
    }
    s_wheelleg.foot_test_phase = phase;
    if (WheelLegSendStateCmd(map->front,
                                WheelLegKinematicToRaw(front_target,
                                                          front_raw_zero,
                                                          front_dir,
                                                          front_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                torque) == 0u ||
        WheelLegSendStateCmd(map->back,
                                WheelLegKinematicToRaw(back_target,
                                                          back_raw_zero,
                                                          back_dir,
                                                          back_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                torque) == 0u)
    {
        return 0u;
    }
    return WheelLegFootTestApplyWheel(map, side, &zero, &target, pitch, kp, kd);
}

static uint8_t WheelLegFootTestApply(uint32_t now_ms, fp32 pitch)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const WheelLegActuatorMap *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const WheelLegActuatorMap *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    fp32 torque_limit;
    fp32 torque;
    uint32_t elapsed_ms;
    uint8_t ok_left;
    uint8_t ok_right;

    if (s_wheelleg.foot_test_active == 0u)
    {
        s_wheelleg.foot_test_start_ms = now_ms;
        s_wheelleg.foot_test_active = 1u;
        WheelLegFootTestResetWheelZero();
    }

    WheelLegClearJointTestCmds();

    torque_limit = WheelLegAbs(cfg->foot_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? WheelLegClamp(cfg->foot_test_torque_ff_nm, -torque_limit, torque_limit)
                 : 0.0f;
    elapsed_ms = now_ms - s_wheelleg.foot_test_start_ms;
    s_wheelleg.foot_test_ik_ok = 0u;

    ok_left = WheelLegFootTestApplyLeg(left_map,
                                           WHEELLEG_SIDE_LEFT,
                                           cfg->left_front_zero_rad,
                                           cfg->left_back_zero_rad,
                                           cfg->left_front_dir,
                                           cfg->left_back_dir,
                                           elapsed_ms,
                                           pitch,
                                           cfg->foot_test_kp,
                                           cfg->foot_test_kd,
                                           torque);
    ok_right = WheelLegFootTestApplyLeg(right_map,
                                            WHEELLEG_SIDE_RIGHT,
                                            cfg->right_front_zero_rad,
                                            cfg->right_back_zero_rad,
                                            cfg->right_front_dir,
                                            cfg->right_back_dir,
                                            elapsed_ms,
                                            pitch,
                                            cfg->foot_test_kp,
                                            cfg->foot_test_kd,
                                            torque);
    if (ok_left == 0u || ok_right == 0u)
    {
        WheelLegClearJointTestCmds();
        return 0u;
    }

    s_wheelleg.foot_test_ik_ok = 1u;
    return 1u;
}
