/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

static void wheelleg_clear_test_runtime_flags(void)
{
    s_wheelleg.left_test_active = 0u;
    s_wheelleg.foot_test_active = 0u;
    s_wheelleg.detached_test_active = 0u;
    s_wheelleg.foot_test_phase = 0u;
    s_wheelleg.foot_test_ik_ok = 0u;
}

static void wheelleg_balance_idle_reset(int16_t vx_axis, int16_t yaw_axis, fp32 yaw)
{
    const uint8_t idle =
        (uint8_t)((wheelleg_axis_in_deadband(vx_axis, g_config.wheelleg_mit.rc_deadband) != 0u &&
                   wheelleg_axis_in_deadband(yaw_axis, g_config.wheelleg_mit.rc_deadband) != 0u)
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

static MotorId wheelleg_core_actuator_to_id(wheelleg_core_actuator_e actuator)
{
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];

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

static uint8_t wheelleg_send_core_output(const wheelleg_core_output_t *out)
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

        id = wheelleg_core_actuator_to_id((wheelleg_core_actuator_e)i);
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

static void wheelleg_send_wheel_torques(const fp32 wheel_torque[WHEELLEG_SIDE_COUNT])
{
    wheelleg_core_output_t out;

    if (wheel_torque == NULL)
    {
        return;
    }

    wheelleg_core_output_clear(&out);
    wheelleg_core_set_wheel_torques(&out, wheel_torque);
    (void)wheelleg_send_core_output(&out);
}

static void wheelleg_send_vmc_joint_torques(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    wheelleg_core_output_t out;

    wheelleg_core_output_clear(&out);
    wheelleg_core_set_vmc_joint_torques(&out,
                                        s_wheelleg.leg,
                                        cfg->right_front_dir,
                                        cfg->right_back_dir,
                                        cfg->left_front_dir,
                                        cfg->left_back_dir);
    (void)wheelleg_send_core_output(&out);
}

static void wheelleg_clear_joint_test_cmds(void)
{
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];

    wheelleg_clear_state_cmd(left_map->front);
    wheelleg_clear_state_cmd(left_map->back);
    wheelleg_clear_state_cmd(left_map->wheel);
    wheelleg_clear_state_cmd(right_map->front);
    wheelleg_clear_state_cmd(right_map->back);
    wheelleg_clear_state_cmd(right_map->wheel);
    wheelleg_clear_state_cmd(wheelleg_single_test_actuator());
}

static void wheelleg_bench_hold_prepare_default(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_LEFT] = cfg->left_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_LEFT] = cfg->left_back_zero_rad;
    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_back_zero_rad;

    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_RIGHT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_RIGHT] = 1u;

    s_wheelleg.bench_hold_target_leg_m = wheelleg_bench_default_leg_m();
    s_wheelleg.bench_hold_target_foot_x_m = WHEELLEG_BENCH_TARGET_FOOT_X_M;
    s_wheelleg.bench_hold_pose_valid = 1u;
}

static uint8_t wheelleg_bench_hold_apply_joints(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t side;
    uint8_t sent = 0u;

    wheelleg_bench_hold_prepare_default();

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_actuator_map_t *map = &s_wheelleg.actuator[side];

        if (s_wheelleg.bench_hold_front_valid[side] != 0u)
        {
            sent += wheelleg_send_state_cmd(map->front,
                                            s_wheelleg.bench_hold_front_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
        if (s_wheelleg.bench_hold_back_valid[side] != 0u)
        {
            sent += wheelleg_send_state_cmd(map->back,
                                            s_wheelleg.bench_hold_back_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
    }

    return (sent > 0u) ? 1u : 0u;
}

static void wheelleg_bench_hold_update_pose_target(void)
{
    wheelleg_bench_hold_prepare_default();
}

static MotorId wheelleg_single_test_actuator(void)
{
    const MotorId target = robot_mode_target_motor();
    return (target != MotorCount) ? target : (MotorId)g_config.wheelleg_mit.single_test_actuator;
}

static uint8_t wheelleg_single_test_target_is_wheelleg(MotorId id)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    return (uint8_t)((id == (MotorId)cfg->left_front_actuator) ||
                     (id == (MotorId)cfg->left_back_actuator) ||
                     (id == (MotorId)cfg->left_wheel_actuator) ||
                     (id == (MotorId)cfg->right_front_actuator) ||
                     (id == (MotorId)cfg->right_back_actuator) ||
                     (id == (MotorId)cfg->right_wheel_actuator));
}

static uint8_t wheelleg_single_test_enabled(void)
{
    if (robot_mode_variant() == ROBOT_RUN_VARIANT_WHEELLEG_SINGLE_MOTOR)
    {
        return 1u;
    }

    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_SINGLE_MOTOR &&
                     wheelleg_single_test_target_is_wheelleg(robot_mode_target_motor()) != 0u);
}

static uint8_t wheelleg_single_test_apply(int16_t speed_axis)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const MotorId id = wheelleg_single_test_actuator();
    const fp32 velocity = wheelleg_axis_to_fp32(speed_axis,
                                               cfg->single_test_velocity_radps,
                                               cfg->rc_deadband);
    fp32 torque_limit;
    fp32 torque;

    wheelleg_clear_joint_test_cmds();

    torque_limit = wheelleg_abs(cfg->single_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? wheelleg_clamp(cfg->single_test_torque_nm, -torque_limit, torque_limit)
                 : 0.0f;
    return wheelleg_send_state_cmd(id,
                                   cfg->single_test_position_rad,
                                   velocity,
                                   cfg->single_test_kp,
                                   cfg->single_test_kd,
                                   torque);
}

static fp32 wheelleg_lerp(fp32 start, fp32 end, uint32_t elapsed_ms, uint32_t duration_ms)
{
    return wheelleg_core_lerp(start, end, elapsed_ms, duration_ms);
}

static void wheelleg_foot_point_lerp(const wheelleg_foot_point_t *start,
                                     const wheelleg_foot_point_t *end,
                                     uint32_t elapsed_ms,
                                     uint32_t duration_ms,
                                     wheelleg_foot_point_t *out)
{
    if (start == NULL || end == NULL || out == NULL)
    {
        return;
    }

    out->x_m = wheelleg_lerp(start->x_m, end->x_m, elapsed_ms, duration_ms);
    out->y_m = wheelleg_lerp(start->y_m, end->y_m, elapsed_ms, duration_ms);
    out->length_m = sqrtf(out->x_m * out->x_m + out->y_m * out->y_m);
}

static uint8_t wheelleg_foot_point_from_length_x(fp32 length_m,
                                                 fp32 x_m,
                                                 wheelleg_foot_point_t *out)
{
    fp32 max_x;
    fp32 y_sq;

    if (out == NULL || length_m <= 0.02f)
    {
        return 0u;
    }

    max_x = length_m * 0.98f;
    x_m = wheelleg_clamp(x_m, -max_x, max_x);
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

static uint8_t wheelleg_foot_point_from_xy(fp32 x_m,
                                           fp32 y_m,
                                           wheelleg_foot_point_t *out)
{
    fp32 length_m = 0.0f;

    if (out == NULL)
    {
        return 0u;
    }

    if (wheelleg_limit_foot_xy(&x_m, &y_m, &length_m) == 0u)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = y_m;
    out->length_m = length_m;
    return 1u;
}

static uint8_t wheelleg_inverse_point(const wheelleg_foot_point_t *target,
                                      fp32 front_ref,
                                      fp32 back_ref,
                                      fp32 *front_out,
                                      fp32 *back_out)
{
    const wheelleg_core_geometry_t geo = wheelleg_core_geometry_from_config();
    wheelleg_core_foot_point_t core_target;

    if (target == NULL || front_out == NULL || back_out == NULL)
    {
        return 0u;
    }

    wheelleg_core_point_from_local(target, &core_target);
    return wheelleg_core_inverse_point(&geo, &core_target, front_ref, back_ref, front_out, back_out);
}

static uint8_t wheelleg_kinematic_zero(fp32 *front_zero, fp32 *back_zero)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    wheelleg_foot_point_t zero;
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
        length = wheelleg_clamp(length, 0.02f, cfg->max_leg_length_m);
    }

    zero.x_m = 0.0f;
    zero.y_m = length;
    zero.length_m = length;

    return wheelleg_inverse_point(&zero,
                                  -WHEELLEG_PI,
                                  -WHEELLEG_PI,
                                  front_zero,
                                  back_zero);
}

static uint8_t wheelleg_foot_test_target(const wheelleg_foot_point_t *zero,
                                         uint32_t elapsed_ms,
                                         wheelleg_foot_point_t *target,
                                         uint8_t *phase)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 length_m = wheelleg_clamp((cfg->foot_test_length_m > 0.02f)
                                             ? cfg->foot_test_length_m
                                             : cfg->default_leg_length_m,
                                         cfg->min_leg_length_m,
                                         cfg->max_leg_length_m);
    const fp32 forward_m = wheelleg_abs(cfg->foot_test_forward_m) *
                           ((cfg->foot_test_forward_dir < 0) ? -1.0f : 1.0f);
    uint32_t zero_ms = cfg->foot_test_zero_hold_time_ms;
    uint32_t extend_ms = cfg->foot_test_extend_time_ms;
    uint32_t swing_ms = cfg->foot_test_swing_time_ms;
    uint32_t return_ms = cfg->foot_test_return_time_ms;
    uint32_t cycle_ms;
    uint32_t t;
    wheelleg_foot_point_t center;
    wheelleg_foot_point_t forward;
    wheelleg_foot_point_t backward;

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

    if (wheelleg_foot_point_from_length_x(length_m, 0.0f, &center) == 0u ||
        wheelleg_foot_point_from_length_x(length_m, forward_m, &forward) == 0u ||
        wheelleg_foot_point_from_length_x(length_m, -forward_m, &backward) == 0u)
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
        wheelleg_foot_point_lerp(zero, &center, t, extend_ms, target);
        *phase = 1u;
        return 1u;
    }
    t -= extend_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&center, &forward, t, swing_ms, target);
        *phase = 2u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&forward, &backward, t, swing_ms, target);
        *phase = 3u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&backward, &center, t, swing_ms, target);
        *phase = 4u;
        return 1u;
    }
    t -= swing_ms;
    wheelleg_foot_point_lerp(&center, zero, t, return_ms, target);
    *phase = 5u;
    return 1u;
}

static void wheelleg_foot_test_reset_wheel_zero(void)
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

static fp32 wheelleg_foot_test_ground_x(const wheelleg_foot_point_t *point,
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

static uint8_t wheelleg_foot_test_apply_wheel(const wheelleg_actuator_map_t *map,
                                              uint8_t side,
                                              const wheelleg_foot_point_t *zero,
                                              const wheelleg_foot_point_t *target,
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

    radius = g_config.wheelleg_mit.wheel_radius_m;
    if (radius <= 0.001f)
    {
        return 0u;
    }

    dx = wheelleg_foot_test_ground_x(target, side, pitch) -
         wheelleg_foot_test_ground_x(zero, side, pitch);
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
    return wheelleg_send_state_cmd(map->wheel, wheel_target, 0.0f, kp, kd, 0.0f);
}

static uint8_t wheelleg_foot_test_apply_leg(const wheelleg_actuator_map_t *map,
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
    wheelleg_foot_point_t zero;
    wheelleg_foot_point_t target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_target;
    fp32 back_target;
    uint8_t phase = 0u;

    if (map == NULL)
    {
        return 0u;
    }
    if (wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u)
    {
        return 0u;
    }

    zero.x_m = 0.0f;
    zero.y_m = g_config.wheelleg_mit.min_leg_length_m;
    zero.length_m = zero.y_m;
    if (zero.length_m <= 0.02f)
    {
        zero.y_m = g_config.wheelleg_mit.default_leg_length_m;
        zero.length_m = zero.y_m;
    }

    if (wheelleg_foot_test_target(&zero, elapsed_ms, &target, &phase) == 0u ||
        wheelleg_inverse_point(&target, front_kin_zero, back_kin_zero, &front_target, &back_target) == 0u)
    {
        return 0u;
    }

    if (side < WHEELLEG_SIDE_COUNT)
    {
        s_wheelleg.foot_test_target[side] = target;
    }
    s_wheelleg.foot_test_phase = phase;
    if (wheelleg_send_state_cmd(map->front,
                                wheelleg_kinematic_to_raw(front_target,
                                                          front_raw_zero,
                                                          front_dir,
                                                          front_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                torque) == 0u ||
        wheelleg_send_state_cmd(map->back,
                                wheelleg_kinematic_to_raw(back_target,
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
    return wheelleg_foot_test_apply_wheel(map, side, &zero, &target, pitch, kp, kd);
}

static uint8_t wheelleg_foot_test_apply(uint32_t now_ms, fp32 pitch)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    fp32 torque_limit;
    fp32 torque;
    uint32_t elapsed_ms;
    uint8_t ok_left;
    uint8_t ok_right;

    if (s_wheelleg.foot_test_active == 0u)
    {
        s_wheelleg.foot_test_start_ms = now_ms;
        s_wheelleg.foot_test_active = 1u;
        wheelleg_foot_test_reset_wheel_zero();
    }

    wheelleg_clear_joint_test_cmds();

    torque_limit = wheelleg_abs(cfg->foot_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? wheelleg_clamp(cfg->foot_test_torque_ff_nm, -torque_limit, torque_limit)
                 : 0.0f;
    elapsed_ms = now_ms - s_wheelleg.foot_test_start_ms;
    s_wheelleg.foot_test_ik_ok = 0u;

    ok_left = wheelleg_foot_test_apply_leg(left_map,
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
    ok_right = wheelleg_foot_test_apply_leg(right_map,
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
        wheelleg_clear_joint_test_cmds();
        return 0u;
    }

    s_wheelleg.foot_test_ik_ok = 1u;
    return 1u;
}
