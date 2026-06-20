/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static const fp32 s_default_lqr_poly[12][4] = {
    {-243.932f, 105.148f, -19.1838f, -0.199759f},
    {-6.33721f, 2.6174f, -1.08798f, -0.0047227f},
    {-43.8763f, 16.3233f, -2.10154f, -0.127721f},
    {-52.0411f, 19.3719f, -2.60375f, -0.168927f},
    {-805.793f, 328.293f, -48.8092f, 2.92903f},
    {-40.1396f, 16.7832f, -2.61208f, 0.17602f},
    {-962.682f, 417.508f, -67.9889f, 4.84346f},
    {-89.4595f, 37.3246f, -5.80403f, 0.414703f},
    {-618.557f, 251.264f, -37.1404f, 2.18751f},
    {-800.904f, 324.39f, -47.7682f, 2.80481f},
    {3575.2f, -1332.91f, 172.196f, 9.48315f},
    {202.812f, -76.6035f, 10.1013f, 0.345984f},
};

static fp32 WheelLegClamp(fp32 value, fp32 min_value, fp32 max_value)
{
    return WheelLegCoreClamp(value, min_value, max_value);
}

static fp32 WheelLegAbs(fp32 value)
{
    return WheelLegCoreAbs(value);
}

static uint8_t WheelLegAxisInDeadband(int16_t axis, uint16_t deadband)
{
    return WheelLegCoreAxisInDeadband(axis, deadband);
}

static fp32 WheelLegTargetThetaFromFootX(fp32 foot_x_m, fp32 leg_length_m)
{
    return WheelLegCoreTargetThetaFromFootX(foot_x_m, leg_length_m);
}

static fp32 WheelLegBenchDefaultLegM(void)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    fp32 length = cfg->min_leg_length_m;

    if (length <= 0.02f)
    {
        length = WHEELLEG_BENCH_CENTER_LEG_M;
    }
    if (cfg->max_leg_length_m > 0.02f)
    {
        length = WheelLegClamp(length, 0.02f, cfg->max_leg_length_m);
    }
    return length;
}

static uint8_t WheelLegLimitFootXy(fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    fp32 min_leg = cfg->min_leg_length_m;
    fp32 max_leg = cfg->max_leg_length_m;

    if (x_m == NULL || y_m == NULL || length_m == NULL)
    {
        return 0u;
    }

    if (min_leg <= 0.02f)
    {
        min_leg = 0.02f;
    }
    if (max_leg < min_leg)
    {
        max_leg = min_leg;
    }

    return WheelLegCoreLimitFootXy(min_leg,
                                       max_leg,
                                       WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                       x_m,
                                       y_m,
                                       length_m);
}

static WheelLegCoreTargetSmooth WheelLegTargetSmoothFromState(void)
{
    WheelLegCoreTargetSmooth smooth;

    smooth.foot_x_m = s_wheelleg.target_foot_x_smooth;
    smooth.foot_y_m = s_wheelleg.target_foot_y_smooth;
    smooth.length_m = s_wheelleg.target_leg_smooth;
    smooth.valid = s_wheelleg.target_smooth_valid;
    return smooth;
}

static void WheelLegTargetSmoothToState(const WheelLegCoreTargetSmooth *smooth)
{
    if (smooth == NULL)
    {
        return;
    }

    s_wheelleg.target_foot_x_smooth = smooth->foot_x_m;
    s_wheelleg.target_foot_y_smooth = smooth->foot_y_m;
    s_wheelleg.target_leg_smooth = smooth->length_m;
    s_wheelleg.target_smooth_valid = smooth->valid;
}

static void WheelLegTargetSmoothReset(void)
{
    WheelLegCoreTargetSmooth smooth;
    uint8_t side;

    WheelLegCoreTargetSmoothClear(&smooth);
    WheelLegTargetSmoothToState(&smooth);
    s_wheelleg.auto_leg_stage = 0u;
    s_wheelleg.auto_leg_stage_tick_ms = 0u;
    s_wheelleg.bench_hold_pose_valid = 0u;
    s_wheelleg.bench_hold_target_leg_m = WheelLegBenchDefaultLegM();
    s_wheelleg.bench_hold_target_foot_x_m = WHEELLEG_BENCH_TARGET_FOOT_X_M;
    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.bench_hold_front_valid[side] = 0u;
        s_wheelleg.bench_hold_back_valid[side] = 0u;
        s_wheelleg.bench_hold_front_rad[side] = 0.0f;
        s_wheelleg.bench_hold_back_rad[side] = 0.0f;
    }
}

static void WheelLegBalanceStateReset(void)
{
    WheelLegTargetSmoothReset();
    s_wheelleg.x_m = 0.0f;
    s_wheelleg.v_mps = 0.0f;
    s_wheelleg.pitch_auto_offset_rad = 0.0f;
    s_wheelleg.pitch_trim_v_lpf_mps = 0.0f;
    s_wheelleg.balance_idle_latched = 0u;
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].first = 0u;
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].first = 0u;
}

static void WheelLegTargetSmoothUpdateXy(fp32 target_foot_x, fp32 target_foot_y, fp32 dt)
{
    WheelLegCoreTargetSmooth smooth = WheelLegTargetSmoothFromState();
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;
    const fp32 measured_leg =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length) * 0.5f;
    const fp32 measured_alpha =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].alpha + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].alpha) * 0.5f;

    if (WheelLegCoreTargetSmoothUpdateXy(&smooth,
                                              target_foot_x,
                                              target_foot_y,
                                              measured_leg,
                                              measured_alpha,
                                              cfg->min_leg_length_m,
                                              cfg->max_leg_length_m,
                                              WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                              WHEELLEG_TARGET_FOOT_X_SLEW_MPS,
                                              WHEELLEG_TARGET_LEG_SLEW_MPS,
                                              0.003f,
                                              dt) != 0u)
    {
        WheelLegTargetSmoothToState(&smooth);
    }
}

#if WHEELLEG_ENABLE_VMC_BALANCE
static fp32 WheelLegAutoLegTargetByStage(uint8_t stage)
{
    switch (stage)
    {
    case 0u:
        return 0.100f;
    case 1u:
        return 0.120f;
    default:
        return 0.085f;
    }
}

static fp32 WheelLegMeasuredLegAverage(void)
{
    return (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length +
            s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length) *
           0.5f;
}

static fp32 WheelLegAutoLegTarget(uint32_t now_ms)
{
    fp32 target = WheelLegAutoLegTargetByStage(s_wheelleg.auto_leg_stage);

    if (s_wheelleg.auto_leg_stage < 2u &&
        s_wheelleg.target_smooth_valid != 0u &&
        WheelLegAbs(s_wheelleg.target_leg_smooth - target) <= WHEELLEG_AUTO_LEG_REACH_EPS_M &&
        WheelLegAbs(WheelLegMeasuredLegAverage() - target) <= WHEELLEG_AUTO_LEG_REACH_EPS_M)
    {
        if (s_wheelleg.auto_leg_stage_tick_ms == 0u)
        {
            s_wheelleg.auto_leg_stage_tick_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_wheelleg.auto_leg_stage_tick_ms) >= WHEELLEG_AUTO_LEG_DWELL_MS)
        {
            s_wheelleg.auto_leg_stage++;
            s_wheelleg.auto_leg_stage_tick_ms = 0u;
            target = WheelLegAutoLegTargetByStage(s_wheelleg.auto_leg_stage);
        }
    }
    else
    {
        s_wheelleg.auto_leg_stage_tick_ms = 0u;
    }

    return target;
}
#endif

static fp32 WheelLegJointToRaw(fp32 kinematics_position, fp32 zero_position, int8_t dir)
{
    return WheelLegCoreJointToRaw(kinematics_position, zero_position, dir);
}

static fp32 WheelLegRawToKinematic(fp32 raw_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return WheelLegCoreRawToKinematic(raw_position, raw_zero_position, kinematic_zero_position, dir);
}

static fp32 WheelLegKinematicToRaw(fp32 kinematic_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return WheelLegCoreKinematicToRaw(kinematic_position, raw_zero_position, kinematic_zero_position, dir);
}

static uint16_t WheelLegPeriodMs(void)
{
    return RobotProfileWheellegMitControlPeriodMs();
}

static fp32 WheelLegPeriodS(void)
{
    return (fp32)WheelLegPeriodMs() * 0.001f;
}

static uint32_t WheelLegTickMs(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint8_t WheellegControlMgrAllows(uint32_t tick_ms, fp32 dt)
{
    ControlCtx context = {0};

    context.tick_ms = tick_ms;
    context.dt_s = dt;

    if (ControlMgrUpdateDomain(ControlDomainWheelleg, &context) != ControlResultOk)
    {
        return 0u;
    }

    return (ControlMgrActiveId(ControlDomainWheelleg) == ControlIdWheellegMitBalance) ? 1u : 0u;
}

static void WheelLegPidApply(WheelLegPid *pid, const PidParam *cfg)
{
    if (pid == NULL || cfg == NULL)
    {
        return;
    }
    WheelLegCorePidConfigure(pid,
                                cfg->kp,
                                cfg->ki,
                                cfg->kd,
                                cfg->max_out,
                                cfg->max_iout);
}

static void WheelLegPidClear(WheelLegPid *pid)
{
    WheelLegCorePidClear(pid);
}

static fp32 WheelLegPidCalc(WheelLegPid *pid, fp32 ref, fp32 set)
{
    return WheelLegCorePidCalc(pid, ref, set);
}

static fp32 WheelLegPoly(const fp32 coe[4], fp32 len)
{
    return WheelLegCorePoly4(coe, len);
}

static uint8_t WheelLegLqrRowIsZero(const fp32 coe[4])
{
    return WheelLegCoreLqrRowIsZero(coe);
}

static const fp32 *WheelLegLqrRow(uint8_t index)
{
    const fp32 *configured;

    if (index >= 12u)
    {
        return &s_default_lqr_poly[0][0];
    }
    configured = &g_config.WheelLegMit.lqr_poly[index][0];
    return (WheelLegLqrRowIsZero(configured) == 0u) ? configured : &s_default_lqr_poly[index][0];
}

static void WheelLegEvalLqr(fp32 leg_length, fp32 out[12])
{
    uint8_t i;
    fp32 length = leg_length;

    if (out == NULL)
    {
        return;
    }

    if (g_config.WheelLegMit.min_leg_length_m < g_config.WheelLegMit.max_leg_length_m)
    {
        length = WheelLegClamp(length,
                                g_config.WheelLegMit.min_leg_length_m,
                                g_config.WheelLegMit.max_leg_length_m);
    }

    for (i = 0u; i < 12u; i++)
    {
        out[i] = WheelLegPoly(WheelLegLqrRow(i), length);
    }
}

static uint8_t WheelLegManualEnabledBySwitch(void)
{
    return (uint8_t)(ControlInputSwitchIsPos(input_switch(INPUT_SW_CHASSIS_MODE),
                                                 g_config.manual_input.semantics.ChassisSafePos) == 0u);
}

static fp32 WheelLegImuAngle(const fp32 *angle, uint8_t offset, fp32 sign)
{
    return (angle != NULL) ? angle[offset] * sign : 0.0f;
}

static const fp32 *WheelLegImuGyroAligned(const fp32 *gyro, fp32 aligned[3])
{
    if (gyro == NULL || aligned == NULL)
    {
        return NULL;
    }

    aligned[INS_GYRO_X_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_X_OFFSET] * WHEELLEG_IMU_GYRO_X_SIGN;
    aligned[INS_GYRO_Y_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_Y_OFFSET] * WHEELLEG_IMU_GYRO_Y_SIGN;
    aligned[INS_GYRO_Z_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_Z_OFFSET] * WHEELLEG_IMU_GYRO_Z_SIGN;
    return aligned;
}
