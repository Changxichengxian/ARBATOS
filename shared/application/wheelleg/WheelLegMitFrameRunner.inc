/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by WheelLegMitTask.c.
 */

static uint8_t WheelLegInputSnapshotConfigValid(void)
{
    return (uint8_t)(g_config.input.axis[INPUT_AXIS_CHASSIS_X].rc_ch < 5u &&
                     g_config.input.axis[INPUT_AXIS_CHASSIS_Y].rc_ch < 5u &&
                     g_config.input.axis[INPUT_AXIS_CHASSIS_WZ].rc_ch < 5u &&
                     g_config.input.axis[INPUT_AXIS_CALIB_3].rc_ch < 5u &&
                     g_config.input.sw[INPUT_SW_CHASSIS_MODE].rc_sw < 2u);
}

static int16_t WheelLegInputAxisFromSnapshot(const ManualInputState *input, input_axis_e axis)
{
    const input_axis_map_t *map;
    int16_t value;

    if (input == NULL || (uint32_t)axis >= (uint32_t)INPUT_AXIS_COUNT)
    {
        return 0;
    }
    map = &g_config.input.axis[axis];
    if (map->rc_ch >= 5u)
    {
        return 0;
    }

    value = input->rc.ch[map->rc_ch];
    return (map->invert != 0u) ? (int16_t)-value : value;
}

static uint8_t WheelLegInputSwitchFromSnapshot(const ManualInputState *input, input_switch_e sw)
{
    const input_switch_map_t *map;
    uint8_t value;

    if (input == NULL || (uint32_t)sw >= (uint32_t)INPUT_SW_COUNT)
    {
        return RC_SW_UP;
    }
    map = &g_config.input.sw[sw];
    if (map->rc_sw >= 2u)
    {
        return RC_SW_UP;
    }

    value = (uint8_t)input->rc.s[map->rc_sw];
    if (map->invert != 0u)
    {
        if (value == RC_SW_UP)
        {
            value = RC_SW_DOWN;
        }
        else if (value == RC_SW_DOWN)
        {
            value = RC_SW_UP;
        }
    }
    return value;
}

static void WheelLegInputSnapshotCapture(WheelLegTaskFrame *frame)
{
    uint8_t input_valid;

    if (frame == NULL)
    {
        return;
    }

    input_valid = ManualInputGetCurrentCopy(&frame->manual_input);
    input_valid = (uint8_t)(input_valid != 0u &&
                            toe_is_error(DBUS_TOE) == 0u &&
                            WheelLegInputSnapshotConfigValid() != 0u);
    frame->manual_input_valid = input_valid;
    frame->vx_axis = WheelLegInputAxisFromSnapshot(&frame->manual_input, INPUT_AXIS_CHASSIS_X);
    frame->leg_axis = WheelLegInputAxisFromSnapshot(&frame->manual_input, INPUT_AXIS_CHASSIS_Y);
    frame->yaw_axis = WheelLegInputAxisFromSnapshot(&frame->manual_input, INPUT_AXIS_CHASSIS_WZ);
    frame->single_test_axis = WheelLegInputAxisFromSnapshot(&frame->manual_input, INPUT_AXIS_CALIB_3);
    frame->chassis_switch = WheelLegInputSwitchFromSnapshot(&frame->manual_input, INPUT_SW_CHASSIS_MODE);
    frame->manual_on = WheelLegManualEnabledFromSnapshot(input_valid, frame->chassis_switch);
    frame->recovery_input_safe = WheelLegRecoveryInputSafeFromSnapshot(input_valid,
                                                                       frame->chassis_switch,
                                                                       frame->vx_axis,
                                                                       frame->leg_axis,
                                                                       frame->yaw_axis,
                                                                       frame->single_test_axis);
}

static void WheelLegSingleTestScopeUpdate(WheelLegTaskFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    frame->single_test_target = MotorCount;
    frame->single_test_member = (uint8_t)WheelLegMemberCount;
    if (frame->single_test == 0u)
    {
        return;
    }

    frame->single_test_target = WheelLegSingleTestActuator();
    frame->single_test_target_valid =
        WheelLegDomainMemberResolve(frame->single_test_target, &frame->single_test_member);
    if (frame->single_test_target_valid == 0u)
    {
        frame->domain_faults |= WHEELLEG_FAULT_CONTROLLER;
        return;
    }

    frame->single_test_target_online =
        (uint8_t)((s_wheelleg.domain_online_mask &
                   (uint8_t)(1u << frame->single_test_member)) != 0u);
    if (frame->single_test_target_online == 0u)
    {
        frame->domain_faults |= s_wheelleg.domain_member[frame->single_test_member].offline_fault;
    }
}

static void WheelLegFaultDomainUpdate(WheelLegTaskFrame *frame)
{
    FaultMgrResult result = FaultMgrResultOk;
    uint32_t domain_safe_mask = 0u;
    uint8_t single_test_transition;
    uint8_t config_fault_active;
    FaultAction previous_action;
    FaultAction requested_action;
    uint8_t device_id;

    if (frame == NULL)
    {
        return;
    }

    single_test_transition =
        (uint8_t)(s_wheelleg.single_test_last != frame->single_test ||
                  (frame->single_test != 0u &&
                   s_wheelleg.single_test_target_last != frame->single_test_target));
    config_fault_active = (frame->single_test != 0u) ?
                              (uint8_t)(frame->single_test_target_valid == 0u) :
                              (uint8_t)(s_wheelleg.domain_binding_valid == 0u);
    previous_action = s_wheelleg.domain_last_action;

    if (s_wheelleg.fault_mgr_ready != 0u)
    {
        for (device_id = 0u; device_id < (uint8_t)WheelLegMemberCount; device_id++)
        {
            uint8_t fault_active =
                (uint8_t)((s_wheelleg.domain_online_mask & (uint8_t)(1u << device_id)) == 0u);

            if (frame->single_test != 0u &&
                (frame->single_test_target_valid == 0u || device_id != frame->single_test_member))
            {
                fault_active = 0u;
            }

            if (FaultMgrSetDeviceFault(&s_wheelleg.fault_mgr,
                                       device_id,
                                       WHEELLEG_DOMAIN_REASON_MOTOR_OFFLINE,
                                       fault_active,
                                       frame->now_ms) != FaultMgrResultOk)
            {
                result = FaultMgrResultNotReady;
            }
        }
        if (FaultMgrSetDeviceFault(&s_wheelleg.fault_mgr,
                                   (uint8_t)WheelLegFaultDeviceImu,
                                   WHEELLEG_DOMAIN_REASON_IMU_OFFLINE,
                                   (uint8_t)(frame->imu_ok == 0u),
                                   frame->now_ms) != FaultMgrResultOk)
        {
            result = FaultMgrResultNotReady;
        }
        if (FaultMgrSetDeviceFault(&s_wheelleg.fault_mgr,
                                   (uint8_t)WheelLegFaultDeviceGuard,
                                   WHEELLEG_DOMAIN_REASON_CONFIG,
                                   config_fault_active,
                                   frame->now_ms) != FaultMgrResultOk)
        {
            result = FaultMgrResultNotReady;
        }
        if (FaultMgrSetDeviceFault(&s_wheelleg.fault_mgr,
                                   (uint8_t)WheelLegFaultDeviceGuard,
                                   WHEELLEG_DOMAIN_REASON_TEST_TRANSITION,
                                   single_test_transition,
                                   frame->now_ms) != FaultMgrResultOk)
        {
            result = FaultMgrResultNotReady;
        }

        if (frame->recovery_input_safe != 0u)
        {
            domain_safe_mask = 1u << WheelLegFaultDomainDrive;
        }
        if (result == FaultMgrResultOk)
        {
            result = FaultMgrUpdate(&s_wheelleg.fault_mgr,
                                    frame->now_ms,
                                    0u,
                                    domain_safe_mask,
                                    1u);
        }
        if (result == FaultMgrResultOk)
        {
            result = FaultMgrGetDomainStatus(&s_wheelleg.fault_mgr,
                                             (uint8_t)WheelLegFaultDomainDrive,
                                             &s_wheelleg.fault_domain_status);
        }
    }
    else
    {
        result = FaultMgrResultNotReady;
    }

    if (result != FaultMgrResultOk)
    {
        (void)memset(&s_wheelleg.fault_domain_status, 0, sizeof(s_wheelleg.fault_domain_status));
        s_wheelleg.fault_domain_status.action = FaultActionStopDomain;
        s_wheelleg.fault_domain_status.blockingMemberMask = WHEELLEG_FAULT_DOMAIN_MEMBER_MASK;
        s_wheelleg.fault_mgr_ready = 0u;
    }

    requested_action = s_wheelleg.fault_domain_status.action;
    if (requested_action != FaultActionRun)
    {
        if (previous_action == FaultActionRun)
        {
            s_wheelleg.domain_stop_clear_complete = 0u;
        }
        (void)WheelLegDomainInhibitHold();
    }
    else if (previous_action != FaultActionRun ||
             s_wheelleg.domain_inhibit_complete != 0u ||
             s_wheelleg.domain_stop_clear_complete != 0u)
    {
        if (WheelLegDomainInhibitRelease() == 0u)
        {
            /* 清零和整批释放没有全部成功前，故障管理器即使已恢复也不能放行。 */
            s_wheelleg.fault_domain_status.action = FaultActionStopDomain;
            s_wheelleg.fault_domain_status.recoveryPending = 1u;
        }
    }

    frame->domain_action = s_wheelleg.fault_domain_status.action;
    frame->domain_recovery_pending = s_wheelleg.fault_domain_status.recoveryPending;
    frame->domain_recovered_now =
        (uint8_t)(previous_action != FaultActionRun && frame->domain_action == FaultActionRun);
    if (frame->domain_action != FaultActionRun)
    {
        WheelLegResetDomainControlState();
    }
    else if (frame->domain_recovered_now != 0u)
    {
        /* 释放前已经用 SAFETY 清零；解锁当帧只清控制状态，不允许再写测试或控制命令。 */
        WheelLegResetDomainControlState();
    }
    s_wheelleg.domain_last_action = frame->domain_action;
    s_wheelleg.single_test_last = frame->single_test;
    s_wheelleg.single_test_target_last =
        (frame->single_test != 0u) ? frame->single_test_target : MotorCount;
}

static void WheelLegTaskFrameInit(WheelLegTaskFrame *frame)
{
    InsSnapshot imu_snapshot = {0};
    const fp32 *angle = NULL;
    const fp32 *gyro = NULL;
    uint8_t imu_snapshot_ok;

    if (frame == NULL)
    {
        return;
    }

    (void)memset(frame, 0, sizeof(*frame));
    WatchTaskBeat(WATCH_TASK_WHEELLEG_MIT);

    frame->loop_start_ms = WheelLegTickMs();
    frame->period_ms = WheelLegPeriodMs();
    frame->dt = WheelLegPeriodS();
    frame->now_ms = WheelLegTickMs();

    imu_snapshot_ok = (uint8_t)InsSnapshotRead(&imu_snapshot);
    if (imu_snapshot_ok != 0u)
    {
        angle = imu_snapshot.angle;
        gyro = imu_snapshot.gyro;
    }
    frame->gyro_wheelleg = WheelLegImuGyroAligned(gyro, frame->gyro_aligned);
    frame->pitch = WheelLegImuAngle(angle, WHEELLEG_IMU_BODY_PITCH_OFFSET, WHEELLEG_IMU_PITCH_SIGN);
    frame->roll = WheelLegImuAngle(angle, WHEELLEG_IMU_BODY_ROLL_OFFSET, WHEELLEG_IMU_ROLL_SIGN);
    frame->yaw = WheelLegImuAngle(angle, WHEELLEG_IMU_BODY_YAW_OFFSET, WHEELLEG_IMU_YAW_SIGN);

    WheelLegInputSnapshotCapture(frame);
    frame->control_stage = WheelLegControlStageFromConfig();

    frame->target_leg = 0.100f;
    frame->target_foot_y = 0.100f;
    frame->profile_on = RobotProfileIsWheelLegMit();
    frame->imu_ok = (uint8_t)(imu_snapshot_ok != 0u &&
                              imu_snapshot.age_ms <= (uint32_t)frame->period_ms * 3u + 2u &&
                              toe_is_error(BOARD_GYRO_TOE) == 0u &&
                              toe_is_error(BOARD_ACCEL_TOE) == 0u);
    frame->single_test = WheelLegSingleTestEnabled();
    frame->left_leg_test = robot_mode_wheelleg_left_leg_swing();
    frame->foot_test = robot_mode_wheelleg_foot_trajectory();
    frame->operation_test_active =
        (frame->single_test != 0u || frame->left_leg_test != 0u || frame->foot_test != 0u) ? 1u : 0u;

    WheelLegPidApply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT], &g_config.WheelLegMit.leg_length_pid);
    WheelLegPidApply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT], &g_config.WheelLegMit.leg_length_pid);
    WheelLegPidApply(&s_wheelleg.split_pid, &g_config.WheelLegMit.leg_split_pid);
    WheelLegControlStageUpdate(frame->control_stage);

    frame->feedback_faults = WheelLegUpdateFeedback(frame->now_ms);
    s_wheelleg.feedback_faults = frame->feedback_faults;
    frame->domain_faults = (frame->single_test != 0u) ? WHEELLEG_FAULT_NONE : frame->feedback_faults;
    WheelLegSingleTestScopeUpdate(frame);
    s_wheelleg.manual_on = frame->manual_on;
    s_wheelleg.recovery_input_safe = frame->recovery_input_safe;
    if (frame->imu_ok == 0u)
    {
        frame->domain_faults |= WHEELLEG_FAULT_IMU_OFFLINE;
    }
    s_wheelleg.domain_faults = frame->domain_faults;
    WheelLegFaultDomainUpdate(frame);

    if (frame->manual_input_valid == 0u)
    {
        frame->faults |= WHEELLEG_FAULT_MANUAL_OFFLINE;
    }
    if (frame->single_test != 0u)
    {
        frame->faults |= frame->domain_faults;
    }
    else
    {
        frame->faults |= frame->feedback_faults;
        if (frame->imu_ok == 0u)
        {
            frame->faults |= WHEELLEG_FAULT_IMU_OFFLINE;
        }
        if (WheelLegAbs(frame->pitch) > g_config.WheelLegMit.attitude_limit_rad ||
            WheelLegAbs(frame->roll) > g_config.WheelLegMit.attitude_limit_rad)
        {
            frame->faults |= WHEELLEG_FAULT_ATTITUDE_LIMIT;
        }
    }
    if (frame->domain_action != FaultActionRun || frame->domain_recovered_now != 0u)
    {
        frame->faults |= WHEELLEG_FAULT_DOMAIN_RECOVERY;
    }

    if (frame->profile_on != 0u &&
        frame->feedback_faults == WHEELLEG_FAULT_NONE &&
        frame->domain_action == FaultActionRun)
    {
        frame->kinematics_ok = WheelLegUpdateLegKinematics((frame->imu_ok != 0u) ? frame->pitch : 0.0f,
                                                              (frame->imu_ok != 0u) ? frame->gyro_wheelleg : NULL,
                                                              frame->dt);
    }

    frame->enabled = (uint8_t)(frame->profile_on != 0u &&
                               frame->manual_on != 0u &&
                               frame->domain_recovered_now == 0u &&
                               frame->faults == WHEELLEG_FAULT_NONE);
    s_wheelleg.last_mode = s_wheelleg.mode;
}

static void WheelLegTaskFinishFrame(const WheelLegTaskFrame *frame,
                                       uint16_t publish_faults,
                                       uint64_t profiler_start_us,
                                       TickType_t *last_wake,
                                       uint8_t count_overrun)
{
    if (frame == NULL || last_wake == NULL)
    {
        return;
    }

    WheelLegPublish(publish_faults,
                     s_wheelleg.mode,
                     frame->pitch,
                     frame->roll,
                     frame->yaw,
                     frame->gyro_wheelleg,
                     frame->target_v,
                     frame->target_leg,
                     frame->target_foot_x,
                     frame->target_yaw_rate,
                     frame->wheel_torque,
                     frame->controller_active);

    if (count_overrun != 0u &&
        (uint32_t)(WheelLegTickMs() - frame->loop_start_ms) > frame->period_ms)
    {
        s_wheelleg.overrun_count++;
    }
    RtProfEnd(RtProfWheellegMitLoop, profiler_start_us);
    vTaskDelayUntil(last_wake, pdMS_TO_TICKS(frame->period_ms));
}

static void WheelLegResetDomainControlState(void)
{
    uint8_t side;

    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    WheelLegPidClear(&s_wheelleg.split_pid);
    WheelLegBalanceStateReset();
    WheelLegClearLegVirtualOutputs();
    WheelLegClearTestRuntimeFlags();
    WheelLegFootTestResetWheelZero();
    s_wheelleg.yaw_set = 0.0f;
    s_wheelleg.last_yaw = 0.0f;
    s_wheelleg.yaw_inited = 0u;
    s_wheelleg.ever_commanded = 0u;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        (void)memset(&s_wheelleg.foot_test_target[side], 0, sizeof(s_wheelleg.foot_test_target[side]));
    }
}

static void WheelLegHandleDisabledFrame(WheelLegTaskFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    if (frame->domain_recovered_now != 0u ||
        (frame->domain_action != FaultActionRun && frame->domain_recovery_pending != 0u))
    {
        s_wheelleg.mode = WHEELLEG_MODE_RECOVERY;
    }
    else
    {
        s_wheelleg.mode =
            (frame->faults == WHEELLEG_FAULT_NONE) ? WHEELLEG_MODE_DISABLED : WHEELLEG_MODE_FAULT;
    }
    if (frame->profile_on != 0u)
    {
        (void)WheelLegStopDomainOutputs();
    }
    WheelLegResetDomainControlState();
    frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
    frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
    frame->controller_active = 0u;
}

static void WheelLegOperationTestResetCommon(void)
{
    WheelLegBalanceStateReset();
    s_wheelleg.detached_test_active = 0u;
}

static void WheelLegMarkTestResult(WheelLegTaskFrame *frame, uint8_t ok)
{
    if (frame == NULL)
    {
        return;
    }

    if (ok != 0u)
    {
        frame->controller_active = 1u;
        s_wheelleg.ever_commanded = 1u;
    }
    else
    {
        frame->faults |= WHEELLEG_FAULT_CONTROLLER;
        s_wheelleg.mode = WHEELLEG_MODE_FAULT;
    }
}

static uint8_t WheelLegRunOperationTest(WheelLegTaskFrame *frame)
{
    if (frame == NULL)
    {
        return 0u;
    }

    if (frame->single_test != 0u)
    {
        WheelLegOperationTestResetCommon();
        s_wheelleg.left_test_active = 0u;
        s_wheelleg.foot_test_active = 0u;
        s_wheelleg.foot_test_phase = 0u;
        s_wheelleg.foot_test_ik_ok = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        WheelLegMarkTestResult(frame, WheelLegSingleTestApply(frame->single_test_axis));
        return 1u;
    }

    if (frame->left_leg_test != 0u)
    {
        WheelLegOperationTestResetCommon();
        s_wheelleg.foot_test_active = 0u;
        s_wheelleg.foot_test_phase = 0u;
        s_wheelleg.foot_test_ik_ok = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        WheelLegMarkTestResult(frame, WheelLegLeftLegTestApply(frame->now_ms));
        return 1u;
    }

    if (frame->foot_test != 0u)
    {
        WheelLegOperationTestResetCommon();
        s_wheelleg.left_test_active = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        WheelLegMarkTestResult(frame, WheelLegFootTestApply(frame->now_ms, frame->pitch));
        return 1u;
    }

    return 0u;
}

static void WheelLegPrepareBalanceFrame(WheelLegTaskFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    WheelLegClearTestRuntimeFlags();
    s_wheelleg.mode = WheelLegModeFromControlStage(frame->control_stage);
    frame->target_v = WheelLegAxisToFp32(frame->vx_axis,
                                            g_config.WheelLegMit.max_v_mps,
                                            g_config.WheelLegMit.rc_deadband);
    frame->target_yaw_rate = -WheelLegAxisToFp32(frame->yaw_axis,
                                                    g_config.WheelLegMit.max_yaw_rate_radps,
                                                    g_config.WheelLegMit.rc_deadband);
}

static void WheelLegRunBenchStage(WheelLegTaskFrame *frame)
{
    uint8_t joint_hold_ok;
    uint8_t wheel_lqr_ok = 0u;

    if (frame == NULL)
    {
        return;
    }

    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    WheelLegPidClear(&s_wheelleg.split_pid);
    joint_hold_ok = WheelLegBenchHoldApplyJoints();
    frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
    frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;

    if ((frame->faults & WHEELLEG_FAULT_CONTROLLER) == 0u &&
        frame->feedback_faults == WHEELLEG_FAULT_NONE)
    {
        if (frame->kinematics_ok != 0u)
        {
            WheelLegBenchHoldUpdatePoseTarget();
            frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
            frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
            WheelLegUpdateObserver(frame->dt, frame->gyro_wheelleg);
            WheelLegBalanceIdleReset(frame->vx_axis, frame->yaw_axis, frame->yaw);
            WheelLegPitchTrimUpdate(frame->dt,
                                       frame->vx_axis,
                                       frame->yaw_axis,
                                       frame->target_v,
                                       frame->target_yaw_rate);
            if (WheelLegBenchLqrStep(frame->dt,
                                        frame->pitch,
                                        frame->yaw,
                                        frame->gyro_wheelleg,
                                        frame->target_v,
                                        frame->target_leg,
                                        frame->target_foot_x,
                                        frame->target_yaw_rate,
                                        0u,
                                        frame->wheel_torque) != 0u)
            {
                wheel_lqr_ok = 1u;
            }
            else
            {
                frame->faults |= WHEELLEG_FAULT_CONTROLLER;
            }
        }
        else
        {
            frame->faults |= WHEELLEG_FAULT_CONTROLLER;
        }
    }

    if ((frame->faults & WHEELLEG_FAULT_CONTROLLER) == 0u)
    {
        frame->controller_active = (uint8_t)((joint_hold_ok != 0u || wheel_lqr_ok != 0u) ? 1u : 0u);
        if (wheel_lqr_ok == 0u)
        {
            frame->wheel_torque[WHEELLEG_SIDE_RIGHT] = 0.0f;
            frame->wheel_torque[WHEELLEG_SIDE_LEFT] = 0.0f;
        }
        WheelLegSendWheelTorques(frame->wheel_torque);
    }
}

static void WheelLegRunPositionStage(WheelLegTaskFrame *frame)
{
    uint8_t position_ok = 0u;
    uint8_t wheel_lqr_ok = 0u;

    if (frame == NULL)
    {
        return;
    }

    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    WheelLegPidClear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    WheelLegPidClear(&s_wheelleg.split_pid);
    WheelLegClearLegVirtualOutputs();
    frame->target_foot_x = WheelLegManualXTargetFromAxis(0);
    frame->target_foot_y = WheelLegManualYTargetFromAxis(frame->leg_axis);
    if (WheelLegLimitFootXy(&frame->target_foot_x, &frame->target_foot_y, &frame->target_leg) == 0u)
    {
        frame->faults |= WHEELLEG_FAULT_CONTROLLER;
    }
    if (frame->feedback_faults != WHEELLEG_FAULT_NONE)
    {
        frame->faults |= frame->feedback_faults;
    }
    if (frame->faults == WHEELLEG_FAULT_NONE && frame->kinematics_ok == 0u)
    {
        frame->faults |= WHEELLEG_FAULT_CONTROLLER;
    }

    if (frame->faults == WHEELLEG_FAULT_NONE)
    {
        WheelLegTargetSmoothUpdateXy(frame->target_foot_x, frame->target_foot_y, frame->dt);
        frame->target_leg = s_wheelleg.target_leg_smooth;
        frame->target_foot_x = s_wheelleg.target_foot_x_smooth;
        frame->target_foot_y = s_wheelleg.target_foot_y_smooth;
        if (WheelLegManualPositionApply(frame->target_foot_x, frame->target_foot_y) != 0u)
        {
            position_ok = 1u;
            WheelLegUpdateObserver(frame->dt, frame->gyro_wheelleg);
            WheelLegBalanceIdleReset(frame->vx_axis, frame->yaw_axis, frame->yaw);
            WheelLegPitchTrimUpdate(frame->dt,
                                       frame->vx_axis,
                                       frame->yaw_axis,
                                       frame->target_v,
                                       frame->target_yaw_rate);
            if (WheelLegBenchLqrStep(frame->dt,
                                        frame->pitch,
                                        frame->yaw,
                                        frame->gyro_wheelleg,
                                        frame->target_v,
                                        frame->target_leg,
                                        frame->target_foot_x,
                                        frame->target_yaw_rate,
                                        1u,
                                        frame->wheel_torque) != 0u)
            {
                wheel_lqr_ok = 1u;
            }
            else
            {
                frame->faults |= WHEELLEG_FAULT_CONTROLLER;
            }
        }
        else
        {
            frame->faults |= WHEELLEG_FAULT_CONTROLLER;
        }
    }

    if (frame->faults == WHEELLEG_FAULT_NONE)
    {
        frame->controller_active = (uint8_t)((position_ok != 0u || wheel_lqr_ok != 0u) ? 1u : 0u);
        WheelLegSendWheelTorques(frame->wheel_torque);
    }
}

static void WheelLegRunVmcStage(WheelLegTaskFrame *frame)
{
    const uint8_t use_lqr_hip =
        (frame != NULL && frame->control_stage == WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR) ? 1u : 0u;

    if (frame == NULL)
    {
        return;
    }

    frame->target_foot_x = WheelLegManualXTargetFromAxis(0);
    frame->target_foot_y = WheelLegManualYTargetFromAxis(frame->leg_axis);
    if (WheelLegLimitFootXy(&frame->target_foot_x, &frame->target_foot_y, &frame->target_leg) == 0u)
    {
        frame->faults |= WHEELLEG_FAULT_CONTROLLER;
    }
    if (frame->feedback_faults != WHEELLEG_FAULT_NONE)
    {
        frame->faults |= frame->feedback_faults;
    }
    if (frame->faults == WHEELLEG_FAULT_NONE && frame->kinematics_ok == 0u)
    {
        frame->faults |= WHEELLEG_FAULT_CONTROLLER;
    }

    if (frame->faults == WHEELLEG_FAULT_NONE)
    {
        WheelLegTargetSmoothUpdateXy(frame->target_foot_x, frame->target_foot_y, frame->dt);
        frame->target_leg = s_wheelleg.target_leg_smooth;
        frame->target_foot_x = s_wheelleg.target_foot_x_smooth;
        frame->target_foot_y = s_wheelleg.target_foot_y_smooth;
        WheelLegUpdateObserver(frame->dt, frame->gyro_wheelleg);
        WheelLegBalanceIdleReset(frame->vx_axis, frame->yaw_axis, frame->yaw);
        WheelLegPitchTrimUpdate(frame->dt,
                                   frame->vx_axis,
                                   frame->yaw_axis,
                                   frame->target_v,
                                   frame->target_yaw_rate);
        if (WheelLegControllerStep(frame->dt,
                                     frame->pitch,
                                     frame->roll,
                                     frame->yaw,
                                     frame->gyro_wheelleg,
                                     frame->target_v,
                                     frame->target_leg,
                                     frame->target_foot_x,
                                     frame->target_yaw_rate,
                                     use_lqr_hip,
                                     frame->wheel_torque) != 0u)
        {
            frame->controller_active = 1u;
            WheelLegSendWheelTorques(frame->wheel_torque);
            WheelLegSendVmcJointTorques();
        }
        else
        {
            frame->faults |= WHEELLEG_FAULT_CONTROLLER;
        }
    }
}

static void WheelLegRunBalanceFrame(WheelLegTaskFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    WheelLegPrepareBalanceFrame(frame);
    if (frame->control_stage == WHEELLEG_CONTROL_STAGE_BENCH_LQR)
    {
        WheelLegRunBenchStage(frame);
    }
    else if (frame->control_stage == WHEELLEG_CONTROL_STAGE_POSITION_LQR)
    {
        WheelLegRunPositionStage(frame);
    }
    else
    {
        WheelLegRunVmcStage(frame);
    }

    if (frame->faults != WHEELLEG_FAULT_NONE)
    {
        s_wheelleg.mode = WHEELLEG_MODE_FAULT;
        (void)WheelLegStopDomainOutputs();
        WheelLegResetDomainControlState();
        frame->controller_active = 0u;
    }
    else if (frame->controller_active != 0u)
    {
        s_wheelleg.ever_commanded = 1u;
    }
}
