/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static fp32 WatchWheelLegDirSign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static void WatchCopyWheelLegMitMotor(WatchWheelLegMitMotor *out,
                                          uint8_t raw_id,
                                          fp32 zero_rad,
                                          int8_t dir,
                                          uint8_t use_rel_position)
{
    MotorCmd cmd = {0};
    MotorState fb = {0};
    MotorApplied applied = {0};
    MotorId id;

    if (out == NULL)
    {
        return;
    }
    out->id = raw_id;
    if ((uint32_t)raw_id >= (uint32_t)MotorCount)
    {
        return;
    }

    id = (MotorId)raw_id;
    if (LowCmdGetMotor(id, &cmd) != 0u)
    {
        out->cmd_active = cmd.active;
        out->cmd_mode = cmd.mode;
        out->cmd_writer = cmd.writer;
        out->cmd_timeout_ms = cmd.timeoutMs;
        out->cmd_seq = cmd.seq;
        out->cmd_tick_ms = cmd.tick;
        out->cmd_position_deg = WatchRadToDeg(cmd.q);
        out->cmd_velocity_deg_s = WatchRadToDeg(cmd.dq);
        out->cmd_kp = cmd.kp;
        out->cmd_kd = cmd.kd;
        out->cmd_torque_nm = cmd.tau;
    }

    if (LowStateGetApplied(id, &applied) != 0u)
    {
        out->applied_active = applied.active;
        out->applied_mode = applied.mode;
        out->applied_drive_state = applied.driveState;
        out->applied_flags = applied.flags;
        out->applied_bus = applied.bus;
        out->applied_transport = applied.transport;
        out->applied_protocol = applied.protocol;
        out->applied_tx_id = applied.txId;
        out->applied_tick_ms = applied.tick;
        out->applied_current = applied.current;
        out->applied_position_deg = WatchRadToDeg(applied.q);
        out->applied_velocity_deg_s = WatchRadToDeg(applied.dq);
        out->applied_kp = applied.kp;
        out->applied_kd = applied.kd;
        out->applied_torque_nm = applied.tau;
    }

    if (LowStateGetMotor(id, &fb) != 0u)
    {
        out->online = fb.online;
        out->fb_bus = fb.bus;
        out->fb_rx_dlc = fb.rxDlc;
        out->fb_rx_data0 = fb.rxData0;
        out->fb_rx_data0_low4 = (uint8_t)(fb.rxData0 & 0x0Fu);
        out->fb_rx_data0_high4 = (uint8_t)(fb.rxData0 >> 4);
        out->fb_motor_id = fb.motorId;
        out->fb_state = fb.state;
        out->fb_rx_id = fb.rxId;
        out->fb_rx_count = fb.rxCount;
        out->fb_last_rx_tick_ms = fb.lastRxTick;
        out->fb_position_deg = WatchRadToDeg(fb.q);
        out->fb_velocity_deg_s = WatchRadToDeg(fb.dq);
        out->fb_torque_nm = fb.tauEst;
        if (use_rel_position != 0u)
        {
            const fp32 rel_position_rad = (fb.q - zero_rad) * WatchWheelLegDirSign(dir);
            out->rel_position_deg = WatchRadToDeg(rel_position_rad);
        }
        if (out->cmd_active != 0u)
        {
            out->cmd_minus_fb_deg = WatchRadToDeg(cmd.q - fb.q);
        }
    }
}

static void WatchCopyWheelLegMitFootTest(uint8_t side)
{
    WatchWheelLegMitFootTest *out;
    fp32 wheel_zero_rad = 0.0f;
    fp32 wheel_comp_rad = 0.0f;
    fp32 wheel_target_rad = 0.0f;

    if (side >= (uint8_t)WHEELLEG_SIDE_COUNT)
    {
        return;
    }

    out = &g_watch.WheelLegMit.foot_test[side];
    WheelLegMitGetFootTestTarget(side,
                                      &out->target_x_m,
                                      &out->target_y_m,
                                      &out->target_length_m);
    WheelLegMitGetFootTestWheel(side,
                                     &out->wheel_zero_valid,
                                     &wheel_zero_rad,
                                     &out->wheel_dx_m,
                                     &wheel_comp_rad,
                                     &wheel_target_rad);
    out->wheel_zero_deg = WatchRadToDeg(wheel_zero_rad);
    out->wheel_comp_deg = WatchRadToDeg(wheel_comp_rad);
    out->wheel_target_deg = WatchRadToDeg(wheel_target_rad);
}

static uint8_t WatchWheelLegManualEnabledBySwitch(uint8_t ChassisSw,
                                                   uint8_t ChassisSafePos,
                                                   uint8_t manualOnline)
{
    return (uint8_t)(manualOnline != 0u &&
                     ControlInputSwitchIsPos(ChassisSw, ChassisSafePos) == 0u);
}

static void WatchCopyWheelLegMit(void)
{
    const uint8_t profile_on = WatchBlockActiveWheelLegMit();
    const uint8_t manualOnline = (uint8_t)(s_watch_manual_input_valid != 0u &&
                                           s_watch_manual_input.online != 0u);
    const uint8_t ChassisSafePos = (s_watch_manual_input_valid != 0u) ?
                                       s_watch_manual_input.semantics.ChassisSafePos :
                                       MANUAL_INPUT_SWITCH_POS_UP;
    const uint8_t ChassisSw = (manualOnline != 0u) ?
                                  s_watch_manual_input.control.sw[INPUT_SW_CHASSIS_MODE] :
                                  ControlInputSwitchPosToRaw(ChassisSafePos);
    const uint8_t run_variant = (uint8_t)robot_mode_variant();
    WheelLegStatus status;
    WheelLegState state;
    LowCmdDiag lowcmd_diag = {0};
    uint8_t status_valid;
    uint8_t state_valid;

    memset(&g_watch.WheelLegMit, 0, sizeof(g_watch.WheelLegMit));
    if (profile_on == 0u)
    {
        return;
    }

    status_valid = WheelLegStatusRead(&status);
    state_valid = WheelLegStateRead(&state);
    (void)LowCmdGetDiag(&lowcmd_diag);

    g_watch.WheelLegMit.status_valid = status_valid;
    g_watch.WheelLegMit.state_valid = state_valid;
    g_watch.WheelLegMit.profile_on = profile_on;
    g_watch.WheelLegMit.lowcmd_seq = lowcmd_diag.seq;
    g_watch.WheelLegMit.lowcmd_rejected_count = lowcmd_diag.rejected_count;
    g_watch.WheelLegMit.lowcmd_emergency_stop_count = lowcmd_diag.emergency_stop_count;
    g_watch.WheelLegMit.lowcmd_last_reject_tick_ms = lowcmd_diag.last_reject_tick;
    g_watch.WheelLegMit.lowcmd_last_reject_writer = lowcmd_diag.last_reject_writer;
    g_watch.WheelLegMit.lowcmd_last_reject_owner = lowcmd_diag.last_reject_owner;
    g_watch.WheelLegMit.lowcmd_emergency_writer = lowcmd_diag.emergency_writer;
    g_watch.WheelLegMit.lowcmd_emergency_active = lowcmd_diag.emergency_active;
    g_watch.WheelLegMit.input_chassis_switch = ChassisSw;
    g_watch.WheelLegMit.enable_switch_pos = g_config.WheelLegMit.enable_switch_pos;
    g_watch.WheelLegMit.manual_on = WatchWheelLegManualEnabledBySwitch(
        ChassisSw,
        ChassisSafePos,
        manualOnline);
    g_watch.WheelLegMit.test_mode = run_variant;
    g_watch.WheelLegMit.foot_test_phase = WheelLegMitGetFootTestPhase();
    g_watch.WheelLegMit.foot_test_ik_ok = WheelLegMitGetFootTestIkOk();
    WatchCopyWheelLegMitFootTest((uint8_t)WHEELLEG_SIDE_LEFT);
    WatchCopyWheelLegMitFootTest((uint8_t)WHEELLEG_SIDE_RIGHT);

    if (status_valid != 0u)
    {
        g_watch.WheelLegMit.mode = status.mode;
        g_watch.WheelLegMit.last_mode = status.last_mode;
        g_watch.WheelLegMit.fault_flags = status.fault_flags;
        g_watch.WheelLegMit.domain_fault_flags = status.domain_fault_flags;
        g_watch.WheelLegMit.domain_member_count = status.domain_member_count;
        g_watch.WheelLegMit.domain_online_mask = status.domain_online_mask;
        g_watch.WheelLegMit.domain_binding_valid = status.domain_binding_valid;
        g_watch.WheelLegMit.recovery_input_safe = status.recovery_input_safe;
        g_watch.WheelLegMit.domain_outputs_active = status.domain_outputs_active;
        g_watch.WheelLegMit.domain_inhibit_complete = status.domain_inhibit_complete;
        g_watch.WheelLegMit.domain_action = status.domain_action;
        g_watch.WheelLegMit.domain_recovery_pending = status.domain_recovery_pending;
        g_watch.WheelLegMit.domain_fault_active = status.domain_fault_active;
        g_watch.WheelLegMit.domain_ever_faulted = status.domain_ever_faulted;
        g_watch.WheelLegMit.domain_device_count = status.domain_device_count;
        g_watch.WheelLegMit.domain_active_member_mask = status.domain_active_member_mask;
        g_watch.WheelLegMit.domain_blocking_member_mask = status.domain_blocking_member_mask;
        g_watch.WheelLegMit.domain_active_reason_mask = status.domain_active_reason_mask;
        g_watch.WheelLegMit.domain_blocking_reason_mask = status.domain_blocking_reason_mask;
        g_watch.WheelLegMit.domain_history_reason_mask = status.domain_history_reason_mask;
        g_watch.WheelLegMit.domain_first_fault_ms = status.domain_first_fault_ms;
        g_watch.WheelLegMit.domain_last_fault_ms = status.domain_last_fault_ms;
        g_watch.WheelLegMit.domain_healthy_since_ms = status.domain_healthy_since_ms;
        g_watch.WheelLegMit.domain_stop_count = status.domain_stop_count;
        g_watch.WheelLegMit.domain_stop_fail_count = status.domain_stop_fail_count;
        g_watch.WheelLegMit.domain_inhibit_fail_count = status.domain_inhibit_fail_count;
        g_watch.WheelLegMit.domain_inhibit_release_fail_count =
            status.domain_inhibit_release_fail_count;
        g_watch.WheelLegMit.domain_last_stop_tick_ms = status.domain_last_stop_tick_ms;
        g_watch.WheelLegMit.health = status.health;
        g_watch.WheelLegMit.controller_active = status.controller_active;
        g_watch.WheelLegMit.active_controller_id = status.active_controller_id;
        g_watch.WheelLegMit.target_v_mps = status.target_v_mps;
        g_watch.WheelLegMit.target_yaw_rate_radps = status.target_yaw_rate_radps;
        g_watch.WheelLegMit.target_leg_length_m = status.target_leg_length_m;
        g_watch.WheelLegMit.target_foot_x_m = status.target_foot_x_m;
        g_watch.WheelLegMit.target_foot_y_m = status.target_foot_y_m;
        g_watch.WheelLegMit.target_leg_theta_deg = WatchRadToDeg(status.target_leg_theta_rad);
        g_watch.WheelLegMit.pitch_deg = WatchRadToDeg(status.pitch_rad);
        g_watch.WheelLegMit.x_dot_mps = status.x_dot_mps;
        g_watch.WheelLegMit.enabled =
            (uint8_t)(g_watch.WheelLegMit.manual_on != 0u &&
                      (status.fault_flags == WHEELLEG_FAULT_NONE ||
                       status.controller_active != 0u));

        g_watch.WheelLegMit.leg[0].length_m = status.leg_length_m[WHEELLEG_SIDE_LEFT];
        g_watch.WheelLegMit.leg[1].length_m = status.leg_length_m[WHEELLEG_SIDE_RIGHT];
        g_watch.WheelLegMit.leg[0].theta_deg = WatchRadToDeg(status.leg_theta_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.WheelLegMit.leg[1].theta_deg = WatchRadToDeg(status.leg_theta_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.WheelLegMit.leg[0].alpha_deg = WatchRadToDeg(status.leg_alpha_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.WheelLegMit.leg[1].alpha_deg = WatchRadToDeg(status.leg_alpha_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.WheelLegMit.leg[0].support_force_n = status.support_force_n[WHEELLEG_SIDE_LEFT];
        g_watch.WheelLegMit.leg[1].support_force_n = status.support_force_n[WHEELLEG_SIDE_RIGHT];
        g_watch.WheelLegMit.leg[0].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_LEFT];
        g_watch.WheelLegMit.leg[1].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_RIGHT];
    }

    if (state_valid != 0u)
    {
        g_watch.WheelLegMit.leg[0].front_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[0];
        g_watch.WheelLegMit.leg[0].back_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[1];
        g_watch.WheelLegMit.leg[1].front_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[0];
        g_watch.WheelLegMit.leg[1].back_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[1];
        g_watch.WheelLegMit.leg[0].wheel_online = state.wheel_online[WHEELLEG_SIDE_LEFT];
        g_watch.WheelLegMit.leg[1].wheel_online = state.wheel_online[WHEELLEG_SIDE_RIGHT];
    }

    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.joint[0],
                                  g_config.WheelLegMit.left_front_actuator,
                                  g_config.WheelLegMit.left_front_zero_rad,
                                  g_config.WheelLegMit.left_front_dir,
                                  1u);
    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.joint[1],
                                  g_config.WheelLegMit.left_back_actuator,
                                  g_config.WheelLegMit.left_back_zero_rad,
                                  g_config.WheelLegMit.left_back_dir,
                                  1u);
    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.joint[2],
                                  g_config.WheelLegMit.right_front_actuator,
                                  g_config.WheelLegMit.right_front_zero_rad,
                                  g_config.WheelLegMit.right_front_dir,
                                  1u);
    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.joint[3],
                                  g_config.WheelLegMit.right_back_actuator,
                                  g_config.WheelLegMit.right_back_zero_rad,
                                  g_config.WheelLegMit.right_back_dir,
                                  1u);
    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.wheel[0],
                                  g_config.WheelLegMit.left_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
    WatchCopyWheelLegMitMotor(&g_watch.WheelLegMit.wheel[1],
                                  g_config.WheelLegMit.right_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
}

#endif
