/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

static void wheelleg_task_frame_init(wheelleg_task_frame_t *frame)
{
    const fp32 *angle;
    const fp32 *gyro;

    if (frame == NULL)
    {
        return;
    }

    (void)memset(frame, 0, sizeof(*frame));
    watch_task_beat(WATCH_TASK_WHEELLEG_MIT);

    frame->loop_start_ms = wheelleg_tick_ms();
    frame->period_ms = wheelleg_period_ms();
    frame->dt = wheelleg_period_s();
    frame->now_ms = wheelleg_tick_ms();

    angle = get_INS_angle_point();
    gyro = get_gyro_data_point();
    frame->gyro_wheelleg = wheelleg_imu_gyro_aligned(gyro, frame->gyro_aligned);
    frame->pitch = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_PITCH_OFFSET, WHEELLEG_IMU_PITCH_SIGN);
    frame->roll = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_ROLL_OFFSET, WHEELLEG_IMU_ROLL_SIGN);
    frame->yaw = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_YAW_OFFSET, WHEELLEG_IMU_YAW_SIGN);

    frame->vx_axis = input_axis(INPUT_AXIS_CHASSIS_X);
    frame->yaw_axis = input_axis(INPUT_AXIS_CHASSIS_WZ);
    frame->single_test_axis = input_axis(INPUT_AXIS_CALIB_3);
    frame->leg_axis = input_axis(INPUT_AXIS_CHASSIS_Y);
    frame->control_stage = wheelleg_control_stage_from_config();

    frame->target_leg = 0.100f;
    frame->target_foot_y = 0.100f;
    frame->profile_on = robot_profile_is_wheelleg_mit();
    frame->manual_on = wheelleg_manual_enabled_by_switch();
    frame->imu_ok =
        (toe_is_error(BOARD_GYRO_TOE) == 0u && toe_is_error(BOARD_ACCEL_TOE) == 0u) ? 1u : 0u;
    frame->single_test = wheelleg_single_test_enabled();
    frame->left_leg_test = robot_mode_wheelleg_left_leg_swing();
    frame->foot_test = robot_mode_wheelleg_foot_trajectory();
    frame->operation_test_active =
        (frame->single_test != 0u || frame->left_leg_test != 0u || frame->foot_test != 0u) ? 1u : 0u;

    wheelleg_configure_actuators();
    wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT], &g_config.wheelleg_mit.leg_length_pid);
    wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT], &g_config.wheelleg_mit.leg_length_pid);
    wheelleg_pid_apply(&s_wheelleg.split_pid, &g_config.wheelleg_mit.leg_split_pid);
    wheelleg_control_stage_update(frame->control_stage);

    frame->feedback_faults = wheelleg_update_feedback(frame->now_ms);
    s_wheelleg.feedback_faults = frame->feedback_faults;

    if (toe_is_error(DBUS_TOE))
    {
        frame->faults |= WHEELLEG_FAULT_MANUAL_OFFLINE;
    }
    if (frame->operation_test_active == 0u)
    {
        if (frame->imu_ok == 0u)
        {
            frame->faults |= WHEELLEG_FAULT_IMU_OFFLINE;
        }
        if (wheelleg_abs(frame->pitch) > g_config.wheelleg_mit.attitude_limit_rad ||
            wheelleg_abs(frame->roll) > g_config.wheelleg_mit.attitude_limit_rad)
        {
            frame->faults |= WHEELLEG_FAULT_ATTITUDE_LIMIT;
        }
    }

    if (frame->profile_on != 0u &&
        frame->feedback_faults == WHEELLEG_FAULT_NONE)
    {
        frame->kinematics_ok = wheelleg_update_leg_kinematics((frame->imu_ok != 0u) ? frame->pitch : 0.0f,
                                                              (frame->imu_ok != 0u) ? frame->gyro_wheelleg : NULL,
                                                              frame->dt);
    }

    frame->enabled = (uint8_t)(frame->profile_on != 0u &&
                               frame->manual_on != 0u &&
                               frame->faults == WHEELLEG_FAULT_NONE);
    s_wheelleg.last_mode = s_wheelleg.mode;
}

static void wheelleg_task_finish_frame(const wheelleg_task_frame_t *frame,
                                       uint16_t publish_faults,
                                       uint64_t profiler_start_us,
                                       TickType_t *last_wake,
                                       uint8_t count_overrun)
{
    if (frame == NULL || last_wake == NULL)
    {
        return;
    }

    wheelleg_publish(publish_faults,
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
        (uint32_t)(wheelleg_tick_ms() - frame->loop_start_ms) > frame->period_ms)
    {
        s_wheelleg.overrun_count++;
    }
    rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
    vTaskDelayUntil(last_wake, pdMS_TO_TICKS(frame->period_ms));
}

static void wheelleg_handle_disabled_frame(wheelleg_task_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    s_wheelleg.mode = (frame->faults == WHEELLEG_FAULT_NONE) ? WHEELLEG_MODE_DISABLED : WHEELLEG_MODE_FAULT;
    if (frame->profile_on != 0u)
    {
        wheelleg_clear_all_control_cmds();
    }
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    wheelleg_pid_clear(&s_wheelleg.split_pid);
    wheelleg_balance_state_reset();
    s_wheelleg.yaw_inited = 0u;
    wheelleg_clear_test_runtime_flags();
    frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
    frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
    frame->controller_active = 0u;
}

static void wheelleg_operation_test_reset_common(void)
{
    wheelleg_balance_state_reset();
    s_wheelleg.detached_test_active = 0u;
}

static void wheelleg_mark_test_result(wheelleg_task_frame_t *frame, uint8_t ok)
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

static uint8_t wheelleg_run_operation_test(wheelleg_task_frame_t *frame)
{
    if (frame == NULL)
    {
        return 0u;
    }

    if (frame->single_test == 0u)
    {
        wheelleg_clear_state_cmd(wheelleg_single_test_actuator());
    }

    if (frame->single_test != 0u)
    {
        wheelleg_operation_test_reset_common();
        s_wheelleg.left_test_active = 0u;
        s_wheelleg.foot_test_active = 0u;
        s_wheelleg.foot_test_phase = 0u;
        s_wheelleg.foot_test_ik_ok = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        wheelleg_mark_test_result(frame, wheelleg_single_test_apply(frame->single_test_axis));
        return 1u;
    }

    if (frame->left_leg_test != 0u)
    {
        wheelleg_operation_test_reset_common();
        s_wheelleg.foot_test_active = 0u;
        s_wheelleg.foot_test_phase = 0u;
        s_wheelleg.foot_test_ik_ok = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        wheelleg_mark_test_result(frame, wheelleg_left_leg_test_apply(frame->now_ms));
        return 1u;
    }

    if (frame->foot_test != 0u)
    {
        wheelleg_operation_test_reset_common();
        s_wheelleg.left_test_active = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
        wheelleg_mark_test_result(frame, wheelleg_foot_test_apply(frame->now_ms, frame->pitch));
        return 1u;
    }

    return 0u;
}

static void wheelleg_prepare_balance_frame(wheelleg_task_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    wheelleg_clear_test_runtime_flags();
    s_wheelleg.mode = wheelleg_mode_from_control_stage(frame->control_stage);
    frame->target_v = wheelleg_axis_to_fp32(frame->vx_axis,
                                            g_config.wheelleg_mit.max_v_mps,
                                            g_config.wheelleg_mit.rc_deadband);
    frame->target_yaw_rate = -wheelleg_axis_to_fp32(frame->yaw_axis,
                                                    g_config.wheelleg_mit.max_yaw_rate_radps,
                                                    g_config.wheelleg_mit.rc_deadband);
}

static void wheelleg_run_bench_stage(wheelleg_task_frame_t *frame)
{
    uint8_t joint_hold_ok;
    uint8_t wheel_lqr_ok = 0u;

    if (frame == NULL)
    {
        return;
    }

    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    wheelleg_pid_clear(&s_wheelleg.split_pid);
    joint_hold_ok = wheelleg_bench_hold_apply_joints();
    frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
    frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;

    if ((frame->faults & WHEELLEG_FAULT_CONTROLLER) == 0u &&
        frame->feedback_faults == WHEELLEG_FAULT_NONE)
    {
        if (frame->kinematics_ok != 0u)
        {
            wheelleg_bench_hold_update_pose_target();
            frame->target_leg = s_wheelleg.bench_hold_target_leg_m;
            frame->target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
            wheelleg_update_observer(frame->dt, frame->gyro_wheelleg);
            wheelleg_balance_idle_reset(frame->vx_axis, frame->yaw_axis, frame->yaw);
            wheelleg_pitch_trim_update(frame->dt,
                                       frame->vx_axis,
                                       frame->yaw_axis,
                                       frame->target_v,
                                       frame->target_yaw_rate);
            if (wheelleg_bench_lqr_step(frame->dt,
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
        wheelleg_send_wheel_torques(frame->wheel_torque);
    }
}

static void wheelleg_run_position_stage(wheelleg_task_frame_t *frame)
{
    uint8_t position_ok = 0u;
    uint8_t wheel_lqr_ok = 0u;

    if (frame == NULL)
    {
        return;
    }

    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    wheelleg_pid_clear(&s_wheelleg.split_pid);
    wheelleg_clear_leg_virtual_outputs();
    frame->target_foot_x = wheelleg_manual_x_target_from_axis(0);
    frame->target_foot_y = wheelleg_manual_y_target_from_axis(frame->leg_axis);
    if (wheelleg_limit_foot_xy(&frame->target_foot_x, &frame->target_foot_y, &frame->target_leg) == 0u)
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
        wheelleg_target_smooth_update_xy(frame->target_foot_x, frame->target_foot_y, frame->dt);
        frame->target_leg = s_wheelleg.target_leg_smooth;
        frame->target_foot_x = s_wheelleg.target_foot_x_smooth;
        frame->target_foot_y = s_wheelleg.target_foot_y_smooth;
        if (wheelleg_manual_position_apply(frame->target_foot_x, frame->target_foot_y) != 0u)
        {
            position_ok = 1u;
            wheelleg_update_observer(frame->dt, frame->gyro_wheelleg);
            wheelleg_balance_idle_reset(frame->vx_axis, frame->yaw_axis, frame->yaw);
            wheelleg_pitch_trim_update(frame->dt,
                                       frame->vx_axis,
                                       frame->yaw_axis,
                                       frame->target_v,
                                       frame->target_yaw_rate);
            if (wheelleg_bench_lqr_step(frame->dt,
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
        wheelleg_send_wheel_torques(frame->wheel_torque);
    }
}

static void wheelleg_run_vmc_stage(wheelleg_task_frame_t *frame)
{
    const uint8_t use_lqr_hip =
        (frame != NULL && frame->control_stage == WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR) ? 1u : 0u;

    if (frame == NULL)
    {
        return;
    }

    frame->target_foot_x = wheelleg_manual_x_target_from_axis(0);
    frame->target_foot_y = wheelleg_manual_y_target_from_axis(frame->leg_axis);
    if (wheelleg_limit_foot_xy(&frame->target_foot_x, &frame->target_foot_y, &frame->target_leg) == 0u)
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
        wheelleg_target_smooth_update_xy(frame->target_foot_x, frame->target_foot_y, frame->dt);
        frame->target_leg = s_wheelleg.target_leg_smooth;
        frame->target_foot_x = s_wheelleg.target_foot_x_smooth;
        frame->target_foot_y = s_wheelleg.target_foot_y_smooth;
        wheelleg_update_observer(frame->dt, frame->gyro_wheelleg);
        wheelleg_balance_idle_reset(frame->vx_axis, frame->yaw_axis, frame->yaw);
        wheelleg_pitch_trim_update(frame->dt,
                                   frame->vx_axis,
                                   frame->yaw_axis,
                                   frame->target_v,
                                   frame->target_yaw_rate);
        if (wheelleg_controller_step(frame->dt,
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
            wheelleg_send_wheel_torques(frame->wheel_torque);
            wheelleg_send_vmc_joint_torques();
        }
        else
        {
            frame->faults |= WHEELLEG_FAULT_CONTROLLER;
        }
    }
}

static void wheelleg_run_balance_frame(wheelleg_task_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    wheelleg_prepare_balance_frame(frame);
    if (frame->control_stage == WHEELLEG_CONTROL_STAGE_BENCH_LQR)
    {
        wheelleg_run_bench_stage(frame);
    }
    else if (frame->control_stage == WHEELLEG_CONTROL_STAGE_POSITION_LQR)
    {
        wheelleg_run_position_stage(frame);
    }
    else
    {
        wheelleg_run_vmc_stage(frame);
    }

    if (frame->faults != WHEELLEG_FAULT_NONE)
    {
        s_wheelleg.mode = WHEELLEG_MODE_FAULT;
        wheelleg_target_smooth_reset();
        wheelleg_clear_all_control_cmds();
        frame->controller_active = 0u;
    }
    else if (frame->controller_active != 0u)
    {
        s_wheelleg.ever_commanded = 1u;
    }
}
