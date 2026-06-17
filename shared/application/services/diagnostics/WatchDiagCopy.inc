/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

static void WatchCopyDiag(void)
{
    memset(&g_watch.diag, 0, sizeof(g_watch.diag));

#if WATCH_ENABLE_DIAG_COPY
    SdLogStats sd_stats = {0};

    g_watch.diag.offline_need_geometry = 1u;
    g_watch.diag.offline_need_mass_inertia = 1u;
    g_watch.diag.offline_need_com_inertia = 1u;
    g_watch.diag.offline_need_motor_limits = 0u;
    g_watch.diag.offline_need_lqr_model = 1u;
    g_watch.diag.offline_need_stage_k = 1u;
    g_watch.diag.offline_need_qr_weights = 1u;
    g_watch.diag.offline_need_sign_table = 0u;

    g_watch.diag.power_need_imu_sign = 0u;
    g_watch.diag.power_need_motor_online = 0u;
    g_watch.diag.power_need_joint_zero_dir = 0u;
    g_watch.diag.power_need_wheel_dir = 0u;
    g_watch.diag.power_need_fk_leg_state = 1u;
    g_watch.diag.power_need_jacobian_dir = 1u;
    g_watch.diag.power_need_observer_speed = 1u;
    g_watch.diag.power_need_lqr_response = 1u;

    SdLogGetStats(&sd_stats);
    g_watch.diag.live_sdlog_active = sd_stats.active;
    g_watch.diag.live_operation_mode = (uint8_t)robot_mode_current();
    g_watch.diag.live_imu_online =
        (uint8_t)((toe_is_error(BOARD_GYRO_TOE) == 0u && toe_is_error(BOARD_ACCEL_TOE) == 0u) ? 1u : 0u);
    g_watch.diag.pitch_deg = g_watch.imu.angle_deg[INS_PITCH_ADDRESS_OFFSET];
    g_watch.diag.pitch_gyro_deg_s = g_watch.imu.gyro_dps[INS_GYRO_Y_ADDRESS_OFFSET];

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    {
        WheelLegDebug debug = {0};
        const uint8_t WheelLegDebugValid =
            (RobotProfileIsWheelLegMit() != 0u) ? WheelLegDebugRead(&debug) : 0u;

        g_watch.diag.live_profile_on = g_watch.WheelLegMit.profile_on;
        g_watch.diag.live_manual_on = g_watch.WheelLegMit.manual_on;
        g_watch.diag.live_enabled = g_watch.WheelLegMit.enabled;
        g_watch.diag.live_control_stage = (uint8_t)g_config.WheelLegMit.control_stage;
        g_watch.diag.live_mode = g_watch.WheelLegMit.mode;
        g_watch.diag.live_status_valid = g_watch.WheelLegMit.status_valid;
        g_watch.diag.live_state_valid = g_watch.WheelLegMit.state_valid;
        g_watch.diag.live_fault_flags = g_watch.WheelLegMit.fault_flags;
        g_watch.diag.live_lqr_debug_valid = WheelLegDebugValid;
        g_watch.diag.live_all_mit_online =
            (uint8_t)((g_watch.WheelLegMit.joint[0].online != 0u &&
                       g_watch.WheelLegMit.joint[1].online != 0u &&
                       g_watch.WheelLegMit.joint[2].online != 0u &&
                       g_watch.WheelLegMit.joint[3].online != 0u &&
                       g_watch.WheelLegMit.wheel[0].online != 0u &&
                       g_watch.WheelLegMit.wheel[1].online != 0u)
                          ? 1u
                          : 0u);

        g_watch.diag.lqr_wheel_scale = g_config.WheelLegMit.lqr_wheel_torque_scale;
        g_watch.diag.lqr_hip_scale = g_config.WheelLegMit.lqr_hip_torque_scale;
        g_watch.diag.max_wheel_torque_nm = g_config.WheelLegMit.max_wheel_torque_nm;
        g_watch.diag.max_joint_torque_nm = g_config.WheelLegMit.max_joint_torque_nm;

        g_watch.diag.target_v_mps = g_watch.WheelLegMit.target_v_mps;
        g_watch.diag.target_yaw_rate_radps = g_watch.WheelLegMit.target_yaw_rate_radps;
        g_watch.diag.target_leg_length_m = g_watch.WheelLegMit.target_leg_length_m;
        g_watch.diag.target_foot_x_m = g_watch.WheelLegMit.target_foot_x_m;
        g_watch.diag.target_foot_y_m = g_watch.WheelLegMit.target_foot_y_m;
        g_watch.diag.target_leg_theta_deg = g_watch.WheelLegMit.target_leg_theta_deg;

        if (g_watch.WheelLegMit.status_valid != 0u)
        {
            g_watch.diag.pitch_deg = g_watch.WheelLegMit.pitch_deg;
        }
        g_watch.diag.x_dot_mps = g_watch.WheelLegMit.x_dot_mps;
        g_watch.diag.left_leg_length_m = g_watch.WheelLegMit.leg[0].length_m;
        g_watch.diag.right_leg_length_m = g_watch.WheelLegMit.leg[1].length_m;
        g_watch.diag.left_leg_theta_deg = g_watch.WheelLegMit.leg[0].theta_deg;
        g_watch.diag.right_leg_theta_deg = g_watch.WheelLegMit.leg[1].theta_deg;
        g_watch.diag.left_front_rel_deg = g_watch.WheelLegMit.joint[0].rel_position_deg;
        g_watch.diag.left_back_rel_deg = g_watch.WheelLegMit.joint[1].rel_position_deg;
        g_watch.diag.right_front_rel_deg = g_watch.WheelLegMit.joint[2].rel_position_deg;
        g_watch.diag.right_back_rel_deg = g_watch.WheelLegMit.joint[3].rel_position_deg;
        g_watch.diag.left_wheel_vel_deg_s = g_watch.WheelLegMit.wheel[0].fb_velocity_deg_s;
        g_watch.diag.right_wheel_vel_deg_s = g_watch.WheelLegMit.wheel[1].fb_velocity_deg_s;
        g_watch.diag.left_wheel_cmd_torque_nm = g_watch.WheelLegMit.wheel[0].cmd_torque_nm;
        g_watch.diag.right_wheel_cmd_torque_nm = g_watch.WheelLegMit.wheel[1].cmd_torque_nm;
        g_watch.diag.wheel_balance_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm - g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.wheel_turn_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm + g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.left_wheel_fb_torque_nm = g_watch.WheelLegMit.wheel[0].fb_torque_nm;
        g_watch.diag.right_wheel_fb_torque_nm = g_watch.WheelLegMit.wheel[1].fb_torque_nm;
        g_watch.diag.left_support_force_n = g_watch.WheelLegMit.leg[0].support_force_n;
        g_watch.diag.right_support_force_n = g_watch.WheelLegMit.leg[1].support_force_n;
        g_watch.diag.lqr_pitch_err_deg =
            g_watch.diag.pitch_deg - WatchRadToDeg(g_config.WheelLegMit.pitch_balance_offset_right_rad);
        g_watch.diag.lqr_left_pitch_err_deg =
            -g_watch.diag.pitch_deg - WatchRadToDeg(g_config.WheelLegMit.pitch_balance_offset_left_rad);

        if (WheelLegDebugValid != 0u)
        {
            g_watch.diag.left_hip_torque_nm = debug.lqr.output[3];
            g_watch.diag.right_hip_torque_nm = debug.lqr.output[2];
            g_watch.diag.lqr_theta_err_deg = WatchRadToDeg(debug.lqr.error[0]);
            g_watch.diag.lqr_dtheta_deg_s = WatchRadToDeg(debug.lqr.error[1]);
            g_watch.diag.lqr_x_m = debug.lqr.error[2];
            g_watch.diag.lqr_v_err_mps = debug.lqr.error[3];
            g_watch.diag.lqr_pitch_err_deg = WatchRadToDeg(debug.lqr.error[4]);
            g_watch.diag.lqr_left_pitch_err_deg =
                -g_watch.diag.pitch_deg - WatchRadToDeg(g_config.WheelLegMit.pitch_balance_offset_left_rad);
            g_watch.diag.lqr_pitch_gyro_deg_s = WatchRadToDeg(debug.lqr.error[5]);
            g_watch.diag.lqr_right_output_nm = debug.lqr.output[0];
            g_watch.diag.lqr_left_output_nm = debug.lqr.output[1];
        }

        WatchWheelLegRunCaptureUpdate();
        WatchWheelLegRunCaptureCopy();
    }
#endif
#endif
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared watch can link even if a target doesn't export them.
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
__weak uint32_t GimbalHighWater = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
__weak uint32_t ChassisHighWater = 0u;
#endif
__weak uint32_t DetectTaskStack = 0u;
__weak uint32_t CalibrateTaskStack = 0u;
#endif

static void WatchCopyRtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_watch.rtos.heap_free = heap_get_free();
    g_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_watch.rtos.WatchUpdateCount++;
    g_watch.rtos.WatchUpdateTickMs = HAL_GetTick();
    g_watch.rtos.scheduler_state = (uint32_t)xTaskGetSchedulerState();
    g_watch.rtos.task_count = (uint32_t)uxTaskGetNumberOfTasks();
    current_task = xTaskGetCurrentTaskHandle();
    g_watch.rtos.current_task_handle = (uint32_t)current_task;
    current_name = pcTaskGetTaskName(NULL);
    memset(g_watch.rtos.current_task_name, 0, sizeof(g_watch.rtos.current_task_name));
    if (current_name != NULL)
    {
        (void)strncpy(g_watch.rtos.current_task_name, current_name, sizeof(g_watch.rtos.current_task_name) - 1u);
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
    g_watch.rtos.stack_default = uxTaskGetStackHighWaterMark(NULL);
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = GimbalHighWater;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = ChassisHighWater;
#endif
    g_watch.rtos.stack_detect = DetectTaskStack;
    g_watch.rtos.stack_calibrate = CalibrateTaskStack;
#else
    g_watch.rtos.stack_default = 0u;
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = 0u;
#endif
    g_watch.rtos.stack_detect = 0u;
    g_watch.rtos.stack_calibrate = 0u;
#endif
}
