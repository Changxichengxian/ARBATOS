/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by watch.c.
 */

static void watch_copy_diag(void)
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
        wheelleg_debug_t wheelleg_debug = {0};
        const uint8_t wheelleg_debug_valid =
            (robot_profile_is_wheelleg_mit() != 0u) ? wheelleg_debug_read(&wheelleg_debug) : 0u;

        g_watch.diag.live_profile_on = g_watch.wheelleg_mit.profile_on;
        g_watch.diag.live_manual_on = g_watch.wheelleg_mit.manual_on;
        g_watch.diag.live_enabled = g_watch.wheelleg_mit.enabled;
        g_watch.diag.live_control_stage = (uint8_t)g_config.wheelleg_mit.control_stage;
        g_watch.diag.live_mode = g_watch.wheelleg_mit.mode;
        g_watch.diag.live_status_valid = g_watch.wheelleg_mit.status_valid;
        g_watch.diag.live_state_valid = g_watch.wheelleg_mit.state_valid;
        g_watch.diag.live_fault_flags = g_watch.wheelleg_mit.fault_flags;
        g_watch.diag.live_lqr_debug_valid = wheelleg_debug_valid;
        g_watch.diag.live_all_mit_online =
            (uint8_t)((g_watch.wheelleg_mit.joint[0].online != 0u &&
                       g_watch.wheelleg_mit.joint[1].online != 0u &&
                       g_watch.wheelleg_mit.joint[2].online != 0u &&
                       g_watch.wheelleg_mit.joint[3].online != 0u &&
                       g_watch.wheelleg_mit.wheel[0].online != 0u &&
                       g_watch.wheelleg_mit.wheel[1].online != 0u)
                          ? 1u
                          : 0u);

        g_watch.diag.lqr_wheel_scale = g_config.wheelleg_mit.lqr_wheel_torque_scale;
        g_watch.diag.lqr_hip_scale = g_config.wheelleg_mit.lqr_hip_torque_scale;
        g_watch.diag.max_wheel_torque_nm = g_config.wheelleg_mit.max_wheel_torque_nm;
        g_watch.diag.max_joint_torque_nm = g_config.wheelleg_mit.max_joint_torque_nm;

        g_watch.diag.target_v_mps = g_watch.wheelleg_mit.target_v_mps;
        g_watch.diag.target_yaw_rate_radps = g_watch.wheelleg_mit.target_yaw_rate_radps;
        g_watch.diag.target_leg_length_m = g_watch.wheelleg_mit.target_leg_length_m;
        g_watch.diag.target_foot_x_m = g_watch.wheelleg_mit.target_foot_x_m;
        g_watch.diag.target_foot_y_m = g_watch.wheelleg_mit.target_foot_y_m;
        g_watch.diag.target_leg_theta_deg = g_watch.wheelleg_mit.target_leg_theta_deg;

        if (g_watch.wheelleg_mit.status_valid != 0u)
        {
            g_watch.diag.pitch_deg = g_watch.wheelleg_mit.pitch_deg;
        }
        g_watch.diag.x_dot_mps = g_watch.wheelleg_mit.x_dot_mps;
        g_watch.diag.left_leg_length_m = g_watch.wheelleg_mit.leg[0].length_m;
        g_watch.diag.right_leg_length_m = g_watch.wheelleg_mit.leg[1].length_m;
        g_watch.diag.left_leg_theta_deg = g_watch.wheelleg_mit.leg[0].theta_deg;
        g_watch.diag.right_leg_theta_deg = g_watch.wheelleg_mit.leg[1].theta_deg;
        g_watch.diag.left_front_rel_deg = g_watch.wheelleg_mit.joint[0].rel_position_deg;
        g_watch.diag.left_back_rel_deg = g_watch.wheelleg_mit.joint[1].rel_position_deg;
        g_watch.diag.right_front_rel_deg = g_watch.wheelleg_mit.joint[2].rel_position_deg;
        g_watch.diag.right_back_rel_deg = g_watch.wheelleg_mit.joint[3].rel_position_deg;
        g_watch.diag.left_wheel_vel_deg_s = g_watch.wheelleg_mit.wheel[0].fb_velocity_deg_s;
        g_watch.diag.right_wheel_vel_deg_s = g_watch.wheelleg_mit.wheel[1].fb_velocity_deg_s;
        g_watch.diag.left_wheel_cmd_torque_nm = g_watch.wheelleg_mit.wheel[0].cmd_torque_nm;
        g_watch.diag.right_wheel_cmd_torque_nm = g_watch.wheelleg_mit.wheel[1].cmd_torque_nm;
        g_watch.diag.wheel_balance_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm - g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.wheel_turn_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm + g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.left_wheel_fb_torque_nm = g_watch.wheelleg_mit.wheel[0].fb_torque_nm;
        g_watch.diag.right_wheel_fb_torque_nm = g_watch.wheelleg_mit.wheel[1].fb_torque_nm;
        g_watch.diag.left_support_force_n = g_watch.wheelleg_mit.leg[0].support_force_n;
        g_watch.diag.right_support_force_n = g_watch.wheelleg_mit.leg[1].support_force_n;
        g_watch.diag.lqr_pitch_err_deg =
            g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_right_rad);
        g_watch.diag.lqr_left_pitch_err_deg =
            -g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_left_rad);

        if (wheelleg_debug_valid != 0u)
        {
            g_watch.diag.left_hip_torque_nm = wheelleg_debug.lqr.output[3];
            g_watch.diag.right_hip_torque_nm = wheelleg_debug.lqr.output[2];
            g_watch.diag.lqr_theta_err_deg = watch_rad_to_deg(wheelleg_debug.lqr.error[0]);
            g_watch.diag.lqr_dtheta_deg_s = watch_rad_to_deg(wheelleg_debug.lqr.error[1]);
            g_watch.diag.lqr_x_m = wheelleg_debug.lqr.error[2];
            g_watch.diag.lqr_v_err_mps = wheelleg_debug.lqr.error[3];
            g_watch.diag.lqr_pitch_err_deg = watch_rad_to_deg(wheelleg_debug.lqr.error[4]);
            g_watch.diag.lqr_left_pitch_err_deg =
                -g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_left_rad);
            g_watch.diag.lqr_pitch_gyro_deg_s = watch_rad_to_deg(wheelleg_debug.lqr.error[5]);
            g_watch.diag.lqr_right_output_nm = wheelleg_debug.lqr.output[0];
            g_watch.diag.lqr_left_output_nm = wheelleg_debug.lqr.output[1];
        }

        watch_wheelleg_run_capture_update();
        watch_wheelleg_run_capture_copy();
    }
#endif
#endif
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared watch can link even if a target doesn't export them.
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
__weak uint32_t gimbal_high_water = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
__weak uint32_t chassis_high_water = 0u;
#endif
__weak uint32_t detect_task_stack = 0u;
__weak uint32_t calibrate_task_stack = 0u;
#endif

static void watch_copy_rtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_watch.rtos.heap_free = heap_get_free();
    g_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_watch.rtos.watch_update_count++;
    g_watch.rtos.watch_update_tick_ms = HAL_GetTick();
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
    g_watch.rtos.stack_gimbal = gimbal_high_water;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = chassis_high_water;
#endif
    g_watch.rtos.stack_detect = detect_task_stack;
    g_watch.rtos.stack_calibrate = calibrate_task_stack;
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
