// C 板经典控制初值源自 INFANTRY-TIAN-A，须按新车确认 PID、几何、方向和零位。


    .gimbal =
        {
            .yaw_speed_pid = {5156.620f, 0.0f, 0.0f, 16384.0f, 1000.0f},


            .pitch_speed_pid = {2400.0f, 0.0f, 0.0f, 16384.0f, 1000.0f},
            .yaw_encode_angle_pid = {10.0f, 0.0f, 0.0f, 40.0f, 5.0f},
            .pitch_encode_angle_pid = {12.0f, 0.0f, 0.0f, 15.0f, 0.0f},

            .task_init_time_ms = 201,
            .control_period_ms = 1,

            .channel_yaw = 0,
            .channel_pitch = 1,
            .channel_mode = 0,

            .yaw_rc_sen = 0.000014544410f,
            .pitch_rc_sen = 0.000007272205f,
            .yaw_mouse_sen = 0.00005f,
            .pitch_mouse_sen = 0.00015f,
            .yaw_encode_sen = 0.01f,
            .pitch_encode_sen = 0.01f,
            .rc_deadband = 10,

            .init_angle_error = 0.1f,
            .init_stop_time_ms = 100,
            .init_time_ms = 6000,
            .init_pitch_speed = 0.004f,
            .init_yaw_speed = 0.005f,
            .init_pitch_set = 0.0f,
            .init_yaw_set = 0.0f,

            .yaw_middle_ecd = 0,
            .pitch_middle_ecd = 0,
            .pitch_middle_ecd_enable = 0u,
            .yaw_spin_ff_gain = 0.0f,
            .yaw_spin_ff_limit = 9000.0f,
            .pitch_kick_up_current = 0.0f,
            .pitch_kick_down_current = 0.0f,

            .pitch_soft_limit_up = 0.35f,
            .pitch_soft_limit_down = -0.35f,
            .pitch_current_limit = 16384.0f,


            .pitch_cali =
                {
                    .enable = 0u,
                    .angle_points = 11u,
                    .bullet_points = 1u,
                    .bullet_source = (uint8_t)PITCH_CALI_BULLET_SRC_REFEREE,
                    .bullet_min = 0u,
                    .bullet_max = 50u,
                    .bullet_manual = 0u,
                    .angle_margin = 0.05f,
                    .stable_angle_err = 0.01f,
                    .stable_gyro_err = 0.05f,
                    .stable_time_ms = 300u,
                    .seek_k = 0.002f,
                    .hold_avg_time_ms = 200u,
                    .breakaway_step_current = 50u,
                    .breakaway_step_period_ms = 10u,
                    .breakaway_max_extra_current = 4000u,
                    .breakaway_gyro_threshold = 0.3f,
                    .breakaway_angle_threshold = 0.01f,
                    .recover_time_ms = 200u,
                },

            .half_ecd_range = 4096,
            .full_ecd_range = 8191,
            .motor_ecd_to_rad = 0.000766990394f,

            .cali_redundant_angle = 0.1f,
            .cali_motor_set = 8000,
            .cali_step_time_ms = 2000,
            .cali_gyro_limit = 0.1f,
            .cali_pitch_max_step = 1,
            .cali_pitch_min_step = 2,
            .cali_yaw_max_step = 3,
            .cali_yaw_min_step = 4,
            .cali_start_step = 1,
            .cali_end_step = 5,

            .motionless_rc_deadline = 10.0f,
            .motionless_time_max_ms = 3000,

            .turn_speed = 0.04f,
            .turn_key_mask = 1u << 9,
            .test_key_mask = 1u << 8,

            .yaw_turn = 0,
            .pitch_turn = 1,
        },


    .chassis =
        {

            .motor_speed_pid = {14429.676f, 0.0f, 0.0f, 10000.0f, 0.0f},




            .follow_gimbal_pid = {-3.5f, 0.0f, 1.0f, 2.0f, 0.2f},
            .motor_dir = {1, 1, 1, 1},
            .wheel_type = CHASSIS_WHEEL_TYPE_XDRIVE,

            .task_init_time_ms = 357,
            .control_period_ms = 2,

            .channel_vx = 3,
            .channel_vy = 2,
            .channel_wz = 0,
            .channel_mode = 0,

            .vx_rc_sen = 0.006f,
            .vy_rc_sen = -0.005f,
            .angle_z_rc_sen = 0.000002f,
            .wz_rc_sen = 0.0051f,
            .accel_x_first_order = 0.1666666667f,
            .accel_y_first_order = 0.3333333333f,
            .rc_deadband = 10,

            .motor_speed_to_chassis_vx = 0.25f,
            .motor_speed_to_chassis_vy = 0.25f,
            .motor_speed_to_chassis_wz = 0.25f,
            .motor_distance_to_center = 0.395f,
            .rpm_to_vector = 0.000415809748903494517209f,

            .max_wheel_speed = 4.0f,
            .max_vx_forward = 2.0f,
            .max_vx_backward = 2.0f,
            .max_vy_left = 1.5f,
            .max_vy_right = 1.5f,

            .wz_set_scale = 0.1f,
            .swing_no_move_angle = 3.5f,
            .swing_move_angle = 3.5f,

            .max_motor_can_current = 10000.0f,

            .swing_key_mask = 1u << 5,
            .swing_mode_key_mask = 1u << 10,
            .gyro_spin_var_key_mask = 1u << 15,
            .spinSineRc = 1u,
            .spinSinePeriodMs = 1500u,
            .spinSineMinScale = 0.65f,
            .swing_amp_rad = 0.2617993878f,
            .swing_half_period_ms = 300,
            .swing_center_hold_min_ms = 5000,
            .swing_center_hold_max_ms = 20000,
            .key_front_mask = 1u << 0,
            .key_back_mask = 1u << 1,
            .key_left_mask = 1u << 2,
            .key_right_mask = 1u << 3,
        },

    .WheelLegMit = {0},


    .shoot =
        {
            .fric_speed_rpm = 4000.0f,
            .fric_speed_off_rpm = 0.0f,
            .fric_speed_step_rpm_s = 20000.0f,
            .fric_ready_ratio = 0.90f,
            .fric_speed_pid = {9.0f, 0.0f, 0.0f, 13000.0f, 0.0f},
            .fric_motor_dir = {1, -1, 0, 0},

            .rc_mode_channel = 1,
            .control_period_ms = 1,

            .key_on_mask = 0u,
            .key_off_mask = 0u,

            .ShootDoneKeyOffTimeMs = 15,
            .press_long_time_ms = 400,
            .rc_s_long_time_ms = 2000,
            .up_add_time_ms = 100,

            .half_ecd_range = 4096,
            .full_ecd_range = 8191,

            .motor_rpm_to_speed = 0.00290888208665721596153948461415f,
            .motor_ecd_to_angle = 0.000021305288720633905968306772076277f,
            .full_count = 18,

            .trigger_speed_single = 3.0f,
            .trigger_speed_continuous = 4.0f,
            .trigger_speed_ready = 3.0f,

            .key_off_judge_time_ms = 500,
            .switch_trigger_on = 0,
            .switch_trigger_off = 1,

            .block_trigger_speed = 1.0f,
            .block_time_ms = 700,
            .reverse_time_ms = 500,
            .reverse_speed_limit = 13.0f,

            .pi_over_four = 0.78539816339744830961566084581988f,
            .pi_over_ten = 0.314f,

            .trigger_angle_pid = {800.0f, 0.5f, 0.0f, 10000.0f, 7000.0f},
            .trigger_bullet_pid_max_out = 10000.0f,
            .trigger_bullet_pid_max_iout = 9000.0f,
            .trigger_ready_pid_max_out = 10000.0f,
            .trigger_ready_pid_max_iout = 7000.0f,

            .heat_remain_value = 80,
        },


    .ArmJ0Unitree = {0},

    .power =
        {
            .limitMode = ChassisPowerLimitAuto,
            .power_limit = 80.0f,
            .warning_power = 40.0f,
            .warning_power_buffer = 50.0f,


            .no_judge_total_current_limit = 36000.0f,
            .buffer_total_current_limit = 16000.0f,
            .power_total_current_limit = 20000.0f,
        },


    .imu =
        {
            .fusion_mode = IMU_FUSION_MAHONY_6AXIS,
            .temperature_pid = {1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f},
            .temperature_pid_max_out = 4500.0f,
            .temperature_pid_max_iout = 4400.0f,
            .imu_temp_pwm_max = 5000,
            .task_init_time_ms = 7,
        },


    .voltage =
        {
            .full_battery_voltage = 25.2f,
            .low_battery_voltage = 21.6f,
            .voltage_drop = 0.0f,
            .lowAlarmDelayMs = 3000u,
            .lowAlarmEnabled = 0u,
        },


    .buzzer =
    {
        .pcm =
        {
            .carrier_min_hz = 48000u,
            .sample_rate_hz = 12000u,
            .retry_ms = 500u,
            .volume = 255u,
            .loop = 1u,
            .gain_q8 = 1024u,
        },
        .soft_beep_psc = 1,
        .soft_beep_duration_ms = 80,
        .enable = 1,

        .GimbalWarnPsc = 31,
        .GimbalWarnPwm = 20000,

        .imu_cali_psc = 95,
        .imu_cali_pwm = 10000,
        .GimbalCaliPsc = 31,
        .GimbalCaliPwm = 19999,

        .rc_cali_middle_time_ms = 10000,
        .rc_cali_start_time_ms = 0,
        .rc_cali_cycle_time_ms = 400,
        .rc_cali_pause_time_ms = 200,
        .rc_cmd_long_time_ms = 2000,
    },


    .led =
        {
            .slot_on_ms = 300,
            .slot_off_ms = 200,
            .slot_gap_ms = 1000,
        },
