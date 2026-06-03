// Included by config.c inside g_config's initializer; do not compile directly.
    // Wheel-leg MIT config
    .wheelleg_mit =
        {
            .task_init_time_ms = 1000u,
            .control_period_ms = 3u,
            .l1_m = 0.05000f,
            .l2_m = 0.11404f,
            .l3_m = 0.11404f,
            .l4_m = 0.05000f,
            .l5_m = 0.06000f,
            .wheel_radius_m = 0.03275f,
            .lqr_poly =
                {
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
                },
            .support_bias_n = 11.0f,
            .leg_mass_kg = 0.12f,
            .default_leg_length_m = 0.100f,
            .min_leg_length_m = 0.085f,
            .max_leg_length_m = 0.120f,
            .max_wheel_torque_nm = 0.45f,
            .max_joint_torque_nm = 3.0f,
            .max_jump_joint_torque_nm = 3.0f,
            .max_support_force_n = 60.0f,
            .attitude_limit_rad = 0.45f,
            .observer_lpf = 0.18f,
            .leg_length_pid = {280.0f, 0.0f, 1200.0f, 35.0f, 0.0f},
            .leg_split_pid = {30.0f, 0.0f, 1.0f, 2.0f, 0.0f},
            .turn_pid = {0.6f, 0.0f, 0.08f, 0.20f, 0.0f},
            .roll_pid = {25.0f, 0.0f, 3.0f, 15.0f, 0.0f},
            .pitch_balance_offset_right_rad = 0.0f,
            .pitch_balance_offset_left_rad = 0.0f,
            .max_v_mps = 0.45f,
            .max_yaw_rate_radps = 1.20f,
            .rc_deadband = 10u,
            .enable_switch_pos = MANUAL_INPUT_SWITCH_POS_MID,
            .single_test_actuator = (uint8_t)Motor17, // [600] 单电机测试目标：达妙 H3510，CAN1 ID 5
            .single_test_position_rad = 0.0f, // [601] MIT 位置给定；kp=0 时不生效
            .single_test_velocity_radps = 30.0f, // [602] ch3 满杆速度，rad/s；改负数可反向
            .single_test_kp = 0.0f, // [603] 位置刚度，默认关闭位置环
            .single_test_kd = 0.10f, // [604] 速度阻尼
            .single_test_torque_nm = 0.0f, // [605] 单电机测试力矩，默认 0
            .single_test_torque_limit_nm = 0.30f, // [606] 测试力矩限幅
            .left_test_zero_time_ms = 1000u, // [607] 上电后先回 0 并保持
            .left_test_move_time_ms = 1500u, // [608] 每段 90deg 扫动时间
            .left_test_angle_rad = 1.5707963268f, // [609] 90deg
            .left_test_kp = 2.0f, // [610] 位置刚度，测试先给到能明显跟随
            .left_test_kd = 0.16f, // [611] 速度阻尼
            .left_test_torque_ff_nm = 0.0f, // [612] 前馈力矩
            .left_test_torque_limit_nm = 0.30f, // [613] 前馈限幅
            .left_test_front_dir = -1, // [614]
            .left_test_back_dir = 1, // [615]
            .right_test_front_dir = 1, // [616]
            .right_test_back_dir = -1, // [617]
            .left_front_zero_rad = -1.0145340144f, // [618] short-leg reference pose
            .left_back_zero_rad = 0.3362703268f, // [619]
            .right_front_zero_rad = 0.2740898096f, // [620]
            .right_back_zero_rad = -2.0628290601f, // [621]
            .left_front_dir = -1, // [622]
            .left_back_dir = 1, // [623]
            .right_front_dir = 1, // [624]
            .right_back_dir = -1, // [625]
            .foot_test_zero_hold_time_ms = 1000u, // [626] 先回 0 并保持
            .foot_test_extend_time_ms = 4000u, // [627] 从 0 点伸到目标腿长
            .foot_test_swing_time_ms = 3000u, // [628] 足端每段前后移动时间
            .foot_test_return_time_ms = 4000u, // [629] 回到 0 点时间
            .foot_test_length_m = 0.120f, // [630] 伸腿目标 12cm
            .foot_test_forward_m = 0.030f, // [631] 足端前后 3cm
            .foot_test_kp = 2.0f, // [632] 足端测试位置刚度，太小会基本不动
            .foot_test_kd = 0.16f, // [633]
            .foot_test_torque_ff_nm = 0.0f, // [634]
            .foot_test_torque_limit_nm = 0.30f, // [635]
            .foot_test_forward_dir = 1, // [636] 前后方向反了就改成 -1
            .control_stage = 0u, // [637] 0 bench, 1 position+LQR, 2 VMC height, 3 VMC assist
            .lqr_wheel_torque_scale = 0.18f, // [638]
            .lqr_hip_torque_scale = 0.18f, // [639]
            .right_front_actuator = (uint8_t)Motor15,
            .right_back_actuator = (uint8_t)Motor16,
            .right_wheel_actuator = (uint8_t)Motor12,
            .left_front_actuator = (uint8_t)Motor13,
            .left_back_actuator = (uint8_t)Motor14,
            .left_wheel_actuator = (uint8_t)Motor17,
        },

    // Shoot config
    .shoot =
        {
            .fric_speed_rpm = 4500.0f,         // [113] 摩擦轮目标转速（RPM）
            .fric_speed_off_rpm = 0.0f,        // [114] 停止
            .fric_speed_step_rpm_s = 20000.0f, // [115] 转速斜坡（RPM/s）
            .fric_ready_ratio = 0.90f,         // [116] 到速判定比例
            .fric_speed_pid = {8.0f, 0.05f, 0.0f, 16000.0f, 8000.0f}, // [117]kp [118]ki [119]kd [120]max_out [121]max_iout | 速度环 PID（输出电流）
            .fric_motor_dir = {-1, 1, 0, 0},    // [122]0x201 [123]0x202 [124]0x203 [125]0x204 | 四路摩擦轮方向（按实际安装可改为 1/-1/0）

            .rc_mode_channel = 1,   // [126] 射击模式通道
            .control_period_ms = 1, // [127] 控制周期

            .key_on_mask = 1u << 6, // [128] Q
            .key_off_mask = 1u << 7,// [129] E

            .shoot_done_key_off_time_ms = 15,  // [130] 防抖时间：发射后需松键超过该时间才停止
            .press_long_time_ms = 400,         // [131] 长按判定：超过则进入连发逻辑
            .rc_s_long_time_ms = 2000,         // [132] 拨杆长按：用于模式切换
            .up_add_time_ms = 100,              // [133] 斜率时间：摩擦轮爬升时间，越小起转越快但电流大

            .half_ecd_range = 4096,            // 拨盘编码器半范围
            .full_ecd_range = 8191,            // 拨盘编码器全范围
            .motor_rpm_to_speed = 0.00290888208665721596153948461415f, // [136] RPM->速度
            .motor_ecd_to_angle = 0.000021305288720633905968306772076277f, // [137] 编码器->角度
            .full_count = 18,                  // [138] 拨盘计数一圈

            .trigger_speed_single = 3.0f,     // [139] 单发速度：越大单发时间缩短，过大可能卡弹
            .trigger_speed_continuous = 4.0f, // [140] 连发速度：越大射速越高，热量/卡弹风险升
            .trigger_speed_ready = 3.0f,       // [141] 预备速度：小速度防止预备阶段抖动

            .key_off_judge_time_ms = 500,      // [142] 松键判定时间
            .switch_trigger_on = 0,            // [143] 拨杆开火位
            .switch_trigger_off = 1,           // [144] 拨杆停火位

            .block_trigger_speed = 1.0f,       // [145] 卡弹判定速度：低于该值认为卡弹
            .block_time_ms = 700,              // [146] 卡弹时间：持续低速超过此时间触发反转
            .reverse_time_ms = 500,            // [147] 反转时间：反转持续时长
            .reverse_speed_limit = 13.0f,      // [148] 反转限速：防止反转过猛

            .pi_over_four = 0.78539816339744830961566084581988f, // [149] π/4
            .pi_over_ten = 0.314f,                                 // [150] π/10

            .trigger_angle_pid = {800.0f, 0.5f, 0.0f, 10000.0f, 7000.0f}, // [151]kp [152]ki [153]kd | 拨盘角度 PID：kp 增大定位更硬，ki 增大消除跟随误差
            .trigger_bullet_pid_max_out = 10000.0f,   // [156] 发射 PID 上限：限制输出力矩
            .trigger_bullet_pid_max_iout = 9000.0f,   // [157] 发射 PID 积分上限：防止积分过饱和
            .trigger_ready_pid_max_out = 10000.0f,    // [158] 预备 PID 上限
            .trigger_ready_pid_max_iout = 7000.0f,    // [159] 预备 PID 积分上限

            .heat_remain_value = 80,           // [160] 发热余量：裁判热量低于此值禁射
        },

    // 功率配置
    .power =
        {
            .power_limit = 80.0f,                  // [161] 功率上限：裁判功率阈值，越大越易超限
            .warning_power = 40.0f,                // [162] 告警功率：超过开始线性降流
            .warning_power_buffer = 50.0f,         // [163] 缓冲阈值：缓冲能量低于此值强制限流
            // 无裁判电流上限：调试用，防止失控。参考（粗略按裁判功率比例）：
            // 40W≈18000，60W≈27000，80W≈36000，放开测试可用 64000
            .no_judge_total_current_limit = 36000.0f, // [164]
            .buffer_total_current_limit = 16000.0f,   // [165] 缓冲段电流：最低保底电流
            .power_total_current_limit = 20000.0f,    // [166] 正常段电流：在告警区间线性缩放
        },

    // IMU/温控配置
    .imu =
        {
            .fusion_mode = IMU_FUSION_MAHONY_6AXIS,                 // [218] IMU fusion: 0=Mahony6Axis, 1=AHRS9Axis
            .temperature_pid = {1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f}, // 温控 PID
            .temperature_pid_max_out = 4500.0f,                       // 输出上限
            .temperature_pid_max_iout = 4400.0f,                      // 积分上限
            // PWM 上限需与 CubeMX 中 IMU Heat_PWM(TIM10_CH1) 的 ARR+1 对齐：
            // - 当前 TIM10 ARR=4999 (Period=4999) => pwm_max=5000
            .imu_temp_pwm_max = 5000,                                 // [219] PWM 上限
            .task_init_time_ms = 7,                                   // 任务延时
        },

    // 电压配置
    .voltage =
        {
            .full_battery_voltage = 25.2f, // [221] 满电电压
            .low_battery_voltage = 21.6f,  // [222] 低电阈值
            .voltage_drop = 0.0f,          // [223] 压降补偿
        },

    // 蜂鸣器配置
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
        .soft_beep_psc = 49,               // [224] 软提示分频：越小音调越高
        .soft_beep_duration_ms = 180,      // [225] 软提示时长：ms
        .enable = 1,                       // [226] 蜂鸣器总开关（0=关，1=开）

        .gimbal_warn_psc = 31,             // [227] 云台警告分频
        .gimbal_warn_pwm = 20000,          // [228] 云台警告占空

        .imu_cali_psc = 95,                // [229] IMU 校准音分频
        .imu_cali_pwm = 10000,             // [230] IMU 校准音占空
        .gimbal_cali_psc = 31,             // [231] 云台校准音分频
        .gimbal_cali_pwm = 19999,          // [232] 云台校准音占空

        .rc_cali_middle_time_ms = 10000,   // [233] 遥控校准中段时间
        .rc_cali_start_time_ms = 0,        // [234] 遥控校准起始时间
        .rc_cali_cycle_time_ms = 400,      // [235] 遥控校准蜂鸣周期
        .rc_cali_pause_time_ms = 200,      // [236] 遥控校准蜂鸣暂停
        .rc_cmd_long_time_ms = 2000,       // [237] 遥控长按判定
    },

    // LED 配置（状态巡视：单模块亮灭时间）
    .led =
        {
            .slot_on_ms = 300,   // [238] 亮灯时长 ms
            .slot_off_ms = 200,  // [239] 熄灭时长 ms
            .slot_gap_ms = 1000, // [240] 每轮结束额外熄灭时长 ms
        },
