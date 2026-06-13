// Included by config.c inside g_config's initializer; do not compile directly.
    // 云台配置
    .gimbal =
        {
            .yaw_speed_pid = {500.0f, 1.0f, 0.0f, 7000.0f, 1000.0f},   // [001]kp [002]ki [003]kd [004]max_out [005]max_iout | 小 yaw 轻，速度环先按轻云台压低防震荡
            // PITCH 速度环：
            // - 控制周期为 1ms，PID_calc 内部未乘 dt，因此 Ki 属于“每次迭代”的积分系数；过大极易导致抖动/风up
            // - 默认以 P 为主（接近 YAW 的量级），I 默认先关；若需要重力补偿再通过 AUX 口逐步加一点 Ki
            .pitch_speed_pid = {2000.0f, 5.0f, 0.0f, 14000.0f, 1000.0f},   // [006]kp [007]ki [008]kd [009]max_out [010]max_iout
            .yaw_encode_angle_pid = {9.0f, 0.0f, 0.0f, 25.0f, 3.0f},      // [011]kp [012]ki [013]kd [014]max_out [015]max_iout | 小 yaw 角度环，先保守止振再往上加
            .pitch_encode_angle_pid = {6.0f, 0.0f, 0.0f, 20.0f, 0.0f},   // [016]kp [017]ki [018]kd [019]max_out [020]max_iout | PITCH 角度环（编码器）：同上

            .task_init_time_ms = 201,   // 任务启动延时
            .control_period_ms = 1,     // [022] 控制周期 1ms

            .channel_yaw = 0,           // 云台 yaw 逻辑轴，保留给兼容代码
            .channel_pitch = 1,         // 云台 pitch 逻辑轴，保留给兼容代码
            .channel_mode = 0,          // 云台模式逻辑拨杆，保留给兼容代码

            .yaw_rc_sen = -0.000003f,    // [026] 遥控 yaw 灵敏度（ch0 正=右转）
            .pitch_rc_sen = 0.000005f,  // [027] 遥控 pitch 灵敏度（ch1 正=抬头）
            .yaw_mouse_sen = 0.000023f,  // [028] 鼠标 yaw 灵敏度
            .pitch_mouse_sen = 0.00015f,// [029] 鼠标 pitch 灵敏度
            .yaw_encode_sen = 0.01f,    // [030] 编码器 yaw 灵敏度
            .pitch_encode_sen = 0.01f,  // [031] 编码器 pitch 灵敏度
            .rc_deadband = 10,          // [032] 遥控死区

            .init_angle_error = 0.1f,   // 初始化容差：偏差小于此值视为对准，过小易长时间无法完成
            .init_stop_time_ms = 100,   // 初始化停顿时间：保持稳定的判定窗口，过短易误判，过长拖慢启动
            .init_time_ms = 6000,       // 初始化最长时间：超时即放弃，防止卡死
            .init_pitch_speed = 0.004f, // 初始化 pitch 速度：越大收敛快但可能越过
            .init_yaw_speed = 0.005f,   // 初始化 yaw 速度：同上
            .init_pitch_set = 0.0f,     // 初始 pitch 设定：机械零点，偏移会导致上电姿态不同
            .init_yaw_set = 0.0f,       // 初始 yaw 设定：同上

            .yaw_middle_ecd = 6150,     // yaw 云台在车身中位时的编码器值（常用调参项）
            .yaw_soft_limit_max = 1.342f, // [062] 小 yaw 正向软限位：右 8000，留约 100 计数余量
            .yaw_soft_limit_min = -1.572f,// [063] 小 yaw 负向软限位：左 4000，留约 100 计数余量
            .yaw_kick_current = 500.0f, // [064] 小 yaw 起步电流：按目标速度方向给正负，不是重力保持
            .pitch_kick_up_current = 1000.0f,   // [041] pitch 抬头起步电流（静摩擦/重力补偿，持续叠加到 PID 输出上）
            .pitch_kick_down_current = 100.0f, // [042] pitch 低头起步电流（静摩擦/重力补偿，持续叠加到 PID 输出上）
            // pitch 软限位（以 VOFA ch0= gimbal_pitch_motor.angle 为准）：
            // - 本车实测：上 3400、中 3600、下 4400；当前 pitch_turn=1，控制角度为上方正、下方负
            // - 机械范围：约 +0.153 ~ -0.614 rad；下面配置已留余量
            .pitch_soft_limit_up = 0.115f,      // [043] 抬头方向软限位：上 3400，留约 50 计数余量
            .pitch_soft_limit_down = -0.575f, // [044] 低头方向软限位：下 4400，留约 50 计数余量
            .pitch_current_limit = 14000.0f,         // [045] pitch 输出电流限幅：6000 重力保持外再留 PID 修正余量

            // pitch 补偿校准（重力维持/静摩擦起动）：数据存 SD，正常模式可作为前馈使用
            .pitch_cali =
                {
                    .enable = 1u, // 默认开启：若没有校准文件则自动回退到常量参数
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

            .half_ecd_range = 4096,     // 编码器半范围：用于跳变判断
            .full_ecd_range = 8192,     // 编码器全范围：满圈计数
            .motor_ecd_to_rad = 0.000766990394f, // [048] 编码器计数->弧度：增大等于调大角度读数

            .cali_redundant_angle = 0.1f, // [049] 校准冗余角：留 0.1rad 保护
            .cali_motor_set = 8000,       // [050] 校准驱动值：编码器扫边时的给定
            .cali_step_time_ms = 2000,    // [051] 校准单步时间：每步等待 2s
            .cali_gyro_limit = 0.1f,      // [052] 校准陀螺限幅：陀螺波动大于此值则中断
            .cali_pitch_max_step = 1,     // [053] 校准顺序 pitch max
            .cali_pitch_min_step = 2,     // [054] 校准顺序 pitch min
            .cali_yaw_max_step = 3,       // [055] 校准顺序 yaw max
            .cali_yaw_min_step = 4,       // [056] 校准顺序 yaw min
            .cali_start_step = 1,         // [057] 校准起始步
            .cali_end_step = 5,           // [058] 校准结束步

            .motionless_rc_deadline = 10.0f, // [059] 静止判定阈值：遥杆变化小于此值认为未操作
            .motionless_time_max_ms = 3000,  // [060] 静止最长时间：超过则视为长时间无操作

            .turn_speed = 0.04f,          // [061] 一键转身速度：越大转身越快，过大会抖
            .turn_key_mask = 1u << 9,     // 一键转身键：F
            .test_key_mask = 1u << 8,     // 测试键：R

            .yaw_turn = 0,                // 云台 yaw 方向：0=不反转，1=反转
            .pitch_turn = 1,              // 云台 pitch 方向：0=不反转，1=反转
        },

    // 底盘配置
    .chassis =
        {
            .motor_speed_pid = {4000.0f, 10.0f, 0.0f, 16000.0f, 2000.0f}, // [066]kp [067]ki [068]kd [069]max_out [070]max_iout | 3508 速度环
            // 跟随云台 yaw 外环（输出底盘 wz_set，单位 rad/s）
            // - 误差：云台-底盘相对 yaw（单位 rad）
            // - Kd：对底盘 wz（rad/s）的阻尼项（不是“每 tick 误差差分”的 Kd）
            // 轮距参数修正后（motor_distance_to_center=0.395m），为保持原手感，将 wz_set 相关量按 0.2/0.395≈0.506 缩放。
            .follow_gimbal_pid = {10.13f, 0.0f, 1.0f, 3.04f, 0.2f}, // [071]kp [072]ki [073]kd [074]max_out [075]max_iout
            .motor_dir = {1, -1, 1, -1},          // 底盘四轮方向系数：LF / RF / LR / RR
            .wheel_type = CHASSIS_WHEEL_TYPE_MECANUM,

            .task_init_time_ms = 357,  // 任务启动延时
            .control_period_ms = 2,    // [081] 控制周期 2ms（500Hz）

            .channel_vx = 3,           // 底盘前后逻辑轴，保留给兼容代码
            .channel_vy = 2,           // 底盘左右逻辑轴，保留给兼容代码
            .channel_wz = 0,           // 底盘旋转逻辑轴，保留给兼容代码
            .channel_mode = 0,         // 底盘模式逻辑拨杆，保留给兼容代码

            .vx_rc_sen = 0.006f,       // [086] 前后灵敏度
            .vy_rc_sen = 0.005f,       // [087] 左右灵敏度
            .angle_z_rc_sen = 0.000002f, // [088] 跟随角度灵敏度
            .wz_rc_sen = 0.0051f,        // [089] 旋转灵敏度：轮距修正后按 0.2/0.395 缩放
            .accel_x_first_order = 0.1666666667f, // [090] vx 一阶滤波
            .accel_y_first_order = 0.3333333333f, // [091] vy 一阶滤波
            .rc_deadband = 10,         // [092] 遥控死区

            .motor_speed_to_chassis_vx = 0.25f, // 底盘解算：电机转速到 vx 的换算
            .motor_speed_to_chassis_vy = 0.25f, // 底盘解算：电机转速到 vy 的换算
            .motor_speed_to_chassis_wz = 0.25f, // 底盘解算：电机转速到 wz 的换算
            .motor_distance_to_center = 0.395f,   // 底盘几何中心半径：(前后390mm + 左右400mm) / 2
            .rpm_to_vector = 0.000415809748903494517209f, // RPM 到线速度的换算

            .max_wheel_speed = 4.0f,   // [098] 单轮最大速度：限制到电机安全区，过大易过流
            .max_vx_forward = 2.0f,    // [099] 最大前进速度：增大则前进更快，但超功率风险高
            .max_vx_backward = 2.0f,   // [100] 最大后退速度：同理
            .max_vy_left = 1.5f,       // [101] 最大左移：过大侧移易打滑
            .max_vy_right = 1.5f,      // [102] 最大右移

            .wz_set_scale = 0.1f,      // [103] 预留：wz 缩放（当前未使用）
            .swing_no_move_angle = 3.5f, // [104] 小陀螺自转角速度(rad/s)：轮距修正后按 0.2/0.395 缩放
            .swing_move_angle = 3.5f,    // [105] 小陀螺移动/静止保持同速

            .max_motor_can_current = 16000.0f, // [106] 电机电流上限：对应裁判功率限制，过大易掉线

            .swing_key_mask = 1u << 5, // 摇摆键：CTRL
            .swing_mode_key_mask = 1u << 10, // 小陀螺键：G
            .gyro_spin_var_key_mask = 1u << 15, // 变速小陀螺键：B
            .swing_amp_rad = 0.2617993878f, // [245] 15deg
            .swing_half_period_ms = 300, // [246]
            .swing_center_hold_min_ms = 5000, // [247]
            .swing_center_hold_max_ms = 20000, // [248]
            .key_front_mask = 1u << 0, // 前进键：W
            .key_back_mask = 1u << 1,  // 后退键：S
            .key_left_mask = 1u << 2,  // 左移键：A
            .key_right_mask = 1u << 3, // 右移键：D
        },

    // Wheel-leg MIT config
    .wheelleg_mit =
        {
            .task_init_time_ms = 250u,
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
            .default_leg_length_m = 0.095f,
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
            .turn_pid = {0.6f, 0.0f, 0.08f, 0.10f, 0.0f},
            .roll_pid = {25.0f, 0.0f, 3.0f, 15.0f, 0.0f},
            .pitch_balance_offset_right_rad = 0.0f,
            .pitch_balance_offset_left_rad = 0.0f,
            .max_v_mps = 0.30f,
            .max_yaw_rate_radps = 0.60f,
            .rc_deadband = 10u,
            .enable_switch_pos = MANUAL_INPUT_SWITCH_POS_MID,
            .single_test_actuator = (uint8_t)Motor13, // [600] 单电机测试目标
            .single_test_position_rad = 0.0f, // [601] MIT 位置给定；kp=0 时不生效
            .single_test_velocity_radps = 0.0f, // [602] MIT 速度给定
            .single_test_kp = 0.0f, // [603] 位置刚度，默认关闭位置环
            .single_test_kd = 0.10f, // [604] 速度阻尼
            .single_test_torque_nm = 0.0f, // [605] 单电机测试力矩，默认 0
            .single_test_torque_limit_nm = 0.30f, // [606] 测试力矩限幅
            .left_test_zero_time_ms = 1000u, // [607] 上电后先回 0 并保持
            .left_test_move_time_ms = 1500u, // [608] 每段 90deg 扫动时间
            .left_test_angle_rad = 1.5707963268f, // [609] 90deg
            .left_test_kp = 1.0f, // [610] 位置刚度，先保守
            .left_test_kd = 0.12f, // [611] 速度阻尼
            .left_test_torque_ff_nm = 0.0f, // [612] 前馈力矩
            .left_test_torque_limit_nm = 0.30f, // [613] 前馈限幅
            .left_test_front_dir = 1, // [614]
            .left_test_back_dir = 1, // [615]
            .right_test_front_dir = 1, // [616]
            .right_test_back_dir = 1, // [617]
            .left_front_zero_rad = 0.0f, // [618]
            .left_back_zero_rad = 0.0f, // [619]
            .right_front_zero_rad = 0.0f, // [620]
            .right_back_zero_rad = 0.0f, // [621]
            .left_front_dir = 1, // [622]
            .left_back_dir = 1, // [623]
            .right_front_dir = 1, // [624]
            .right_back_dir = 1, // [625]
            .right_front_actuator = (uint8_t)Motor13,
            .right_back_actuator = (uint8_t)Motor14,
            .right_wheel_actuator = (uint8_t)Motor12,
            .left_front_actuator = (uint8_t)Motor16,
            .left_back_actuator = (uint8_t)Motor17,
            .left_wheel_actuator = (uint8_t)Motor15,
        },

    // Shoot config
    .shoot =
        {
            .fric_speed_rpm = 4500.0f,         // [113] 摩擦轮目标转速（RPM）
            .fric_speed_off_rpm = 0.0f,        // [114] 停止
            .fric_speed_step_rpm_s = 20000.0f, // [115] 转速斜坡（RPM/s）
            .fric_ready_ratio = 0.90f,         // [116] 到速判定比例
            .fric_speed_pid = {8.0f, 0.05f, 0.0f, 16000.0f, 8000.0f}, // [117]kp [118]ki [119]kd [120]max_out [121]max_iout | 速度环 PID（输出电流）
            .fric_motor_dir = {-1, 1, 0, 0},    // 两路摩擦轮方向：0x201 / 0x202，后两路关闭

            .rc_mode_channel = 1,   // 射击模式逻辑拨杆，保留给兼容代码
            .control_period_ms = 1, // [127] 控制周期

            .key_on_mask = 1u << 6, // 启动键：Q
            .key_off_mask = 1u << 7,// 停止键：E

            .shoot_done_key_off_time_ms = 15,  // [130] 防抖时间：发射后需松键超过该时间才停止
            .press_long_time_ms = 400,         // [131] 长按判定：超过则进入连发逻辑
            .rc_s_long_time_ms = 2000,         // [132] 拨杆长按：用于模式切换
            .up_add_time_ms = 100,              // [133] 斜率时间：摩擦轮爬升时间，越小起转越快但电流大

            .half_ecd_range = 4096,            // 拨盘编码器半范围
            .full_ecd_range = 8192,            // 拨盘编码器全范围
            .motor_rpm_to_speed = 0.00290888208665721596153948461415f, // 拨盘电机 RPM 到角速度的换算
            .motor_ecd_to_angle = 0.000021305288720633905968306772076277f, // 编码器计数到角度的换算
            .full_count = 18,                  // 拨盘一圈对应的计数

            .trigger_speed_single = 3.0f,     // [139] 单发速度：越大单发时间缩短，过大可能卡弹
            .trigger_speed_continuous = 4.0f, // [140] 连发速度：越大射速越高，热量/卡弹风险升
            .trigger_speed_ready = 3.0f,       // [141] 预备速度：小速度防止预备阶段抖动

            .key_off_judge_time_ms = 500,      // [142] 松键判定时间
            .switch_trigger_on = 0,            // 拨盘微动开关导通电平
            .switch_trigger_off = 1,           // 拨盘微动开关断开电平

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
    .arm_j0_unitree =
        {
            .control_period_ms = 5u,
            .key_speed_rad_s = 1.0f,
            .hold_kd = 0.2f,
            .drive_kd = 0.4f,
        },

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
            .fusion_mode = IMU_FUSION_MAHONY_6AXIS,                 // 姿态融合算法：0=Mahony 六轴，1=AHRS 九轴
            .temperature_pid = {1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f}, // 温控 PID
            .temperature_pid_max_out = 4500.0f,                       // 输出上限
            .temperature_pid_max_iout = 4400.0f,                      // 积分上限
            .imu_temp_pwm_max = 50,                                   // [219] PWM 上限
            .task_init_time_ms = 7,                                   // 任务延时
        },

    // 电压配置
    .voltage =
        {
            .full_battery_voltage = 25.2f, // [221] 满电电压
            .low_battery_voltage = 22.2f,  // [222] 低电阈值
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
        .soft_beep_psc = 1,                // [224] 软提示分频：越小音调越高
        .soft_beep_duration_ms = 80,       // [225] 软提示时长：ms
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
