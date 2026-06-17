// Included by config.c inside g_config's initializer; do not compile directly.
    // 手动输入配置
    // ===== 稳定配置：硬件归属 / 输入语义 / 调试默认值，只在代码里改 =====
    // 这些项不会出现在 AUX 动态参数表里。
    .manual_input =
        {
            .active_source = MANUAL_INPUT_SRC_AUTO,     // 默认自动选择最近活跃输入源
            .mix_mode = MANUAL_INPUT_MIX_SELECT_LATEST, // 默认谁最近在发就用谁
            .source_timeout_ms = 100,                   // 活跃源超时窗口(ms)
            .semantics =
                {
                    .GimbalSafePos = MANUAL_INPUT_SWITCH_POS_UP,              // 云台安全档
                    .ChassisSafePos = MANUAL_INPUT_SWITCH_POS_UP,             // 底盘安全档
                    .ChassisFollowPos = MANUAL_INPUT_SWITCH_POS_MID,          // 底盘普通档
                    .ChassisSpinPos = MANUAL_INPUT_SWITCH_POS_DOWN,           // 底盘小陀螺档
                    .ShootStopPos = MANUAL_INPUT_SWITCH_POS_UP,               // 停火档
                    .ShootReadyPos = MANUAL_INPUT_SWITCH_POS_MID,             // 预热档
                    .ShootFirePos = MANUAL_INPUT_SWITCH_POS_DOWN,             // 开火档
                    .image_vt13_shoot_switch_input = MANUAL_INPUT_IMAGE_SWITCH_CHASSIS, // VT13 射击兼容读底盘模式拨杆
                },

            .BoardKeyKeyMask = 0,                    // 板载按键映射的键值，0 表示关闭
            .vt13 =
                {
                    .switch1_safe_value = 0u,                         // VT13 switch1 的安全值
                    .switch1_normal_value = 1u,                       // VT13 switch1 的普通值
                    .switch1_spin_value = 2u,                         // VT13 switch1 的小陀螺值
                    .switch2_pause_pos = MANUAL_INPUT_SWITCH_POS_UP,  // pause 翻成哪一档
                    .switch2_btn_l_pos = MANUAL_INPUT_SWITCH_POS_MID, // btn_l 翻成哪一档
                    .switch2_btn_r_pos = MANUAL_INPUT_SWITCH_POS_DOWN, // btn_r 翻成哪一档
                    .auto_aim_pause_enable = 1u,                      // pause 是否触发自瞄
                    .auto_aim_mouse_r_enable = 1u,                    // 右键是否触发自瞄
                    .AuxFireBtnLEnable = 1u,                      // btn_l 是否触发辅助开火
                    .AuxFireMouseLEnable = 1u,                    // 左键是否触发辅助开火
                },
        },

    .input =
        {
            // 摇杆/拨杆映射：稳定配置，只在代码里改。
            .ElrsChMap = {0, 1, 2, 3, 6},             // ELRS 轴顺序
            .ElrsSwMap = {4, 5},                      // ELRS 拨杆顺序

            .axis =
                {
                    [INPUT_AXIS_CHASSIS_X] = {.rc_ch = 3u, .invert = 0u},     // 底盘前后 <- rc.ch3
                    [INPUT_AXIS_CHASSIS_Y] = {.rc_ch = 2u, .invert = 0u},     // 底盘左右 <- rc.ch2
                    [INPUT_AXIS_CHASSIS_WZ] = {.rc_ch = 0u, .invert = 0u},    // 底盘旋转 <- rc.ch0
                    [INPUT_AXIS_GIMBAL_YAW] = {.rc_ch = 0u, .invert = 0u},    // 云台 yaw <- rc.ch0
                    [INPUT_AXIS_GIMBAL_PITCH] = {.rc_ch = 1u, .invert = 0u},  // 云台 pitch <- rc.ch1
                    [INPUT_AXIS_CALIB_0] = {.rc_ch = 0u, .invert = 0u},       // 校准杆 0 <- rc.ch0
                    [INPUT_AXIS_CALIB_1] = {.rc_ch = 1u, .invert = 0u},       // 校准杆 1 <- rc.ch1
                    [INPUT_AXIS_CALIB_2] = {.rc_ch = 2u, .invert = 0u},       // 校准杆 2 <- rc.ch2
                    [INPUT_AXIS_CALIB_3] = {.rc_ch = 3u, .invert = 0u},       // 校准杆 3 <- rc.ch3
                },
            .sw =
                {
                    [INPUT_SW_GIMBAL_MODE] = {.rc_sw = 0u, .invert = 0u},     // 云台模式拨杆 <- rc.s0
                    [INPUT_SW_CHASSIS_MODE] = {.rc_sw = 0u, .invert = 0u},    // 底盘模式拨杆 <- rc.s0
                    [INPUT_SW_SHOOT_MODE] = {.rc_sw = 1u, .invert = 0u},      // 射击模式拨杆 <- rc.s1
                    [INPUT_SW_CALIB_L] = {.rc_sw = 0u, .invert = 0u},         // 校准左拨杆 <- rc.s0
                    [INPUT_SW_CALIB_R] = {.rc_sw = 1u, .invert = 0u},         // 校准右拨杆 <- rc.s1
                },
        },
