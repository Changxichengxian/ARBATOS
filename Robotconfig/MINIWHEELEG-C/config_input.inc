// Included by config.c inside g_config's initializer; do not compile directly.
    // 手动输入配置
    // Manual control input (SBUS/DBUS / ELRS(CRSF) / USB CDC / board key).
    .manual_input =
        {
            .active_source = MANUAL_INPUT_SRC_AUTO,     // [300] auto select
            .mix_mode = MANUAL_INPUT_MIX_SELECT_LATEST, // [301] safe default: keep legacy "latest wins"
            .source_timeout_ms = 100,                   // [302] active within 100ms
            .semantics =
                {
                    .gimbal_safe_pos = MANUAL_INPUT_SWITCH_POS_UP,              // [303]
                    .chassis_safe_pos = MANUAL_INPUT_SWITCH_POS_UP,             // [304]
                    .chassis_follow_pos = MANUAL_INPUT_SWITCH_POS_MID,          // [305]
                    .chassis_spin_pos = MANUAL_INPUT_SWITCH_POS_DOWN,           // [306]
                    .shoot_stop_pos = MANUAL_INPUT_SWITCH_POS_UP,               // [307]
                    .shoot_ready_pos = MANUAL_INPUT_SWITCH_POS_MID,             // [308]
                    .shoot_fire_pos = MANUAL_INPUT_SWITCH_POS_DOWN,             // [309]
                    .image_vt13_shoot_switch_input = MANUAL_INPUT_IMAGE_SWITCH_CHASSIS, // [318]
                },

            .board_key_key_mask = 0,                    // [317] 0=disabled
            .vt13 =
                {
                    .switch1_safe_value = 0u,                         // [369]
                    .switch1_normal_value = 1u,                       // [370]
                    .switch1_spin_value = 2u,                         // [371]
                    .switch2_pause_pos = MANUAL_INPUT_SWITCH_POS_UP,  // [372]
                    .switch2_btn_l_pos = MANUAL_INPUT_SWITCH_POS_MID, // [373]
                    .switch2_btn_r_pos = MANUAL_INPUT_SWITCH_POS_DOWN, // [374]
                    .auto_aim_pause_enable = 1u,                      // [375]
                    .auto_aim_mouse_r_enable = 1u,                    // [376]
                    .aux_fire_btn_l_enable = 1u,                      // [377]
                    .aux_fire_mouse_l_enable = 1u,                    // [378]
                },
        },

    .input =
        {
            // Default ELRS(CRSF) -> RC_ctrl_t mapping (matches legacy hardcode in elrs_task.c):
            // - axes: ch0..3 => crsf0..3, ch4(AUX3) => crsf6
            // - switches: s0/s1 => AUX1/AUX2 => crsf4/crsf5
            .elrs_ch_map = {0, 1, 2, 3, 6},             // [310..314]
            .elrs_sw_map = {4, 5},                      // [315..316]

            .axis =
                {
                    [INPUT_AXIS_CHASSIS_X] = {.rc_ch = 1u, .invert = 0u},     // [320] ch1 forward/back
                    [INPUT_AXIS_CHASSIS_Y] = {.rc_ch = 3u, .invert = 0u},     // [322] ch3 reserved; bench mode does not move legs
                    [INPUT_AXIS_CHASSIS_WZ] = {.rc_ch = 0u, .invert = 0u},    // [324] ch [325] invert
                    [INPUT_AXIS_GIMBAL_YAW] = {.rc_ch = 0u, .invert = 0u},    // [326] ch [327] invert
                    [INPUT_AXIS_GIMBAL_PITCH] = {.rc_ch = 1u, .invert = 0u},  // [328] ch [329] invert
                    [INPUT_AXIS_CALIB_0] = {.rc_ch = 0u, .invert = 0u},       // [330] ch [331] invert
                    [INPUT_AXIS_CALIB_1] = {.rc_ch = 1u, .invert = 0u},       // [332] ch [333] invert
                    [INPUT_AXIS_CALIB_2] = {.rc_ch = 2u, .invert = 0u},       // [334] ch2 reserved; bench mode does not move legs
                    [INPUT_AXIS_CALIB_3] = {.rc_ch = 3u, .invert = 0u},       // [336] ch [337] invert
                },
            .sw =
                {
                    [INPUT_SW_GIMBAL_MODE] = {.rc_sw = 0u, .invert = 0u},     // [338] sw [339] invert
                    [INPUT_SW_CHASSIS_MODE] = {.rc_sw = 0u, .invert = 0u},    // [340] sw [341] invert
                    [INPUT_SW_SHOOT_MODE] = {.rc_sw = 1u, .invert = 0u},      // [342] sw [343] invert
                    [INPUT_SW_CALIB_L] = {.rc_sw = 0u, .invert = 0u},         // [344] sw [345] invert
                    [INPUT_SW_CALIB_R] = {.rc_sw = 1u, .invert = 0u},         // [346] sw [347] invert
                },
        },
