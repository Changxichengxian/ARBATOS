// Included by config.c inside g_config's initializer; do not compile directly.
    // 离线检测配置
    .detect =
        {
            .items =
                {
                    {30, 40, 15},   // [167]off [168]on [169]pri | DBUS_TOE
                    {10, 10, 11},   // [170]off [171]on [172]pri | CHASSIS_MOTOR1_TOE
                    {10, 10, 10},   // [173]off [174]on [175]pri | CHASSIS_MOTOR2_TOE
                    {10, 10, 9},    // [176]off [177]on [178]pri | CHASSIS_MOTOR3_TOE
                    {10, 10, 8},    // [179]off [180]on [181]pri | CHASSIS_MOTOR4_TOE
                    {50, 20, 14},   // [182]off [183]on [184]pri | YAW_GIMBAL_MOTOR_TOE
                    {50, 20, 13},   // [185]off [186]on [187]pri | PITCH_GIMBAL_MOTOR_TOE
                    {10, 10, 12},   // [188]off [189]on [190]pri | TRIGGER_MOTOR_TOE
                    {2, 3, 7},      // [191]off [192]on [193]pri | BOARD_GYRO_TOE
                    {5, 5, 7},      // [194]off [195]on [196]pri | BOARD_ACCEL_TOE
                    {40, 200, 7},   // [197]off [198]on [199]pri | BOARD_MAG_TOE
                    {100, 100, 5},  // [200]off [201]on [202]pri | REFEREE_TOE
                    {10, 10, 7},    // [203]off [204]on [205]pri | RM_IMU_TOE
                    {100, 100, 1},  // [206]off [207]on [208]pri | OLED_TOE
                },
            .enable_mask = 0x1F1F,    // 关闭 yaw/pitch/trigger 检测，保留底盘和板载检测
            .task_init_time_ms = 57,  // 检测任务启动延时
            .control_period_ms = 10,  // [211] 检测轮询周期
        },

	    // AUX 口遥测默认表：这里只定默认行为，是否临时看更多通道改代码即可。
	    // - TF/SD 仍会持续记录更详细的运行日志，这里只影响 AUX 口实时输出。
	    // - channel_num=0 时发送内置默认列表；非 0 时按 channel_map 的前 channel_num 项发送。
	    // - 这里写的是默认表，不再作为 AUX 动态配置项暴露。
	    // - 真要临时换表，直接改这里的默认值最清楚。
		    .aux_telem =
		        {
						.enable = 1,      // AUX 遥测模式：0=关，1=JustFloat，2=定时报平安，4=兼容 JustFloat
	            .period_ms = 0,  // 发送周期：0=自动；非 0 会按最小非阻塞周期自动限速
	            .channel_num = 14, // 发送通道数：0=内置默认列表；非 0=发送前 channel_num 项
	            .channel_map =
	                {
		                    // ===== 自定义发送列表（按顺序填ID；长度=channel_num）=====
		                    // 摩擦轮调试：尽量少通道，VOFA+ 刷新更快。
	                    AUX_TELEM_SIG_SYS_TICK_MS,
	                    AUX_TELEM_SIG_PACK_OFFLINE,
	                    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_CMD,
	                    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_CMD,
	                    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_CMD,
	                    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD,
	                    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_FB,
	                    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_FB,
	                    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_FB,
	                    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_FB,
	                    AUX_TELEM_SIG_SHOOT_FRIC0_RPM,
	                    AUX_TELEM_SIG_SHOOT_FRIC1_RPM,
	                    AUX_TELEM_SIG_SHOOT_FRIC2_RPM,
	                    AUX_TELEM_SIG_SHOOT_FRIC3_RPM,


                                     },
         },
    .sdlog =
        {
            // F4 用 1/2 差不多；更弱的芯片用 1/4，想更省可自己扩到 1/8；H7 级别可以全开。F4 全开基本带不动。
            .high_rate_div = 2u,
        },
