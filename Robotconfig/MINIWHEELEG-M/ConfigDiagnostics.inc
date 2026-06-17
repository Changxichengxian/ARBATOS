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
            .enable_mask = 0x1FFF,    // [209] bits[0..12]=1，OLED(bit13)=0
            .task_init_time_ms = 57,  // 检测任务启动延时
            .control_period_ms = 10,  // [211] 检测轮询周期
        },

	    // AUX 口 VOFA+/FireWater (JustFloat) 遥测：N*fp32 + INF 尾（实时/带宽有限）
	    // - TF/SD 会一直记录更详细的运行日志（sdlog：IMU/PID/loop/CAN/裁判/视觉等），不受这里开关影响
	    // - 默认列表：channel_num=0 时，发送内置默认列表（全量精简：去掉 mode/offline/err，保留 PACK_MODE/PACK_OFFLINE），共 219 通道
	    // - 自定义列表：channel_num!=0 时，发送 channel_map[0..channel_num-1]（元素为“信号编号(ID)”=AuxTelemSig 数值）
	    // - VOFA+ 通道数 N：等于实际发送通道数（默认 N=219；自定义 N=channel_num）
		    .AuxTelem =
		        {
						.enable = 1,      // [241] 遥测模式(0=关,1=UART JustFloat,2=UART心跳,3=预留,4=UART JustFloat(兼容))
	            .period_ms = 0,  // [242] 发送周期(ms)，0=自动(最小非阻塞周期+额外50%回退，适合无线)；非0会自动限速到最小非阻塞周期(含25%回退)
	            .channel_num = 14, // [243] 0=默认列表；非0=发送 channel_num 个通道
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
