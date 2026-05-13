/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "config.h"

/*
 * AUX 口临时改参：发送 "<id>:<value>"（例如 "1:1000"）
 * - 编号见本文件 g_config 初始化处每行末尾的 [ID] 注释
 * - 未标 [ID] 的参数：只在 init 使用 / 未运行时应用（AUX 口不支持改）
 * - 仅修改 RAM 中的 g_config，重启后恢复默认值
 */

// 电机型号表：每种电机的固定参数。
const motor_config_t g_motor_config =
{
    .model =
        {
            [MOTOR_MODEL_3508] = {.can_id_base = 0x200u, .max_current = 16000, .reduction_ratio = 19.0f},
            [MOTOR_MODEL_3510] = {.can_id_base = 0x200u, .max_current = 16000, .reduction_ratio = 19.0f},
            [MOTOR_MODEL_2006] = {.can_id_base = 0x200u, .max_current = 10000, .reduction_ratio = 25.0f},
            [MOTOR_MODEL_6020] = {.can_id_base = 0x204u, .max_current = 30000, .reduction_ratio = 1.0f},
            [MOTOR_MODEL_6623] = {.can_id_base = 0x200u, .max_current = 16000, .reduction_ratio = 1.0f},
            [MOTOR_MODEL_DM_J4310_2EC_V11] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 10.0f},
            [MOTOR_MODEL_DM_J4310_2EC_V12] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 10.0f},
            [MOTOR_MODEL_DM_J8009_2EC_V10] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 9.0f},
            [MOTOR_MODEL_DM_J8006_2EC_V11] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 6.0f},
            [MOTOR_MODEL_DM_J8006_2EC_V10] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 6.0f},
            [MOTOR_MODEL_UNITREE_GO_M8010_6] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 6.33f},
            [MOTOR_MODEL_DM_J4310_WHEELLEG_REF] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 10.0f},
            [MOTOR_MODEL_DM_6215_WHEELLEG_REF] = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 1.0f},
        },
};

config_t g_config = {
    .profile =
        {
            .locomotion_family = LOCOMOTION_FAMILY_CLASSIC_CHASSIS,
            .gimbal_family = GIMBAL_FAMILY_SINGLE,
            .arm_family = ARM_FAMILY_UNIFIED,
        },
    .motor =
        {
            .chassis =
                {
                    {MOTOR_MODEL_3508, 1u},
                    {MOTOR_MODEL_3508, 2u},
                    {MOTOR_MODEL_3508, 7u},
                    {MOTOR_MODEL_3508, 5u},
                },
            .friction =
                {
                    {MOTOR_MODEL_3510, 1u},
                    {MOTOR_MODEL_3510, 2u},
                    {MOTOR_MODEL_3510, 3u},
                    {MOTOR_MODEL_3510, 4u},
                },
            .yaw = {MOTOR_MODEL_6020, 1u},
            .yaw_upper = {MOTOR_MODEL_6020, 0u},
            .pitch = {MOTOR_MODEL_3510, 6u},
            .trigger = {MOTOR_MODEL_3510, 7u},
            .arm =
                {
                    {MOTOR_MODEL_6020, 1u},
                    {MOTOR_MODEL_DM_J8009_2EC_V10, 1u, 2u, (uint8_t)MOTOR_PROTOCOL_INHERIT, (uint8_t)MOTOR_CONTROL_MODE_INHERIT, 0u, (uint8_t)MOTOR_TRANSPORT_CAN},
                    {MOTOR_MODEL_DM_J8006_2EC_V10, 2u, 2u, (uint8_t)MOTOR_PROTOCOL_INHERIT, (uint8_t)MOTOR_CONTROL_MODE_INHERIT, 0u, (uint8_t)MOTOR_TRANSPORT_CAN},
                    {MOTOR_MODEL_DM_J4310_2EC_V11, 3u, 2u, (uint8_t)MOTOR_PROTOCOL_INHERIT, (uint8_t)MOTOR_CONTROL_MODE_INHERIT, 0u, (uint8_t)MOTOR_TRANSPORT_CAN},
                    {MOTOR_MODEL_DM_J4310_2EC_V12, 4u, 2u, (uint8_t)MOTOR_PROTOCOL_INHERIT, (uint8_t)MOTOR_CONTROL_MODE_INHERIT, 0u, (uint8_t)MOTOR_TRANSPORT_CAN},
                    {MOTOR_MODEL_DM_J4310_2EC_V12, 5u, 2u, (uint8_t)MOTOR_PROTOCOL_INHERIT, (uint8_t)MOTOR_CONTROL_MODE_INHERIT, 0u, (uint8_t)MOTOR_TRANSPORT_CAN},
                },
        },
    // 云台配置
    .gimbal =
        {
            .yaw_speed_pid = {2000.0f, 0.0f, 0.0f, 20000.0f, 5000.0f},   // [001]kp [002]ki [003]kd [004]max_out [005]max_iout | YAW 速度环：输出上限 20000（电压模式）
            // PITCH 速度环：
            // - 控制周期为 1ms，PID_calc 内部未乘 dt，因此 Ki 属于“每次迭代”的积分系数；过大极易导致抖动/风up
            // - 默认以 P 为主（接近 YAW 的量级），I 默认先关；若需要重力补偿再通过 AUX 口逐步加一点 Ki
            .pitch_speed_pid = {1800.0f, 0.0f, 0.0f, 10000.0f, 1000.0f},   // [006]kp [007]ki [008]kd [009]max_out [010]max_iout
            .yaw_encode_angle_pid = {30.0f, 0.0f, 0.0f, 40.0f, 5.0f},      // [011]kp [012]ki [013]kd [014]max_out [015]max_iout | YAW 角度环（编码器）：用于编码器模式；kp 增大提高位置刚性
            .pitch_encode_angle_pid = {4.5f, 0.0f, 0.0f, 15.0f, 0.0f},   // [016]kp [017]ki [018]kd [019]max_out [020]max_iout | PITCH 角度环（编码器）：同上

            .task_init_time_ms = 201,   // 任务启动延时
            .control_period_ms = 1,     // [022] 控制周期 1ms

            .channel_yaw = 0,           // [023] 遥控通道 yaw
            .channel_pitch = 1,         // [024] 遥控通道 pitch
            .channel_mode = 0,          // [025] 模式通道

            .yaw_rc_sen = 0.000005f,    // [026] 遥控 yaw 灵敏度（ch0 正=右转）
            .pitch_rc_sen = 0.00001f,  // [027] 遥控 pitch 灵敏度（ch1 正=抬头）
            .yaw_mouse_sen = 0.00005f,  // [028] 鼠标 yaw 灵敏度
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

            .yaw_middle_ecd = 1677,     // yaw 云台在车身中位时的编码器值（常用调参项）
            .pitch_kick_up_current = 1000.0f,   // [041] pitch 抬头起步电流（静摩擦/重力补偿，持续叠加到 PID 输出上）
            .pitch_kick_down_current = 100.0f, // [042] pitch 低头起步电流（静摩擦/重力补偿，持续叠加到 PID 输出上）
            // pitch 软限位（以 VOFA ch0= gimbal_pitch_motor.angle 为准）：
            // - 符号：正值为抬头，负值为低头（注意：IMU angle_deg[] 为 INS 原始坐标，可能符号相反）
            // - 机械范围：约 +0.8 ~ -0.49 rad；
            .pitch_soft_limit_up = 0.75f,      // [043] 抬头方向软限位（rad，正）
            .pitch_soft_limit_down = -0.45f, // [044] 低头方向软限位（rad，负）
            .pitch_current_limit = 4000.0f,         // [045] pitch 输出电流限幅

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
            .full_ecd_range = 8191,     // 编码器全范围：满圈计数
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
            .turn_key_mask = 1u << 9,     // [062] F 键
            .test_key_mask = 1u << 8,     // [063] R 键

            .yaw_turn = 0,                // [064] yaw 不反转
            .pitch_turn = 1,              // [065] pitch 反转
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
            .motor_dir = {1, -1, 1, -1},          // [076]LF [077]RF [078]LR [079]RR | 单轮方向系数 LF/RF/LR/RR（右侧朝内安装取 -1）
            .wheel_type = CHASSIS_WHEEL_TYPE_MECANUM, // wheel_type: 0=mecanum, 1=xdrive

            .task_init_time_ms = 357,  // 任务启动延时
            .control_period_ms = 2,    // [081] 控制周期 2ms（500Hz）

            .channel_vx = 3,           // [082] 遥控通道 前后
            .channel_vy = 2,           // [083] 遥控通道 左右
            .channel_wz = 0,           // [084] 遥控通道 旋转
            .channel_mode = 0,         // [085] 模式通道

            .vx_rc_sen = 0.006f,       // [086] 前后灵敏度
            .vy_rc_sen = 0.005f,       // [087] 左右灵敏度
            .angle_z_rc_sen = 0.000002f, // [088] 跟随角度灵敏度
            .wz_rc_sen = 0.0051f,        // [089] 旋转灵敏度：轮距修正后按 0.2/0.395 缩放
            .accel_x_first_order = 0.1666666667f, // [090] vx 一阶滤波
            .accel_y_first_order = 0.3333333333f, // [091] vy 一阶滤波
            .rc_deadband = 10,         // [092] 遥控死区

            .motor_speed_to_chassis_vx = 0.25f, // [093] 电机转速->vx：比例越大同样转速车越快
            .motor_speed_to_chassis_vy = 0.25f, // [094] 电机转速->vy
            .motor_speed_to_chassis_wz = 0.25f, // [095] 电机转速->wz
            .motor_distance_to_center = 0.395f,   // [096] (前后390mm + 左右400mm)/2 = 0.395m
            .rpm_to_vector = 0.000415809748903494517209f, // [097] RPM->线速度

            .max_wheel_speed = 4.0f,   // [098] 单轮最大速度：限制到电机安全区，过大易过流
            .max_vx_forward = 2.0f,    // [099] 最大前进速度：增大则前进更快，但超功率风险高
            .max_vx_backward = 2.0f,   // [100] 最大后退速度：同理
            .max_vy_left = 1.5f,       // [101] 最大左移：过大侧移易打滑
            .max_vy_right = 1.5f,      // [102] 最大右移

            .wz_set_scale = 0.1f,      // [103] 预留：wz 缩放（当前未使用）
            .swing_no_move_angle = 3.5f, // [104] 小陀螺自转角速度(rad/s)：轮距修正后按 0.2/0.395 缩放
            .swing_move_angle = 3.5f,    // [105] 小陀螺移动/静止保持同速

            .max_motor_can_current = 16000.0f, // [106] 电机电流上限：对应裁判功率限制，过大易掉线

            .swing_key_mask = 1u << 5, // [107] CTRL 键
            .swing_mode_key_mask = 1u << 10, // [249] G
            .gyro_spin_var_key_mask = 1u << 15, // [250] B
            .swing_amp_rad = 0.2617993878f, // [245] 15deg
            .swing_half_period_ms = 300, // [246]
            .swing_center_hold_min_ms = 5000, // [247]
            .swing_center_hold_max_ms = 20000, // [248]
            .key_front_mask = 1u << 0, // [108] W
            .key_back_mask = 1u << 1,  // [109] S
            .key_left_mask = 1u << 2,  // [110] A
            .key_right_mask = 1u << 3, // [111] D
        },

    // 射击配置
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
    // Arm J0 Unitree timing/tuning; motor identity lives in g_config.motor.arm[0].
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

    // IMU/温控配置
    .imu =
        {
            .fusion_mode = IMU_FUSION_MAHONY_6AXIS,                 // [218] IMU fusion: 0=Mahony6Axis, 1=AHRS9Axis
            .temperature_pid = {1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f}, // 温控 PID
            .temperature_pid_max_out = 4500.0f,                       // 输出上限
            .temperature_pid_max_iout = 4400.0f,                      // 积分上限
            // PWM 上限需与 CubeMX 中 Heat_PWM(TIM3_CH2) 的 ARR+1 对齐：
            // - 当前 TIM3 ARR=49 (Period=49) => pwm_max=50
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
            .mid_file = "YOU.U8",
            .down_file = "hajimi.U8",
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

    // 测试模式（单组件）
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
                    [INPUT_AXIS_CHASSIS_X] = {.rc_ch = 3u, .invert = 0u},     // [320] ch [321] invert
                    [INPUT_AXIS_CHASSIS_Y] = {.rc_ch = 2u, .invert = 0u},     // [322] ch [323] invert
                    [INPUT_AXIS_CHASSIS_WZ] = {.rc_ch = 0u, .invert = 0u},    // [324] ch [325] invert
                    [INPUT_AXIS_GIMBAL_YAW] = {.rc_ch = 0u, .invert = 0u},    // [326] ch [327] invert
                    [INPUT_AXIS_GIMBAL_PITCH] = {.rc_ch = 1u, .invert = 0u},  // [328] ch [329] invert
                    [INPUT_AXIS_CALIB_0] = {.rc_ch = 0u, .invert = 0u},       // [330] ch [331] invert
                    [INPUT_AXIS_CALIB_1] = {.rc_ch = 1u, .invert = 0u},       // [332] ch [333] invert
                    [INPUT_AXIS_CALIB_2] = {.rc_ch = 2u, .invert = 0u},       // [334] ch [335] invert
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

    .test =
        {
            .mode = TEST_MODE_NONE, // [244] 测试模式（AUX 口发送 244:<value>）
            // 可直接复制的取值（枚举值 / AUX value）：
            // 0: TEST_MODE_NONE,
            // 1: TEST_MODE_CHASSIS_ONLY,
            // 2: TEST_MODE_YAW_ONLY,
            // 3: TEST_MODE_YAW_EASY_TEST,
            // 4: TEST_MODE_PITCH_ONLY,
            // 5: TEST_MODE_GIMBAL_DUAL,
            // 6: TEST_MODE_FRIC_ONLY,
            // 7: TEST_MODE_TRIGGER_ONLY,
            // 8: TEST_MODE_SHOOT_COMBO,
            // 9: TEST_MODE_ENTERTAIN,
            // 10: TEST_MODE_PITCH_CALI,
        },

	    // AUX 口 VOFA+/FireWater (JustFloat) 遥测：N*fp32 + INF 尾（实时/带宽有限）
	    // - TF/SD 会一直记录更详细的运行日志（sdlog：IMU/PID/loop/CAN/裁判/视觉等），不受这里开关影响
	    // - 默认列表：channel_num=0 时，发送内置默认列表（全量精简：去掉 mode/offline/err，保留 PACK_MODE/PACK_OFFLINE），共 219 通道
	    // - 自定义列表：channel_num!=0 时，发送 channel_map[0..channel_num-1]（元素为“信号编号(ID)”=aux_telem_sig_e 数值）
	    // - VOFA+ 通道数 N：等于实际发送通道数（默认 N=219；自定义 N=channel_num）
		    .aux_telem =
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
 };

// 这些函数只判断某个参数块当前是否有效，不负责修改配置内容。
static uint8_t config_block_active_always(void)
{
    return 1u;
}

static uint8_t config_block_active_gimbal_single(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_SINGLE);
}

static uint8_t config_block_active_gimbal_dual(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_DUAL);
}

static uint8_t config_block_active_locomotion_classic(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_CLASSIC_CHASSIS);
}

static uint8_t config_block_active_wheelleg_servo(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_SERVO);
}

static uint8_t config_block_active_wheelleg_mit(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_MIT);
}

static uint8_t config_block_active_arm(void)
{
    return (uint8_t)(g_config.profile.arm_family != ARM_FAMILY_NONE);
}

// AUX 调参表：只有列在这里的块能被运行时改，g_config.motor 这种装配信息不放进来。
static const config_block_desc_t g_config_blocks[] = {
    {CONFIG_BLOCK_PROFILE, "profile", "", &g_config.profile, sizeof(g_config.profile), config_block_active_always},
    {CONFIG_BLOCK_GIMBAL_SINGLE, "gimbal.single", "001-022,026-061,350-368", &g_config.gimbal, sizeof(g_config.gimbal), config_block_active_gimbal_single},
    {CONFIG_BLOCK_GIMBAL_DUAL, "gimbal.dual", "400-499", &g_config.dual_gimbal, sizeof(g_config.dual_gimbal), config_block_active_gimbal_dual},
    {CONFIG_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", "066-075,081,086-092,098-106,245-248", &g_config.chassis, sizeof(g_config.chassis), config_block_active_locomotion_classic},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.wheelleg_servo", "500-599", &g_config.wheelleg_servo, sizeof(g_config.wheelleg_servo), config_block_active_wheelleg_servo},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.wheelleg_mit", "600-699", &g_config.wheelleg_mit, sizeof(g_config.wheelleg_mit), config_block_active_wheelleg_mit},
    {CONFIG_BLOCK_SHOOT_RM, "shoot.rm", "113-121,127,130-133,139-142,145-160", &g_config.shoot, sizeof(g_config.shoot), config_block_active_always},
    {CONFIG_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", "800-803", &g_config.arm_j0_unitree, sizeof(g_config.arm_j0_unitree), config_block_active_arm},
    {CONFIG_BLOCK_COMMON_POWER, "common.power", "161-166", &g_config.power, sizeof(g_config.power), config_block_active_always},
    {CONFIG_BLOCK_COMMON_DETECT, "common.detect", "167-208,211", &g_config.detect, sizeof(g_config.detect), config_block_active_always},
    {CONFIG_BLOCK_COMMON_IMU, "common.imu", "218-219", &g_config.imu, sizeof(g_config.imu), config_block_active_always},
    {CONFIG_BLOCK_COMMON_VOLTAGE, "common.voltage", "221-223", &g_config.voltage, sizeof(g_config.voltage), config_block_active_always},
    {CONFIG_BLOCK_COMMON_BUZZER, "common.buzzer", "224-237", &g_config.buzzer, sizeof(g_config.buzzer), config_block_active_always},
    {CONFIG_BLOCK_COMMON_LED, "common.led", "238-240", &g_config.led, sizeof(g_config.led), config_block_active_always},
    {CONFIG_BLOCK_COMMON_MANUAL_INPUT, "common.manual_input", "300-309,317-318,369-378", &g_config.manual_input, sizeof(g_config.manual_input), config_block_active_always},
    {CONFIG_BLOCK_COMMON_INPUT, "common.input", "310-316,320-347", &g_config.input, sizeof(g_config.input), config_block_active_always},
    {CONFIG_BLOCK_COMMON_AUX_TELEM, "common.aux_telem", "241-243", &g_config.aux_telem, sizeof(g_config.aux_telem), config_block_active_always},
    {CONFIG_BLOCK_COMMON_TEST, "common.test", "244", &g_config.test, sizeof(g_config.test), config_block_active_always},
    {CONFIG_BLOCK_COMMON_SDLOG, "common.sdlog", "249", &g_config.sdlog, sizeof(g_config.sdlog), config_block_active_always},
};

// 返回调参块表，同时可选返回块数量。
const config_block_desc_t *config_get_block_table(uint32_t *count)
{
    if (count != NULL)
    {
        *count = (uint32_t)(sizeof(g_config_blocks) / sizeof(g_config_blocks[0]));
    }
    return g_config_blocks;
}

// 按块 ID 查找调参块描述；找不到返回 NULL。
const config_block_desc_t *config_find_block(config_block_id_e id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_config_blocks) / sizeof(g_config_blocks[0])); i++)
    {
        if (g_config_blocks[i].id == id)
        {
            return &g_config_blocks[i];
        }
    }
    return NULL;
}

// 对外判断某个调参块当前是否启用。
uint8_t config_block_is_active(config_block_id_e id)
{
    const config_block_desc_t *block = config_find_block(id);
    if (block == NULL)
    {
        return 0u;
    }
    if (block->is_active == NULL)
    {
        return 1u;
    }
    return block->is_active();
}
