/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>
#include "struct_typedef.h"

// 说明：
// - 本文件集中管理可调参数，作为配置源；默认值在 config.c 内填写。
// - 运行时读取方式：直接使用 g_config.xxx
// - 电机型号表使用 const g_motor_config；轴电机装配在 g_config.motor，启动后不改。
// - 若需从 Flash/裁判数据覆盖，可在启动时拷贝到 RAM 并按需修改。

// 通用 PID 参数
typedef struct
{
    fp32 kp;       // 比例
    fp32 ki;       // 积分
    fp32 kd;       // 微分
    fp32 max_out;  // 输出上限
    fp32 max_iout; // 积分上限
} pid_param_t;

// 测试模式选项
typedef enum
{
    TEST_MODE_NONE = 0,      // 關閉測試，正常模式
    TEST_MODE_CHASSIS_ONLY,  // 只測底盤
    TEST_MODE_YAW_ONLY,      // 只測雲台 yaw
    TEST_MODE_YAW_EASY_TEST, // yaw 轴简单上电：固定电流输出
    TEST_MODE_PITCH_ONLY,    // 只測雲台 pitch
    TEST_MODE_GIMBAL_DUAL,   // 雲台雙軸
    TEST_MODE_FRIC_ONLY,     // 只開摩擦輪
    TEST_MODE_TRIGGER_ONLY,  // 只撥盤
    TEST_MODE_SHOOT_COMBO,   // 摩擦+撥盤
    TEST_MODE_ENTERTAIN,     // 娛樂模式：禁用摩擦輪/撥盤，左撥杆控制 TF 音樂（上=停，中/下=播）
    TEST_MODE_PITCH_CALI,    // pitch 校准：测重力维持电流 & 静摩擦起动电流（存 SD，非安全档才运行）
} test_mode_e;

typedef struct
{
    uint8_t mode; // test_mode_e
} test_config_t;

// Pitch compensation (gravity hold + static friction breakaway) calibration.
typedef enum
{
    PITCH_CALI_BULLET_SRC_REFEREE = 0u, // use referee bullet_remaining_t.bullet_remaining_num
    PITCH_CALI_BULLET_SRC_MANUAL = 1u,  // use gimbal.pitch_cali.bullet_manual
} pitch_cali_bullet_src_e;

typedef struct
{
    uint8_t enable;        // 1: use loaded table in normal control; 0: fallback to constants
    uint8_t angle_points;  // scatter points along pitch soft limits (>=2)
    uint8_t bullet_points; // scatter points along bullet count axis (>=1)
    uint8_t bullet_source; // pitch_cali_bullet_src_e

    uint16_t bullet_min;    // min bullet count for calibration grid
    uint16_t bullet_max;    // max bullet count for calibration grid
    uint16_t bullet_manual; // manual bullet count (when bullet_source=MANUAL)

    fp32 angle_margin; // keep away from soft limit edges (rad)

    fp32 stable_angle_err; // stable criteria: |angle-target| < err (rad)
    fp32 stable_gyro_err;  // stable criteria: |gyro| < err (rad/s)
    uint16_t stable_time_ms;
    fp32 seek_k; // calibration: adjust angle_set += (target-angle)*seek_k per tick (helps when PID Ki=0)

    uint16_t hold_avg_time_ms; // averaging window for hold current (ms)

    uint16_t breakaway_step_current;     // delta current step (abs)
    uint16_t breakaway_step_period_ms;   // step update period (ms)
    uint16_t breakaway_max_extra_current; // max delta current allowed (abs)
    fp32 breakaway_gyro_threshold;       // detect motion by gyro (rad/s)
    fp32 breakaway_angle_threshold;      // detect motion by angle delta (rad)
    uint16_t recover_time_ms;            // recover/settle time after breakaway (ms)
} pitch_cali_config_t;

// 云台参数
typedef struct
{
    pid_param_t yaw_speed_pid;          // YAW 速度环
    pid_param_t pitch_speed_pid;        // PITCH 速度环
    pid_param_t yaw_encode_angle_pid;   // YAW 角度（编码器）环
    pid_param_t pitch_encode_angle_pid; // PITCH 角度（编码器）环

    uint16_t task_init_time_ms;   // 任务初始延时
    uint16_t control_period_ms;   // 控制周期

    uint8_t channel_yaw;          // 遥控 YAW 通道
    uint8_t channel_pitch;        // 遥控 PITCH 通道
    uint8_t channel_mode;         // 云台模式通道

    fp32 yaw_rc_sen;              // 遥控 YAW 灵敏度
    fp32 pitch_rc_sen;            // 遥控 PITCH 灵敏度
    fp32 yaw_mouse_sen;           // 鼠标 YAW 灵敏度
    fp32 pitch_mouse_sen;         // 鼠标 PITCH 灵敏度
    fp32 yaw_encode_sen;          // 编码器 YAW 灵敏度
    fp32 pitch_encode_sen;        // 编码器 PITCH 灵敏度
    uint16_t rc_deadband;         // 遥控死区

    fp32 init_angle_error;        // 初始化角度容差
    uint16_t init_stop_time_ms;   // 初始化停滞时间
    uint16_t init_time_ms;        // 初始化最大时间
    fp32 init_pitch_speed;        // 初始化 PITCH 速度
    fp32 init_yaw_speed;          // 初始化 YAW 速度
    fp32 init_pitch_set;          // 初始化 PITCH 目标
    fp32 init_yaw_set;            // 初始化 YAW 目标

    uint16_t yaw_middle_ecd;      // yaw 云台在车身中位时的编码器值（常用调参项，影响底盘跟随与相对角计算）
    fp32 pitch_kick_up_current;   // pitch 上抬起步电流（静摩擦补偿，摇杆有输入时生效）
    fp32 pitch_kick_down_current; // pitch 下压起步电流（静摩擦补偿，摇杆有输入时生效）
    // pitch 软限位（rad，符号以 gimbal_pitch_motor.angle / AUX-VOFA ch0 为准）：
    // - 抬头为正，低头为负
    // - 注意：watch.imu.angle_deg[] 是 INS 原始坐标的欧拉角，可能与此处符号相反（由 PITCH_TURN 适配电机/IMU 安装方向）
    fp32 pitch_soft_limit_up;     // 抬头方向极限（rad，通常为正）
    fp32 pitch_soft_limit_down;   // 低头方向极限（rad，通常为负）
    fp32 pitch_current_limit;     // pitch 输出电流限幅（绝对值）

    pitch_cali_config_t pitch_cali; // pitch 补偿校准参数（SD 持久化）

    uint16_t half_ecd_range;      // 编码器半范围
    uint16_t full_ecd_range;      // 编码器全范围
    fp32 motor_ecd_to_rad;        // 编码器计数转弧度系数

    fp32 cali_redundant_angle;    // 校准冗余角
    uint16_t cali_motor_set;      // 校准电机给定
    uint16_t cali_step_time_ms;   // 校准步时
    fp32 cali_gyro_limit;         // 校准陀螺限幅
    uint8_t cali_pitch_max_step;  // 校准步骤：Pitch 最大
    uint8_t cali_pitch_min_step;  // 校准步骤：Pitch 最小
    uint8_t cali_yaw_max_step;    // 校准步骤：Yaw 最大
    uint8_t cali_yaw_min_step;    // 校准步骤：Yaw 最小
    uint8_t cali_start_step;      // 校准起始步
    uint8_t cali_end_step;        // 校准结束步

    fp32 motionless_rc_deadline;  // 判静止的遥控阈值
    uint16_t motionless_time_max_ms; // 判静止的最大时间

    fp32 turn_speed;              // 一键转身速度
    uint16_t turn_key_mask;       // 一键转身键位
    uint16_t test_key_mask;       // 测试键位

    uint8_t yaw_turn;             // YAW 方向翻转标志
    uint8_t pitch_turn;           // PITCH 方向翻转标志
} gimbal_config_t;

// Chassis wheel type / kinematics selection.
typedef enum
{
    CHASSIS_WHEEL_TYPE_MECANUM = 0u, // mecanum wheel
    CHASSIS_WHEEL_TYPE_XDRIVE = 1u,  // 45deg omni (X-drive)
} chassis_wheel_type_e;

// High-level task family selection.
// Small geometry changes stay inside one family.
// Major control changes get a new family and usually a new task.
typedef enum
{
    LOCOMOTION_FAMILY_NONE = 0u,
    LOCOMOTION_FAMILY_CLASSIC_CHASSIS = 1u,
    LOCOMOTION_FAMILY_WHEELLEG_SERVO = 2u,
    LOCOMOTION_FAMILY_WHEELLEG_MIT = 3u,
} locomotion_family_e;

typedef enum
{
    GIMBAL_FAMILY_NONE = 0u,
    GIMBAL_FAMILY_SINGLE = 1u,
    GIMBAL_FAMILY_DUAL = 2u,
} gimbal_family_e;

typedef enum
{
    ARM_FAMILY_NONE = 0u,
    ARM_FAMILY_UNIFIED = 1u,
} arm_family_e;

typedef struct
{
    uint8_t locomotion_family; // locomotion_family_e
    uint8_t gimbal_family;     // gimbal_family_e
    uint8_t arm_family;        // arm_family_e
} task_profile_t;

// 底盘参数
typedef struct
{
    pid_param_t motor_speed_pid;     // 3508 速度环
    pid_param_t follow_gimbal_pid;   // 跟随云台角度环

    int8_t motor_dir[4];             // 单轮方向系数（±1，顺序 LF/RF/LR/RR）

    uint16_t task_init_time_ms;      // 任务初始延时
    uint16_t control_period_ms;      // 控制周期

    uint8_t channel_vx;              // 遥控通道：前后
    uint8_t channel_vy;              // 遥控通道：左右
    uint8_t channel_wz;              // 遥控通道：旋转
    uint8_t channel_mode;            // 模式通道

    fp32 vx_rc_sen;                  // 前后灵敏度
    fp32 vy_rc_sen;                  // 左右灵敏度
    fp32 angle_z_rc_sen;             // 跟随模式角度灵敏度
    fp32 wz_rc_sen;                  // 旋转速度灵敏度
    fp32 accel_x_first_order;        // vx 一阶滤波系数
    fp32 accel_y_first_order;        // vy 一阶滤波系数
    uint16_t rc_deadband;            // 遥控死区

    fp32 motor_speed_to_chassis_vx;  // 电机转速->车体 vx 比例
    fp32 motor_speed_to_chassis_vy;  // 电机转速->车体 vy 比例
    fp32 motor_speed_to_chassis_wz;  // 电机转速->车体 wz 比例
    fp32 motor_distance_to_center;   // 轮到中心距离
    fp32 rpm_to_vector;              // RPM -> 线速度比例

    fp32 max_wheel_speed;            // 单轮最大速度
    fp32 max_vx_forward;             // 最大前进速度
    fp32 max_vx_backward;            // 最大后退速度
    fp32 max_vy_left;                // 最大左移速度
    fp32 max_vy_right;               // 最大右移速度

    fp32 wz_set_scale;               // 摇摆 wz 缩放
    fp32 swing_no_move_angle;        // 小陀螺原地自转角速度(rad/s)
    fp32 swing_move_angle;           // 小陀螺移动自转角速度(rad/s)

    fp32 max_motor_can_current;      // 电机 CAN 电流上限

    uint16_t swing_key_mask;         // 摇摆按键
    uint16_t swing_mode_key_mask;    // 真正摇摆模式按键
    uint16_t gyro_spin_var_key_mask; // 变速小陀螺按键
    fp32 swing_amp_rad;              // 摇摆幅度(rad)
    uint16_t swing_half_period_ms;   // 摇摆半周期(ms)
    uint16_t swing_center_hold_min_ms; // 中心保持最短时间(ms)
    uint16_t swing_center_hold_max_ms; // 中心保持最长时间(ms)
    uint16_t key_front_mask;         // 前键
    uint16_t key_back_mask;          // 后键
    uint16_t key_left_mask;          // 左键
    uint16_t key_right_mask;         // 右键

    uint8_t wheel_type;              // chassis_wheel_type_e
} chassis_config_t;

// 射击/摩擦轮参数
typedef struct
{
    fp32 fric_speed_rpm;                // 摩擦轮目标转速（RPM，电机反馈值）
    fp32 fric_speed_off_rpm;            // 摩擦轮停止目标转速（一般为 0）
    fp32 fric_speed_step_rpm_s;         // 目标转速斜坡（RPM/s）
    fp32 fric_ready_ratio;              // 就绪判定比例：|rpm| >= |set|*ratio 认为到速
    pid_param_t fric_speed_pid;         // 摩擦轮速度环 PID（输出为电流）
    int8_t fric_motor_dir[4];           // CAN2 0x201~0x204 方向：1/-1/0(禁用)

    uint8_t rc_mode_channel;            // 射击模式通道
    uint16_t control_period_ms;         // 控制周期

    uint16_t key_on_mask;               // 启动键
    uint16_t key_off_mask;              // 停止键

    uint16_t shoot_done_key_off_time_ms; // 发射后防抖时间
    uint16_t press_long_time_ms;         // 长按判定时间
    uint16_t rc_s_long_time_ms;          // 遥控拨杆长按时间
    uint16_t up_add_time_ms;             // 斜率时间

    uint16_t half_ecd_range;            // 拨盘编码器半范围
    uint16_t full_ecd_range;            // 拨盘编码器全范围
    fp32 motor_rpm_to_speed;            // 拨盘 RPM->速度
    fp32 motor_ecd_to_angle;            // 拨盘编码器->角度
    uint8_t full_count;                 // 拨盘一圈计数

    fp32 trigger_speed_single;          // 单发拨盘速度
    fp32 trigger_speed_continuous;      // 连发拨盘速度
    fp32 trigger_speed_ready;           // 预备拨盘速度

    uint16_t key_off_judge_time_ms;     // 松键判定时间
    uint8_t switch_trigger_on;          // 拨杆开火位
    uint8_t switch_trigger_off;         // 拨杆停火位

    fp32 block_trigger_speed;           // 卡弹检测速度
    uint16_t block_time_ms;             // 卡弹判定时间
    uint16_t reverse_time_ms;           // 反转时间
    fp32 reverse_speed_limit;           // 反转限速

    fp32 pi_over_four;                  // π/4
    fp32 pi_over_ten;                   // π/10

    pid_param_t trigger_angle_pid;      // 拨盘角度 PID
    fp32 trigger_bullet_pid_max_out;    // 拨盘发射 PID 输出上限
    fp32 trigger_bullet_pid_max_iout;   // 拨盘发射 PID 积分上限
    fp32 trigger_ready_pid_max_out;     // 拨盘预备 PID 输出上限
    fp32 trigger_ready_pid_max_iout;    // 拨盘预备 PID 积分上限

    uint16_t heat_remain_value;         // 发热余量阈值
} shoot_config_t;

// 功率控制参数
typedef struct
{
    fp32 power_limit;                 // 功率上限
    fp32 warning_power;               // 告警功率阈值
    fp32 warning_power_buffer;        // 告警缓冲阈值
    fp32 no_judge_total_current_limit; // 无裁判时电流上限
    fp32 buffer_total_current_limit;  // 缓冲段电流上限
    fp32 power_total_current_limit;   // 正常段电流上限
} power_config_t;

// 装甲板/设备离线检测参数
// 检测参数（设备离线判定）
typedef struct
{
    uint16_t offline_time_ms;  // 判定离线的超时时间
    uint16_t online_time_ms;   // 判定上线的稳定时间
    uint8_t priority;          // 优先级
} detect_item_t;

typedef struct
{
    detect_item_t items[14];    // 顺序与 detect_task.h 的 errorList 对齐
    uint16_t enable_mask;       // bit=1 开启检测，默认全开（OLED 默认关闭）
    uint16_t task_init_time_ms; // 任务启动延时
    uint16_t control_period_ms; // 检测轮询周期
} detect_config_t;

typedef enum
{
    IMU_FUSION_MAHONY_6AXIS = 0,
    IMU_FUSION_AHRS_9AXIS = 1,
} imu_fusion_mode_e;

// IMU 与温控
typedef struct
{
    uint8_t fusion_mode; // imu_fusion_mode_e
    pid_param_t temperature_pid;    // 温控 PID
    fp32 temperature_pid_max_out;   // 温控 PID 输出上限
    fp32 temperature_pid_max_iout;  // 温控 PID 积分上限
    uint16_t imu_temp_pwm_max;      // 温控 PWM 上限
    uint16_t task_init_time_ms;     // 任务初始延时
} imu_config_t;

// 电压/电池
typedef struct
{
    fp32 full_battery_voltage; // 满电电压
    fp32 low_battery_voltage;  // 低电阈值
    fp32 voltage_drop;         // 线路压降补偿
} voltage_config_t;

// Buzzer PCM playback config (PWM+DMA, u8 samples on TF/SD).
// - name/path: if it contains ':' treat as absolute path (e.g. "0:/YOU_12K.U8"),
//              otherwise prefix "0:/" at runtime.
#define CONFIG_BUZZER_MUSIC_NAME_MAX 32u

typedef struct
{
    uint32_t carrier_min_hz; // minimum carrier frequency (Hz), 0=default (48kHz)
    uint32_t sample_rate_hz; // PCM sample rate (Hz), must match the U8 file
    uint16_t retry_ms;       // restart retry interval (ms) when file open/read fails
    uint8_t volume;          // volume (0..255)
    uint8_t loop;            // loop (0/1)
    uint16_t gain_q8;        // extra digital gain (Q8, 256=1.0x, 512=2.0x), saturates to 8-bit output

    char mid_file[CONFIG_BUZZER_MUSIC_NAME_MAX];  // ENTERTAIN: RC switch MID music file name/path
    char down_file[CONFIG_BUZZER_MUSIC_NAME_MAX]; // ENTERTAIN: RC switch DOWN music file name/path
} buzzer_pcm_config_t;

// 蜂鸣器参数
typedef struct
{
    uint16_t soft_beep_psc;          // 软提示音分频（PSC），越大音调越低
    uint16_t soft_beep_duration_ms;  // 软提示音时长，越大鸣响时间越长
    uint8_t enable;                  // 蜂鸣器总开关，0 关闭，1 开启（默认 0 便于硬件有故障时静音）

    uint16_t gimbal_warn_psc;        // 云台告警分频
    uint16_t gimbal_warn_pwm;        // 云台告警占空（PWM）

    uint16_t imu_cali_psc;           // IMU 校准音 分频
    uint16_t imu_cali_pwm;           // IMU 校准音 占空
    uint16_t gimbal_cali_psc;        // 云台校准音 分频
    uint16_t gimbal_cali_pwm;        // 云台校准音 占空

    uint16_t rc_cali_middle_time_ms; // 遥控校准中段时间
    uint16_t rc_cali_start_time_ms;  // 遥控校准起始时间
    uint16_t rc_cali_cycle_time_ms;  // 遥控校准蜂鸣周期
    uint16_t rc_cali_pause_time_ms;  // 遥控校准蜂鸣暂停
    uint16_t rc_cmd_long_time_ms;    // 遥控长按判定

    buzzer_pcm_config_t pcm;         // PCM (entertain/TF) config
} buzzer_config_t;

// LED 参数
typedef struct
{
    uint16_t slot_on_ms;   // 单模块点亮时长
    uint16_t slot_off_ms;  // 单模块灭灯时长
    uint16_t slot_gap_ms;  // 每轮结束额外熄灭时间
} led_config_t;

typedef struct
{
    uint8_t high_rate_div; // 1=all, 2=1/2, 4=1/4 for high-rate sdlog streams
} sdlog_config_t;

typedef enum
{
    MOTOR_MODEL_3508 = 0,
    MOTOR_MODEL_3510,
    MOTOR_MODEL_2006,
    MOTOR_MODEL_6020,
    MOTOR_MODEL_6623, // position-only feedback (ecd)
    MOTOR_MODEL_DM_J4310_2EC_V11,
    MOTOR_MODEL_DM_J4310_2EC_V12,
    MOTOR_MODEL_DM_J8009_2EC_V10,
    MOTOR_MODEL_DM_J8006_2EC_V11,
    MOTOR_MODEL_DM_J8006_2EC_V10,
    MOTOR_MODEL_UNITREE_GO_M8010_6,
    MOTOR_MODEL__COUNT
} motor_model_e;

typedef enum
{
    MOTOR_PROTOCOL_INHERIT = 0,
    MOTOR_PROTOCOL_RM_GROUP,
    MOTOR_PROTOCOL_DM_3MODE,
    MOTOR_PROTOCOL_DM_EXT_V1,
    MOTOR_PROTOCOL_DM_EXT_V2,
    MOTOR_PROTOCOL_UNITREE_RS485,
} motor_protocol_e;

typedef enum
{
    MOTOR_CONTROL_MODE_INHERIT = 0,
    MOTOR_CONTROL_MODE_CURRENT,
    MOTOR_CONTROL_MODE_MIT,
    MOTOR_CONTROL_MODE_POS_VEL,
    MOTOR_CONTROL_MODE_SPEED,
    MOTOR_CONTROL_MODE_FORCE_POS,
} motor_control_mode_e;

typedef enum
{
    MOTOR_TRANSPORT_INHERIT = 0,
    MOTOR_TRANSPORT_CAN,
    MOTOR_TRANSPORT_RS485,
} motor_transport_e;

typedef struct
{
    uint16_t can_id_base;   // base ID (0x200 for most, 0x204 for 6020)
    int16_t max_current;    // abs current limit
    fp32 reduction_ratio;   // gear ratio
} motor_model_param_t;

typedef struct
{
    motor_model_e model;
    uint8_t can_id; // motor id; CAN uses DIP id (1..8), RS485 uses device id
    uint8_t can_bus; // 0=legacy path decides bus, otherwise 1/2
    uint8_t protocol; // motor_protocol_e, 0=inherit from model table
    uint8_t control_mode; // motor_control_mode_e, 0=inherit from model/protocol
    uint16_t master_id; // feedback ID for motors with separate rx/tx IDs
    uint8_t transport; // motor_transport_e, 0=CAN/default
    uint8_t rs485_port; // RS485 port when transport=RS485
    uint32_t baudrate; // RS485 baudrate, 0=driver/default
    uint16_t rx_timeout_ms; // RS485 offline timeout, 0=driver/default
} motor_node_param_t;

typedef struct
{
    motor_model_param_t model[MOTOR_MODEL__COUNT];
} motor_config_t;

#define MOTOR_ARM_JOINT_COUNT 6u

typedef struct
{
    motor_node_param_t chassis[4];
    motor_node_param_t friction[4];
    motor_node_param_t yaw;
    motor_node_param_t yaw_upper;
    motor_node_param_t pitch;
    motor_node_param_t trigger;
    motor_node_param_t arm[MOTOR_ARM_JOINT_COUNT];
} motor_mount_config_t;

// AUX telemetry signal IDs.
// - AUX JustFloat telemetry (VOFA+/FireWater): N * float32 (little-endian) + tail 0x7F800000 (INF).
#define AUX_TELEM_MAX_CH 320u

typedef enum
{
    // ===== System =====
    AUX_TELEM_SIG_SYS_TICK_MS = 0, // [000] 系统运行时间tick(ms)
    AUX_TELEM_SIG_SYS_AUX_CMD_SEQ, // [001] 辅助链路命令序号（用于确认命令被执行）
    AUX_TELEM_SIG_SYS_BATTERY_VOLT, // [002] 电池电压(V)
    AUX_TELEM_SIG_SYS_BATTERY_PERCENT, // [003] 电池电量(%)

    // ===== Remote control (SBUS/DBUS) =====
    AUX_TELEM_SIG_RC_CH0, // [004] 遥控通道0（摇杆原始值）
    AUX_TELEM_SIG_RC_CH1, // [005] 遥控通道1（摇杆原始值）
    AUX_TELEM_SIG_RC_CH2, // [006] 遥控通道2（摇杆原始值）
    AUX_TELEM_SIG_RC_CH3, // [007] 遥控通道3（摇杆原始值）
    AUX_TELEM_SIG_RC_CH4, // [008] 遥控通道4（摇杆原始值）
    AUX_TELEM_SIG_RC_S0, // [009] 遥控拨码开关S0
    AUX_TELEM_SIG_RC_S1, // [010] 遥控拨码开关S1
    AUX_TELEM_SIG_RC_MOUSE_X, // [011] 鼠标X轴移动
    AUX_TELEM_SIG_RC_MOUSE_Y, // [012] 鼠标Y轴移动
    AUX_TELEM_SIG_RC_MOUSE_Z, // [013] 鼠标滚轮
    AUX_TELEM_SIG_RC_MOUSE_L, // [014] 鼠标左键(1按下)
    AUX_TELEM_SIG_RC_MOUSE_R, // [015] 鼠标右键(1按下)
    AUX_TELEM_SIG_RC_KEY, // [016] 键盘按键位掩码(key.v)
    AUX_TELEM_SIG_RC_ERROR, // [017] 遥控数据错误标志(1=异常)

    // ===== IMU / INS =====
    AUX_TELEM_SIG_IMU_Q0, // [018] 姿态四元数q0（ins_quat）
    AUX_TELEM_SIG_IMU_Q1, // [019] 姿态四元数q1（ins_quat）
    AUX_TELEM_SIG_IMU_Q2, // [020] 姿态四元数q2（ins_quat）
    AUX_TELEM_SIG_IMU_Q3, // [021] 姿态四元数q3（ins_quat）
    AUX_TELEM_SIG_IMU_ANGLE_YAW_RAD, // [022] INS姿态角 yaw（rad）
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_RAD, // [023] INS姿态角 roll（rad）
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_RAD, // [024] INS姿态角 pitch（rad）
    AUX_TELEM_SIG_IMU_ANGLE_YAW_DEG, // [025] INS姿态角 yaw（deg）
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_DEG, // [026] INS姿态角 roll（deg）
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_DEG, // [027] INS姿态角 pitch（deg）
    AUX_TELEM_SIG_IMU_GYRO_X_RAD_S, // [028] 陀螺仪角速度 X（rad/s）
    AUX_TELEM_SIG_IMU_GYRO_Y_RAD_S, // [029] 陀螺仪角速度 Y（rad/s）
    AUX_TELEM_SIG_IMU_GYRO_Z_RAD_S, // [030] 陀螺仪角速度 Z（rad/s）
    AUX_TELEM_SIG_IMU_GYRO_X_DPS, // [031] 陀螺仪角速度 X（deg/s）
    AUX_TELEM_SIG_IMU_GYRO_Y_DPS, // [032] 陀螺仪角速度 Y（deg/s）
    AUX_TELEM_SIG_IMU_GYRO_Z_DPS, // [033] 陀螺仪角速度 Z（deg/s）
    AUX_TELEM_SIG_IMU_ACCEL_X, // [034] 加速度计 X（单位同INS输出）
    AUX_TELEM_SIG_IMU_ACCEL_Y, // [035] 加速度计 Y（单位同INS输出）
    AUX_TELEM_SIG_IMU_ACCEL_Z, // [036] 加速度计 Z（单位同INS输出）

    // ===== Gimbal (runtime) =====
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE, // [037] 云台YAW 角度(angle,rad)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_SET, // [038] 云台YAW 角度设定(angle_set,rad)
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO, // [039] 云台YAW 角速度反馈(motor_gyro)
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO_SET, // [040] 云台YAW 角速度设定(motor_gyro_set)
    AUX_TELEM_SIG_GIMBAL_YAW_MOTOR_SPEED, // [041] 云台YAW 电机速度(motor_speed)
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_SET, // [042] 云台YAW 电流设定(current_set)
    AUX_TELEM_SIG_GIMBAL_YAW_GIVEN_CURRENT, // [043] 云台YAW 电流反馈(given_current)
    AUX_TELEM_SIG_GIMBAL_YAW_RAW_CMD_CURRENT, // [044] 云台YAW 原始电流指令(raw_cmd_current)
    AUX_TELEM_SIG_GIMBAL_YAW_ECD, // [045] 云台YAW 编码器计数(ecd)
    AUX_TELEM_SIG_GIMBAL_YAW_OFFSET_ECD, // [046] 云台YAW 零偏编码器(offset_ecd)
    AUX_TELEM_SIG_GIMBAL_YAW_RPM, // [047] 云台YAW 转速(rpm)
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_FB, // [048] 云台YAW 电流反馈(given_current)
    AUX_TELEM_SIG_GIMBAL_YAW_TEMP, // [049] 云台YAW 温度(℃)

    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE, // [050] 云台PITCH 角度(angle,rad)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_SET, // [051] 云台PITCH 角度设定(angle_set,rad)
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO, // [052] 云台PITCH 角速度反馈(motor_gyro)
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO_SET, // [053] 云台PITCH 角速度设定(motor_gyro_set)
    AUX_TELEM_SIG_GIMBAL_PITCH_MOTOR_SPEED, // [054] 云台PITCH 电机速度(motor_speed)
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_SET, // [055] 云台PITCH 电流设定(current_set)
    AUX_TELEM_SIG_GIMBAL_PITCH_GIVEN_CURRENT, // [056] 云台PITCH 电流反馈(given_current)
    AUX_TELEM_SIG_GIMBAL_PITCH_RAW_CMD_CURRENT, // [057] 云台PITCH 原始电流指令(raw_cmd_current)
    AUX_TELEM_SIG_GIMBAL_PITCH_ECD, // [058] 云台PITCH 编码器计数(ecd)
    AUX_TELEM_SIG_GIMBAL_PITCH_OFFSET_ECD, // [059] 云台PITCH 零偏编码器(offset_ecd)
    AUX_TELEM_SIG_GIMBAL_PITCH_RPM, // [060] 云台PITCH 转速(rpm)
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_FB, // [061] 云台PITCH 电流反馈(given_current)
    AUX_TELEM_SIG_GIMBAL_PITCH_TEMP, // [062] 云台PITCH 温度(℃)

    // ===== Gimbal PID (angle loop, gimbal_PID_t) =====
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET, // [063] 云台YAW 角度环PID 设定值(set)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_GET, // [064] 云台YAW 角度环PID 反馈值(get)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_POUT, // [065] 云台YAW 角度环PID P项输出(Pout)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_IOUT, // [066] 云台YAW 角度环PID I项输出(Iout)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_DOUT, // [067] 云台YAW 角度环PID D项输出(Dout)
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_OUT, // [068] 云台YAW 角度环PID 总输出(out)

    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET, // [069] 云台PITCH 角度环PID 设定值(set)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_GET, // [070] 云台PITCH 角度环PID 反馈值(get)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_POUT, // [071] 云台PITCH 角度环PID P项输出(Pout)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_IOUT, // [072] 云台PITCH 角度环PID I项输出(Iout)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_DOUT, // [073] 云台PITCH 角度环PID D项输出(Dout)
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_OUT, // [074] 云台PITCH 角度环PID 总输出(out)

    // ===== Gimbal PID (speed loop, pid_type_def) =====
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET, // [075] 云台YAW 速度环PID 设定值(set)
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_FDB, // [076] 云台YAW 速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DBUF0, // [077] 云台YAW 速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_POUT, // [078] 云台YAW 速度环PID P项输出(Pout)
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_IOUT, // [079] 云台YAW 速度环PID I项输出(Iout)
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DOUT, // [080] 云台YAW 速度环PID D项输出(Dout)
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_OUT, // [081] 云台YAW 速度环PID 总输出(out)

    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET, // [082] 云台PITCH 速度环PID 设定值(set)
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_FDB, // [083] 云台PITCH 速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DBUF0, // [084] 云台PITCH 速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_POUT, // [085] 云台PITCH 速度环PID P项输出(Pout)
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_IOUT, // [086] 云台PITCH 速度环PID I项输出(Iout)
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DOUT, // [087] 云台PITCH 速度环PID D项输出(Dout)
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT, // [088] 云台PITCH 速度环PID 总输出(out)

    // ===== Chassis (runtime) =====
    AUX_TELEM_SIG_CHASSIS_VX_SET, // [089] 底盘vx设定(vx_set)
    AUX_TELEM_SIG_CHASSIS_VY_SET, // [090] 底盘vy设定(vy_set)
    AUX_TELEM_SIG_CHASSIS_WZ_SET, // [091] 底盘wz设定(wz_set)
    AUX_TELEM_SIG_CHASSIS_VX, // [092] 底盘vx反馈(vx)
    AUX_TELEM_SIG_CHASSIS_VY, // [093] 底盘vy反馈(vy)
    AUX_TELEM_SIG_CHASSIS_WZ, // [094] 底盘wz反馈(wz)
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET, // [095] 底盘yaw偏置(chassis_yaw_offset)
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET_SET, // [096] 底盘yaw偏置设定(chassis_yaw_offset_set)
    AUX_TELEM_SIG_CHASSIS_YAW_SET, // [097] 底盘yaw设定(chassis_yaw_set)
    AUX_TELEM_SIG_CHASSIS_YAW, // [098] 底盘yaw反馈(chassis_yaw)
    AUX_TELEM_SIG_CHASSIS_PITCH, // [099] 底盘pitch反馈(chassis_pitch)
    AUX_TELEM_SIG_CHASSIS_ROLL, // [100] 底盘roll反馈(chassis_roll)
    AUX_TELEM_SIG_CHASSIS_SWING_KEY, // [101] 底盘小陀螺按键(1=开启)

    AUX_TELEM_SIG_CHASSIS_M0_RPM, // [102] 底盘电机0 转速(rpm)
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_CMD, // [103] 底盘电机0 电流指令(give_current)
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_FB, // [104] 底盘电机0 电流反馈(given_current)
    AUX_TELEM_SIG_CHASSIS_M0_SPEED_SET, // [105] 底盘电机0 速度设定(speed_set)
    AUX_TELEM_SIG_CHASSIS_M0_SPEED, // [106] 底盘电机0 速度反馈(speed)
    AUX_TELEM_SIG_CHASSIS_M0_ACCEL, // [107] 底盘电机0 加速度(accel)
    AUX_TELEM_SIG_CHASSIS_M0_ECD, // [108] 底盘电机0 编码器计数(ecd)
    AUX_TELEM_SIG_CHASSIS_M0_TEMP, // [109] 底盘电机0 温度(℃)

    AUX_TELEM_SIG_CHASSIS_M1_RPM, // [110] 底盘电机1 转速(rpm)
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_CMD, // [111] 底盘电机1 电流指令(give_current)
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_FB, // [112] 底盘电机1 电流反馈(given_current)
    AUX_TELEM_SIG_CHASSIS_M1_SPEED_SET, // [113] 底盘电机1 速度设定(speed_set)
    AUX_TELEM_SIG_CHASSIS_M1_SPEED, // [114] 底盘电机1 速度反馈(speed)
    AUX_TELEM_SIG_CHASSIS_M1_ACCEL, // [115] 底盘电机1 加速度(accel)
    AUX_TELEM_SIG_CHASSIS_M1_ECD, // [116] 底盘电机1 编码器计数(ecd)
    AUX_TELEM_SIG_CHASSIS_M1_TEMP, // [117] 底盘电机1 温度(℃)

    AUX_TELEM_SIG_CHASSIS_M2_RPM, // [118] 底盘电机2 转速(rpm)
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_CMD, // [119] 底盘电机2 电流指令(give_current)
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_FB, // [120] 底盘电机2 电流反馈(given_current)
    AUX_TELEM_SIG_CHASSIS_M2_SPEED_SET, // [121] 底盘电机2 速度设定(speed_set)
    AUX_TELEM_SIG_CHASSIS_M2_SPEED, // [122] 底盘电机2 速度反馈(speed)
    AUX_TELEM_SIG_CHASSIS_M2_ACCEL, // [123] 底盘电机2 加速度(accel)
    AUX_TELEM_SIG_CHASSIS_M2_ECD, // [124] 底盘电机2 编码器计数(ecd)
    AUX_TELEM_SIG_CHASSIS_M2_TEMP, // [125] 底盘电机2 温度(℃)

    AUX_TELEM_SIG_CHASSIS_M3_RPM, // [126] 底盘电机3 转速(rpm)
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_CMD, // [127] 底盘电机3 电流指令(give_current)
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_FB, // [128] 底盘电机3 电流反馈(given_current)
    AUX_TELEM_SIG_CHASSIS_M3_SPEED_SET, // [129] 底盘电机3 速度设定(speed_set)
    AUX_TELEM_SIG_CHASSIS_M3_SPEED, // [130] 底盘电机3 速度反馈(speed)
    AUX_TELEM_SIG_CHASSIS_M3_ACCEL, // [131] 底盘电机3 加速度(accel)
    AUX_TELEM_SIG_CHASSIS_M3_ECD, // [132] 底盘电机3 编码器计数(ecd)
    AUX_TELEM_SIG_CHASSIS_M3_TEMP, // [133] 底盘电机3 温度(℃)

    // ===== Chassis follow PID (pid_type_def) =====
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET, // [134] 底盘跟随PID 设定值(set)
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_FDB, // [135] 底盘跟随PID 反馈值(fdb)
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DBUF0, // [136] 底盘跟随PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_POUT, // [137] 底盘跟随PID P项输出(Pout)
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_IOUT, // [138] 底盘跟随PID I项输出(Iout)
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DOUT, // [139] 底盘跟随PID D项输出(Dout)
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_OUT, // [140] 底盘跟随PID 总输出(out)

    // ===== Chassis motor speed PID (pid_type_def) =====
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET, // [141] 底盘电机0速度环PID 设定值(set)
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_FDB, // [142] 底盘电机0速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DBUF0, // [143] 底盘电机0速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_POUT, // [144] 底盘电机0速度环PID P项输出(Pout)
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_IOUT, // [145] 底盘电机0速度环PID I项输出(Iout)
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DOUT, // [146] 底盘电机0速度环PID D项输出(Dout)
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_OUT, // [147] 底盘电机0速度环PID 总输出(out)

    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_SET, // [148] 底盘电机1速度环PID 设定值(set)
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_FDB, // [149] 底盘电机1速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DBUF0, // [150] 底盘电机1速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_POUT, // [151] 底盘电机1速度环PID P项输出(Pout)
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_IOUT, // [152] 底盘电机1速度环PID I项输出(Iout)
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DOUT, // [153] 底盘电机1速度环PID D项输出(Dout)
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_OUT, // [154] 底盘电机1速度环PID 总输出(out)

    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_SET, // [155] 底盘电机2速度环PID 设定值(set)
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_FDB, // [156] 底盘电机2速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DBUF0, // [157] 底盘电机2速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_POUT, // [158] 底盘电机2速度环PID P项输出(Pout)
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_IOUT, // [159] 底盘电机2速度环PID I项输出(Iout)
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DOUT, // [160] 底盘电机2速度环PID D项输出(Dout)
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_OUT, // [161] 底盘电机2速度环PID 总输出(out)

    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_SET, // [162] 底盘电机3速度环PID 设定值(set)
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_FDB, // [163] 底盘电机3速度环PID 反馈值(fdb)
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DBUF0, // [164] 底盘电机3速度环PID 微分缓存(Dbuf[0])
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_POUT, // [165] 底盘电机3速度环PID P项输出(Pout)
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_IOUT, // [166] 底盘电机3速度环PID I项输出(Iout)
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DOUT, // [167] 底盘电机3速度环PID D项输出(Dout)
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT, // [168] 底盘电机3速度环PID 总输出(out)

    // ===== Shoot (runtime) =====
    AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET, // [169] 摩擦轮目标转速(fric_speed_set)
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET, // [170] 拨弹目标速度(trigger_speed_set)
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED, // [171] 拨弹速度(speed)
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE, // [172] 拨弹角度(angle)
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE_SET, // [173] 拨弹角度设定(set_angle)
    AUX_TELEM_SIG_SHOOT_TRIGGER_GIVEN_CURRENT, // [174] 拨弹给定电流(given_current)
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT, // [175] 拨弹累计编码器计数(ecd_count)
    AUX_TELEM_SIG_SHOOT_PRESS_L, // [176] 左键/左拨(press_l)
    AUX_TELEM_SIG_SHOOT_PRESS_R, // [177] 右键/右拨(press_r)
    AUX_TELEM_SIG_SHOOT_KEY, // [178] 射击按键(key)
    AUX_TELEM_SIG_SHOOT_HEAT_LIMIT, // [179] 热量上限(heat_limit)
    AUX_TELEM_SIG_SHOOT_HEAT, // [180] 当前热量(heat)

    AUX_TELEM_SIG_SHOOT_FRIC0_RPM, // [181] 摩擦轮0 转速(rpm)
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_FB, // [182] 摩擦轮0 电流反馈(given_current)
    AUX_TELEM_SIG_SHOOT_FRIC0_TEMP, // [183] 摩擦轮0 温度(℃)
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_CMD, // [184] 摩擦轮0 电流指令(CAN2)

    AUX_TELEM_SIG_SHOOT_FRIC1_RPM, // [185] 摩擦轮1 转速(rpm)
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_FB, // [186] 摩擦轮1 电流反馈(given_current)
    AUX_TELEM_SIG_SHOOT_FRIC1_TEMP, // [187] 摩擦轮1 温度(℃)
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_CMD, // [188] 摩擦轮1 电流指令(CAN2)

    AUX_TELEM_SIG_SHOOT_FRIC2_RPM, // [189] 摩擦轮2 转速(rpm)
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_FB, // [190] 摩擦轮2 电流反馈(given_current)
    AUX_TELEM_SIG_SHOOT_FRIC2_TEMP, // [191] 摩擦轮2 温度(℃)
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_CMD, // [192] 摩擦轮2 电流指令(CAN2)

    AUX_TELEM_SIG_SHOOT_FRIC3_RPM, // [193] 摩擦轮3 转速(rpm)
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_FB, // [194] 摩擦轮3 电流反馈(given_current)
    AUX_TELEM_SIG_SHOOT_FRIC3_TEMP, // [195] 摩擦轮3 温度(℃)
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD, // [196] 摩擦轮3 电流指令(CAN2)

    AUX_TELEM_SIG_SHOOT_TRIGGER_RPM, // [197] 拨弹电机转速(rpm)
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD, // [198] 拨弹电机编码器(ecd)
    AUX_TELEM_SIG_SHOOT_TRIGGER_TEMP, // [199] 拨弹电机温度(℃)
    AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB, // [200] 拨弹电机电流反馈(given_current)

    // ===== 诊断 =====
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS0_CURRENT, // [201] chassis0 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS1_CURRENT, // [202] chassis1 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_PITCH_CURRENT, // [203] pitch 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_TRIGGER_CURRENT, // [204] trigger 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS3_CURRENT, // [205] chassis3 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_YAW_CURRENT, // [206] yaw 电流指令
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS2_CURRENT, // [207] chassis2 电流指令
    AUX_TELEM_SIG_DIAG_RM_GROUP_1FF_STATUS, // [208] RM 0x1FF 发送状态(用于诊断)
    AUX_TELEM_SIG_DIAG_CAN_BUS1_ERR, // [209] CAN bus1 错误码(最近一次)
    AUX_TELEM_SIG_DIAG_ZERO_FORCE, // [210] 云台零力模式(1=进入)

    // ===== 射击 PID（拨弹，pid_type_def） =====
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT, // [211] 拨弹PID I项输出(Iout)
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT, // [212] 拨弹PID 总输出(out)

    // ===== 内存（自定义堆） =====
    AUX_TELEM_SIG_MEM_HEAP_FREE, // [213] 剩余堆空间(byte)
    AUX_TELEM_SIG_MEM_HEAP_EVER_FREE, // [214] 历史最小剩余堆空间(byte)

    // ===== 板载 =====
    AUX_TELEM_SIG_BOARD_KEY_DOWN, // [215] 板载按键电平(1=按下)
    AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT, // [216] 板载按键按下次数(去抖)

    // ===== 打包状态（推荐用于无线） =====
    // 模式打包（十进制位权）：shoot_mode*100000 + last_chassis_mode*10000 + chassis_mode*1000 + pitch_motor_mode*100 + yaw_motor_mode*10 + gimbal_behaviour
    AUX_TELEM_SIG_PACK_MODE, // [217] 模式打包（十进制位权，见 README）
    AUX_TELEM_SIG_PACK_OFFLINE, // [218] 离线 bitmask（1=离线/错误，见 README）

    AUX_TELEM_SIG__COUNT, // [219] 枚举计数（不是遥测信号）
} aux_telem_sig_e;

typedef struct
{
    // AUX 口实时遥测（JustFloat, TX）：
    // - TF/SD 的 sdlog 遥测日志默认一直开启，不使用此配置。
    uint8_t enable;           // 0=关闭, 1=AUX JustFloat, 2=AUX 定时报平安, 3=预留, 4=AUX JustFloat（兼容）
    uint16_t period_ms;       // 遥测周期(ms)：0=auto（最小不阻塞 + 额外回退）；非 0 会钳制到最小周期（并加 25% 回退）
    // 遥测通道列表（AUX）：
    // - channel_num == 0：使用内置默认列表（当前 219 通道）。
    // - channel_num != 0：发送 channel_map[] 的前 channel_num 个条目（并钳制到 AUX_TELEM_MAX_CH）。
    uint16_t channel_num;     // 0=默认列表，否则为显式长度（<= AUX_TELEM_MAX_CH）
    uint16_t channel_map[AUX_TELEM_MAX_CH]; // “信号 ID 列表”：aux_telem_sig_e 的数值（0 是合法 ID）
} aux_telem_config_t;

// 手动控制输入源（SBUS/DBUS / ELRS(CRSF) / USB CDC / 板载按键）。
// - SBUS/DBUS：USART3 IRQ（BSP）推帧；rc_sbus_task 解析并更新 RC_ctrl_t。
// - ELRS/CRSF：由 elrs_task.c 更新 RC_ctrl_t，实际串口走 AUX 口配置。
// - USB/板载按键：作为额外输入源，可与其它源做合并。
#define MANUAL_INPUT_SRC_AUTO 0u  // 自动：选择最近活跃的源（或混合，见 mix_mode）
#define MANUAL_INPUT_SRC_DBUS 1u  // USART3 SBUS/DBUS
#define MANUAL_INPUT_SRC_ELRS 2u  // AUX 口 ELRS/CRSF
#define MANUAL_INPUT_SRC_USB  3u  // USB CDC（预留）

#define MANUAL_INPUT_MIX_SELECT_LATEST 0u // 输出=选中的源（强制或最近活跃）
#define MANUAL_INPUT_MIX_MERGE         1u // 输出=合并多个活跃源

#define MANUAL_INPUT_SWITCH_POS_UP   0u
#define MANUAL_INPUT_SWITCH_POS_MID  1u
#define MANUAL_INPUT_SWITCH_POS_DOWN 2u
#define MANUAL_INPUT_SWITCH_POS_MAX  MANUAL_INPUT_SWITCH_POS_DOWN

#define MANUAL_INPUT_IMAGE_SWITCH_SHOOT   0u
#define MANUAL_INPUT_IMAGE_SWITCH_CHASSIS 1u
#define MANUAL_INPUT_IMAGE_SWITCH_GIMBAL  2u
#define MANUAL_INPUT_IMAGE_SWITCH_MAX     MANUAL_INPUT_IMAGE_SWITCH_GIMBAL

typedef struct
{
    uint8_t gimbal_safe_pos;             // [303] MANUAL_INPUT_SWITCH_POS_*
    uint8_t chassis_safe_pos;            // [304] MANUAL_INPUT_SWITCH_POS_*
    uint8_t chassis_follow_pos;          // [305] MANUAL_INPUT_SWITCH_POS_*
    uint8_t chassis_spin_pos;            // [306] MANUAL_INPUT_SWITCH_POS_*
    uint8_t shoot_stop_pos;              // [307] MANUAL_INPUT_SWITCH_POS_*
    uint8_t shoot_ready_pos;             // [308] MANUAL_INPUT_SWITCH_POS_*
    uint8_t shoot_fire_pos;              // [309] MANUAL_INPUT_SWITCH_POS_*
    uint8_t image_vt13_shoot_switch_input; // [318] MANUAL_INPUT_IMAGE_SWITCH_*
} manual_input_semantics_config_t;

typedef struct
{
    uint8_t switch1_safe_value;          // [369] VT13 switch1 原始值，默认 0=安全
    uint8_t switch1_normal_value;        // [370] VT13 switch1 原始值，默认 1=普通
    uint8_t switch1_spin_value;          // [371] VT13 switch1 原始值，默认 2=小陀螺
    uint8_t switch2_pause_pos;           // [372] pause 映射到哪个三档位
    uint8_t switch2_btn_l_pos;           // [373] btn_l 映射到哪个三档位
    uint8_t switch2_btn_r_pos;           // [374] btn_r 映射到哪个三档位
    uint8_t auto_aim_pause_enable;       // [375] pause 是否触发自瞄
    uint8_t auto_aim_mouse_r_enable;     // [376] 鼠标右键是否触发自瞄
    uint8_t aux_fire_btn_l_enable;       // [377] btn_l 是否触发辅助开火
    uint8_t aux_fire_mouse_l_enable;     // [378] 鼠标左键是否触发辅助开火
} manual_input_vt13_config_t;

typedef struct
{
    uint8_t active_source;       // [300] MANUAL_INPUT_SRC_*
    uint8_t mix_mode;            // [301] MANUAL_INPUT_MIX_*
    uint16_t source_timeout_ms;  // [302] consider a source active within this window

    manual_input_semantics_config_t semantics;

    // 板载按键按下时：将该 mask OR 到 rc.key.v（0=禁用）。
    uint16_t board_key_key_mask; // [317]
    manual_input_vt13_config_t vt13;
} manual_input_config_t;

// Logical input mapping (RC channels -> app axes/switches).
typedef enum
{
    INPUT_AXIS_CHASSIS_X = 0,
    INPUT_AXIS_CHASSIS_Y,
    INPUT_AXIS_CHASSIS_WZ,
    INPUT_AXIS_GIMBAL_YAW,
    INPUT_AXIS_GIMBAL_PITCH,
    INPUT_AXIS_CALIB_0,
    INPUT_AXIS_CALIB_1,
    INPUT_AXIS_CALIB_2,
    INPUT_AXIS_CALIB_3,
    INPUT_AXIS_COUNT
} input_axis_e;

typedef enum
{
    INPUT_SW_GIMBAL_MODE = 0,
    INPUT_SW_CHASSIS_MODE,
    INPUT_SW_SHOOT_MODE,
    INPUT_SW_CALIB_L,
    INPUT_SW_CALIB_R,
    INPUT_SW_COUNT
} input_switch_e;

typedef struct
{
    uint8_t rc_ch;  // 0..4
    uint8_t invert; // 0=normal, 1=invert
} input_axis_map_t;

typedef struct
{
    uint8_t rc_sw;  // 0..1
    uint8_t invert; // 0=normal, 1=swap up/down
} input_switch_map_t;

typedef struct
{
    // ELRS/CRSF 通道映射 (0..15) -> RC_ctrl_t 字段。
    // - axes: rc.ch[0..4] <= crsf.ch[elrs_ch_map[i]]
    // - switches: rc.s[0..1] <= crsf.ch[elrs_sw_map[i]]
    uint8_t elrs_ch_map[5];      // [310..314]
    uint8_t elrs_sw_map[2];      // [315..316]

    input_axis_map_t axis[INPUT_AXIS_COUNT];
    input_switch_map_t sw[INPUT_SW_COUNT];
} input_config_t;

// Reserved config blocks for task families that are not implemented yet.
// Keep them in the top-level table now so later new tasks only need to
// add fields and a block-table entry, without reshaping the whole config.
typedef struct
{
    uint8_t reserved0;
} dual_gimbal_config_t;

typedef struct
{
    uint8_t reserved0;
} wheelleg_servo_config_t;

typedef struct
{
    uint8_t reserved0;
} wheelleg_mit_config_t;

typedef struct
{
    uint16_t control_period_ms; // [800] executor period
    fp32 key_speed_rad_s;       // [801] bringup key speed on output side
    fp32 hold_kd;               // [802] damping when no key is pressed
    fp32 drive_kd;              // [803] damping when key is pressed
} arm_j0_unitree_config_t;

typedef enum
{
    CONFIG_BLOCK_PROFILE = 0u,
    CONFIG_BLOCK_GIMBAL_SINGLE,
    CONFIG_BLOCK_GIMBAL_DUAL,
    CONFIG_BLOCK_LOCOMOTION_CLASSIC,
    CONFIG_BLOCK_LOCOMOTION_WHEELLEG_SERVO,
    CONFIG_BLOCK_LOCOMOTION_WHEELLEG_MIT,
    CONFIG_BLOCK_SHOOT_RM,
    CONFIG_BLOCK_ARM_J0_UNITREE,
    CONFIG_BLOCK_COMMON_POWER,
    CONFIG_BLOCK_COMMON_DETECT,
    CONFIG_BLOCK_COMMON_IMU,
    CONFIG_BLOCK_COMMON_VOLTAGE,
    CONFIG_BLOCK_COMMON_BUZZER,
    CONFIG_BLOCK_COMMON_LED,
    CONFIG_BLOCK_COMMON_MANUAL_INPUT,
    CONFIG_BLOCK_COMMON_INPUT,
    CONFIG_BLOCK_COMMON_AUX_TELEM,
    CONFIG_BLOCK_COMMON_TEST,
    CONFIG_BLOCK_COMMON_SDLOG,
    CONFIG_BLOCK_COUNT
} config_block_id_e;

typedef uint8_t (*config_block_active_fn_t)(void);

typedef struct
{
    config_block_id_e id;
    const char *name;
    const char *param_range; // e.g. "300-302,317"; empty when no tunable params
    const void *data;
    uint32_t size;
    config_block_active_fn_t is_active;
} config_block_desc_t;

typedef struct
{
    task_profile_t profile;
    motor_mount_config_t motor;
    gimbal_config_t gimbal;
    dual_gimbal_config_t dual_gimbal;
    chassis_config_t chassis;
    wheelleg_servo_config_t wheelleg_servo;
    wheelleg_mit_config_t wheelleg_mit;
    shoot_config_t shoot;
    arm_j0_unitree_config_t arm_j0_unitree;
    power_config_t power;
    detect_config_t detect;
    imu_config_t imu;
    voltage_config_t voltage;
    buzzer_config_t buzzer;
    led_config_t led;
    manual_input_config_t manual_input;
    input_config_t input;
    aux_telem_config_t aux_telem;
    test_config_t test;
    sdlog_config_t sdlog;
} config_t;

extern config_t g_config;
extern const motor_config_t g_motor_config;
const config_block_desc_t *config_get_block_table(uint32_t *count);
const config_block_desc_t *config_find_block(config_block_id_e id);
uint8_t config_block_is_active(config_block_id_e id);
