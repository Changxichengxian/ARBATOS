/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SD_LOG_H
#define SD_LOG_H

#include <stdint.h>

// ===== File format =====
//
// Current sdlog files use:
//   File header (sdlog_file_header_t), followed by a stream of blocks:
//     [sdlog_block_header_t][block bytes...]
//   Each block stores raw record-stream bytes, or an LZ4-compressed blob that
//   decompresses to those bytes when SDLOG_BLOCK_FLAG_COMPRESSED is set.
//
// Record stream inside each raw block:
//   record := dt_ms(varint u32) + tag(varint u32) + len(varint u32) + payload(len bytes)
// where dt_ms is the delta from the previous record tick, starting from
// sdlog_file_header_t::boot_tick_ms. No padding/alignment is used.
//
// All fields are little-endian.

#define SDLOG_FILE_MAGIC 0x474C4453u /* 'SDLG' */
#define SDLOG_SCHEMA_VERSION 1u

#define SDLOG_BLOCK_MAGIC 0x4B424453u /* 'SDBK' */
#define SDLOG_BLOCK_FLAG_COMPRESSED 0x0001u
// If set, sdlog_block_header_t::reserved stores CRC32(IEEE) of the raw (decompressed) block bytes.
#define SDLOG_BLOCK_FLAG_CRC32 0x0002u

typedef struct __attribute__((packed))
{
    uint32_t magic;       // SDLOG_FILE_MAGIC
    uint16_t header_size; // sizeof(sdlog_file_header_t)
    uint16_t flags;       // reserved, must be 0 for now
    uint32_t boot_tick_ms;
    uint32_t reserved;
} sdlog_file_header_t;

typedef char _check_sdlog_file_header_size[(sizeof(sdlog_file_header_t) == 16) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint32_t magic;       // SDLOG_BLOCK_MAGIC
    uint16_t flags;       // SDLOG_BLOCK_FLAG_*
    uint16_t header_size; // sizeof(sdlog_block_header_t)
    uint32_t raw_len;     // bytes after decompression
    uint32_t data_len;    // bytes stored in file (raw or compressed)
    uint32_t reserved;    // CRC32(raw bytes) if SDLOG_BLOCK_FLAG_CRC32 set, otherwise reserved
} sdlog_block_header_t;

// Tag rule: keep tag payloads append-only. If a payload changes incompatibly,
// bump its payload version or allocate a new tag instead of reusing the old layout.
typedef enum
{
    SDLOG_TAG_META = 0x0000u,
    SDLOG_TAG_IMU = 0x0001u,
    SDLOG_TAG_RC_CRSF = 0x0002u,
    SDLOG_TAG_ACTUATOR_CURRENT = 0x0003u,
    SDLOG_TAG_BATTERY = 0x0004u,
    SDLOG_TAG_PID = 0x0005u,
    SDLOG_TAG_GIMBAL_LOOP = 0x0010u,
    SDLOG_TAG_CHASSIS_LOOP = 0x0011u,

    // Referee system (payload is raw bytes or packed structs from Referee.h)
    SDLOG_TAG_REFEREE_RAW = 0x0020u,
    SDLOG_TAG_REF_GAME_STATE = 0x0021u,
    SDLOG_TAG_REF_GAME_RESULT = 0x0022u,
    SDLOG_TAG_REF_GAME_ROBOT_HP = 0x0023u,
    SDLOG_TAG_REF_EVENT = 0x0024u,
    SDLOG_TAG_REF_SUPPLY_ACTION = 0x0025u,
    SDLOG_TAG_REF_SUPPLY_BOOKING = 0x0026u,
    SDLOG_TAG_REF_WARNING = 0x0027u,
    SDLOG_TAG_REF_ROBOT_STATE = 0x0028u,
    SDLOG_TAG_REF_POWER_HEAT = 0x0029u,
    SDLOG_TAG_REF_ROBOT_POS = 0x002Au,
    SDLOG_TAG_REF_BUFF = 0x002Bu,
    SDLOG_TAG_REF_AERIAL_ENERGY = 0x002Cu,
    SDLOG_TAG_REF_ROBOT_HURT = 0x002Du,
    SDLOG_TAG_REF_SHOOT_DATA = 0x002Eu,
    SDLOG_TAG_REF_BULLET_REMAINING = 0x002Fu,

    // CAN bus
    SDLOG_TAG_CAN_RX = 0x0030u,

    // High-level state/health snapshots
    SDLOG_TAG_WATCH = 0x0031u,      // payload: Watch (see Watch.h)
    SDLOG_TAG_DETECT_STATUS = 0x0032u,  // payload: sdlog_detect_status_t
    SDLOG_TAG_CHASSIS_POWER_LIMIT = 0x0033u,
    SDLOG_TAG_GIMBAL_LIMIT = 0x0034u,
    SDLOG_TAG_REF_RFID_STATUS = 0x0035u,
    SDLOG_TAG_REF_GROUND_ROBOT_POSITION = 0x0036u,
    SDLOG_TAG_REF_RADAR_MARK = 0x0037u,
    SDLOG_TAG_REF_SENTRY_INFO = 0x0038u,
    SDLOG_TAG_REF_RADAR_INFO = 0x0039u,
    SDLOG_TAG_REF_ROBOT_INTERACTIVE = 0x003Au,

    // Configuration / system / events
    SDLOG_TAG_CONFIG = 0x0040u, // payload: sdlog_config_header_t + raw config bytes
    SDLOG_TAG_SYS_STATS = 0x0041u,
    SDLOG_TAG_EVENT = 0x0042u,

    // Vision link RX (current transport: USB CDC)
    SDLOG_TAG_VISION_RX = 0x0043u, // payload: sdlog_vision_to_gimbal_t (same as VisionToGimbal)

    // Aux tuning command (ASCII line)
    SDLOG_TAG_AUX_TUNE = 0x0044u, // payload: uint32_t seq + NUL-terminated command string

    // Raw manual input packets (DBUS / ELRS / image link)
    SDLOG_TAG_MANUAL_INPUT_RAW = 0x0046u,
    SDLOG_TAG_IMAGE_LINK_STATS = 0x0047u,
    SDLOG_TAG_IMU_TRUST = 0x0048u,
    SDLOG_TAG_CONTROL_SUMMARY = 0x0049u,
    SDLOG_TAG_PITCH_CALI = 0x004Au,
    SDLOG_TAG_CHASSIS_BASE_STREAM = 0x004Bu,
    SDLOG_TAG_GIMBAL_BASE_STREAM = 0x004Cu,
    SDLOG_TAG_IMU_BASE_STREAM = 0x004Du,
    SDLOG_TAG_RT_PROFILER = 0x004Eu,
    SDLOG_TAG_WHEELLEG_MIT_CONFIG = 0x004Fu,
    SDLOG_TAG_WHEELLEG_MIT_STATUS = 0x0050u,
    SDLOG_TAG_BUILD_INFO = 0x0051u,
    SDLOG_TAG_RUNTIME_DEVICE = 0x0052u,
    SDLOG_TAG_WHEELLEG_MIT_MOTOR_DIAG = 0x0053u,
} sdlog_tag_e;

typedef enum
{
    SDLOG_PID_IMU_TEMP = 1u,

    SDLOG_PID_GIMBAL_YAW_ANGLE = 10u,
    SDLOG_PID_GIMBAL_YAW_SPEED = 11u,
    SDLOG_PID_GIMBAL_PITCH_ANGLE = 12u,
    SDLOG_PID_GIMBAL_PITCH_SPEED = 13u,

    SDLOG_PID_CHASSIS_M1_SPEED = 20u,
    SDLOG_PID_CHASSIS_M2_SPEED = 21u,
    SDLOG_PID_CHASSIS_M3_SPEED = 22u,
    SDLOG_PID_CHASSIS_M4_SPEED = 23u,
    SDLOG_PID_CHASSIS_FOLLOW = 24u,

    SDLOG_PID_SHOOT_FRIC1_SPEED = 30u,
    SDLOG_PID_SHOOT_FRIC2_SPEED = 31u,
    SDLOG_PID_SHOOT_FRIC3_SPEED = 32u,
    SDLOG_PID_SHOOT_FRIC4_SPEED = 33u,
    SDLOG_PID_SHOOT_TRIGGER = 34u,
} sdlog_pid_id_e;

// ===== Payload structs (for offline decode reference) =====

typedef struct __attribute__((packed))
{
    float quat[4];  // wxyz
    float gyro[3];  // rad/s
    float accel[3]; // m/s^2 (or board units)
    float temp;     // degC (sensor)
} sdlog_imu_t;

typedef struct __attribute__((packed))
{
    uint16_t ch_raw[16]; // CRSF 11-bit channels (0..2047)
    uint8_t rc_ctrl[22]; // ManualInputState (packed) snapshot for exact control values used by the system
} sdlog_rc_crsf_t;

typedef struct __attribute__((packed))
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} sdlog_vision_to_gimbal_t;

typedef char _check_sdlog_vision_to_gimbal_size[(sizeof(sdlog_vision_to_gimbal_t) == 29) ? 1 : -1];

// Aux tuning command record: [u32 seq][ASCII bytes...][0]
typedef struct __attribute__((packed))
{
    uint32_t seq;
    char cmd[1]; // NUL-terminated ASCII line (variable-length payload)
} sdlog_aux_tune_t;

#define SDLOG_CONFIG_VERSION 1u
#define SDLOG_BUILD_INFO_VERSION 3u
#define SDLOG_RUNTIME_DEVICE_VERSION 1u
#define SDLOG_BUILD_INFO_TEXT_LEN 32u
#define SDLOG_BUILD_INFO_GIT_SHA_LEN 16u
#define SDLOG_BUILD_INFO_DATE_LEN 12u
#define SDLOG_BUILD_INFO_TIME_LEN 9u

typedef struct __attribute__((packed))
{
    uint16_t version;     // SDLOG_CONFIG_VERSION
    uint16_t header_size; // sizeof(sdlog_config_header_t)
    uint16_t ConfigSize; // raw config bytes copied after the header
    uint16_t flags;       // reserved
} sdlog_config_header_t;

typedef struct __attribute__((packed))
{
    uint16_t version;        // SDLOG_BUILD_INFO_VERSION
    uint16_t header_size;    // sizeof(sdlog_build_info_t)
    uint16_t schema_version; // SDLOG_SCHEMA_VERSION
    uint16_t flags;          // reserved

    uint32_t ConfigSize;    // sizeof(Config)
    uint32_t ConfigCrc32;   // CRC32(Config bytes at log start)

    uint8_t task_module_count;
    uint8_t high_rate_div;
    uint8_t compression_enabled;
    uint8_t build_dirty;
    uint32_t task_module_mask;
    uint8_t runtime_device_count;
    uint8_t motorInstCount;
    uint8_t controller_count;
    uint8_t profile_kind;
    uint8_t BoardKind;
    uint8_t rtProfCount;
    uint8_t BoardCanBusCount;
    uint32_t BoardCpuHz;

    char target[SDLOG_BUILD_INFO_TEXT_LEN];
    char board[SDLOG_BUILD_INFO_TEXT_LEN];
    char git_sha[SDLOG_BUILD_INFO_GIT_SHA_LEN];
    char build_date[SDLOG_BUILD_INFO_DATE_LEN];
    char build_time[SDLOG_BUILD_INFO_TIME_LEN];
} sdlog_build_info_t;

typedef char _check_sdlog_build_info_size[(sizeof(sdlog_build_info_t) == 136) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint8_t version;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t enabled;
    uint8_t bus;
    uint16_t source_id;
} sdlog_runtime_device_t;

typedef char _check_sdlog_runtime_device_size[(sizeof(sdlog_runtime_device_t) == 8) ? 1 : -1];

typedef struct __attribute__((packed))
{
    int16_t chassis[4];
    int16_t yaw;
    int16_t pitch;
    int16_t trigger;
    int16_t friction[4];
} sdlog_actuator_current_t;

typedef struct __attribute__((packed))
{
    float voltage;
    float percent; // 0..1
} sdlog_battery_t;

typedef struct __attribute__((packed))
{
    uint8_t bus; // 1: CAN1, 2: CAN2
    uint8_t dlc;
    uint16_t std_id;
    uint8_t data[8];
} sdlog_can_rx_t;

typedef enum
{
    SDLOG_MANUAL_INPUT_PROTO_DBUS = 1u,
    SDLOG_MANUAL_INPUT_PROTO_CRSF = 2u,
    SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM = 3u,
    SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13 = 4u,
} sdlog_manual_input_proto_e;

typedef enum
{
    SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT = 1u,
    SDLOG_MANUAL_INPUT_RANGE_CENTERED_660 = 2u,
    SDLOG_MANUAL_INPUT_RANGE_CENTERED_1024 = 3u,
} sdlog_manual_input_range_e;

typedef struct __attribute__((packed))
{
    uint8_t source;        // MANUAL_INPUT_SRC_*
    uint8_t proto;         // sdlog_manual_input_proto_e
    uint8_t range_mode;    // sdlog_manual_input_range_e
    uint8_t channel_count; // valid count in ch_raw[]
    int16_t ch_raw[16];
    uint8_t sw[2];
    uint8_t mouse_btns; // bit0: left, bit1: right
    uint8_t reserved0;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t key_value;
} sdlog_manual_input_raw_t;

typedef struct __attribute__((packed))
{
    uint32_t last_rx_tick_ms;
    uint32_t frame_count;
    uint32_t controller_frame_count;
    uint32_t client_frame_count;
    uint32_t vt13_frame_count;
    uint32_t crc_error_count;
    uint32_t parse_error_count;
    uint32_t restart_count;
    uint16_t last_cmd_id;
    uint8_t port_active;
    uint8_t last_range_mode;
} sdlog_image_link_stats_t;

typedef struct __attribute__((packed))
{
    float acc_norm_g;
    float acc_ref_g;
    float norm_err_g;
    float angle_deg;
    float trust;
    float kp_gain;
    float gyro_norm_dps;
    uint8_t acc_healthy;
    uint8_t acc_rejected;
    uint8_t fusion_mode;
    uint8_t reserved;
} sdlog_imu_trust_t;

typedef struct __attribute__((packed))
{
    uint8_t manual_source;
    uint8_t ChassisMode;
    uint8_t yaw_mode;
    uint8_t pitch_mode;
    uint8_t ShootMode;
    int8_t rc_s0;
    int8_t rc_s1;
    uint8_t reserved0;
    int16_t rc_ch[4];
    float ChassisVxSet;
    float ChassisVySet;
    float ChassisWzSet;
    float ChassisVx;
    float ChassisVy;
    float ChassisWz;
    float yaw_set_deg;
    float yaw_deg;
    float pitch_set_deg;
    float pitch_deg;
    int16_t yaw_current;
    int16_t pitch_current;
    int16_t trigger_current;
    int16_t fric_speed_set_rpm;
} sdlog_control_summary_t;

typedef struct __attribute__((packed))
{
    uint8_t state;
    uint8_t angle_idx;
    uint8_t bullet_idx;
    uint8_t angle_points;
    uint8_t bullet_points;
    uint8_t bullet_ready;
    uint8_t is_stable;
    uint8_t motor_raw_mode;
    uint16_t target_bullet;
    uint16_t bullet_now;
    uint16_t completed_cells;
    uint16_t grid_cells;
    uint32_t state_elapsed_ms;
    uint32_t stable_elapsed_ms;
    int32_t last_error;
    float target_angle;
    float cmd_angle;
    float angle;
    float gyro;
    float current;
    float hold_avg;
    float raw_current_cmd;
    float delta;
} sdlog_pitch_cali_t;

typedef struct __attribute__((packed))
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;

    uint16_t set_offline_time_ms;
    uint16_t set_online_time_ms;

    uint8_t enable;
    uint8_t priority;
    uint8_t error_exist;
    uint8_t is_lost;
    uint8_t data_is_error;
    uint8_t reserved[3];

    float frequency;
} sdlog_detect_item_t;

typedef struct __attribute__((packed))
{
    sdlog_detect_item_t items[14]; // DetectTask.h: DETECT_ERROR_COUNT
} sdlog_detect_status_t;

typedef struct __attribute__((packed))
{
    uint16_t enable_mask;
    uint16_t lost_mask;
    uint16_t data_error_mask;
    uint16_t error_exist_mask;
    uint8_t display_toe;
    uint8_t lost_count;
    uint8_t data_error_count;
    uint8_t error_count;
} sdlog_detect_summary_t;

typedef struct __attribute__((packed))
{
    float ChassisPower;
    float ChassisPowerBuffer;
    float total_current;
    float total_current_limit;
    float current_scale;
    uint8_t RefereeOffline;
    uint8_t robot_id;
    uint8_t reserved[2];
} sdlog_chassis_power_limit_t;

typedef struct __attribute__((packed))
{
    uint8_t axis; // 0: yaw, 1: pitch
    uint8_t soft_limited;
    uint8_t current_limited;
    uint8_t reserved;

    float angle;
    float angle_min;
    float angle_max;
    float gyro_set;

    float current_before;
    float current_after;
    float current_limit;
} sdlog_gimbal_limit_t;

typedef struct __attribute__((packed))
{
    uint8_t active;
    uint8_t reserved8;
    uint16_t reserved16;

    uint32_t dropped;
    uint32_t ring_used;
    uint32_t ring_free;
    uint32_t bytes_flushed;
    uint32_t last_sync_ms;
    int32_t last_error;
} SdLogStats;

typedef struct __attribute__((packed))
{
    uint8_t sd_mounted;
    uint8_t sdlog_active;
    uint16_t reserved16;

    uint32_t sdlog_dropped;
    uint32_t sdlog_ring_used;
    uint32_t sdlog_ring_free;
    uint32_t sdlog_bytes_flushed;
    uint32_t sdlog_last_sync_ms;
    int32_t sdlog_last_error;

    uint32_t heap_free;
    uint32_t heap_ever_free;

    uint32_t stack_gimbal;
    uint32_t stack_chassis;
    uint32_t stack_detect;
    uint32_t stack_calibrate;

    uint32_t GimbalLoopCnt;
    uint32_t ChassisLoopCnt;

    uint16_t cpu_load_permille; // 0..1000
    uint16_t reserved16_2;
} sdlog_sys_stats_t;

#define SDLOG_RT_PROFILER_MAX 12u

typedef struct __attribute__((packed))
{
    uint8_t id;
    uint8_t reserved8[3];
    uint32_t count;
    uint32_t last_us;
    uint32_t max_us;
    uint32_t avg_us;
    uint32_t budget_us;
    uint32_t overrun_count;
} SdLogRtProfEntry;

typedef struct __attribute__((packed))
{
    uint8_t count;
    uint8_t reserved8[3];
    SdLogRtProfEntry entry[SDLOG_RT_PROFILER_MAX];
} SdLogRtProf;

#define SDLOG_WHEELLEG_MIT_CONFIG_VERSION 1u
#define SDLOG_WHEELLEG_MIT_STATUS_VERSION 1u
#define SDLOG_WHEELLEG_MIT_MOTOR_DIAG_VERSION 3u
#define SDLOG_WHEELLEG_MIT_MOTOR_DIAG_COUNT 6u

typedef struct __attribute__((packed))
{
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;
} sdlog_pid_param_t;

typedef struct __attribute__((packed))
{
    uint8_t version;
    uint8_t enable_switch_pos;
    uint16_t control_period_ms;
    uint16_t rc_deadband;
    uint16_t lqr_default_mask; // bit i = lqr row i uses firmware default instead of config row

    uint8_t actuator_id[6]; // left front/back/wheel, right front/back/wheel
    int8_t joint_dir[4];    // left front/back, right front/back
    uint8_t reserved8[2];

    float joint_zero_rad[4]; // left front/back, right front/back

    float l1_m;
    float l2_m;
    float l3_m;
    float l4_m;
    float l5_m;
    float wheel_radius_m;

    float default_leg_length_m;
    float min_leg_length_m;
    float max_leg_length_m;

    float support_bias_n;
    float leg_mass_kg;

    float max_wheel_torque_nm;
    float max_joint_torque_nm;
    float max_jump_joint_torque_nm;
    float max_support_force_n;

    float attitude_limit_rad;
    float observer_lpf;

    float pitch_balance_offset_right_rad;
    float pitch_balance_offset_left_rad;
    float max_v_mps;
    float max_yaw_rate_radps;

    sdlog_pid_param_t leg_length_pid;
    sdlog_pid_param_t leg_split_pid;
    sdlog_pid_param_t turn_pid;
    sdlog_pid_param_t roll_pid;

    float effective_lqr_poly[12][4];
} sdlog_wheelleg_mit_config_t;

typedef struct __attribute__((packed))
{
    uint8_t version;
    uint8_t mode;
    uint8_t last_mode;
    uint8_t controller_active;

    uint16_t fault_flags;
    uint16_t feedback_faults;

    uint8_t test_mode;
    uint8_t manual_on;
    uint8_t profile_on;
    uint8_t reserved8;

    float pitch_rad;
    float roll_rad;
    float yaw_rad;
    float gyro_radps[3];

    float target_v_mps;
    float target_yaw_rate_radps;
    float target_leg_length_m;
    float target_foot_x_m;
    float target_leg_theta_rad;

    float observer_x_m;
    float observer_v_mps;

    float leg_length_m[2];      // left, right
    float leg_theta_rad[2];     // left, right
    float leg_d_length_mps[2];  // left, right
    float leg_d_theta_radps[2]; // left, right

    float support_force_n[2];   // left, right
    float hip_torque_nm[2];     // left, right
    float joint_torque_nm[2][2]; // [side][front/back]

    float wheel_pos_rad[2];     // left, right
    float wheel_vel_radps[2];   // left, right
    float wheel_torque_nm[2];   // left, right

    float lqr_error[6];  // right theta, right dtheta, x, v, right pitch, right gyro
    float lqr_output[4]; // right wheel, left wheel, right hip, left hip

    uint8_t contact[2];       // left, right
    uint8_t motor_online_bits; // bit0 LF, bit1 LB, bit2 LW, bit3 RF, bit4 RB, bit5 RW
    uint8_t reserved8_2;

    uint32_t overrun_count;
} sdlog_wheelleg_mit_status_t;

typedef char _check_sdlog_wheelleg_mit_config_size[(sizeof(sdlog_wheelleg_mit_config_t) == 392) ? 1 : -1];
typedef char _check_sdlog_wheelleg_mit_status_size[(sizeof(sdlog_wheelleg_mit_status_t) == 200) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint8_t role; // 0:LF, 1:LB, 2:LW, 3:RF, 4:RB, 5:RW
    uint8_t actuator_id;
    uint8_t fresh;
    uint8_t fb_online;
    uint8_t cmd_active;
    uint8_t cmd_mode;
    uint8_t applied_active;
    uint8_t applied_mode;
    uint8_t applied_drive_state;
    uint8_t applied_flags;
    uint8_t fb_bus;
    uint8_t fb_rx_dlc;
    uint8_t fb_rx_data0;
    uint8_t fb_rx_data0_low4;
    uint8_t fb_rx_data0_high4;
    uint8_t fb_motor_id;
    uint8_t fb_state;
    uint8_t reserved8;
    uint16_t fb_rx_id;
    uint16_t applied_tx_id;
    uint16_t cmd_writer;
    uint16_t cmd_timeout_ms;
    uint32_t fb_rx_count;
    uint32_t tx_id_count;
    uint32_t fb_last_rx_tick_ms;
    uint32_t cmd_seq;
    uint32_t cmd_tick_ms;
    uint32_t applied_tick_ms;
    float cmd_position_rad;
    float cmd_velocity_radps;
    float cmd_kp;
    float cmd_kd;
    float cmd_torque_nm;
    float applied_torque_nm;
    float fb_position_rad;
    float fb_velocity_radps;
    float fb_torque_nm;
} sdlog_wheelleg_mit_motor_diag_entry_t;

typedef struct __attribute__((packed))
{
    uint8_t version;
    uint8_t count;
    uint16_t reserved16;
    uint32_t can1_rx_count;
    uint32_t can1_rx_drop;
    uint32_t can1_tx_count;
    uint32_t can1_tx_fail;
    uint32_t can2_rx_count;
    uint32_t can2_rx_drop;
    uint32_t can2_tx_count;
    uint32_t can2_tx_fail;
    sdlog_wheelleg_mit_motor_diag_entry_t motor[SDLOG_WHEELLEG_MIT_MOTOR_DIAG_COUNT];
} sdlog_wheelleg_mit_motor_diag_t;

typedef char _check_sdlog_wheelleg_mit_motor_diag_entry_size[(sizeof(sdlog_wheelleg_mit_motor_diag_entry_t) == 86) ? 1 : -1];
typedef char _check_sdlog_wheelleg_mit_motor_diag_size[(sizeof(sdlog_wheelleg_mit_motor_diag_t) == 552) ? 1 : -1];

typedef enum
{
    SDLOG_EVT_TOE_LOST = 1u,
    SDLOG_EVT_TOE_RECOVER = 2u,
    SDLOG_EVT_TOE_DATA_ERROR = 3u,
    SDLOG_EVT_CONFIG_CHANGED = 10u,
    SDLOG_EVT_SDLOG_DROPPED = 20u,
    SDLOG_EVT_SD_CARD_MOUNT = 30u,
    SDLOG_EVT_SDLOG_RESTART_INFO = 31u,
    SDLOG_EVT_TEST_MODE = 40u,
    SDLOG_EVT_GIMBAL_BEHAVIOUR = 41u,
    SDLOG_EVT_CHASSIS_BEHAVIOUR = 42u,
    SDLOG_EVT_SHOOT_MODE = 43u,
    SDLOG_EVT_MANUAL_SOURCE_SWITCH = 44u,
} sdlog_event_id_e;

typedef struct __attribute__((packed))
{
    uint16_t event_id; // sdlog_event_id_e
    uint16_t arg0_u16;
    uint32_t arg1_u32;
    uint32_t arg2_u32;
} sdlog_event_t;

// Runtime view of PidTypeDef, enough for offline replay.
typedef struct
{
    uint16_t PidId;
    uint8_t mode;
    uint8_t reserved;

    float set;
    float fdb;
    float out;
} sdlog_pid_runtime_t;

// Legacy full snapshot of PidTypeDef, kept for old log decode reference.
typedef struct
{
    uint16_t PidId;
    uint8_t mode;
    uint8_t reserved;

    float Kp;
    float Ki;
    float Kd;
    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;

    float Dbuf[3];
    float error[3];
} sdlog_pid_snapshot_t;

// Legacy full snapshot of GimbalPid, kept for old log decode reference.
typedef struct
{
    uint16_t PidId;
    uint16_t reserved;

    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} sdlog_gimbal_pid_snapshot_t;

#define SDLOG_FRIC_NUM 4u

typedef struct
{
    uint32_t loop_cnt;
    int16_t yaw_current;
    int16_t pitch_current;
    int16_t trigger_current;
    uint16_t reserved16;
    uint8_t yaw_mode;
    uint8_t pitch_mode;
    uint8_t ShootMode;
    uint8_t reserved8;

    sdlog_pid_runtime_t yaw_angle_pid;
    sdlog_pid_runtime_t yaw_speed_pid;
    sdlog_pid_runtime_t pitch_angle_pid;
    sdlog_pid_runtime_t pitch_speed_pid;

    sdlog_pid_runtime_t ShootTriggerPid;
    sdlog_pid_runtime_t ShootFricPid[SDLOG_FRIC_NUM];
} sdlog_gimbal_loop_t;

typedef struct
{
    uint32_t loop_cnt;
    uint8_t ChassisMode;
    uint8_t last_chassis_mode;
    uint16_t reserved;

    float vx;
    float vy;
    float wz;
    float vx_set;
    float vy_set;
    float wz_set;

    sdlog_pid_runtime_t motor_speed_pid[4];
    sdlog_pid_runtime_t ChassisAnglePid;
} sdlog_chassis_loop_t;

#define SDLOG_CHASSIS_BASE_STREAM_VERSION 1u
#define SDLOG_GIMBAL_BASE_STREAM_VERSION 1u
#define SDLOG_IMU_BASE_STREAM_VERSION_FLOAT 1u
#define SDLOG_IMU_BASE_STREAM_VERSION_FIXED_TEMP 2u
#define SDLOG_IMU_BASE_STREAM_VERSION_FIXED 3u

#ifndef SDLOG_IMU_BASE_ENABLE_TEMP
#define SDLOG_IMU_BASE_ENABLE_TEMP 1
#endif

#define SDLOG_IMU_BASE_QUAT_SCALE 32767.0f
#define SDLOG_IMU_BASE_GYRO_SCALE 512.0f
#define SDLOG_IMU_BASE_ACCEL_SCALE 128.0f
#define SDLOG_IMU_BASE_TEMP_SCALE 100.0f

#if SDLOG_IMU_BASE_ENABLE_TEMP
#define SDLOG_IMU_BASE_STREAM_VERSION SDLOG_IMU_BASE_STREAM_VERSION_FIXED_TEMP
#else
#define SDLOG_IMU_BASE_STREAM_VERSION SDLOG_IMU_BASE_STREAM_VERSION_FIXED
#endif

typedef struct __attribute__((packed))
{
    uint32_t start_tick_ms;
    uint16_t period_us;
    uint8_t sample_count;
    uint8_t version;
} sdlog_chassis_base_stream_header_t;

typedef struct __attribute__((packed))
{
    uint8_t ChassisMode;
    uint8_t last_chassis_mode;
    uint16_t reserved;
    int16_t wheel_rpm[4];
    int16_t current_request[4];
    int16_t current_output[4];
} sdlog_chassis_base_sample_t;

typedef char _check_sdlog_chassis_base_sample_size[(sizeof(sdlog_chassis_base_sample_t) == 28) ? 1 : -1];
typedef char _check_sdlog_chassis_base_stream_header_size[(sizeof(sdlog_chassis_base_stream_header_t) == 8) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint32_t start_tick_ms;
    uint16_t period_us;
    uint8_t sample_count;
    uint8_t version;
} sdlog_gimbal_base_stream_header_t;

typedef struct __attribute__((packed))
{
    uint8_t GimbalBehaviour;
    uint8_t test_mode;
    uint8_t yaw_motor_mode;
    uint8_t pitch_motor_mode;
    float yaw_angle;
    float pitch_angle;
    float yaw_gyro;
    float pitch_gyro;
    int16_t yaw_current_request;
    int16_t pitch_current_request;
    int16_t yaw_current_output;
    int16_t pitch_current_output;
} sdlog_gimbal_base_sample_t;

typedef char _check_sdlog_gimbal_base_sample_size[(sizeof(sdlog_gimbal_base_sample_t) == 28) ? 1 : -1];
typedef char _check_sdlog_gimbal_base_stream_header_size[(sizeof(sdlog_gimbal_base_stream_header_t) == 8) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint32_t start_tick_ms;
    uint16_t period_us;
    uint8_t sample_count;
    uint8_t version;
} sdlog_imu_base_stream_header_t;

typedef struct __attribute__((packed))
{
    int16_t quat_q15[4];
    int16_t gyro_rad_s_q9[3];
    int16_t accel_m_s2_q7[3];
#if SDLOG_IMU_BASE_ENABLE_TEMP
    int16_t temp_centi_c;
#endif
} sdlog_imu_base_sample_t;

#if SDLOG_IMU_BASE_ENABLE_TEMP
typedef char _check_sdlog_imu_base_sample_size[(sizeof(sdlog_imu_base_sample_t) == 22) ? 1 : -1];
#else
typedef char _check_sdlog_imu_base_sample_size[(sizeof(sdlog_imu_base_sample_t) == 20) ? 1 : -1];
#endif
typedef char _check_sdlog_imu_base_stream_header_size[(sizeof(sdlog_imu_base_stream_header_t) == 8) ? 1 : -1];

static inline int16_t SdLogI16FromScaledFloat(float value, float scale)
{
    if (!(value == value))
    {
        return 0;
    }

    const float scaled = value * scale;
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static inline void SdLogImuBaseSampleSet(sdlog_imu_base_sample_t *sample,
                                         const float quat[4],
                                         const float gyro[3],
                                         const float accel[3],
                                         float temp_c)
{
    if (sample == 0)
    {
        return;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        const float v = (quat != 0) ? quat[i] : 0.0f;
        sample->quat_q15[i] = SdLogI16FromScaledFloat(v, SDLOG_IMU_BASE_QUAT_SCALE);
    }
    for (uint8_t i = 0u; i < 3u; i++)
    {
        const float g = (gyro != 0) ? gyro[i] : 0.0f;
        const float a = (accel != 0) ? accel[i] : 0.0f;
        sample->gyro_rad_s_q9[i] = SdLogI16FromScaledFloat(g, SDLOG_IMU_BASE_GYRO_SCALE);
        sample->accel_m_s2_q7[i] = SdLogI16FromScaledFloat(a, SDLOG_IMU_BASE_ACCEL_SCALE);
    }
#if SDLOG_IMU_BASE_ENABLE_TEMP
    sample->temp_centi_c = SdLogI16FromScaledFloat(temp_c, SDLOG_IMU_BASE_TEMP_SCALE);
#else
    (void)temp_c;
#endif
}

// ===== Runtime API =====

int SdLogIsActive(void);
uint32_t SdLogDropped(void);

// Start (open new log file) if the TF/SD card is mounted.
// Returns 0 on success, non-zero on failure.
int SdLogStart(void);

// Stop logging and close the current log file (safe to call even if inactive).
void SdLogStop(void);

// Append one record into the RAM ring buffer (non-blocking, drops if full).
void SdLogWrite(uint16_t tag, const void *payload, uint16_t len);

// Same as SdLogWrite(), but callable from ISR context.
void SdLogWriteIsr(uint16_t tag, const void *payload, uint16_t len);

// Read-only stats snapshot (thread-safe, lightweight).
void SdLogGetStats(SdLogStats *out);

// Runtime divider for high-rate streams. Returns 1, 2, or 4.
uint8_t SdLogHighRateDiv(void);

// Flush pending buffered records to the TF/SD card (call from a low-priority task).
void SdLogPoll(void);

#endif
