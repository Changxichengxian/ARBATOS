/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "host_link_task.h"
#include "elrs_task.h"

#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "config.h"
#include "control_input.h"
#include "watch.h"
#include "user_lib.h"
#include "chassis_control_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_control_task.h"
#include "mem_mang.h"
#include "manual_input.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "shoot.h"
#include "bsp_time.h"
#include "bsp_usart.h"
#include "battery_monitor_task.h"
#include "referee.h"
#include "bsp_key.h"
#include "sdlog.h"

// ===== Aux tuning/telemetry / image remote link =====
#define AUX_TUNE_RX_LINE_MAX     96u
#define AUX_TUNE_BAUD            230400u
// Extra backoff applied when cfg->period_ms==0 (auto). This is intentionally
// conservative for wireless UART bridges that may buffer/retransmit and have
// lower effective throughput than the UART baud rate.
#define AUX_TELEM_AUTO_EXTRA_BACKOFF_PCT 50u
#ifndef AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
#define AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP 0u
#endif

typedef struct
{
    const manual_input_state_t *rc;
    const fp32 *quat;
    const fp32 *angle;
    const fp32 *gyro;
    const fp32 *accel;
    const gimbal_motor_t *yaw;
    const gimbal_motor_t *pitch;
    const chassis_move_t *chassis;
    const motor_measure_t *trigger_meas;
    const motor_measure_t *fric_meas[FRIC_MOTOR_NUM];
} aux_telem_ctx_t;

typedef enum
{
    AUX_AUTOTUNE_TARGET_NONE = 0u,
    AUX_AUTOTUNE_TARGET_PITCH_SPEED,
    AUX_AUTOTUNE_TARGET_PITCH_ANGLE,
    AUX_AUTOTUNE_TARGET_YAW_SPEED,
    AUX_AUTOTUNE_TARGET_YAW_ANGLE,
    AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW,
    AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED,
} aux_autotune_target_e;

typedef struct
{
    uint8_t enabled;
    uint8_t target; // aux_autotune_target_e
    uint16_t period_ms;
    uint32_t last_tick_ms;
} aux_autotune_stream_t;

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
typedef enum
{
    CONFIG_PARAM_SCOPE_COMMON = 0u,
    CONFIG_PARAM_SCOPE_GIMBAL_SINGLE,
    CONFIG_PARAM_SCOPE_GIMBAL_DUAL,
    CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT,
} config_param_scope_e;
#endif

typedef enum
{
    CONFIG_PARAM_APPLY_NONE = 0u,
    CONFIG_PARAM_APPLY_REMOTE_REFRESH,
    CONFIG_PARAM_APPLY_GIMBAL_YAW_SPEED_PID,
    CONFIG_PARAM_APPLY_GIMBAL_PITCH_SPEED_PID,
    CONFIG_PARAM_APPLY_GIMBAL_YAW_ANGLE_PID,
    CONFIG_PARAM_APPLY_GIMBAL_PITCH_ANGLE_PID,
    CONFIG_PARAM_APPLY_CHASSIS_MOTOR_SPEED_PID,
    CONFIG_PARAM_APPLY_CHASSIS_FOLLOW_PID,
    CONFIG_PARAM_APPLY_SHOOT_FRIC_SPEED_PID,
    CONFIG_PARAM_APPLY_SHOOT_TRIGGER_PID,
} config_param_apply_e;

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
typedef struct
{
    uint16_t id;
    const char *name;
    uint8_t scope; // config_param_scope_e
} config_param_desc_t;

static const config_param_desc_t g_config_param_descs[] =
{
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
};
#endif

static char aux_rx_line[AUX_TUNE_RX_LINE_MAX];
static volatile uint16_t aux_rx_len = 0;
static char aux_cmd_line[AUX_TUNE_RX_LINE_MAX];
static volatile bool_t aux_cmd_ready = 0;

static uint32_t aux_telem_tick = 0;
static volatile uint32_t aux_cmd_seq = 0;
static uint8_t aux_telem_frame[(AUX_TELEM_MAX_CH + 1u) * 4u];
static aux_autotune_stream_t aux_autotune = {0};
static uint8_t aux_autotune_frame[(8u + 1u) * 4u];

// ===== Aux port mode / ELRS(CRSF) RX / image remote link =====
// ELRS/CRSF RX is implemented in elrs_task.c; this file only routes aux-port callbacks.

// Default list (legacy full list with reduced bandwidth):
// - Remove per-signal offline/mode/err channels
// - Keep PACK_MODE / PACK_OFFLINE for status
static const aux_telem_sig_e aux_telem_default_list[] =
{
    AUX_TELEM_SIG_SYS_TICK_MS,
    AUX_TELEM_SIG_SYS_AUX_CMD_SEQ,
    AUX_TELEM_SIG_SYS_BATTERY_VOLT,
    AUX_TELEM_SIG_SYS_BATTERY_PERCENT,
    AUX_TELEM_SIG_RC_CH0,
    AUX_TELEM_SIG_RC_CH1,
    AUX_TELEM_SIG_RC_CH2,
    AUX_TELEM_SIG_RC_CH3,
    AUX_TELEM_SIG_RC_CH4,
    AUX_TELEM_SIG_RC_S0,
    AUX_TELEM_SIG_RC_S1,
    AUX_TELEM_SIG_RC_MOUSE_X,
    AUX_TELEM_SIG_RC_MOUSE_Y,
    AUX_TELEM_SIG_RC_MOUSE_Z,
    AUX_TELEM_SIG_RC_MOUSE_L,
    AUX_TELEM_SIG_RC_MOUSE_R,
    AUX_TELEM_SIG_RC_KEY,
    AUX_TELEM_SIG_RC_ERROR,
    AUX_TELEM_SIG_IMU_Q0,
    AUX_TELEM_SIG_IMU_Q1,
    AUX_TELEM_SIG_IMU_Q2,
    AUX_TELEM_SIG_IMU_Q3,
    AUX_TELEM_SIG_IMU_ANGLE_YAW_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_YAW_DEG,
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_DEG,
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_DEG,
    AUX_TELEM_SIG_IMU_GYRO_X_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_Y_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_Z_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_X_DPS,
    AUX_TELEM_SIG_IMU_GYRO_Y_DPS,
    AUX_TELEM_SIG_IMU_GYRO_Z_DPS,
    AUX_TELEM_SIG_IMU_ACCEL_X,
    AUX_TELEM_SIG_IMU_ACCEL_Y,
    AUX_TELEM_SIG_IMU_ACCEL_Z,
    AUX_TELEM_SIG_PACK_MODE,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO,
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_MOTOR_SPEED,
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_GIVEN_CURRENT,
    AUX_TELEM_SIG_GIMBAL_YAW_RAW_CMD_CURRENT,
    AUX_TELEM_SIG_GIMBAL_YAW_ECD,
    AUX_TELEM_SIG_GIMBAL_YAW_OFFSET_ECD,
    AUX_TELEM_SIG_GIMBAL_YAW_RPM,
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_FB,
    AUX_TELEM_SIG_GIMBAL_YAW_TEMP,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO,
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_MOTOR_SPEED,
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_GIVEN_CURRENT,
    AUX_TELEM_SIG_GIMBAL_PITCH_RAW_CMD_CURRENT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ECD,
    AUX_TELEM_SIG_GIMBAL_PITCH_OFFSET_ECD,
    AUX_TELEM_SIG_GIMBAL_PITCH_RPM,
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_FB,
    AUX_TELEM_SIG_GIMBAL_PITCH_TEMP,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_GET,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_GET,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_FDB,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DBUF0,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_FDB,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DBUF0,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_VX_SET,
    AUX_TELEM_SIG_CHASSIS_VY_SET,
    AUX_TELEM_SIG_CHASSIS_WZ_SET,
    AUX_TELEM_SIG_CHASSIS_VX,
    AUX_TELEM_SIG_CHASSIS_VY,
    AUX_TELEM_SIG_CHASSIS_WZ,
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET,
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET_SET,
    AUX_TELEM_SIG_CHASSIS_YAW_SET,
    AUX_TELEM_SIG_CHASSIS_YAW,
    AUX_TELEM_SIG_CHASSIS_PITCH,
    AUX_TELEM_SIG_CHASSIS_ROLL,
    AUX_TELEM_SIG_CHASSIS_SWING_KEY,
    AUX_TELEM_SIG_CHASSIS_M0_RPM,
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M0_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M0_SPEED,
    AUX_TELEM_SIG_CHASSIS_M0_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M0_ECD,
    AUX_TELEM_SIG_CHASSIS_M0_TEMP,
    AUX_TELEM_SIG_CHASSIS_M1_RPM,
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M1_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M1_SPEED,
    AUX_TELEM_SIG_CHASSIS_M1_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M1_ECD,
    AUX_TELEM_SIG_CHASSIS_M1_TEMP,
    AUX_TELEM_SIG_CHASSIS_M2_RPM,
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M2_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M2_SPEED,
    AUX_TELEM_SIG_CHASSIS_M2_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M2_ECD,
    AUX_TELEM_SIG_CHASSIS_M2_TEMP,
    AUX_TELEM_SIG_CHASSIS_M3_RPM,
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M3_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M3_SPEED,
    AUX_TELEM_SIG_CHASSIS_M3_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M3_ECD,
    AUX_TELEM_SIG_CHASSIS_M3_TEMP,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT,
    AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_GIVEN_CURRENT,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT,
    AUX_TELEM_SIG_SHOOT_PRESS_L,
    AUX_TELEM_SIG_SHOOT_PRESS_R,
    AUX_TELEM_SIG_SHOOT_KEY,
    AUX_TELEM_SIG_SHOOT_HEAT_LIMIT,
    AUX_TELEM_SIG_SHOOT_HEAT,
    AUX_TELEM_SIG_SHOOT_FRIC0_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC0_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC1_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC1_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC2_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC2_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC3_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC3_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_TRIGGER_RPM,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD,
    AUX_TELEM_SIG_SHOOT_TRIGGER_TEMP,
    AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB,
    AUX_TELEM_SIG_DIAG_CAN1_0X200_M1,
    AUX_TELEM_SIG_DIAG_CAN1_0X200_M2,
    AUX_TELEM_SIG_DIAG_CAN1_0X200_PITCH,
    AUX_TELEM_SIG_DIAG_CAN1_0X200_TRIGGER,
    AUX_TELEM_SIG_DIAG_CAN1_0X1FF_M4,
    AUX_TELEM_SIG_DIAG_CAN1_0X1FF_YAW,
    AUX_TELEM_SIG_DIAG_CAN1_0X1FF_M3,
    AUX_TELEM_SIG_DIAG_CAN1_1FF_STATUS,
    AUX_TELEM_SIG_DIAG_CAN1_ERR,
    AUX_TELEM_SIG_PACK_OFFLINE,
    AUX_TELEM_SIG_DIAG_ZERO_FORCE,
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT,
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT,
    AUX_TELEM_SIG_MEM_HEAP_FREE,
    AUX_TELEM_SIG_MEM_HEAP_EVER_FREE,
    AUX_TELEM_SIG_BOARD_KEY_DOWN,
    AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT,
};

typedef char _check_aux_telem_default_list_fits[(sizeof(aux_telem_default_list) / sizeof(aux_telem_default_list[0]) <= AUX_TELEM_MAX_CH) ? 1 : -1];

static const fp32 *ins_quat;
static const fp32 *ins_angle;
static const fp32 *ins_gyro;
static const fp32 *ins_accel;

extern ext_shoot_data_t shoot_data_t;
extern ext_bullet_remaining_t bullet_remaining_t;
extern shoot_control_t shoot_control;

// Some targets do not wire in battery_monitor_task.c yet. Keep USB telemetry linkable
// by providing weak zero defaults that are overridden when the real task exists.
__weak fp32 battery_voltage = 0.0f;
__weak fp32 electricity_percentage = 0.0f;

static void aux_port_init(void);
static void aux_port_poll(void);
static void aux_port_stop(void);
static bool_t aux_port_apply_baud(uint32_t baud);
static uint8_t aux_port_is_elrs_mode(uint32_t baud);
static uint8_t aux_port_is_image_mode(uint32_t baud);
static uint8_t aux_port_is_tune_mode(uint32_t baud);
static void aux_tune_rx_start(void);
static void aux_tune_on_byte(uint8_t b);
static uint8_t aux_tune_on_uart_error(void);
static bool_t aux_tune_handle_line(const char *line);
static void aux_tune_try_send_telem(void);
static void aux_tune_poll(void);
static void aux_autotune_stop(void);
static bool_t aux_autotune_parse_target(const char *s, aux_autotune_target_e *out);
static bool_t aux_autotune_fill_fields(fp32 *fields, uint32_t *out_tick_ms);
static bool_t aux_autotune_try_send_frame(void);
static bool_t aux_tune_parse_fp32(const char *s, fp32 *out);
static bool_t aux_tune_parse_u16(const char *s, uint16_t *out);
static bool_t aux_tune_parse_u32(const char *s, uint32_t *out);
#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
static const config_param_desc_t *aux_tune_find_config_param_by_name(const char *name);
#endif
static void aux_tune_apply_config_param(config_param_apply_e action);
static bool_t aux_tune_set_app_param(uint16_t id, fp32 value);
static uint16_t aux_telem_min_period_ms(uint16_t channel_num);
static fp32 aux_telem_get_value(const aux_telem_ctx_t *ctx, aux_telem_sig_e sig);

void host_link_task(void const * argument)
{
    ins_quat = get_INS_quat_point();
    ins_angle = get_INS_angle_point();
    ins_gyro = get_gyro_data_point();
    ins_accel = get_accel_data_point();

    vision_link_init(ins_quat, ins_angle, ins_gyro);
    aux_port_init();

    while(1)
    {
        watch_task_beat(WATCH_TASK_HOST_LINK);
        vision_link_poll_tx();
        aux_port_poll();

        osDelay(2);
    }
}

static void aux_port_init(void)
{
    const uint32_t baud = bsp_aux_link_get_baudrate();

    aux_port_stop();

    if (aux_port_is_elrs_mode(baud))
    {
        bsp_aux_link_set_rx_event_cb(elrs_link_on_rx_event);
        bsp_aux_link_set_rx_byte_cb(elrs_link_on_it_byte);
        bsp_aux_link_set_error_cb(elrs_link_on_uart_error);
        elrs_link_rx_start();
    }
    else if (aux_port_is_tune_mode(baud))
    {
        aux_tune_rx_start();
    }
    else if (aux_port_is_image_mode(baud))
    {
        image_remote_link_start();
    }
}

static void aux_port_poll(void)
{
    const uint32_t baud = bsp_aux_link_get_baudrate();

    if (aux_port_is_tune_mode(baud))
    {
        aux_tune_poll();
        if (aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
        {
            aux_tune_try_send_telem();
        }
    }
    else if (aux_port_is_image_mode(baud))
    {
        image_remote_link_poll();
    }
}

static void aux_port_stop(void)
{
    image_remote_link_stop();
    elrs_link_stop();
    aux_rx_len = 0u;
    aux_cmd_ready = 0;

    bsp_aux_link_set_rx_event_cb(NULL);
    bsp_aux_link_set_rx_byte_cb(NULL);
    bsp_aux_link_set_error_cb(NULL);
    bsp_aux_link_rx_it_stop();
}

static bool_t aux_port_apply_baud(uint32_t baud)
{
    if (!aux_port_is_tune_mode(baud) &&
        !aux_port_is_elrs_mode(baud) &&
        !aux_port_is_image_mode(baud))
    {
        return 0;
    }

    const uint32_t old_baud = bsp_aux_link_get_baudrate();
    if (old_baud == baud)
    {
        aux_port_init();
        return 1;
    }

    aux_port_stop();
    if (bsp_aux_link_set_baudrate(baud) != 0)
    {
        (void)bsp_aux_link_set_baudrate(old_baud);
        aux_port_init();
        return 0;
    }

    aux_port_init();
    return 1;
}

static uint8_t aux_port_is_elrs_mode(uint32_t baud)
{
    return (baud == ELRS_LINK_BAUD) ? 1u : 0u;
}

static uint8_t aux_port_is_image_mode(uint32_t baud)
{
    return (baud == IMAGE_REMOTE_LINK_BAUD) ? 1u : 0u;
}

static uint8_t aux_port_is_tune_mode(uint32_t baud)
{
    return (baud == AUX_TUNE_BAUD) ? 1u : 0u;
}

static void aux_tune_rx_start(void)
{
    image_remote_link_stop();
    elrs_link_stop();
    aux_rx_len = 0;
    aux_cmd_ready = 0;
    aux_cmd_seq = 0;
    aux_telem_tick = 0u;
    aux_autotune.last_tick_ms = 0u;

    bsp_aux_link_set_rx_event_cb(NULL);
    bsp_aux_link_set_rx_byte_cb(aux_tune_on_byte);
    bsp_aux_link_set_error_cb(aux_tune_on_uart_error);
    (void)bsp_aux_link_rx_it_start();
}

static void aux_tune_try_send_telem(void)
{
    if (aux_autotune_try_send_frame())
    {
        return;
    }

    const aux_telem_config_t *cfg = &g_config.aux_telem;
    const uint8_t want_uart_justfloat = ((cfg->enable == 1u) || (cfg->enable == 4u)) ? 1u : 0u;
    // Aux telemetry only runs in the dedicated tuning mode.
    const uint8_t aux_can_tx = aux_port_is_tune_mode(bsp_aux_link_get_baudrate());
    if (!(want_uart_justfloat && aux_can_tx))
    {
        return;
    }

    // Telemetry list selection:
    // - channel_num == 0: use built-in default list.
    // - channel_num != 0: send the first channel_num entries in channel_map[].
    uint8_t use_default_list = 0u;
    uint16_t channel_num = cfg->channel_num;
    if (channel_num == 0u)
    {
        use_default_list = 1u;
        channel_num = (uint16_t)(sizeof(aux_telem_default_list) / sizeof(aux_telem_default_list[0]));
    }
    if (channel_num > AUX_TELEM_MAX_CH)
    {
        channel_num = AUX_TELEM_MAX_CH;
    }
    if (channel_num == 0u)
    {
        return;
    }

    uint16_t period_ms = cfg->period_ms;
    const uint16_t min_period_ms = aux_telem_min_period_ms(channel_num);
    if (period_ms == 0u)
    {
        uint32_t auto_ms = ((uint32_t)min_period_ms * (100u + AUX_TELEM_AUTO_EXTRA_BACKOFF_PCT) + 99u) / 100u;
        if (auto_ms < 1u)
        {
            auto_ms = 1u;
        }
        if (auto_ms > 1000u)
        {
            auto_ms = 1000u;
        }
        period_ms = (uint16_t)auto_ms;
    }
    else if (period_ms < min_period_ms)
    {
        period_ms = min_period_ms;
    }

    const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((uint32_t)(now - aux_telem_tick) < period_ms)
    {
        return;
    }

    // If TX DMA is busy, skip (non-blocking).
    if (!bsp_aux_link_tx_ready())
    {
        return;
    }

    aux_telem_ctx_t ctx = {0};
    ctx.rc = get_remote_control_point();
    ctx.quat = ins_quat;
    ctx.angle = ins_angle;
    ctx.gyro = ins_gyro;
    ctx.accel = ins_accel;
    ctx.yaw = get_yaw_motor_point();
    ctx.pitch = get_pitch_motor_point();
    ctx.chassis = get_chassis_move_point();
    ctx.trigger_meas = get_trigger_motor_measure_point();
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        ctx.fric_meas[i] = get_friction_motor_measure_point(i);
    }

    if (use_default_list)
    {
        for (uint32_t i = 0; i < channel_num; i++)
        {
            const aux_telem_sig_e sig = aux_telem_default_list[i];
            const fp32 v = aux_telem_get_value(&ctx, sig);
            memcpy(&aux_telem_frame[i * 4u], &v, 4u);
        }
    }
    else
    {
        for (uint32_t i = 0; i < channel_num; i++)
        {
            uint16_t sig_id = cfg->channel_map[i];
            if (sig_id >= (uint16_t)AUX_TELEM_SIG__COUNT)
            {
                sig_id = 0u;
            }
            const aux_telem_sig_e sig = (aux_telem_sig_e)sig_id;
            const fp32 v = aux_telem_get_value(&ctx, sig);
            memcpy(&aux_telem_frame[i * 4u], &v, 4u);
        }
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&aux_telem_frame[channel_num * 4u], &tail, 4u);

    const uint16_t frame_len = (uint16_t)((channel_num + 1u) * 4u);
    if (bsp_aux_link_tx_dma(aux_telem_frame, frame_len) == 0)
    {
        aux_telem_tick = now;
    }
}

static bool_t aux_tune_handle_line(const char *line)
{
    if (line == NULL)
    {
        return 0;
    }

    char buf[AUX_TUNE_RX_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1u);
    buf[sizeof(buf) - 1u] = '\0';

    // to lower
    for (char *p = buf; *p != '\0'; p++)
    {
        if (*p >= 'A' && *p <= 'Z')
        {
            *p = (char)(*p - 'A' + 'a');
        }
    }

    // Fast path: "<id>:<value>" sets one config parameter.
    char *colon = strchr(buf, ':');
    if (colon != NULL)
    {
        *colon = '\0';
        char *key_s = buf;
        char *v_s = colon + 1;

        while (*key_s == ' ' || *key_s == '\t')
        {
            key_s++;
        }
        while (*v_s == ' ' || *v_s == '\t')
        {
            v_s++;
        }

        char *key_end = key_s + strlen(key_s);
        while (key_end > key_s && (key_end[-1] == ' ' || key_end[-1] == '\t'))
        {
            key_end--;
        }
        *key_end = '\0';

        char *v_end = v_s + strlen(v_s);
        while (v_end > v_s && (v_end[-1] == ' ' || v_end[-1] == '\t'))
        {
            v_end--;
        }
        *v_end = '\0';

        uint16_t id = 0;
        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(v_s, &v))
        {
            return 0;
        }

        if (aux_tune_parse_u16(key_s, &id))
        {
            if (aux_tune_set_app_param(id, v))
            {
                aux_cmd_seq++;
                return 1;
            }
            return 0;
        }

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
        const config_param_desc_t *desc = aux_tune_find_config_param_by_name(key_s);
        if (desc != NULL && aux_tune_set_app_param(desc->id, v))
        {
            aux_cmd_seq++;
            return 1;
        }
#endif

        return 0;
    }

    // split tokens
    char *argv[6];
    int argc = 0;
    char *p = buf;
    while (*p != '\0' && argc < (int)(sizeof(argv) / sizeof(argv[0])))
    {
        while (*p == ' ' || *p == '\t')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        *p = '\0';
        p++;
    }

    if (argc == 0)
    {
        return 0;
    }

    if (strcmp(argv[0], "clear") == 0)
    {
        gimbal_tune_clear_pitch_pid();
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "view") == 0)
    {
        aux_cmd_seq++;
        return 0;
    }

    if (strcmp(argv[0], "aux") == 0 || strcmp(argv[0], "u1") == 0 || strcmp(argv[0], "uart1") == 0)
    {
        if (argc < 3)
        {
            return 0;
        }

        if (strcmp(argv[1], "mode") == 0)
        {
            uint32_t baud = 0u;
            if (strcmp(argv[2], "tune") == 0)
            {
                baud = AUX_TUNE_BAUD;
            }
            else if (strcmp(argv[2], "elrs") == 0)
            {
                baud = ELRS_LINK_BAUD;
            }
            else if (strcmp(argv[2], "image") == 0)
            {
                baud = IMAGE_REMOTE_LINK_BAUD;
            }
            else
            {
                return 0;
            }

            if (!aux_port_apply_baud(baud))
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "baud") == 0)
        {
            uint32_t baud = 0u;
            if (!aux_tune_parse_u32(argv[2], &baud))
            {
                return 0;
            }
            if (!aux_port_apply_baud(baud))
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        return 0;
    }

    if (strcmp(argv[0], "at") == 0 || strcmp(argv[0], "autotune") == 0)
    {
        if (argc < 2)
        {
            return 0;
        }

        if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "stop") == 0)
        {
            aux_autotune_stop();
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "period") == 0)
        {
            uint32_t period_ms = 0u;
            if (argc < 3 || !aux_tune_parse_u32(argv[2], &period_ms))
            {
                return 0;
            }

            if (period_ms > 1000u)
            {
                period_ms = 1000u;
            }

            aux_autotune.period_ms = (uint16_t)period_ms;
            aux_autotune.last_tick_ms = 0u;
            aux_cmd_seq++;
            return 1;
        }

        const char *target_s = argv[1];
        if (strcmp(argv[1], "target") == 0)
        {
            if (argc < 3)
            {
                return 0;
            }
            target_s = argv[2];
        }

        aux_autotune_target_e target = AUX_AUTOTUNE_TARGET_NONE;
        if (!aux_autotune_parse_target(target_s, &target))
        {
            return 0;
        }

        aux_autotune.enabled = 1u;
        aux_autotune.target = (uint8_t)target;
        if (aux_autotune.period_ms == 0u)
        {
            aux_autotune.period_ms = 20u;
        }
        aux_autotune.last_tick_ms = 0u;
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "cf") == 0)
    {
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            chassis_tune_clear_follow_pid();
            aux_cmd_seq++;
            return 1;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        chassis_tune_get_follow_pid(&pid);

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        chassis_tune_set_follow_pid(&pid, 1);
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "cm") == 0)
    {
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        chassis_tune_get_motor_speed_pid(&pid);

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        chassis_tune_set_motor_speed_pid(&pid, 1);
        aux_cmd_seq++;
        return 1;
    }

    const bool_t is_pitch_speed = (strcmp(argv[0], "ps") == 0);
    const bool_t is_pitch_angle = (strcmp(argv[0], "pa") == 0);
    const bool_t is_yaw_speed = (strcmp(argv[0], "ys") == 0);
    const bool_t is_yaw_angle = (strcmp(argv[0], "ya") == 0);
    if (is_pitch_speed || is_pitch_angle || is_yaw_speed || is_yaw_angle)
    {
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            if (is_pitch_speed || is_pitch_angle)
            {
                gimbal_tune_clear_pitch_pid();
            }
            else
            {
                gimbal_tune_clear_yaw_pid();
            }
            aux_cmd_seq++;
            return 1;
        }

        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        if (is_pitch_speed)
        {
            gimbal_tune_get_pitch_speed_pid(&pid);
        }
        else if (is_pitch_angle)
        {
            gimbal_tune_get_pitch_angle_pid(&pid);
        }
        else if (is_yaw_speed)
        {
            gimbal_tune_get_yaw_speed_pid(&pid);
        }
        else
        {
            gimbal_tune_get_yaw_angle_pid(&pid);
        }

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        if (is_pitch_speed)
        {
            gimbal_tune_set_pitch_speed_pid(&pid, 1);
        }
        else if (is_pitch_angle)
        {
            gimbal_tune_set_pitch_angle_pid(&pid, 1);
        }
        else if (is_yaw_speed)
        {
            gimbal_tune_set_yaw_speed_pid(&pid, 1);
        }
        else
        {
            gimbal_tune_set_yaw_angle_pid(&pid, 1);
        }

        aux_cmd_seq++;
        return 1;
    }

    return 0;
}

static void aux_autotune_stop(void)
{
    aux_autotune.enabled = 0u;
    aux_autotune.target = (uint8_t)AUX_AUTOTUNE_TARGET_NONE;
    aux_autotune.last_tick_ms = 0u;
    if (aux_autotune.period_ms == 0u)
    {
        aux_autotune.period_ms = 20u;
    }
}

static bool_t aux_autotune_parse_target(const char *s, aux_autotune_target_e *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (strcmp(s, "ps") == 0 || strcmp(s, "pitch_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_PITCH_SPEED;
        return 1;
    }
    if (strcmp(s, "pa") == 0 || strcmp(s, "pitch_angle") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_PITCH_ANGLE;
        return 1;
    }
    if (strcmp(s, "ys") == 0 || strcmp(s, "yaw_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_YAW_SPEED;
        return 1;
    }
    if (strcmp(s, "ya") == 0 || strcmp(s, "yaw_angle") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_YAW_ANGLE;
        return 1;
    }
    if (strcmp(s, "cf") == 0 || strcmp(s, "chassis_follow") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW;
        return 1;
    }
    if (strcmp(s, "cm") == 0 || strcmp(s, "chassis_motor_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED;
        return 1;
    }

    return 0;
}

static bool_t aux_autotune_fill_fields(fp32 *fields, uint32_t *out_tick_ms)
{
    if (fields == NULL || out_tick_ms == NULL)
    {
        return 0;
    }

    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    *out_tick_ms = now_ms;
    fields[0] = (fp32)now_ms;

    switch ((aux_autotune_target_e)aux_autotune.target)
    {
    case AUX_AUTOTUNE_TARGET_PITCH_SPEED:
    {
        const gimbal_motor_t *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &motor->gimbal_motor_gyro_pid;
        fields[1] = pid->set;
        fields[2] = pid->fdb;
        fields[3] = pid->out;
        fields[4] = pid->error[0];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_PITCH_ANGLE:
    {
        const gimbal_motor_t *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const gimbal_PID_t *pid = &motor->gimbal_motor_angle_pid;
        fields[1] = pid->set;
        fields[2] = pid->get;
        fields[3] = pid->out;
        fields[4] = pid->err;
        fields[5] = pid->kp;
        fields[6] = pid->ki;
        fields[7] = pid->kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_YAW_SPEED:
    {
        const gimbal_motor_t *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &motor->gimbal_motor_gyro_pid;
        fields[1] = pid->set;
        fields[2] = pid->fdb;
        fields[3] = pid->out;
        fields[4] = pid->error[0];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_YAW_ANGLE:
    {
        const gimbal_motor_t *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const gimbal_PID_t *pid = &motor->gimbal_motor_angle_pid;
        fields[1] = pid->set;
        fields[2] = pid->get;
        fields[3] = pid->out;
        fields[4] = pid->err;
        fields[5] = pid->kp;
        fields[6] = pid->ki;
        fields[7] = pid->kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW:
    {
        const chassis_move_t *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &chassis->chassis_angle_pid;
        const fp32 setpoint = chassis->chassis_yaw_offset_set;
        const fp32 input = chassis->chassis_yaw_offset;
        fields[1] = setpoint;
        fields[2] = input;
        fields[3] = pid->out;
        fields[4] = rad_format(setpoint - input);
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED:
    {
        const chassis_move_t *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }

        fp32 set_sum = 0.0f;
        fp32 fdb_sum = 0.0f;
        fp32 out_sum = 0.0f;
        for (uint8_t i = 0u; i < 4u; i++)
        {
            const pid_type_def *pid_i = &chassis->motor_speed_pid[i];
            set_sum += pid_i->set;
            fdb_sum += pid_i->fdb;
            out_sum += pid_i->out;
        }

        const pid_type_def *pid = &chassis->motor_speed_pid[0];
        fields[1] = set_sum * 0.25f;
        fields[2] = fdb_sum * 0.25f;
        fields[3] = out_sum * 0.25f;
        fields[4] = fields[1] - fields[2];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_NONE:
    default:
        return 0;
    }
}

static bool_t aux_autotune_try_send_frame(void)
{
    if (aux_autotune.enabled == 0u)
    {
        return 0;
    }
    if (!aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
    {
        return 0;
    }

    uint16_t period_ms = aux_autotune.period_ms;
    if (period_ms == 0u)
    {
        period_ms = 20u;
    }

    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((uint32_t)(now_ms - aux_autotune.last_tick_ms) < period_ms)
    {
        return 0;
    }

    if (!bsp_aux_link_tx_ready())
    {
        return 0;
    }

    fp32 fields[8] = {0};
    uint32_t sample_tick_ms = 0u;
    if (!aux_autotune_fill_fields(fields, &sample_tick_ms))
    {
        return 0;
    }

    for (uint8_t i = 0u; i < 8u; i++)
    {
        memcpy(&aux_autotune_frame[i * 4u], &fields[i], 4u);
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&aux_autotune_frame[8u * 4u], &tail, 4u);

    if (bsp_aux_link_tx_dma(aux_autotune_frame, (uint16_t)sizeof(aux_autotune_frame)) == 0)
    {
        aux_autotune.last_tick_ms = sample_tick_ms;
        return 1;
    }

    return 0;
}

static uint16_t aux_telem_min_period_ms(uint16_t channel_num)
{
    const uint32_t baud = bsp_aux_link_get_baudrate();
    if (baud == 0u)
    {
        return 100u;
    }

    const uint32_t bytes = ((uint32_t)channel_num + 1u) * 4u;
    const uint32_t bits = bytes * 10u; // 8N1 -> 10 bits per byte
    uint32_t ms = (bits * 1000u + baud - 1u) / baud;
    // Add 25% backoff and round up to reduce drop risk at high utilization.
    ms = (ms * 125u + 99u) / 100u;
    if (ms < 1u)
    {
        ms = 1u;
    }
    if (ms > 1000u)
    {
        ms = 1000u;
    }
    return (uint16_t)ms;
}

static fp32 aux_telem_pid_field(const pid_type_def *pid, uint8_t field)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    switch (field)
    {
    case 0:  return pid->Kp;
    case 1:  return pid->Ki;
    case 2:  return pid->Kd;
    case 3:  return pid->max_out;
    case 4:  return pid->max_iout;
    case 5:  return pid->set;
    case 6:  return pid->fdb;
    case 7:  return pid->error[0];
    case 8:  return pid->error[1];
    case 9:  return pid->Dbuf[0];
    case 10: return pid->Pout;
    case 11: return pid->Iout;
    case 12: return pid->Dout;
    case 13: return pid->out;
    default: return 0.0f;
    }
}

static fp32 aux_telem_gimbal_pid_field(const gimbal_PID_t *pid, uint8_t field)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    switch (field)
    {
    case 0:  return pid->kp;
    case 1:  return pid->ki;
    case 2:  return pid->kd;
    case 3:  return pid->set;
    case 4:  return pid->get;
    case 5:  return pid->err;
    case 6:  return pid->max_out;
    case 7:  return pid->max_iout;
    case 8:  return pid->Pout;
    case 9:  return pid->Iout;
    case 10: return pid->Dout;
    case 11: return pid->out;
    default: return 0.0f;
    }
}

static fp32 aux_telem_get_value(const aux_telem_ctx_t *ctx, aux_telem_sig_e sig)
{
    if (ctx == NULL)
    {
        return 0.0f;
    }

    // Decode repeated groups to keep the switch small.
    if (sig >= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_OUT)
    {
        static const uint8_t s_gimbal_angle_pid_map[] = {3u, 4u, 8u, 9u, 10u, 11u}; // set/get/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET);
        if (off < (uint8_t)(sizeof(s_gimbal_angle_pid_map) / sizeof(s_gimbal_angle_pid_map[0])))
        {
            return aux_telem_gimbal_pid_field(ctx->yaw ? &ctx->yaw->gimbal_motor_angle_pid : NULL, s_gimbal_angle_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_OUT)
    {
        static const uint8_t s_gimbal_angle_pid_map[] = {3u, 4u, 8u, 9u, 10u, 11u}; // set/get/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET);
        if (off < (uint8_t)(sizeof(s_gimbal_angle_pid_map) / sizeof(s_gimbal_angle_pid_map[0])))
        {
            return aux_telem_gimbal_pid_field(ctx->pitch ? &ctx->pitch->gimbal_motor_angle_pid : NULL, s_gimbal_angle_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return aux_telem_pid_field(ctx->yaw ? &ctx->yaw->gimbal_motor_gyro_pid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return aux_telem_pid_field(ctx->pitch ? &ctx->pitch->gimbal_motor_gyro_pid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET && sig <= AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return aux_telem_pid_field(ctx->chassis ? &ctx->chassis->chassis_angle_pid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET && sig <= AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET);
        const uint8_t motor = (uint8_t)(off / 7u);
        const uint8_t field = (uint8_t)(off % 7u);
        if (motor < 4u && ctx->chassis)
        {
            return aux_telem_pid_field(&ctx->chassis->motor_speed_pid[motor], s_pid_map[field]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_M0_RPM && sig <= AUX_TELEM_SIG_CHASSIS_M3_TEMP)
    {
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_CHASSIS_M0_RPM);
        const uint8_t motor = (uint8_t)(off / 8u);
        const uint8_t field = (uint8_t)(off % 8u);
        if (motor < 4u && ctx->chassis)
        {
            const chassis_motor_t *m = &ctx->chassis->motor_chassis[motor];
            const motor_measure_t *mm = m->chassis_motor_measure;
            switch (field)
            {
            case 0: return (fp32)(mm ? mm->speed_rpm : 0);
            case 1: return (fp32)m->give_current;
            case 2: return (fp32)(mm ? mm->given_current : 0);
            case 3: return m->speed_set;
            case 4: return m->speed;
            case 5: return m->accel;
            case 6: return (fp32)(mm ? mm->ecd : 0);
            case 7: return (fp32)(mm ? mm->temperate : 0);
            default: return 0.0f;
            }
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_SHOOT_FRIC0_RPM && sig <= AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD)
    {
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_SHOOT_FRIC0_RPM);
        const uint8_t motor = (uint8_t)(off / 4u);
        const uint8_t field = (uint8_t)(off % 4u);
        if (motor < FRIC_MOTOR_NUM)
        {
            const motor_measure_t *mm = ctx->fric_meas[motor];
            switch (field)
            {
            case 0: return (fp32)(mm ? mm->speed_rpm : 0);
            case 1: return (fp32)(mm ? mm->given_current : 0);
            case 2: return (fp32)(mm ? mm->temperate : 0);
            case 3: return (fp32)actuator_cmd_get_friction_current_can2(motor);
            default: return 0.0f;
            }
        }
        return 0.0f;
    }

    const fp32 rad2deg = 57.29577951308232f;

    switch (sig)
    {
    case AUX_TELEM_SIG_SYS_TICK_MS:
        return (fp32)bsp_time_get_tick_ms();
    case AUX_TELEM_SIG_SYS_AUX_CMD_SEQ:
        return (fp32)aux_cmd_seq;
    case AUX_TELEM_SIG_SYS_BATTERY_VOLT:
        return battery_voltage;
    case AUX_TELEM_SIG_SYS_BATTERY_PERCENT:
        return electricity_percentage * 100.0f;

    case AUX_TELEM_SIG_RC_CH0:
    case AUX_TELEM_SIG_RC_CH1:
    case AUX_TELEM_SIG_RC_CH2:
    case AUX_TELEM_SIG_RC_CH3:
    case AUX_TELEM_SIG_RC_CH4:
        if (ctx->rc)
        {
            const uint8_t idx = (uint8_t)(sig - AUX_TELEM_SIG_RC_CH0);
            return (idx < 5u) ? (fp32)ctx->rc->rc.ch[idx] : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_RC_S0:
        return ctx->rc ? (fp32)ctx->rc->rc.s[0] : 0.0f;
    case AUX_TELEM_SIG_RC_S1:
        return ctx->rc ? (fp32)ctx->rc->rc.s[1] : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_X:
        return ctx->rc ? (fp32)ctx->rc->mouse.x : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_Y:
        return ctx->rc ? (fp32)ctx->rc->mouse.y : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_Z:
        return ctx->rc ? (fp32)ctx->rc->mouse.z : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_L:
        return ctx->rc ? (fp32)ctx->rc->mouse.press_l : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_R:
        return ctx->rc ? (fp32)ctx->rc->mouse.press_r : 0.0f;
    case AUX_TELEM_SIG_RC_KEY:
        return ctx->rc ? (fp32)ctx->rc->key.v : 0.0f;
    case AUX_TELEM_SIG_RC_ERROR:
        return (fp32)RC_data_is_error();

    case AUX_TELEM_SIG_IMU_Q0:
    case AUX_TELEM_SIG_IMU_Q1:
    case AUX_TELEM_SIG_IMU_Q2:
    case AUX_TELEM_SIG_IMU_Q3:
        if (ctx->quat)
        {
            const uint8_t idx = (uint8_t)(sig - AUX_TELEM_SIG_IMU_Q0);
            return (idx < 4u) ? ctx->quat[idx] : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_YAW_RAD:
        return ctx->angle ? ctx->angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_ROLL_RAD:
        return ctx->angle ? ctx->angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_PITCH_RAD:
        return ctx->angle ? ctx->angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_YAW_DEG:
        return ctx->angle ? (ctx->angle[INS_YAW_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_ROLL_DEG:
        return ctx->angle ? (ctx->angle[INS_ROLL_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_PITCH_DEG:
        return ctx->angle ? (ctx->angle[INS_PITCH_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_X_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_X_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Y_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Z_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_X_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_X_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Y_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_Y_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Z_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_Z_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_X:
        return ctx->accel ? ctx->accel[INS_ACCEL_X_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_Y:
        return ctx->accel ? ctx->accel[INS_ACCEL_Y_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_Z:
        return ctx->accel ? ctx->accel[INS_ACCEL_Z_ADDRESS_OFFSET] : 0.0f;

    case AUX_TELEM_SIG_GIMBAL_YAW_ANGLE:
        return ctx->yaw ? ctx->yaw->angle : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_SET:
        return ctx->yaw ? ctx->yaw->angle_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GYRO:
        return ctx->yaw ? ctx->yaw->motor_gyro : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GYRO_SET:
        return ctx->yaw ? ctx->yaw->motor_gyro_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_MOTOR_SPEED:
        return ctx->yaw ? ctx->yaw->motor_speed : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_SET:
        return ctx->yaw ? ctx->yaw->current_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GIVEN_CURRENT:
        return ctx->yaw ? (fp32)ctx->yaw->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_RAW_CMD_CURRENT:
        return ctx->yaw ? ctx->yaw->raw_cmd_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_ECD:
        return (ctx->yaw && ctx->yaw->gimbal_motor_measure) ? (fp32)ctx->yaw->gimbal_motor_measure->ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_OFFSET_ECD:
        return ctx->yaw ? (fp32)ctx->yaw->offset_ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_RPM:
        return (ctx->yaw && ctx->yaw->gimbal_motor_measure) ? (fp32)ctx->yaw->gimbal_motor_measure->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_FB:
        return (ctx->yaw && ctx->yaw->gimbal_motor_measure) ? (fp32)ctx->yaw->gimbal_motor_measure->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_TEMP:
        return (ctx->yaw && ctx->yaw->gimbal_motor_measure) ? (fp32)ctx->yaw->gimbal_motor_measure->temperate : 0.0f;

    case AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE:
        return ctx->pitch ? ctx->pitch->angle : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_SET:
        return ctx->pitch ? ctx->pitch->angle_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GYRO:
        return ctx->pitch ? ctx->pitch->motor_gyro : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GYRO_SET:
        return ctx->pitch ? ctx->pitch->motor_gyro_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_MOTOR_SPEED:
        return ctx->pitch ? ctx->pitch->motor_speed : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_SET:
        return ctx->pitch ? ctx->pitch->current_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GIVEN_CURRENT:
        return ctx->pitch ? (fp32)ctx->pitch->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_RAW_CMD_CURRENT:
        return ctx->pitch ? ctx->pitch->raw_cmd_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_ECD:
        return (ctx->pitch && ctx->pitch->gimbal_motor_measure) ? (fp32)ctx->pitch->gimbal_motor_measure->ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_OFFSET_ECD:
        return ctx->pitch ? (fp32)ctx->pitch->offset_ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_RPM:
        return (ctx->pitch && ctx->pitch->gimbal_motor_measure) ? (fp32)ctx->pitch->gimbal_motor_measure->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_FB:
        return (ctx->pitch && ctx->pitch->gimbal_motor_measure) ? (fp32)ctx->pitch->gimbal_motor_measure->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_TEMP:
        return (ctx->pitch && ctx->pitch->gimbal_motor_measure) ? (fp32)ctx->pitch->gimbal_motor_measure->temperate : 0.0f;

    case AUX_TELEM_SIG_CHASSIS_VX_SET:
        return ctx->chassis ? ctx->chassis->vx_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VY_SET:
        return ctx->chassis ? ctx->chassis->vy_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_WZ_SET:
        return ctx->chassis ? ctx->chassis->wz_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VX:
        return ctx->chassis ? ctx->chassis->vx : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VY:
        return ctx->chassis ? ctx->chassis->vy : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_WZ:
        return ctx->chassis ? ctx->chassis->wz : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_OFFSET:
        return ctx->chassis ? ctx->chassis->chassis_yaw_offset : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_OFFSET_SET:
        return ctx->chassis ? ctx->chassis->chassis_yaw_offset_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_SET:
        return ctx->chassis ? ctx->chassis->chassis_yaw_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW:
        return ctx->chassis ? ctx->chassis->chassis_yaw : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_PITCH:
        return ctx->chassis ? ctx->chassis->chassis_pitch : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_ROLL:
        return ctx->chassis ? ctx->chassis->chassis_roll : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_SWING_KEY:
        if (ctx->chassis && ctx->chassis->chassis_RC)
        {
            const uint16_t sw = (uint16_t)ctx->chassis->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL];
            const uint16_t key = ctx->chassis->chassis_RC->key.v;
            const bool_t swing = ((key & SWING_KEY) != 0u) ||
                                 ((key & CHASSIS_GYRO_SPIN_VAR_KEY) != 0u) ||
                                 switch_is_down(sw);
            return swing ? 1.0f : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET:
        return shoot_control.fric_speed_set;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET:
        return shoot_control.trigger_speed_set;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED:
        return shoot_control.speed;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE:
        return shoot_control.angle;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE_SET:
        return shoot_control.set_angle;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_GIVEN_CURRENT:
        return (fp32)shoot_control.given_current;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT:
        return (fp32)shoot_control.ecd_count;
    case AUX_TELEM_SIG_SHOOT_PRESS_L:
        return (fp32)shoot_control.press_l;
    case AUX_TELEM_SIG_SHOOT_PRESS_R:
        return (fp32)shoot_control.press_r;
    case AUX_TELEM_SIG_SHOOT_KEY:
        return (fp32)shoot_control.key;
    case AUX_TELEM_SIG_SHOOT_HEAT_LIMIT:
        return (fp32)shoot_control.heat_limit;
    case AUX_TELEM_SIG_SHOOT_HEAT:
        return (fp32)shoot_control.heat;

    case AUX_TELEM_SIG_SHOOT_TRIGGER_RPM:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ECD:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->ecd : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_TEMP:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->temperate : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->given_current : 0.0f;

    case AUX_TELEM_SIG_DIAG_CAN1_0X200_M1:
        return (fp32)actuator_cmd_get_chassis_current_can1(0);
    case AUX_TELEM_SIG_DIAG_CAN1_0X200_M2:
        return (fp32)actuator_cmd_get_chassis_current_can1(1);
    case AUX_TELEM_SIG_DIAG_CAN1_0X200_PITCH:
        return (fp32)actuator_cmd_get_pitch_current_can1();
    case AUX_TELEM_SIG_DIAG_CAN1_0X200_TRIGGER:
        return (fp32)actuator_cmd_get_trigger_current_can1();
    case AUX_TELEM_SIG_DIAG_CAN1_0X1FF_M4:
        return (fp32)actuator_cmd_get_chassis_current_can1(3);
    case AUX_TELEM_SIG_DIAG_CAN1_0X1FF_YAW:
        return (fp32)actuator_cmd_get_yaw_current_can1();
    case AUX_TELEM_SIG_DIAG_CAN1_0X1FF_M3:
        return (fp32)actuator_cmd_get_chassis_current_can1(2);
    case AUX_TELEM_SIG_DIAG_CAN1_1FF_STATUS:
        return (fp32)CAN_get_last_1ff_status();
    case AUX_TELEM_SIG_DIAG_CAN1_ERR:
        return (fp32)CAN_get_last_can1_error();
    case AUX_TELEM_SIG_DIAG_ZERO_FORCE:
        return (gimbal_behaviour_watch == GIMBAL_ZERO_FORCE) ? 1.0f : 0.0f;

    case AUX_TELEM_SIG_PACK_MODE:
    {
        const uint32_t gimbal_behaviour = (uint32_t)gimbal_behaviour_watch;
        const uint32_t yaw_motor_mode = (ctx->yaw != NULL) ? (uint32_t)ctx->yaw->gimbal_motor_mode : 0u;
        const uint32_t pitch_motor_mode = (ctx->pitch != NULL) ? (uint32_t)ctx->pitch->gimbal_motor_mode : 0u;
        const uint32_t chassis_mode = (ctx->chassis != NULL) ? (uint32_t)ctx->chassis->chassis_mode : 0u;
        const uint32_t last_chassis_mode = (ctx->chassis != NULL) ? (uint32_t)ctx->chassis->last_chassis_mode : 0u;
        const uint32_t shoot_mode = (uint32_t)shoot_control.shoot_mode;

        const uint32_t packed = gimbal_behaviour +
                                yaw_motor_mode * 10u +
                                pitch_motor_mode * 100u +
                                chassis_mode * 1000u +
                                last_chassis_mode * 10000u +
                                shoot_mode * 100000u;
        return (fp32)packed;
    }
    case AUX_TELEM_SIG_PACK_OFFLINE:
    {
        uint32_t mask = 0u;
        if (toe_is_error(DBUS_TOE))           mask |= 1u << 0;
        if (toe_is_error(CHASSIS_MOTOR1_TOE))  mask |= 1u << 1;
        if (toe_is_error(CHASSIS_MOTOR2_TOE))  mask |= 1u << 2;
        if (toe_is_error(CHASSIS_MOTOR3_TOE))  mask |= 1u << 3;
        if (toe_is_error(CHASSIS_MOTOR4_TOE))  mask |= 1u << 4;
        if (toe_is_error(YAW_GIMBAL_MOTOR_TOE)) mask |= 1u << 5;
        if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) mask |= 1u << 6;
        if (toe_is_error(TRIGGER_MOTOR_TOE))   mask |= 1u << 7;
        if (toe_is_error(REFEREE_TOE))         mask |= 1u << 8;
        if (toe_is_error(RM_IMU_TOE))          mask |= 1u << 9;
        if (toe_is_error(BOARD_GYRO_TOE))      mask |= 1u << 10;
        if (toe_is_error(BOARD_ACCEL_TOE))     mask |= 1u << 11;
        if (toe_is_error(BOARD_MAG_TOE))       mask |= 1u << 12;
        if (toe_is_error(OLED_TOE))            mask |= 1u << 13;
        return (fp32)mask;
    }

    case AUX_TELEM_SIG_MEM_HEAP_FREE:
        return (fp32)heap_get_free();
    case AUX_TELEM_SIG_MEM_HEAP_EVER_FREE:
        return (fp32)heap_get_ever_free();

    case AUX_TELEM_SIG_BOARD_KEY_DOWN:
        return (fp32)bsp_key_read_raw_down();
    case AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT:
        return (fp32)bsp_key_get_press_cnt();

    case AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT:
        return shoot_control.trigger_motor_pid.Iout;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT:
        return shoot_control.trigger_motor_pid.out;

    default:
        return 0.0f;
    }
}

static bool_t aux_tune_parse_fp32(const char *s, fp32 *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    int sign = 1;
    if (*s == '+')
    {
        s++;
    }
    else if (*s == '-')
    {
        sign = -1;
        s++;
    }

    bool_t has_digit = 0;
    int32_t int_part = 0;
    while (*s >= '0' && *s <= '9')
    {
        has_digit = 1;
        int_part = int_part * 10 + (*s - '0');
        s++;
    }

    fp32 value = (fp32)int_part;
    if (*s == '.')
    {
        s++;
        fp32 base = 0.1f;
        while (*s >= '0' && *s <= '9')
        {
            has_digit = 1;
            value += (fp32)(*s - '0') * base;
            base *= 0.1f;
            s++;
        }
    }

    if (!has_digit)
    {
        return 0;
    }

    *out = (fp32)sign * value;
    return 1;
}

static bool_t aux_tune_parse_u16(const char *s, uint16_t *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (*s == '\0')
    {
        return 0;
    }

    uint32_t v = 0;
    bool_t has_digit = 0;
    while (*s >= '0' && *s <= '9')
    {
        has_digit = 1;
        v = v * 10u + (uint32_t)(*s - '0');
        if (v > 65535u)
        {
            v = 65535u;
        }
        s++;
    }

    if (!has_digit || *s != '\0')
    {
        return 0;
    }

    *out = (uint16_t)v;
    return 1;
}

static bool_t aux_tune_parse_u32(const char *s, uint32_t *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (*s == '\0')
    {
        return 0;
    }

    uint32_t v = 0u;
    bool_t has_digit = 0;
    while (*s >= '0' && *s <= '9')
    {
        const uint32_t digit = (uint32_t)(*s - '0');
        has_digit = 1;
        if (v > ((0xFFFFFFFFu - digit) / 10u))
        {
            v = 0xFFFFFFFFu;
        }
        else
        {
            v = v * 10u + digit;
        }
        s++;
    }

    if (!has_digit || *s != '\0')
    {
        return 0;
    }

    *out = v;
    return 1;
}

static uint8_t aux_tune_to_u8(fp32 v)
{
    if (v <= 0.0f)
    {
        return 0u;
    }
    if (v >= 255.0f)
    {
        return 255u;
    }

    fp32 r = v + 0.5f;
    if (r < 0.0f)
    {
        r = 0.0f;
    }
    if (r > 255.0f)
    {
        r = 255.0f;
    }
    return (uint8_t)r;
}

static uint16_t aux_tune_to_u16(fp32 v)
{
    if (v <= 0.0f)
    {
        return 0u;
    }
    if (v >= 65535.0f)
    {
        return 65535u;
    }

    fp32 r = v + 0.5f;
    if (r < 0.0f)
    {
        r = 0.0f;
    }
    if (r > 65535.0f)
    {
        r = 65535.0f;
    }
    return (uint16_t)r;
}

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
static const config_param_desc_t *aux_tune_find_config_param_by_name(const char *name)
{
    if (name == NULL || name[0] == '\0')
    {
        return NULL;
    }

    const uint32_t count = (uint32_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));
    for (uint32_t i = 0; i < count; i++)
    {
        if (strcmp(g_config_param_descs[i].name, name) == 0)
        {
            return &g_config_param_descs[i];
        }
    }
    return NULL;
}
#endif

static void aux_tune_apply_gimbal_yaw_speed_pid(void)
{
    gimbal_tune_set_yaw_speed_pid(&g_config.gimbal.yaw_speed_pid, 1);
}

static void aux_tune_apply_gimbal_pitch_speed_pid(void)
{
    gimbal_tune_set_pitch_speed_pid(&g_config.gimbal.pitch_speed_pid, 1);
}

static void aux_tune_apply_gimbal_yaw_angle_pid(void)
{
    gimbal_tune_set_yaw_angle_pid(&g_config.gimbal.yaw_encode_angle_pid, 1);
}

static void aux_tune_apply_gimbal_pitch_angle_pid(void)
{
    gimbal_tune_set_pitch_angle_pid(&g_config.gimbal.pitch_encode_angle_pid, 1);
}

static void aux_tune_apply_chassis_motor_speed_pid(void)
{
    chassis_tune_set_motor_speed_pid(&g_config.chassis.motor_speed_pid, 1);
}

static void aux_tune_apply_chassis_follow_pid(void)
{
    chassis_tune_set_follow_pid(&g_config.chassis.follow_gimbal_pid, 1);
}

static void aux_tune_apply_shoot_fric_speed_pid(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        pid_type_def *dst = &shoot_control.fric_speed_pid[i];
        dst->Kp = g_config.shoot.fric_speed_pid.kp;
        dst->Ki = g_config.shoot.fric_speed_pid.ki;
        dst->Kd = g_config.shoot.fric_speed_pid.kd;
        dst->max_out = g_config.shoot.fric_speed_pid.max_out;
        dst->max_iout = g_config.shoot.fric_speed_pid.max_iout;
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

static void aux_tune_apply_shoot_trigger_pid(void)
{
    taskENTER_CRITICAL();
    shoot_control.trigger_motor_pid.Kp = g_config.shoot.trigger_angle_pid.kp;
    shoot_control.trigger_motor_pid.Ki = g_config.shoot.trigger_angle_pid.ki;
    shoot_control.trigger_motor_pid.Kd = g_config.shoot.trigger_angle_pid.kd;
    PID_clear(&shoot_control.trigger_motor_pid);
    taskEXIT_CRITICAL();
}

static void aux_tune_apply_config_param(config_param_apply_e action)
{
    switch (action)
    {
    case CONFIG_PARAM_APPLY_NONE:
        return;
    case CONFIG_PARAM_APPLY_REMOTE_REFRESH:
        remote_control_refresh();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_SPEED_PID:
        aux_tune_apply_gimbal_yaw_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_SPEED_PID:
        aux_tune_apply_gimbal_pitch_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_ANGLE_PID:
        aux_tune_apply_gimbal_yaw_angle_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_ANGLE_PID:
        aux_tune_apply_gimbal_pitch_angle_pid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_MOTOR_SPEED_PID:
        aux_tune_apply_chassis_motor_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_FOLLOW_PID:
        aux_tune_apply_chassis_follow_pid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_FRIC_SPEED_PID:
        aux_tune_apply_shoot_fric_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_TRIGGER_PID:
        aux_tune_apply_shoot_trigger_pid();
        return;
    default:
        return;
    }
}

static bool_t aux_tune_set_app_param(uint16_t id, fp32 value)
{
    switch (id)
    {
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        (LVALUE) = value; \
        aux_tune_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        (LVALUE) = aux_tune_to_u16(value); \
        aux_tune_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        (LVALUE) = aux_tune_to_u8(value); \
        aux_tune_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    case ID: \
        (LVALUE) = aux_tune_to_i8(value, (MIN_V), (MAX_V)); \
        aux_tune_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        (LVALUE) = (aux_tune_to_u8(value) != 0u) ? 1u : 0u; \
        aux_tune_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    case ID: \
    { \
        uint8_t v__ = aux_tune_to_u8(value); \
        if (v__ > (uint8_t)(MAX_V)) \
        { \
            v__ = (uint8_t)(MAX_V); \
        } \
        (LVALUE) = v__; \
        aux_tune_apply_config_param(APPLY); \
        return 1; \
    }
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    case ID: \
    { \
        uint8_t v__ = aux_tune_to_u8(value); \
        if (v__ > (uint8_t)(MAX_V)) \
        { \
            v__ = (uint8_t)(DEFAULT_V); \
        } \
        (LVALUE) = v__; \
        aux_tune_apply_config_param(APPLY); \
        return 1; \
    }
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
    default:
        return 0;
    }
}

static void aux_tune_on_byte(uint8_t b)
{
    if (aux_cmd_ready)
    {
        // Drop input until the current command is processed.
    }
    else if (b == '\r' || b == '\n')
    {
        if (aux_rx_len > 0u)
        {
            const uint16_t n = (aux_rx_len >= (AUX_TUNE_RX_LINE_MAX - 1u)) ? (AUX_TUNE_RX_LINE_MAX - 1u) : aux_rx_len;
            aux_rx_line[n] = '\0';
            memcpy(aux_cmd_line, aux_rx_line, n + 1u);
            aux_cmd_ready = 1;
            aux_rx_len = 0;
        }
    }
    else if (b == 0x08u || b == 0x7Fu)
    {
        if (aux_rx_len > 0u)
        {
            aux_rx_len--;
        }
    }
    else
    {
        // Only accept printable ASCII / TAB as tuning commands. Drop binary/noise to
        // avoid accidentally changing config when using a wireless UART bridge.
        if (b == '\t' || (b >= 0x20u && b <= 0x7Eu))
        {
            if (aux_rx_len < (AUX_TUNE_RX_LINE_MAX - 1u))
            {
                aux_rx_line[aux_rx_len++] = (char)b;
            }
            else
            {
                aux_rx_len = 0;
            }
        }
        else
        {
            aux_rx_len = 0;
        }
    }
}

static uint8_t aux_tune_on_uart_error(void)
{
    aux_rx_len = 0;
    return 0u;
}

static void aux_tune_poll(void)
{
    if (aux_cmd_ready)
    {
        char line[AUX_TUNE_RX_LINE_MAX];
        taskENTER_CRITICAL();
        const bool_t ready = aux_cmd_ready;
        aux_cmd_ready = 0;
        if (ready)
        {
            strncpy(line, aux_cmd_line, sizeof(line) - 1u);
            line[sizeof(line) - 1u] = '\0';
        }
        else
        {
            line[0] = '\0';
        }
        taskEXIT_CRITICAL();

        if (line[0] != '\0')
        {
            const bool_t changed = aux_tune_handle_line(line);
            if (changed && sdlog_is_active())
            {
                uint16_t n = 0u;
                while (n < (AUX_TUNE_RX_LINE_MAX - 1u) && line[n] != '\0')
                {
                    n++;
                }

            }
        }
    }
}
