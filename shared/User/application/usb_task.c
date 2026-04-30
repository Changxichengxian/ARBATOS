/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "usb_task.h"
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
#include "app_watch.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "mem_mang.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "shoot.h"
#include "bsp_usart.h"
#include "usbd_cdc_if.h"
#include "voltage_task.h"
#include "referee.h"
#include "bsp_key.h"
#include "sdlog.h"
#include "CRC8_CRC16.h"

// ===== Vision over USB CDC =====
// Frame: head[2] + mode + q[4] + yaw/yaw_vel + pitch/pitch_vel + bullet_speed + bullet_count(u16) + crc16(u16)
typedef struct __attribute__((packed))
{
    uint8_t head[2];
    uint8_t mode;
    float   q[4];     // wxyz
    float   yaw;
    float   yaw_vel;
    float   pitch;
    float   pitch_vel;
    float   bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16; // CRC16 (init 0xFFFF, LSB first)
} VisionTxFrame;

typedef char _check_VisionTxFrame_size[(sizeof(VisionTxFrame) == 43) ? 1 : -1];
typedef char _check_VisionToGimbal_size[(sizeof(VisionToGimbal) == 29) ? 1 : -1];

#define VISION_USB_RX_FRAME_SIZE      ((uint32_t)sizeof(VisionToGimbal))
#define VISION_USB_RX_STREAM_BUF_SIZE 128u

static VisionTxFrame vision_tx;
static VisionToGimbal vision_rx;
static volatile bool vision_rx_updated = false;
static uint8_t vision_usb_rx_stream_buf[VISION_USB_RX_STREAM_BUF_SIZE];
static uint32_t vision_usb_rx_stream_len = 0u;

// ===== UART1 tuning/telemetry / image link =====
#define UART1_TUNE_RX_LINE_MAX     96u
#define UART1_TUNE_BAUD            230400u
#define UART1_IMAGE_LINK_BAUD      921600u
#define UART1_IMAGE_FRAME_SOF      0xA5u
#define UART1_IMAGE_DMA_RX_BUF_SIZE 512u
#define UART1_IMAGE_RM_FRAME_MAX_SIZE 64u
#define UART1_IMAGE_VT13_FRAME_SIZE 21u
#define UART1_IMAGE_RC_MAGIC0      'R'
#define UART1_IMAGE_RC_MAGIC1      'C'
#define UART1_IMAGE_RC_VERSION     1u
#define UART1_IMAGE_RC_RANGE_DBUS  0u
#define UART1_IMAGE_RC_RANGE_VT13  1u
#define UART1_IMAGE_RC_BTN_LEFT    (1u << 0)
#define UART1_IMAGE_RC_BTN_RIGHT   (1u << 1)
#define UART1_IMAGE_RC_ABS_MAX_DBUS ((int16_t)RC_CH_VALUE_ABS_LEGACY)
#define UART1_IMAGE_RC_ABS_MAX_VT13 ((int16_t)RC_CH_VALUE_ABS_MAX)
#define UART1_IMAGE_CMD_CUSTOM_CONTROLLER_RX 0x0302u
#define UART1_IMAGE_CMD_CUSTOM_CLIENT_RX     0x0311u
#define UART1_IMAGE_KEY_FLAG(value, mask) ((((value) & (mask)) != 0u) ? 1u : 0u)
// Extra backoff applied when cfg->period_ms==0 (auto). This is intentionally
// conservative for wireless UART bridges that may buffer/retransmit and have
// lower effective throughput than the UART baud rate.
#define AUX_TELEM_AUTO_EXTRA_BACKOFF_PCT 50u

typedef struct
{
    const RC_ctrl_t *rc;
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

typedef struct __attribute__((packed))
{
    uint8_t magic0;
    uint8_t magic1;
    uint8_t version;
    uint8_t range_mode;
    int16_t ch[5];
    uint8_t sw[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_btns;
    uint16_t key_value;
    uint8_t reserved[5];
} uart1_image_rc_packet_t;

typedef char _check_uart1_image_rc_packet_size[(sizeof(uart1_image_rc_packet_t) == 30u) ? 1 : -1];

typedef enum
{
    UART1_IMAGE_PARSE_IDLE = 0,
    UART1_IMAGE_PARSE_RM,
    UART1_IMAGE_PARSE_VT13,
} uart1_image_parse_mode_e;

static char uart1_rx_line[UART1_TUNE_RX_LINE_MAX];
static volatile uint16_t uart1_rx_len = 0;
static char uart1_cmd_line[UART1_TUNE_RX_LINE_MAX];
static volatile bool_t uart1_cmd_ready = 0;

static uint32_t aux_telem_tick = 0;
static volatile uint32_t uart1_cmd_seq = 0;
static uint8_t uart1_vofa_frame[(AUX_TELEM_MAX_CH + 1u) * 4u];
static uint8_t uart1_image_dma_rx_buf[UART1_IMAGE_DMA_RX_BUF_SIZE];
static volatile uint16_t uart1_image_dma_pos = 0u;
static uint16_t uart1_image_dma_last_pos = 0u;
static volatile uint32_t uart1_image_dma_wrap_cnt = 0u;
static uint32_t uart1_image_dma_last_wrap_cnt = 0u;
static volatile uint8_t uart1_image_dma_active = 0u;
static volatile uint8_t uart1_image_dma_restart_req = 0u;
static uart1_image_parse_mode_e uart1_image_parse_mode = UART1_IMAGE_PARSE_IDLE;
static uint8_t uart1_image_rm_buf[UART1_IMAGE_RM_FRAME_MAX_SIZE];
static uint16_t uart1_image_rm_pos = 0u;
static uint16_t uart1_image_rm_expected = 0u;
static uint8_t uart1_image_vt13_buf[UART1_IMAGE_VT13_FRAME_SIZE];
static uint8_t uart1_image_vt13_pos = 0u;
static volatile uint32_t uart1_image_last_rx_tick_ms = 0u;
static volatile uint32_t uart1_image_frame_cnt = 0u;
static volatile uint32_t uart1_image_controller_frame_cnt = 0u;
static volatile uint32_t uart1_image_client_frame_cnt = 0u;
static volatile uint32_t uart1_image_vt13_frame_cnt = 0u;
static volatile uint32_t uart1_image_crc_error_cnt = 0u;
static volatile uint32_t uart1_image_parse_error_cnt = 0u;
static volatile uint32_t uart1_image_restart_cnt = 0u;
static volatile uint16_t uart1_image_last_cmd_id = 0u;
static volatile uint8_t uart1_image_last_range_mode = 0u;
static uart1_image_remote_state_t uart1_image_remote_state = {0};

// ===== UART1 port mode / ELRS(CRSF) RX / DJI image link =====
// ELRS/CRSF RX is implemented in elrs_task.c; this file only routes UART1 callbacks.

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

// Some targets do not wire in voltage_task.c yet. Keep USB telemetry linkable
// by providing weak zero defaults that are overridden when the real task exists.
__weak fp32 battery_voltage = 0.0f;
__weak fp32 electricity_percentage = 0.0f;

static void vision_usb_send(void);
static void vision_handle_rx(const VisionToGimbal *pkt);
static void vision_usb_rx_stream_consume(const uint8_t *buf, uint32_t len);
static void uart1_port_init(void);
static void uart1_port_poll(void);
static void uart1_port_stop(void);
static bool_t uart1_port_apply_baud(uint32_t baud);
static uint8_t uart1_port_is_elrs_mode(uint32_t baud);
static uint8_t uart1_port_is_image_mode(uint32_t baud);
static uint8_t uart1_port_is_tune_mode(uint32_t baud);
static void uart1_tune_rx_start(void);
static void uart1_tune_on_byte(uint8_t b);
static uint8_t uart1_tune_on_uart_error(void);
static bool_t uart1_tune_handle_line(const char *line);
static void uart1_tune_try_send_telem(void);
static void uart1_tune_poll(void);
static bool_t uart1_tune_parse_fp32(const char *s, fp32 *out);
static bool_t uart1_tune_parse_u16(const char *s, uint16_t *out);
static bool_t uart1_tune_parse_u32(const char *s, uint32_t *out);
static bool_t uart1_tune_set_app_param(uint16_t id, fp32 value);
static uint16_t aux_telem_min_period_ms(uint16_t channel_num);
static fp32 aux_telem_get_value(const aux_telem_ctx_t *ctx, aux_telem_sig_e sig);
static void uart1_image_link_start(void);
static void uart1_image_link_stop(void);
static void uart1_image_link_poll(void);
static void uart1_image_link_on_rx_event(uint16_t size, bsp_uart1_rx_event_e evt);
static uint8_t uart1_image_link_on_uart_error(void);
static void uart1_image_link_process_to(uint16_t pos);
static void uart1_image_link_reset_parser(void);
static void uart1_image_link_feed_byte(uint8_t b);
static void uart1_image_link_handle_rm_frame(const uint8_t *frame, uint16_t frame_len);
static void uart1_image_link_handle_vt13_frame(const uint8_t *frame, uint16_t frame_len);
static bool_t uart1_image_link_try_decode_custom_rc(const uint8_t *data);
static int16_t uart1_image_link_scale_axis(int16_t raw, int16_t raw_abs_max);
static uint8_t uart1_image_link_sanitize_switch(uint8_t value);
static uint8_t uart1_image_link_map_vt13_switch1(uint8_t value);
static uint8_t uart1_image_link_map_vt13_switch2(uint8_t stop, uint8_t left, uint8_t right);
static void uart1_image_remote_store(const uart1_image_remote_state_t *state);

bool vision_take_latest(VisionToGimbal *out)
{
    bool has_new = false;
    if (vision_rx_updated && out != NULL)
    {
        taskENTER_CRITICAL();
        *out = vision_rx;
        vision_rx_updated = false;
        taskEXIT_CRITICAL();
        has_new = true;
    }
    return has_new;
}

bool uart1_image_remote_get_state(uart1_image_remote_state_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *out = uart1_image_remote_state;
    taskEXIT_CRITICAL();
    return (out->valid != 0u);
}

bool uart1_image_auto_aim_requested(void)
{
    uart1_image_remote_state_t state;
    if (!uart1_image_remote_get_state(&state))
    {
        return false;
    }
    if (remote_control_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return false;
    }
    return (state.pause != 0u) || (state.mouse_r != 0u);
}

bool uart1_image_aux_fire_requested(void)
{
    uart1_image_remote_state_t state;
    if (!uart1_image_remote_get_state(&state))
    {
        return false;
    }
    if (remote_control_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return false;
    }
    return (state.btn_l != 0u) || (state.mouse_l != 0u);
}

void usb_task(void const * argument)
{
    ins_quat = get_INS_quat_point();
    ins_angle = get_INS_angle_point();
    ins_gyro = get_gyro_data_point();
    ins_accel = get_accel_data_point();

    uart1_port_init();

    vision_tx.head[0] = 'S';
    vision_tx.head[1] = 'P';
    vision_tx.mode = 0;

    while(1)
    {
        app_watch_task_beat(APP_WATCH_TASK_USB);
        vision_usb_send();
        uart1_port_poll();

        osDelay(2);
    }
}

static void vision_usb_send(void)
{
    /* Default values to avoid stale data if INS is not ready */
    vision_tx.q[0] = 0.0f;
    vision_tx.q[1] = 0.0f;
    vision_tx.q[2] = 0.0f;
    vision_tx.q[3] = 0.0f;
    vision_tx.yaw       = 0.0f;
    vision_tx.yaw_vel   = 0.0f;
    vision_tx.pitch     = 0.0f;
    vision_tx.pitch_vel = 0.0f;
    vision_tx.bullet_speed = 0.0f;
    vision_tx.bullet_count = 0;
    vision_tx.crc16 = 0;

    if (ins_quat)
    {
        vision_tx.q[0] = ins_quat[0];
        vision_tx.q[1] = ins_quat[1];
        vision_tx.q[2] = ins_quat[2];
        vision_tx.q[3] = ins_quat[3];
    }
    if (ins_angle)
    {
        vision_tx.yaw   = ins_angle[INS_YAW_ADDRESS_OFFSET];
        vision_tx.pitch = ins_angle[INS_PITCH_ADDRESS_OFFSET];
    }
    if (ins_gyro)
    {
        vision_tx.yaw_vel = ins_gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        vision_tx.pitch_vel = ins_gyro[INS_GYRO_Y_ADDRESS_OFFSET];
    }

    /* 射速/弹数：来自裁判系统，缺失则保持默认 0 */
    vision_tx.bullet_speed = shoot_data_t.initial_speed;
    vision_tx.bullet_count = bullet_remaining_t.projectile_allowance_17mm;
    append_CRC16_check_sum((uint8_t *)&vision_tx, (uint32_t)sizeof(vision_tx));

    /* If USB is busy, skip this frame and try next tick */
    if (CDC_Transmit_FS((uint8_t*)&vision_tx, sizeof(vision_tx)) == USBD_BUSY)
    {
        return;
    }
}

static void vision_handle_rx(const VisionToGimbal *pkt)
{
    if (pkt->head[0] != 'S' || pkt->head[1] != 'P')
    {
        return;
    }
    if (!verify_CRC16_check_sum((uint8_t *)pkt, (uint32_t)sizeof(*pkt)))
    {
        return;
    }

    // Called from USB CDC RX context (can be ISR). Record the raw control packet for offline analysis.
    // To reduce TF log size, skip packets that carry no RC yaw/pitch command.
    //
    // NOTE: keep this ISR-side filter integer-only (no float ops) to avoid FPU stacking overhead.
    uint32_t yaw_bits = 0u;
    uint32_t pitch_bits = 0u;
    memcpy(&yaw_bits, (const void *)&pkt->yaw, sizeof(yaw_bits));
    memcpy(&pitch_bits, (const void *)&pkt->pitch, sizeof(pitch_bits));
    const bool yaw_has_cmd = ((yaw_bits & 0x7FFFFFFFu) != 0u);
    const bool pitch_has_cmd = ((pitch_bits & 0x7FFFFFFFu) != 0u);
    if (yaw_has_cmd || pitch_has_cmd)
    {
        sdlog_write_isr(SDLOG_TAG_VISION_USB_RX, pkt, (uint16_t)sizeof(*pkt));
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    vision_rx = *pkt;
    vision_rx_updated = true;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

static void vision_usb_rx_stream_consume(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u)
    {
        return;
    }

    // USB CDC behaves like a byte stream. Keep a small rolling buffer so split
    // packets and coalesced packets can still be reconstructed into 29-byte frames.
    if (len >= VISION_USB_RX_STREAM_BUF_SIZE)
    {
        buf += (len - VISION_USB_RX_STREAM_BUF_SIZE);
        len = VISION_USB_RX_STREAM_BUF_SIZE;
        vision_usb_rx_stream_len = 0u;
    }

    if ((vision_usb_rx_stream_len + len) > VISION_USB_RX_STREAM_BUF_SIZE)
    {
        const uint32_t overflow = (vision_usb_rx_stream_len + len) - VISION_USB_RX_STREAM_BUF_SIZE;
        if (overflow >= vision_usb_rx_stream_len)
        {
            vision_usb_rx_stream_len = 0u;
        }
        else
        {
            vision_usb_rx_stream_len -= overflow;
            memmove(vision_usb_rx_stream_buf,
                    &vision_usb_rx_stream_buf[overflow],
                    vision_usb_rx_stream_len);
        }
    }

    memcpy(&vision_usb_rx_stream_buf[vision_usb_rx_stream_len], buf, len);
    vision_usb_rx_stream_len += len;

    uint32_t consume = 0u;
    while ((vision_usb_rx_stream_len - consume) >= VISION_USB_RX_FRAME_SIZE)
    {
        const uint8_t *candidate = &vision_usb_rx_stream_buf[consume];

        if (candidate[0] != 'S' || candidate[1] != 'P')
        {
            consume++;
            continue;
        }

        if (verify_CRC16_check_sum((uint8_t *)candidate, VISION_USB_RX_FRAME_SIZE))
        {
            VisionToGimbal pkt;
            memcpy(&pkt, candidate, sizeof(pkt));
            vision_handle_rx(&pkt);
            consume += VISION_USB_RX_FRAME_SIZE;
            continue;
        }

        consume++;
    }

    if (consume == 0u)
    {
        return;
    }

    vision_usb_rx_stream_len -= consume;
    if (vision_usb_rx_stream_len > 0u)
    {
        memmove(vision_usb_rx_stream_buf,
                &vision_usb_rx_stream_buf[consume],
                vision_usb_rx_stream_len);
    }
}

// Called from usbd_cdc_if.c
void vision_usb_rx_callback(uint8_t *buf, uint32_t len)
{
    vision_usb_rx_stream_consume(buf, len);
}

void uart1_image_link_get_stats(sdlog_image_link_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    taskENTER_CRITICAL();
    out->last_rx_tick_ms = uart1_image_last_rx_tick_ms;
    out->frame_count = uart1_image_frame_cnt;
    out->controller_frame_count = uart1_image_controller_frame_cnt;
    out->client_frame_count = uart1_image_client_frame_cnt;
    out->vt13_frame_count = uart1_image_vt13_frame_cnt;
    out->crc_error_count = uart1_image_crc_error_cnt;
    out->parse_error_count = uart1_image_parse_error_cnt;
    out->restart_count = uart1_image_restart_cnt;
    out->last_cmd_id = uart1_image_last_cmd_id;
    out->port_active = uart1_image_dma_active;
    out->last_range_mode = uart1_image_last_range_mode;
    taskEXIT_CRITICAL();
}

static void uart1_image_remote_store(const uart1_image_remote_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    uart1_image_remote_state = *state;
    taskEXIT_CRITICAL();
}

static void uart1_port_init(void)
{
    const uint32_t baud = bsp_usart1_get_baudrate();

    uart1_port_stop();

    if (uart1_port_is_elrs_mode(baud))
    {
        bsp_usart1_set_rx_event_cb(uart1_elrs_on_rx_event);
        bsp_usart1_set_rx_byte_cb(uart1_elrs_on_it_byte);
        bsp_usart1_set_error_cb(uart1_elrs_on_uart_error);
        uart1_elrs_rx_start();
    }
    else if (uart1_port_is_tune_mode(baud))
    {
        uart1_tune_rx_start();
    }
    else if (uart1_port_is_image_mode(baud))
    {
        uart1_image_link_start();
    }
}

static void uart1_port_poll(void)
{
    const uint32_t baud = bsp_usart1_get_baudrate();

    if (uart1_port_is_tune_mode(baud))
    {
        uart1_tune_poll();
        if (uart1_port_is_tune_mode(bsp_usart1_get_baudrate()))
        {
            uart1_tune_try_send_telem();
        }
    }
    else if (uart1_port_is_image_mode(baud))
    {
        uart1_image_link_poll();
    }
}

static void uart1_port_stop(void)
{
    uart1_image_link_stop();
    uart1_elrs_stop();
    uart1_rx_len = 0u;
    uart1_cmd_ready = 0;

    bsp_usart1_set_rx_event_cb(NULL);
    bsp_usart1_set_rx_byte_cb(NULL);
    bsp_usart1_set_error_cb(NULL);
    bsp_usart1_rx_it_stop();
}

static bool_t uart1_port_apply_baud(uint32_t baud)
{
    if (!uart1_port_is_tune_mode(baud) &&
        !uart1_port_is_elrs_mode(baud) &&
        !uart1_port_is_image_mode(baud))
    {
        return 0;
    }

    const uint32_t old_baud = bsp_usart1_get_baudrate();
    if (old_baud == baud)
    {
        uart1_port_init();
        return 1;
    }

    uart1_port_stop();
    if (bsp_usart1_set_baudrate(baud) != 0)
    {
        (void)bsp_usart1_set_baudrate(old_baud);
        uart1_port_init();
        return 0;
    }

    uart1_port_init();
    return 1;
}

static uint8_t uart1_port_is_elrs_mode(uint32_t baud)
{
    return (baud == UART1_ELRS_BAUD) ? 1u : 0u;
}

static uint8_t uart1_port_is_image_mode(uint32_t baud)
{
    return (baud == UART1_IMAGE_LINK_BAUD) ? 1u : 0u;
}

static uint8_t uart1_port_is_tune_mode(uint32_t baud)
{
    return (baud == UART1_TUNE_BAUD) ? 1u : 0u;
}

static void uart1_image_link_start(void)
{
    uart1_image_link_stop();
    uart1_image_dma_pos = 0u;
    uart1_image_dma_last_pos = 0u;
    uart1_image_dma_wrap_cnt = 0u;
    uart1_image_dma_last_wrap_cnt = 0u;
    uart1_image_dma_restart_req = 0u;
    uart1_image_link_reset_parser();

    bsp_usart1_set_rx_event_cb(uart1_image_link_on_rx_event);
    bsp_usart1_set_rx_byte_cb(NULL);
    bsp_usart1_set_error_cb(uart1_image_link_on_uart_error);

    if (bsp_usart1_rx_has_dma() == 0u)
    {
        return;
    }
    if (bsp_usart1_rx_to_idle_dma_start(uart1_image_dma_rx_buf, (uint16_t)UART1_IMAGE_DMA_RX_BUF_SIZE) != 0)
    {
        return;
    }

    uart1_image_dma_active = 1u;
}

static void uart1_image_link_stop(void)
{
    uart1_image_dma_active = 0u;
    uart1_image_dma_restart_req = 0u;
    uart1_image_dma_pos = 0u;
    uart1_image_dma_last_pos = 0u;
    uart1_image_dma_wrap_cnt = 0u;
    uart1_image_dma_last_wrap_cnt = 0u;
    uart1_image_link_reset_parser();

    bsp_usart1_set_rx_event_cb(NULL);
    bsp_usart1_set_rx_byte_cb(NULL);
    bsp_usart1_set_error_cb(NULL);
    bsp_usart1_rx_it_stop();
}

static void uart1_image_link_poll(void)
{
    if (!uart1_port_is_image_mode(bsp_usart1_get_baudrate()))
    {
        return;
    }
    if (uart1_image_dma_restart_req != 0u)
    {
        uart1_image_dma_restart_req = 0u;
        uart1_image_link_start();
        return;
    }
    if (uart1_image_dma_active == 0u)
    {
        return;
    }

    const uint32_t wrap_cnt = uart1_image_dma_wrap_cnt;
    const uint16_t pos = uart1_image_dma_pos;

    while (uart1_image_dma_last_wrap_cnt != wrap_cnt)
    {
        uart1_image_link_process_to((uint16_t)UART1_IMAGE_DMA_RX_BUF_SIZE);
        uart1_image_dma_last_wrap_cnt++;
    }
    uart1_image_link_process_to(pos);
    uart1_image_dma_last_wrap_cnt = wrap_cnt;
}

static void uart1_image_link_on_rx_event(uint16_t size, bsp_uart1_rx_event_e evt)
{
    if (!uart1_port_is_image_mode(bsp_usart1_get_baudrate()) || uart1_image_dma_active == 0u)
    {
        return;
    }

    if (evt == BSP_UART1_RXEVENT_IDLE && size >= (uint16_t)UART1_IMAGE_DMA_RX_BUF_SIZE)
    {
        return;
    }

    if (evt == BSP_UART1_RXEVENT_TC)
    {
        uart1_image_dma_wrap_cnt++;
    }

    uart1_image_dma_pos = (size >= (uint16_t)UART1_IMAGE_DMA_RX_BUF_SIZE) ? 0u : size;
}

static uint8_t uart1_image_link_on_uart_error(void)
{
    uart1_image_link_reset_parser();
    if (uart1_image_dma_active == 0u)
    {
        return 0u;
    }

    uart1_image_restart_cnt++;
    uart1_image_dma_restart_req = 1u;
    return 1u;
}

static void uart1_image_link_process_to(uint16_t pos)
{
    const uint16_t size = (uint16_t)UART1_IMAGE_DMA_RX_BUF_SIZE;
    uint16_t last = uart1_image_dma_last_pos;

    if (pos > size)
    {
        pos = (uint16_t)(pos % size);
    }
    if (last >= size)
    {
        last = 0u;
    }

    if (last <= pos)
    {
        for (uint16_t i = last; i < pos; i++)
        {
            uart1_image_link_feed_byte(uart1_image_dma_rx_buf[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            uart1_image_link_feed_byte(uart1_image_dma_rx_buf[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            uart1_image_link_feed_byte(uart1_image_dma_rx_buf[i]);
        }
    }

    uart1_image_dma_last_pos = (pos == size) ? 0u : pos;
}

static void uart1_image_link_reset_parser(void)
{
    uart1_image_parse_mode = UART1_IMAGE_PARSE_IDLE;
    uart1_image_rm_pos = 0u;
    uart1_image_rm_expected = 0u;
    uart1_image_vt13_pos = 0u;
}

static void uart1_image_link_feed_byte(uint8_t b)
{
retry_parse:
    switch (uart1_image_parse_mode)
    {
    case UART1_IMAGE_PARSE_IDLE:
        if (b == UART1_IMAGE_FRAME_SOF)
        {
            uart1_image_parse_mode = UART1_IMAGE_PARSE_RM;
            uart1_image_rm_pos = 0u;
            uart1_image_rm_expected = 0u;
            uart1_image_rm_buf[uart1_image_rm_pos++] = b;
        }
        else if (b == 0xA9u)
        {
            uart1_image_parse_mode = UART1_IMAGE_PARSE_VT13;
            uart1_image_vt13_pos = 0u;
            uart1_image_vt13_buf[uart1_image_vt13_pos++] = b;
        }
        return;

    case UART1_IMAGE_PARSE_RM:
        if (uart1_image_rm_pos >= (uint16_t)UART1_IMAGE_RM_FRAME_MAX_SIZE)
        {
            uart1_image_parse_error_cnt++;
            uart1_image_link_reset_parser();
            goto retry_parse;
        }

        uart1_image_rm_buf[uart1_image_rm_pos++] = b;

        if (uart1_image_rm_pos == 5u)
        {
            if (!verify_CRC8_check_sum(uart1_image_rm_buf, 5u))
            {
                uart1_image_crc_error_cnt++;
                uart1_image_link_reset_parser();
                goto retry_parse;
            }

            const uint16_t payload_len = (uint16_t)(uart1_image_rm_buf[1] | ((uint16_t)uart1_image_rm_buf[2] << 8));
            uart1_image_rm_expected = (uint16_t)(payload_len + 9u);
            if (uart1_image_rm_expected < 9u || uart1_image_rm_expected > (uint16_t)UART1_IMAGE_RM_FRAME_MAX_SIZE)
            {
                uart1_image_parse_error_cnt++;
                uart1_image_link_reset_parser();
                goto retry_parse;
            }
        }

        if (uart1_image_rm_expected != 0u && uart1_image_rm_pos >= uart1_image_rm_expected)
        {
            if (verify_CRC16_check_sum(uart1_image_rm_buf, uart1_image_rm_expected))
            {
                uart1_image_link_handle_rm_frame(uart1_image_rm_buf, uart1_image_rm_expected);
            }
            else
            {
                uart1_image_crc_error_cnt++;
            }
            uart1_image_link_reset_parser();
        }
        return;

    case UART1_IMAGE_PARSE_VT13:
        if (uart1_image_vt13_pos == 1u && b != 0x53u)
        {
            uart1_image_parse_error_cnt++;
            uart1_image_link_reset_parser();
            goto retry_parse;
        }
        if (uart1_image_vt13_pos >= UART1_IMAGE_VT13_FRAME_SIZE)
        {
            uart1_image_parse_error_cnt++;
            uart1_image_link_reset_parser();
            goto retry_parse;
        }

        uart1_image_vt13_buf[uart1_image_vt13_pos++] = b;
        if (uart1_image_vt13_pos >= UART1_IMAGE_VT13_FRAME_SIZE)
        {
            uart1_image_link_handle_vt13_frame(uart1_image_vt13_buf, UART1_IMAGE_VT13_FRAME_SIZE);
            uart1_image_link_reset_parser();
        }
        return;

    default:
        uart1_image_parse_error_cnt++;
        uart1_image_link_reset_parser();
        goto retry_parse;
    }
}

static void uart1_image_link_handle_rm_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 9u)
    {
        return;
    }

    const uint16_t payload_len = (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8));
    if ((uint16_t)(payload_len + 9u) != frame_len || payload_len != (uint16_t)sizeof(uart1_image_rc_packet_t))
    {
        uart1_image_parse_error_cnt++;
        return;
    }

    const uint16_t cmd_id = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8));
    const uint8_t *payload = &frame[7];

    if (cmd_id == UART1_IMAGE_CMD_CUSTOM_CONTROLLER_RX || cmd_id == UART1_IMAGE_CMD_CUSTOM_CLIENT_RX)
    {
        uart1_image_last_rx_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uart1_image_frame_cnt++;
        uart1_image_last_cmd_id = cmd_id;
        if (cmd_id == UART1_IMAGE_CMD_CUSTOM_CONTROLLER_RX)
        {
            uart1_image_controller_frame_cnt++;
        }
        else
        {
            uart1_image_client_frame_cnt++;
        }
        (void)uart1_image_link_try_decode_custom_rc(payload);
    }
}

static void uart1_image_link_handle_vt13_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len != UART1_IMAGE_VT13_FRAME_SIZE)
    {
        return;
    }
    if (frame[0] != 0xA9u || frame[1] != 0x53u)
    {
        uart1_image_parse_error_cnt++;
        return;
    }
    if (!verify_CRC16_check_sum((uint8_t *)frame, frame_len))
    {
        uart1_image_crc_error_cnt++;
        return;
    }

    int16_t ch_raw[5] = {0};
    ch_raw[0] = (int16_t)(((uint16_t)frame[2] | ((uint16_t)frame[3] << 8)) & 0x07FFu);
    ch_raw[1] = (int16_t)((((uint16_t)frame[3] >> 3) | ((uint16_t)frame[4] << 5)) & 0x07FFu);
    ch_raw[2] = (int16_t)((((uint16_t)frame[4] >> 6) | ((uint16_t)frame[5] << 2) | ((uint16_t)frame[6] << 10)) & 0x07FFu);
    ch_raw[3] = (int16_t)((((uint16_t)frame[6] >> 1) | ((uint16_t)frame[7] << 7)) & 0x07FFu);
    ch_raw[4] = (int16_t)((((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7)) & 0x07FFu);
    const uint8_t vt13_pause = (uint8_t)((frame[7] >> 6) & 0x01u);
    const uint8_t vt13_btn_l = (uint8_t)((frame[7] >> 7) & 0x01u);
    const uint8_t vt13_btn_r = (uint8_t)(frame[8] & 0x01u);
    const uint16_t vt13_dial = (uint16_t)(((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7));
    const uint8_t vt13_trigger = (uint8_t)((frame[9] >> 4) & 0x01u);
    const uint8_t vt13_mouse_l = (uint8_t)(frame[16] & 0x01u);
    const uint8_t vt13_mouse_r = (uint8_t)((frame[16] >> 1) & 0x01u);
    const uint8_t vt13_mouse_mid = (uint8_t)((frame[16] >> 2) & 0x01u);

    RC_ctrl_t rc = {0};
    rc.rc.ch[0] = uart1_image_link_scale_axis((int16_t)(ch_raw[0] - RC_CH_VALUE_OFFSET), UART1_IMAGE_RC_ABS_MAX_VT13);
    rc.rc.ch[1] = uart1_image_link_scale_axis((int16_t)(ch_raw[1] - RC_CH_VALUE_OFFSET), UART1_IMAGE_RC_ABS_MAX_VT13);
    rc.rc.ch[2] = uart1_image_link_scale_axis((int16_t)(ch_raw[2] - RC_CH_VALUE_OFFSET), UART1_IMAGE_RC_ABS_MAX_VT13);
    rc.rc.ch[3] = uart1_image_link_scale_axis((int16_t)(ch_raw[3] - RC_CH_VALUE_OFFSET), UART1_IMAGE_RC_ABS_MAX_VT13);
    rc.rc.ch[4] = uart1_image_link_scale_axis((int16_t)(ch_raw[4] - RC_CH_VALUE_OFFSET), UART1_IMAGE_RC_ABS_MAX_VT13);

    rc.rc.s[0] = (char)uart1_image_link_map_vt13_switch1((uint8_t)((frame[7] >> 4) & 0x03u));
    rc.rc.s[1] = (char)uart1_image_link_map_vt13_switch2(vt13_pause, vt13_btn_l, vt13_btn_r);

    rc.mouse.x = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    rc.mouse.y = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
    rc.mouse.z = (int16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));
    rc.mouse.press_l = vt13_mouse_l;
    rc.mouse.press_r = vt13_mouse_r;
    rc.key.v = (uint16_t)(frame[17] | ((uint16_t)frame[18] << 8));

    uart1_image_last_rx_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uart1_image_frame_cnt++;
    uart1_image_vt13_frame_cnt++;
    uart1_image_last_cmd_id = 0u;
    uart1_image_last_range_mode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT;
    remote_control_log_raw_source(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  5u,
                                  ch_raw,
                                  NULL,
                                  &rc);
    uart1_image_remote_state_t state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
        .range_mode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
        .raw_ch = { ch_raw[0], ch_raw[1], ch_raw[2], ch_raw[3], ch_raw[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { rc.rc.s[0], rc.rc.s[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = vt13_mouse_l,
        .mouse_r = vt13_mouse_r,
        .mouse_mid = vt13_mouse_mid,
        .pause = vt13_pause,
        .btn_l = vt13_btn_l,
        .btn_r = vt13_btn_r,
        .trigger = vt13_trigger,
        .dial = vt13_dial,
        .key_value = rc.key.v,
        .key_w = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = uart1_image_last_rx_tick_ms,
    };
    uart1_image_remote_store(&state);
    remote_control_set_rc_source(MANUAL_INPUT_SRC_IMAGE, &rc);
}

static bool_t uart1_image_link_try_decode_custom_rc(const uint8_t *data)
{
    if (data == NULL)
    {
        return 0;
    }

    uart1_image_rc_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic0 != (uint8_t)UART1_IMAGE_RC_MAGIC0 ||
        pkt.magic1 != (uint8_t)UART1_IMAGE_RC_MAGIC1 ||
        pkt.version != UART1_IMAGE_RC_VERSION)
    {
        return 0;
    }

    RC_ctrl_t rc = {0};
    int16_t raw_ch[5] = {0};
    uint8_t raw_sw[2] = {0};
    uint8_t log_range_mode = 0u;
    for (uint8_t i = 0u; i < 5u; i++)
    {
        raw_ch[i] = pkt.ch[i];
        if (pkt.range_mode == UART1_IMAGE_RC_RANGE_DBUS)
        {
            log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_660;
            rc.rc.ch[i] = uart1_image_link_scale_axis(pkt.ch[i], UART1_IMAGE_RC_ABS_MAX_DBUS);
        }
        else if (pkt.range_mode == UART1_IMAGE_RC_RANGE_VT13)
        {
            log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_1024;
            rc.rc.ch[i] = uart1_image_link_scale_axis(pkt.ch[i], UART1_IMAGE_RC_ABS_MAX_VT13);
        }
        else
        {
            uart1_image_parse_error_cnt++;
            return 0;
        }
    }

    raw_sw[0] = pkt.sw[0];
    raw_sw[1] = pkt.sw[1];
    rc.rc.s[0] = (char)uart1_image_link_sanitize_switch(pkt.sw[0]);
    rc.rc.s[1] = (char)uart1_image_link_sanitize_switch(pkt.sw[1]);
    rc.mouse.x = pkt.mouse_x;
    rc.mouse.y = pkt.mouse_y;
    rc.mouse.z = pkt.mouse_z;
    rc.mouse.press_l = ((pkt.mouse_btns & UART1_IMAGE_RC_BTN_LEFT) != 0u) ? 1u : 0u;
    rc.mouse.press_r = ((pkt.mouse_btns & UART1_IMAGE_RC_BTN_RIGHT) != 0u) ? 1u : 0u;
    rc.key.v = pkt.key_value;

    remote_control_log_raw_source(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
                                  log_range_mode,
                                  5u,
                                  raw_ch,
                                  raw_sw,
                                  &rc);
    uart1_image_last_range_mode = log_range_mode;
    uart1_image_remote_state_t state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
        .range_mode = log_range_mode,
        .raw_ch = { raw_ch[0], raw_ch[1], raw_ch[2], raw_ch[3], raw_ch[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { rc.rc.s[0], rc.rc.s[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = rc.mouse.press_l,
        .mouse_r = rc.mouse.press_r,
        .mouse_mid = 0u,
        .pause = 0u,
        .btn_l = 0u,
        .btn_r = 0u,
        .trigger = 0u,
        .dial = rc.rc.ch[4],
        .key_value = rc.key.v,
        .key_w = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = UART1_IMAGE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = uart1_image_last_rx_tick_ms,
    };
    uart1_image_remote_store(&state);
    remote_control_set_rc_source(MANUAL_INPUT_SRC_IMAGE, &rc);
    return 1;
}

static int16_t uart1_image_link_scale_axis(int16_t raw, int16_t raw_abs_max)
{
    if (raw_abs_max <= 0)
    {
        return 0;
    }
    return rc_scale_axis_by_abs(raw, raw_abs_max, (int16_t)RC_CH_VALUE_ABS_MAX);
}

static uint8_t uart1_image_link_sanitize_switch(uint8_t value)
{
    if (value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN)
    {
        return value;
    }
    return RC_SW_DOWN;
}

static uint8_t uart1_image_link_map_vt13_switch1(uint8_t value)
{
    // VT13/VTM switch 1 uses 0/1/2 for safe/normal/spin.
    // Normalize it to this project's up/mid/down switch model.
    switch (value)
    {
    case 0u:
        return RC_SW_UP;
    case 1u:
        return RC_SW_MID;
    case 2u:
        return RC_SW_DOWN;
    case 3u:
        return RC_SW_MID;
    default:
        return RC_SW_DOWN;
    }
}

static uint8_t uart1_image_link_map_vt13_switch2(uint8_t stop, uint8_t left, uint8_t right)
{
    if (stop != 0u)
    {
        return RC_SW_UP;
    }
    if (left != 0u)
    {
        return RC_SW_MID;
    }
    if (right != 0u)
    {
        return RC_SW_DOWN;
    }
    return RC_SW_DOWN;
}

static void uart1_tune_rx_start(void)
{
    uart1_image_link_stop();
    uart1_elrs_stop();
    uart1_rx_len = 0;
    uart1_cmd_ready = 0;
    uart1_cmd_seq = 0;

    bsp_usart1_set_rx_event_cb(NULL);
    bsp_usart1_set_rx_byte_cb(uart1_tune_on_byte);
    bsp_usart1_set_error_cb(uart1_tune_on_uart_error);
    (void)bsp_usart1_rx_it_start();
}

static void uart1_tune_try_send_telem(void)
{
    const aux_telem_config_t *cfg = &g_config.aux_telem;
    const uint8_t want_uart_justfloat = ((cfg->enable == 1u) || (cfg->enable == 4u)) ? 1u : 0u;
    // UART1 telemetry only runs in the dedicated tuning mode.
    const uint8_t uart1_can_tx = uart1_port_is_tune_mode(bsp_usart1_get_baudrate());
    if (!(want_uart_justfloat && uart1_can_tx))
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
    if (!bsp_usart1_tx_ready())
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
            memcpy(&uart1_vofa_frame[i * 4u], &v, 4u);
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
            memcpy(&uart1_vofa_frame[i * 4u], &v, 4u);
        }
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&uart1_vofa_frame[channel_num * 4u], &tail, 4u);

    const uint16_t frame_len = (uint16_t)((channel_num + 1u) * 4u);
    if (bsp_usart1_tx_dma(uart1_vofa_frame, frame_len) == 0)
    {
        aux_telem_tick = now;
    }
}

static bool_t uart1_tune_handle_line(const char *line)
{
    if (line == NULL)
    {
        return 0;
    }

    char buf[UART1_TUNE_RX_LINE_MAX];
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

    // Fast path: "<id>:<value>" sets one config parameter by numeric ID.
    char *colon = strchr(buf, ':');
    if (colon != NULL)
    {
        *colon = '\0';
        char *id_s = buf;
        char *v_s = colon + 1;

        while (*id_s == ' ' || *id_s == '\t')
        {
            id_s++;
        }
        while (*v_s == ' ' || *v_s == '\t')
        {
            v_s++;
        }

        char *id_end = id_s + strlen(id_s);
        while (id_end > id_s && (id_end[-1] == ' ' || id_end[-1] == '\t'))
        {
            id_end--;
        }
        *id_end = '\0';

        char *v_end = v_s + strlen(v_s);
        while (v_end > v_s && (v_end[-1] == ' ' || v_end[-1] == '\t'))
        {
            v_end--;
        }
        *v_end = '\0';

        uint16_t id = 0;
        fp32 v = 0.0f;
        if (uart1_tune_parse_u16(id_s, &id) && uart1_tune_parse_fp32(v_s, &v))
        {
            if (uart1_tune_set_app_param(id, v))
            {
                uart1_cmd_seq++;
                return 1;
            }
        }
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
        uart1_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "view") == 0)
    {
        uart1_cmd_seq++;
        return 0;
    }

    if (strcmp(argv[0], "u1") == 0 || strcmp(argv[0], "uart1") == 0)
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
                baud = UART1_TUNE_BAUD;
            }
            else if (strcmp(argv[2], "elrs") == 0)
            {
                baud = UART1_ELRS_BAUD;
            }
            else if (strcmp(argv[2], "image") == 0)
            {
                baud = UART1_IMAGE_LINK_BAUD;
            }
            else
            {
                return 0;
            }

            if (!uart1_port_apply_baud(baud))
            {
                return 0;
            }
            uart1_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "baud") == 0)
        {
            uint32_t baud = 0u;
            if (!uart1_tune_parse_u32(argv[2], &baud))
            {
                return 0;
            }
            if (!uart1_port_apply_baud(baud))
            {
                return 0;
            }
            uart1_cmd_seq++;
            return 1;
        }

        return 0;
    }

    if (strcmp(argv[0], "cf") == 0)
    {
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            chassis_tune_clear_follow_pid();
            uart1_cmd_seq++;
            return 1;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!uart1_tune_parse_fp32(argv[2], &v))
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
        uart1_cmd_seq++;
        return 1;
    }

    const bool_t is_ps = (strcmp(argv[0], "ps") == 0) || (strcmp(argv[0], "ys") == 0);
    const bool_t is_pa = (strcmp(argv[0], "pa") == 0) || (strcmp(argv[0], "ya") == 0);
    if (is_ps || is_pa)
    {
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!uart1_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        if (is_ps)
        {
            gimbal_tune_get_pitch_speed_pid(&pid);
        }
        else
        {
            gimbal_tune_get_pitch_angle_pid(&pid);
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

        if (is_ps)
        {
            gimbal_tune_set_pitch_speed_pid(&pid, 1);
        }
        else
        {
            gimbal_tune_set_pitch_angle_pid(&pid, 1);
        }

        uart1_cmd_seq++;
        return 1;
    }

    return 0;
}

static uint16_t aux_telem_min_period_ms(uint16_t channel_num)
{
    const uint32_t baud = bsp_usart1_get_baudrate();
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
        return (fp32)osKernelSysTick();
    case AUX_TELEM_SIG_SYS_AUX_CMD_SEQ:
        return (fp32)uart1_cmd_seq;
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

static bool_t uart1_tune_parse_fp32(const char *s, fp32 *out)
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

static bool_t uart1_tune_parse_u16(const char *s, uint16_t *out)
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

static bool_t uart1_tune_parse_u32(const char *s, uint32_t *out)
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

static uint8_t uart1_tune_to_u8(fp32 v)
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

static uint16_t uart1_tune_to_u16(fp32 v)
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

static int8_t uart1_tune_to_i8(fp32 v, int8_t min_v, int8_t max_v)
{
    if (min_v > max_v)
    {
        const int8_t t = min_v;
        min_v = max_v;
        max_v = t;
    }

    const fp32 min_f = (fp32)min_v;
    const fp32 max_f = (fp32)max_v;
    if (v <= min_f)
    {
        return min_v;
    }
    if (v >= max_f)
    {
        return max_v;
    }

    int32_t r = (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
    if (r < (int32_t)min_v)
    {
        r = (int32_t)min_v;
    }
    if (r > (int32_t)max_v)
    {
        r = (int32_t)max_v;
    }
    return (int8_t)r;
}

static bool_t uart1_tune_set_pid_field(pid_param_t *pid, uint8_t field, fp32 value)
{
    if (pid == NULL)
    {
        return 0;
    }

    switch (field)
    {
    case 0:
        pid->kp = value;
        return 1;
    case 1:
        pid->ki = value;
        return 1;
    case 2:
        pid->kd = value;
        return 1;
    case 3:
        pid->max_out = value;
        return 1;
    case 4:
        pid->max_iout = value;
        return 1;
    default:
        return 0;
    }
}

static void uart1_tune_apply_shoot_fric_speed_pid(void)
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

static void uart1_tune_apply_shoot_trigger_pid(void)
{
    taskENTER_CRITICAL();
    shoot_control.trigger_motor_pid.Kp = g_config.shoot.trigger_angle_pid.kp;
    shoot_control.trigger_motor_pid.Ki = g_config.shoot.trigger_angle_pid.ki;
    shoot_control.trigger_motor_pid.Kd = g_config.shoot.trigger_angle_pid.kd;
    PID_clear(&shoot_control.trigger_motor_pid);
    taskEXIT_CRITICAL();
}

static bool_t uart1_tune_set_app_param(uint16_t id, fp32 value)
{
    if (id >= 1u && id <= 5u)
    {
        if (!uart1_tune_set_pid_field(&g_config.gimbal.yaw_speed_pid, (uint8_t)(id - 1u), value))
        {
            return 0;
        }
        gimbal_tune_set_yaw_speed_pid(&g_config.gimbal.yaw_speed_pid, 1);
        return 1;
    }
    if (id >= 6u && id <= 10u)
    {
        (void)uart1_tune_set_pid_field(&g_config.gimbal.pitch_speed_pid, (uint8_t)(id - 6u), value);
        gimbal_tune_set_pitch_speed_pid(&g_config.gimbal.pitch_speed_pid, 1);
        return 1;
    }
    if (id >= 11u && id <= 15u)
    {
        (void)uart1_tune_set_pid_field(&g_config.gimbal.yaw_encode_angle_pid, (uint8_t)(id - 11u), value);
        gimbal_tune_set_yaw_angle_pid(&g_config.gimbal.yaw_encode_angle_pid, 1);
        return 1;
    }
    if (id >= 16u && id <= 20u)
    {
        (void)uart1_tune_set_pid_field(&g_config.gimbal.pitch_encode_angle_pid, (uint8_t)(id - 16u), value);
        gimbal_tune_set_pitch_angle_pid(&g_config.gimbal.pitch_encode_angle_pid, 1);
        return 1;
    }

    if (id >= 66u && id <= 70u)
    {
        (void)uart1_tune_set_pid_field(&g_config.chassis.motor_speed_pid, (uint8_t)(id - 66u), value);
        chassis_tune_set_motor_speed_pid(&g_config.chassis.motor_speed_pid, 1);
        return 1;
    }
    if (id >= 71u && id <= 75u)
    {
        (void)uart1_tune_set_pid_field(&g_config.chassis.follow_gimbal_pid, (uint8_t)(id - 71u), value);
        chassis_tune_set_follow_pid(&g_config.chassis.follow_gimbal_pid, 1);
        return 1;
    }
    if (id >= 76u && id <= 79u)
    {
        const uint8_t idx = (uint8_t)(id - 76u);
        if (idx >= 4u)
        {
            return 0;
        }
        g_config.chassis.motor_dir[idx] = uart1_tune_to_i8(value, -1, 1);
        return 1;
    }

    if (id >= 117u && id <= 121u)
    {
        (void)uart1_tune_set_pid_field(&g_config.shoot.fric_speed_pid, (uint8_t)(id - 117u), value);
        uart1_tune_apply_shoot_fric_speed_pid();
        return 1;
    }
    if (id >= 122u && id <= 125u)
    {
        const uint8_t idx = (uint8_t)(id - 122u);
        if (idx >= 4u)
        {
            return 0;
        }
        g_config.shoot.fric_motor_dir[idx] = uart1_tune_to_i8(value, -1, 1);
        return 1;
    }
    if (id >= 151u && id <= 153u)
    {
        (void)uart1_tune_set_pid_field(&g_config.shoot.trigger_angle_pid, (uint8_t)(id - 151u), value);
        uart1_tune_apply_shoot_trigger_pid();
        return 1;
    }

    if (id >= 167u && id <= 208u)
    {
        const uint16_t off = (uint16_t)(id - 167u);
        const uint8_t item = (uint8_t)(off / 3u);
        const uint8_t field = (uint8_t)(off % 3u);
        if (item >= 14u)
        {
            return 0;
        }

        detect_item_t *it = &g_config.detect.items[item];
        switch (field)
        {
        case 0:
            it->offline_time_ms = uart1_tune_to_u16(value);
            return 1;
        case 1:
            it->online_time_ms = uart1_tune_to_u16(value);
            return 1;
        case 2:
            it->priority = uart1_tune_to_u8(value);
            return 1;
        default:
            return 0;
        }
    }

    // ===== app_input mapping =====
    if (id >= 320u && id <= 337u)
    {
        const uint16_t off = (uint16_t)(id - 320u);
        const uint8_t axis = (uint8_t)(off / 2u);
        const uint8_t field = (uint8_t)(off % 2u);
        if (axis >= (uint8_t)INPUT_AXIS_COUNT)
        {
            return 0;
        }

        if (field == 0u)
        {
            uint8_t ch = uart1_tune_to_u8(value);
            if (ch > 4u)
            {
                ch = 4u;
            }
            g_config.input.axis[axis].rc_ch = ch;
        }
        else
        {
            g_config.input.axis[axis].invert = (uart1_tune_to_u8(value) != 0u) ? 1u : 0u;
        }

        remote_control_refresh();
        return 1;
    }
    if (id >= 338u && id <= 347u)
    {
        const uint16_t off = (uint16_t)(id - 338u);
        const uint8_t sw = (uint8_t)(off / 2u);
        const uint8_t field = (uint8_t)(off % 2u);
        if (sw >= (uint8_t)INPUT_SW_COUNT)
        {
            return 0;
        }

        if (field == 0u)
        {
            uint8_t idx = uart1_tune_to_u8(value);
            if (idx > 1u)
            {
                idx = 1u;
            }
            g_config.input.sw[sw].rc_sw = idx;
        }
        else
        {
            g_config.input.sw[sw].invert = (uart1_tune_to_u8(value) != 0u) ? 1u : 0u;
        }

        remote_control_refresh();
        return 1;
    }

    switch (id)
    {
    // ===== gimbal =====
    case 22:
        g_config.gimbal.control_period_ms = uart1_tune_to_u16(value);
        return 1;
    case 23:
        g_config.gimbal.channel_yaw = uart1_tune_to_u8(value);
        return 1;
    case 24:
        g_config.gimbal.channel_pitch = uart1_tune_to_u8(value);
        return 1;
    case 25:
        g_config.gimbal.channel_mode = uart1_tune_to_u8(value);
        return 1;
    case 26:
        g_config.gimbal.yaw_rc_sen = value;
        return 1;
    case 27:
        g_config.gimbal.pitch_rc_sen = value;
        return 1;
    case 28:
        g_config.gimbal.yaw_mouse_sen = value;
        return 1;
    case 29:
        g_config.gimbal.pitch_mouse_sen = value;
        return 1;
    case 30:
        g_config.gimbal.yaw_encode_sen = value;
        return 1;
    case 31:
        g_config.gimbal.pitch_encode_sen = value;
        return 1;
    case 32:
        g_config.gimbal.rc_deadband = uart1_tune_to_u16(value);
        return 1;
    case 41:
        g_config.gimbal.pitch_kick_up_current = value;
        return 1;
    case 42:
        g_config.gimbal.pitch_kick_down_current = value;
        return 1;
    case 43:
        g_config.gimbal.pitch_soft_limit_up = value;
        return 1;
    case 44:
        g_config.gimbal.pitch_soft_limit_down = value;
        return 1;
    case 45:
        g_config.gimbal.pitch_current_limit = value;
        return 1;
    case 48:
        g_config.gimbal.motor_ecd_to_rad = value;
        return 1;
    case 49:
        g_config.gimbal.cali_redundant_angle = value;
        return 1;
    case 50:
        g_config.gimbal.cali_motor_set = uart1_tune_to_u16(value);
        return 1;
    case 51:
        g_config.gimbal.cali_step_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 52:
        g_config.gimbal.cali_gyro_limit = value;
        return 1;
    case 53:
        g_config.gimbal.cali_pitch_max_step = uart1_tune_to_u8(value);
        return 1;
    case 54:
        g_config.gimbal.cali_pitch_min_step = uart1_tune_to_u8(value);
        return 1;
    case 55:
        g_config.gimbal.cali_yaw_max_step = uart1_tune_to_u8(value);
        return 1;
    case 56:
        g_config.gimbal.cali_yaw_min_step = uart1_tune_to_u8(value);
        return 1;
    case 57:
        g_config.gimbal.cali_start_step = uart1_tune_to_u8(value);
        return 1;
    case 58:
        g_config.gimbal.cali_end_step = uart1_tune_to_u8(value);
        return 1;
    case 59:
        g_config.gimbal.motionless_rc_deadline = value;
        return 1;
    case 60:
        g_config.gimbal.motionless_time_max_ms = uart1_tune_to_u16(value);
        return 1;
    case 61:
        g_config.gimbal.turn_speed = value;
        return 1;
    case 62:
        g_config.gimbal.turn_key_mask = uart1_tune_to_u16(value);
        return 1;
    case 63:
        g_config.gimbal.test_key_mask = uart1_tune_to_u16(value);
        return 1;
    case 64:
        g_config.gimbal.yaw_turn = uart1_tune_to_u8(value);
        return 1;
    case 65:
        g_config.gimbal.pitch_turn = uart1_tune_to_u8(value);
        return 1;

    // ===== pitch cali (SD) =====
    case 350:
        g_config.gimbal.pitch_cali.enable = (uart1_tune_to_u8(value) != 0u) ? 1u : 0u;
        return 1;
    case 351:
        g_config.gimbal.pitch_cali.angle_points = uart1_tune_to_u8(value);
        return 1;
    case 352:
        g_config.gimbal.pitch_cali.bullet_points = uart1_tune_to_u8(value);
        return 1;
    case 353:
    {
        uint8_t v = uart1_tune_to_u8(value);
        if (v > (uint8_t)PITCH_CALI_BULLET_SRC_MANUAL)
        {
            v = (uint8_t)PITCH_CALI_BULLET_SRC_REFEREE;
        }
        g_config.gimbal.pitch_cali.bullet_source = v;
        return 1;
    }
    case 354:
        g_config.gimbal.pitch_cali.bullet_min = uart1_tune_to_u16(value);
        return 1;
    case 355:
        g_config.gimbal.pitch_cali.bullet_max = uart1_tune_to_u16(value);
        return 1;
    case 356:
        g_config.gimbal.pitch_cali.bullet_manual = uart1_tune_to_u16(value);
        return 1;
    case 357:
        g_config.gimbal.pitch_cali.angle_margin = value;
        return 1;
    case 358:
        g_config.gimbal.pitch_cali.stable_angle_err = value;
        return 1;
    case 359:
        g_config.gimbal.pitch_cali.stable_gyro_err = value;
        return 1;
    case 360:
        g_config.gimbal.pitch_cali.stable_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 368:
        g_config.gimbal.pitch_cali.seek_k = value;
        return 1;
    case 361:
        g_config.gimbal.pitch_cali.hold_avg_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 362:
        g_config.gimbal.pitch_cali.breakaway_step_current = uart1_tune_to_u16(value);
        return 1;
    case 363:
        g_config.gimbal.pitch_cali.breakaway_step_period_ms = uart1_tune_to_u16(value);
        return 1;
    case 364:
        g_config.gimbal.pitch_cali.breakaway_max_extra_current = uart1_tune_to_u16(value);
        return 1;
    case 365:
        g_config.gimbal.pitch_cali.breakaway_gyro_threshold = value;
        return 1;
    case 366:
        g_config.gimbal.pitch_cali.breakaway_angle_threshold = value;
        return 1;
    case 367:
        g_config.gimbal.pitch_cali.recover_time_ms = uart1_tune_to_u16(value);
        return 1;

    // ===== chassis =====
    case 81:
        g_config.chassis.control_period_ms = uart1_tune_to_u16(value);
        return 1;
    case 82:
        g_config.chassis.channel_vx = uart1_tune_to_u8(value);
        return 1;
    case 83:
        g_config.chassis.channel_vy = uart1_tune_to_u8(value);
        return 1;
    case 84:
        g_config.chassis.channel_wz = uart1_tune_to_u8(value);
        return 1;
    case 85:
        g_config.chassis.channel_mode = uart1_tune_to_u8(value);
        return 1;
    case 86:
        g_config.chassis.vx_rc_sen = value;
        return 1;
    case 87:
        g_config.chassis.vy_rc_sen = value;
        return 1;
    case 88:
        g_config.chassis.angle_z_rc_sen = value;
        return 1;
    case 89:
        g_config.chassis.wz_rc_sen = value;
        return 1;
    case 90:
        g_config.chassis.accel_x_first_order = value;
        return 1;
    case 91:
        g_config.chassis.accel_y_first_order = value;
        return 1;
    case 92:
        g_config.chassis.rc_deadband = uart1_tune_to_u16(value);
        return 1;
    case 93:
        g_config.chassis.motor_speed_to_chassis_vx = value;
        return 1;
    case 94:
        g_config.chassis.motor_speed_to_chassis_vy = value;
        return 1;
    case 95:
        g_config.chassis.motor_speed_to_chassis_wz = value;
        return 1;
    case 96:
        g_config.chassis.motor_distance_to_center = value;
        return 1;
    case 97:
        g_config.chassis.rpm_to_vector = value;
        return 1;
    case 98:
        g_config.chassis.max_wheel_speed = value;
        return 1;
    case 99:
        g_config.chassis.max_vx_forward = value;
        return 1;
    case 100:
        g_config.chassis.max_vx_backward = value;
        return 1;
    case 101:
        g_config.chassis.max_vy_left = value;
        return 1;
    case 102:
        g_config.chassis.max_vy_right = value;
        return 1;
    case 103:
        g_config.chassis.wz_set_scale = value;
        return 1;
    case 104:
        g_config.chassis.swing_no_move_angle = value;
        return 1;
    case 105:
        g_config.chassis.swing_move_angle = value;
        return 1;
    case 106:
        g_config.chassis.max_motor_can_current = value;
        return 1;
    case 107:
        g_config.chassis.swing_key_mask = uart1_tune_to_u16(value);
        return 1;
    case 108:
        g_config.chassis.key_front_mask = uart1_tune_to_u16(value);
        return 1;
    case 109:
        g_config.chassis.key_back_mask = uart1_tune_to_u16(value);
        return 1;
    case 110:
        g_config.chassis.key_left_mask = uart1_tune_to_u16(value);
        return 1;
    case 111:
        g_config.chassis.key_right_mask = uart1_tune_to_u16(value);
        return 1;

    // ===== shoot =====
    case 112:
        g_config.shoot.fric_speed_up_rpm = value;
        return 1;
    case 113:
        g_config.shoot.fric_speed_down_rpm = value;
        return 1;
    case 114:
        g_config.shoot.fric_speed_off_rpm = value;
        return 1;
    case 115:
        g_config.shoot.fric_speed_step_rpm_s = value;
        return 1;
    case 116:
        g_config.shoot.fric_ready_ratio = value;
        return 1;
    case 126:
        g_config.shoot.rc_mode_channel = uart1_tune_to_u8(value);
        return 1;
    case 127:
        g_config.shoot.control_period_ms = uart1_tune_to_u16(value);
        return 1;
    case 128:
        g_config.shoot.key_on_mask = uart1_tune_to_u16(value);
        return 1;
    case 129:
        g_config.shoot.key_off_mask = uart1_tune_to_u16(value);
        return 1;
    case 130:
        g_config.shoot.shoot_done_key_off_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 131:
        g_config.shoot.press_long_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 132:
        g_config.shoot.rc_s_long_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 133:
        g_config.shoot.up_add_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 136:
        g_config.shoot.motor_rpm_to_speed = value;
        return 1;
    case 137:
        g_config.shoot.motor_ecd_to_angle = value;
        return 1;
    case 138:
        g_config.shoot.full_count = uart1_tune_to_u8(value);
        return 1;
    case 139:
        g_config.shoot.trigger_speed_single = value;
        return 1;
    case 140:
        g_config.shoot.trigger_speed_continuous = value;
        return 1;
    case 141:
        g_config.shoot.trigger_speed_ready = value;
        return 1;
    case 142:
        g_config.shoot.key_off_judge_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 143:
        g_config.shoot.switch_trigger_on = uart1_tune_to_u8(value);
        return 1;
    case 144:
        g_config.shoot.switch_trigger_off = uart1_tune_to_u8(value);
        return 1;
    case 145:
        g_config.shoot.block_trigger_speed = value;
        return 1;
    case 146:
        g_config.shoot.block_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 147:
        g_config.shoot.reverse_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 148:
        g_config.shoot.reverse_speed_limit = value;
        return 1;
    case 149:
        g_config.shoot.pi_over_four = value;
        return 1;
    case 150:
        g_config.shoot.pi_over_ten = value;
        return 1;
    case 156:
        g_config.shoot.trigger_bullet_pid_max_out = value;
        return 1;
    case 157:
        g_config.shoot.trigger_bullet_pid_max_iout = value;
        return 1;
    case 158:
        g_config.shoot.trigger_ready_pid_max_out = value;
        return 1;
    case 159:
        g_config.shoot.trigger_ready_pid_max_iout = value;
        return 1;
    case 160:
        g_config.shoot.heat_remain_value = uart1_tune_to_u16(value);
        return 1;

    // ===== power =====
    case 161:
        g_config.power.power_limit = value;
        return 1;
    case 162:
        g_config.power.warning_power = value;
        return 1;
    case 163:
        g_config.power.warning_power_buffer = value;
        return 1;
    case 164:
        g_config.power.no_judge_total_current_limit = value;
        return 1;
    case 165:
        g_config.power.buffer_total_current_limit = value;
        return 1;
    case 166:
        g_config.power.power_total_current_limit = value;
        return 1;

    // ===== detect =====
    case 209:
        g_config.detect.enable_mask = uart1_tune_to_u16(value);
        return 1;
    case 211:
        g_config.detect.control_period_ms = uart1_tune_to_u16(value);
        return 1;

    // ===== imu =====
    case 218:
        g_config.imu.fusion_mode = (imu_fusion_mode_e)uart1_tune_to_i8(value, 0, (int8_t)IMU_FUSION_AHRS_9AXIS);
        return 1;
    case 219:
        g_config.imu.imu_temp_pwm_max = uart1_tune_to_u16(value);
        return 1;

    // ===== voltage =====
    case 221:
        g_config.voltage.full_battery_voltage = value;
        return 1;
    case 222:
        g_config.voltage.low_battery_voltage = value;
        return 1;
    case 223:
        g_config.voltage.voltage_drop = value;
        return 1;

    // ===== buzzer =====
    case 224:
        g_config.buzzer.soft_beep_psc = uart1_tune_to_u16(value);
        return 1;
    case 225:
        g_config.buzzer.soft_beep_duration_ms = uart1_tune_to_u16(value);
        return 1;
    case 226:
        g_config.buzzer.enable = uart1_tune_to_u8(value);
        return 1;
    case 227:
        g_config.buzzer.gimbal_warn_psc = uart1_tune_to_u16(value);
        return 1;
    case 228:
        g_config.buzzer.gimbal_warn_pwm = uart1_tune_to_u16(value);
        return 1;
    case 229:
        g_config.buzzer.imu_cali_psc = uart1_tune_to_u16(value);
        return 1;
    case 230:
        g_config.buzzer.imu_cali_pwm = uart1_tune_to_u16(value);
        return 1;
    case 231:
        g_config.buzzer.gimbal_cali_psc = uart1_tune_to_u16(value);
        return 1;
    case 232:
        g_config.buzzer.gimbal_cali_pwm = uart1_tune_to_u16(value);
        return 1;
    case 233:
        g_config.buzzer.rc_cali_middle_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 234:
        g_config.buzzer.rc_cali_start_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 235:
        g_config.buzzer.rc_cali_cycle_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 236:
        g_config.buzzer.rc_cali_pause_time_ms = uart1_tune_to_u16(value);
        return 1;
    case 237:
        g_config.buzzer.rc_cmd_long_time_ms = uart1_tune_to_u16(value);
        return 1;

    // ===== led =====
    case 238:
        g_config.led.slot_on_ms = uart1_tune_to_u16(value);
        return 1;
    case 239:
        g_config.led.slot_off_ms = uart1_tune_to_u16(value);
        return 1;
    case 240:
        g_config.led.slot_gap_ms = uart1_tune_to_u16(value);
        return 1;

    // ===== aux_telem =====
    case 241:
        g_config.aux_telem.enable = uart1_tune_to_u8(value);
        return 1;
    case 242:
        g_config.aux_telem.period_ms = uart1_tune_to_u16(value);
        return 1;
    case 243:
        g_config.aux_telem.channel_num = uart1_tune_to_u16(value);
        return 1;

    // ===== test =====
    case 244:
        g_config.test.mode = (test_mode_e)uart1_tune_to_i8(value, 0, (int8_t)TEST_MODE_PITCH_CALI);
        return 1;

    // ===== chassis swing =====
    case 245:
        g_config.chassis.swing_amp_rad = value;
        return 1;
    case 246:
        g_config.chassis.swing_half_period_ms = uart1_tune_to_u16(value);
        return 1;
    case 247:
        g_config.chassis.swing_center_hold_min_ms = uart1_tune_to_u16(value);
        return 1;
    case 248:
        g_config.chassis.swing_center_hold_max_ms = uart1_tune_to_u16(value);
        return 1;
    case 249:
        g_config.chassis.swing_mode_key_mask = uart1_tune_to_u16(value);
        return 1;
    case 250:
        g_config.chassis.gyro_spin_var_key_mask = uart1_tune_to_u16(value);
        return 1;

    // ===== manual_input =====
    case 300:
    {
        uint8_t v = uart1_tune_to_u8(value);
        if (v > (uint8_t)MANUAL_INPUT_SRC_MAX)
        {
            v = (uint8_t)MANUAL_INPUT_SRC_AUTO;
        }
        g_config.manual_input.active_source = v;
        remote_control_refresh();
        return 1;
    }
    case 301:
    {
        uint8_t v = uart1_tune_to_u8(value);
        if (v > (uint8_t)MANUAL_INPUT_MIX_MERGE)
        {
            v = (uint8_t)MANUAL_INPUT_MIX_SELECT_LATEST;
        }
        g_config.manual_input.mix_mode = v;
        remote_control_refresh();
        return 1;
    }
    case 302:
        g_config.manual_input.source_timeout_ms = uart1_tune_to_u16(value);
        remote_control_refresh();
        return 1;

    // ===== app_input: ELRS mapping =====
    case 310:
        g_config.input.elrs_ch_map[0] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 311:
        g_config.input.elrs_ch_map[1] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 312:
        g_config.input.elrs_ch_map[2] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 313:
        g_config.input.elrs_ch_map[3] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 314:
        g_config.input.elrs_ch_map[4] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 315:
        g_config.input.elrs_sw_map[0] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;
    case 316:
        g_config.input.elrs_sw_map[1] = uart1_tune_to_u8(value);
        remote_control_refresh();
        return 1;

    // ===== manual_input: board key =====
    case 317:
        g_config.manual_input.board_key_key_mask = uart1_tune_to_u16(value);
        remote_control_refresh();
        return 1;

    default:
        return 0;
    }
}

static void uart1_tune_on_byte(uint8_t b)
{
    if (uart1_cmd_ready)
    {
        // Drop input until the current command is processed.
    }
    else if (b == '\r' || b == '\n')
    {
        if (uart1_rx_len > 0u)
        {
            const uint16_t n = (uart1_rx_len >= (UART1_TUNE_RX_LINE_MAX - 1u)) ? (UART1_TUNE_RX_LINE_MAX - 1u) : uart1_rx_len;
            uart1_rx_line[n] = '\0';
            memcpy(uart1_cmd_line, uart1_rx_line, n + 1u);
            uart1_cmd_ready = 1;
            uart1_rx_len = 0;
        }
    }
    else if (b == 0x08u || b == 0x7Fu)
    {
        if (uart1_rx_len > 0u)
        {
            uart1_rx_len--;
        }
    }
    else
    {
        // Only accept printable ASCII / TAB as tuning commands. Drop binary/noise to
        // avoid accidentally changing config when using a wireless UART bridge.
        if (b == '\t' || (b >= 0x20u && b <= 0x7Eu))
        {
            if (uart1_rx_len < (UART1_TUNE_RX_LINE_MAX - 1u))
            {
                uart1_rx_line[uart1_rx_len++] = (char)b;
            }
            else
            {
                uart1_rx_len = 0;
            }
        }
        else
        {
            uart1_rx_len = 0;
        }
    }
}

static uint8_t uart1_tune_on_uart_error(void)
{
    uart1_rx_len = 0;
    return 0u;
}

static void uart1_tune_poll(void)
{
    if (uart1_cmd_ready)
    {
        char line[UART1_TUNE_RX_LINE_MAX];
        taskENTER_CRITICAL();
        const bool_t ready = uart1_cmd_ready;
        uart1_cmd_ready = 0;
        if (ready)
        {
            strncpy(line, uart1_cmd_line, sizeof(line) - 1u);
            line[sizeof(line) - 1u] = '\0';
        }
        else
        {
            line[0] = '\0';
        }
        taskEXIT_CRITICAL();

        if (line[0] != '\0')
        {
            const bool_t changed = uart1_tune_handle_line(line);
            if (changed && sdlog_is_active())
            {
                uint16_t n = 0u;
                while (n < (UART1_TUNE_RX_LINE_MAX - 1u) && line[n] != '\0')
                {
                    n++;
                }

                uint8_t payload[4u + UART1_TUNE_RX_LINE_MAX];
                const uint32_t seq = uart1_cmd_seq;
                memcpy(payload, &seq, 4u);
                memcpy(&payload[4u], line, n);
                payload[4u + n] = '\0';
                sdlog_write(SDLOG_TAG_UART1_TUNE, payload, (uint16_t)(4u + n + 1u));
            }
        }
    }
}
