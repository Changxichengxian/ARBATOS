/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "LowCmd.h"
#include "ControlMgr.h"
#include "RobotConfig.h"
#include "RuntimeInstance.h"
#include "Types.h"

#ifndef WATCH_ENABLE_LOCOMOTION_CLASSIC
#define WATCH_ENABLE_LOCOMOTION_CLASSIC 1
#endif
#ifndef WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
#define WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO 1
#endif
#ifndef WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
#define WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT 1
#endif
#ifndef WATCH_ENABLE_GIMBAL_SINGLE
#define WATCH_ENABLE_GIMBAL_SINGLE 1
#endif
#ifndef WATCH_ENABLE_GIMBAL_DUAL
#define WATCH_ENABLE_GIMBAL_DUAL 1
#endif
#ifndef WATCH_ENABLE_SHOOT_RM
#define WATCH_ENABLE_SHOOT_RM 1
#endif
#ifndef WATCH_ENABLE_ARM_J0_UNITREE
#define WATCH_ENABLE_ARM_J0_UNITREE 1
#endif
#ifndef WATCH_ENABLE_RUNTIME_COPY
#define WATCH_ENABLE_RUNTIME_COPY 1
#endif
#ifndef WATCH_ENABLE_COMM_COPY
#define WATCH_ENABLE_COMM_COPY 1
#endif
#ifndef WATCH_ENABLE_DIAG_COPY
#define WATCH_ENABLE_DIAG_COPY 1
#endif
#ifndef WATCH_RUNTIME_MAX_MOTORS
#define WATCH_RUNTIME_MAX_MOTORS MotorCount
#endif
#ifndef WATCH_RUNTIME_MAX_CONTROLLERS
#define WATCH_RUNTIME_MAX_CONTROLLERS CONTROL_MGR_MAX_CONTROLLERS
#endif
#ifndef WATCH_RUNTIME_MAX_TASK_MODULES
#define WATCH_RUNTIME_MAX_TASK_MODULES ROBOT_TASK_MODULE_MAX
#endif
#ifndef WATCH_RUNTIME_MAX_ENTRIES
#define WATCH_RUNTIME_MAX_ENTRIES 64u
#endif

// 为减少任务间耦合，本文件不直接依赖 chassis/gimbal/shoot 等头文件；
// mode 字段使用 Watch 内部枚举，数值与对应模块枚举保持一致（用于调试观测）。

typedef enum
{
    WATCH_CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW = 0,
    WATCH_CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    WATCH_CHASSIS_VECTOR_NO_FOLLOW_YAW,
    WATCH_CHASSIS_VECTOR_RAW,
} WatchChassisMode;

typedef enum
{
    WATCH_GIMBAL_MOTOR_RAW = 0,
    WATCH_GIMBAL_MOTOR_ENCODER,
} WatchGimbalMotorMode;

typedef enum
{
    WATCH_SHOOT_STOP = 0,
    WATCH_SHOOT_READY_FRIC,
    WATCH_SHOOT_READY_BULLET,
    WATCH_SHOOT_READY,
    WATCH_SHOOT_BULLET,
    WATCH_SHOOT_CONTINUE_BULLET,
    WATCH_SHOOT_DONE,
} WatchShootMode;

typedef enum
{
    WATCH_BOOT_STAGE_NONE = 0,
    WATCH_BOOT_STAGE_HAL_INIT_DONE,
    WATCH_BOOT_STAGE_SYS_CLOCK_OSC,
    WATCH_BOOT_STAGE_SYS_CLOCK_BUS,
    WATCH_BOOT_STAGE_GPIO_INIT,
    WATCH_BOOT_STAGE_DMA_INIT,
    WATCH_BOOT_STAGE_CAN1_INIT,
    WATCH_BOOT_STAGE_CAN2_INIT,
    WATCH_BOOT_STAGE_USART1_INIT,
    WATCH_BOOT_STAGE_USART3_INIT,
    WATCH_BOOT_STAGE_USART6_INIT,
    WATCH_BOOT_STAGE_UART5_INIT,
    WATCH_BOOT_STAGE_UART7_INIT,
    WATCH_BOOT_STAGE_UART8_INIT,
    WATCH_BOOT_STAGE_SPI5_INIT,
    WATCH_BOOT_STAGE_SDIO_INIT,
    WATCH_BOOT_STAGE_SDIO_CARD_INIT,
    WATCH_BOOT_STAGE_SDIO_WIDE_BUS,
    WATCH_BOOT_STAGE_SDIO_DMA_RX_INIT,
    WATCH_BOOT_STAGE_SDIO_DMA_TX_INIT,
    WATCH_BOOT_STAGE_TIM3_INIT,
    WATCH_BOOT_STAGE_TIM12_INIT,
    WATCH_BOOT_STAGE_CAN_FILTER_INIT,
    WATCH_BOOT_STAGE_BUZZER_INIT,
    WATCH_BOOT_STAGE_REMOTE_CONTROL_INIT,
    WATCH_BOOT_STAGE_FREERTOS_INIT,
    WATCH_BOOT_STAGE_SCHEDULER_START,
    WATCH_BOOT_STAGE_DEFAULT_TASK_START,
    WATCH_BOOT_STAGE_USB_DEVICE_USBD_INIT,
    WATCH_BOOT_STAGE_USB_DEVICE_REGISTER_CLASS,
    WATCH_BOOT_STAGE_USB_DEVICE_REGISTER_IF,
    WATCH_BOOT_STAGE_USB_DEVICE_START,
    WATCH_BOOT_STAGE_RUN,
} WatchBootStage;

typedef enum
{
    WATCH_TASK_DEFAULT = 0,
    WATCH_TASK_DETECT,
    WATCH_TASK_IMU,
    WATCH_TASK_GIMBAL_CONTROL,
    WATCH_TASK_CHASSIS_CONTROL,
    WATCH_TASK_CAN_FEEDBACK_RX,
    WATCH_TASK_CAN_COMMAND_TX,
    WATCH_TASK_RC_SBUS,
    WATCH_TASK_HOST_LINK,
    WATCH_TASK_ELRS,
    WATCH_TASK_ARM,
    WATCH_TASK_WHEELLEG_MIT,
} WatchTaskId;

typedef enum
{
    WATCH_IMU_STAGE_NONE = 0,
    WATCH_IMU_STAGE_ENTER = 1,
    WATCH_IMU_STAGE_INIT_DELAY_DONE = 2,
    WATCH_IMU_STAGE_BMI088_INIT_TRY = 3,
    WATCH_IMU_STAGE_BMI088_INIT_RETRY = 4,
    WATCH_IMU_STAGE_BMI088_INIT_OK = 5,
    WATCH_IMU_STAGE_GYRO_BOOT_CALIB = 6,
    WATCH_IMU_STAGE_FUSION_LOOP = 7,
} WatchImuStage;

typedef enum
{
    WATCH_IRQ_IST8310_EXTI = 0,
    WATCH_IRQ_IMU_EXTI,
    WATCH_IRQ_SD_EXTI,
    WATCH_IRQ_CAN1_RX0,
    WATCH_IRQ_CAN2_RX0,
    WATCH_IRQ_USART1,
    WATCH_IRQ_UART7,
    WATCH_IRQ_UART8,
    WATCH_IRQ_OTG_FS,
    WATCH_IRQ_TIM6_DAC,
    WATCH_IRQ_DMA_USART1_RX,
    WATCH_IRQ_DMA_SPI5_TX,
    WATCH_IRQ_DMA_SPI5_RX,
    WATCH_IRQ_DMA_SDIO_TX,
} WatchIrqId;

typedef struct
{
    uint32_t beat_count;
    uint32_t last_tick_ms;
    uint32_t max_gap_ms;
    uint32_t wait_count;
    uint32_t timeout_count;
    uint32_t error_count;
} WatchTaskDiagEntry;

typedef struct
{
    WatchTaskDiagEntry default_task;
    WatchTaskDiagEntry DetectTask;
    WatchTaskDiagEntry imu_task;
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    WatchTaskDiagEntry GimbalControlTask;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    WatchTaskDiagEntry ChassisControlTask;
#endif
    WatchTaskDiagEntry CanRxTask;
    WatchTaskDiagEntry CanTxTask;
    WatchTaskDiagEntry RcSbusTask;
    WatchTaskDiagEntry HostLinkTask;
    WatchTaskDiagEntry ElrsTask;
#if WATCH_ENABLE_ARM_J0_UNITREE
    WatchTaskDiagEntry ArmTask;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    WatchTaskDiagEntry WheelLegMitTask;
#endif
} WatchTaskDiag;

typedef struct
{
    uint32_t hit_count;
    uint32_t last_tick_ms;
} WatchIrqDiagEntry;

typedef struct
{
    WatchIrqDiagEntry ist8310_exti;
    WatchIrqDiagEntry imu_exti;
    WatchIrqDiagEntry sd_exti;
    WatchIrqDiagEntry can1_rx0;
    WatchIrqDiagEntry can2_rx0;
    WatchIrqDiagEntry usart1;
    WatchIrqDiagEntry uart7;
    WatchIrqDiagEntry uart8;
    WatchIrqDiagEntry otg_fs;
    WatchIrqDiagEntry tim6_dac;
    WatchIrqDiagEntry dma_usart1_rx;
    WatchIrqDiagEntry dma_spi5_tx;
    WatchIrqDiagEntry dma_spi5_rx;
    WatchIrqDiagEntry dma_sdio_tx;
} WatchIrqDiag;

typedef struct
{
    int16_t ch[5];
    char s[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint16_t key;
} WatchRc;

typedef struct
{
    uint8_t valid;
    uint8_t proto;
    uint8_t range_mode;
    int16_t raw_ch[5];
    int16_t ch[5];
    char s[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint8_t mouse_mid;
    uint8_t pause;
    uint8_t btn_l;
    uint8_t btn_r;
    uint8_t trigger;
    uint16_t dial;
    uint16_t key_value;
    uint8_t key_w;
    uint8_t key_s;
    uint8_t key_a;
    uint8_t key_d;
    uint8_t key_shift;
    uint8_t key_ctrl;
    uint8_t key_q;
    uint8_t key_e;
    uint8_t key_r;
    uint8_t key_f;
    uint8_t key_g;
    uint8_t key_z;
    uint8_t key_x;
    uint8_t key_c;
    uint8_t key_v;
    uint8_t key_b;
    uint32_t last_rx_tick_ms;
} WatchNewrc;

typedef struct
{
    WatchChassisMode mode;
    WatchChassisMode last_mode;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 yaw_deg;
    int16_t motor_rpm[4];
    int16_t motor_current[4];
    fp32 motor_speed_set[4];
    uint16_t motor_ecd[4];
    uint8_t motor_temp[4];
    uint8_t fault_configured_mask;
    uint8_t fault_active_mask;
    uint8_t fault_blocking_mask;
    uint8_t fault_recovery_mask;
    uint8_t fault_inhibit_mask;
    uint8_t fault_hold_zero_mask;
    uint8_t fault_recovery_input_safe;
    uint8_t reserved0;
    uint16_t motor_reason_mask[4];
    uint32_t motor_feedback_age_ms[4];
    uint32_t fault_inhibit_fail_count;
    uint32_t fault_release_fail_count;
} WatchChassis;

typedef struct
{
    WatchGimbalMotorMode yaw_mode;
    WatchGimbalMotorMode pitch_mode;
    fp32 yaw_angle_deg;
    fp32 yaw_set_deg;
    fp32 yaw_gyro_dps;
    int16_t yaw_current;
    int16_t yaw_rpm;
    int16_t yaw_current_fb;
    uint16_t yaw_ecd;
    fp32 yaw_ecd_deg;
    uint8_t yaw_temp;
    fp32 pitch_angle_deg;
    fp32 pitch_set_deg;
    fp32 pitch_gyro_dps;
    int16_t pitch_current;
    int16_t pitch_rpm;
    int16_t pitch_current_fb;
    uint16_t pitch_ecd;
    fp32 pitch_ecd_deg;
    uint8_t pitch_temp;
    uint8_t yaw_online;
    uint8_t pitch_online;
    uint8_t imu_online;
    uint8_t follow_available;
    uint8_t fault_configured_mask;
    uint8_t fault_active_mask;
    uint8_t fault_blocking_mask;
    uint8_t fault_recovery_mask;
    uint8_t fault_inhibit_mask;
    uint8_t fault_hold_zero_mask;
    uint8_t fault_imu_required_mask;
    uint8_t fault_recovery_input_safe;
    uint16_t offline_mask;
    uint16_t required_offline_mask;
    uint16_t yaw_reason_mask;
    uint16_t pitch_reason_mask;
    uint32_t yaw_feedback_age_ms;
    uint32_t pitch_feedback_age_ms;
    uint32_t imu_age_ms;
    uint32_t fault_inhibit_fail_count;
    uint32_t fault_release_fail_count;
} WatchGimbal;

typedef struct
{
    WatchShootMode mode;
    int16_t fric_speed_set_rpm;
    int16_t fric_current_cmd[4];
    fp32 trigger_angle_deg;
    fp32 trigger_set_deg;
    fp32 trigger_speed;
    fp32 trigger_speed_set;
    int16_t trigger_current;
    int16_t trigger_rpm;
    uint16_t trigger_ecd;
    uint8_t trigger_temp;
    uint16_t heat_limit;
    uint16_t heat;
    int16_t fric_rpm[4];
    int16_t fric_current[4];
    uint8_t fric_temp[4];
} WatchShoot;

typedef struct
{
    fp32 quat[4];      // wxyz
    fp32 gyro_dps[3];  // deg/s
    fp32 accel[3];     // m/s^2
    fp32 angle_deg[3]; // INS 原始欧拉角 yaw/roll/pitch in deg (see InsTask.h offsets); 未应用 PITCH_TURN/YAW_TURN
} WatchImu;

typedef struct
{
    uint8_t reason;
    uint8_t motion_on;
    uint8_t flags;
    uint8_t gate_bits;
    uint8_t mode;
    uint8_t frame;
    uint8_t t10ms;
    uint8_t cur_on;

    uint16_t crc_rx;
    uint16_t crc_calc;
    uint32_t usb_rx;
    uint32_t lc_ok;
    uint32_t lc_fail;
    uint32_t lc_age_ms;
    uint32_t motion_age_ms;
    uint32_t alg;
    uint32_t alg_age_ms;

    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
    int16_t alg_vx_cmps;
    int16_t alg_vy_cmps;
    int16_t alg_wz_mradps;
    int16_t cur[4];

    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;

#if WATCH_ENABLE_DIAG_COPY
    uint8_t offline_need_geometry;
    uint8_t offline_need_mass_inertia;
    uint8_t offline_need_com_inertia;
    uint8_t offline_need_motor_limits;
    uint8_t offline_need_lqr_model;
    uint8_t offline_need_stage_k;
    uint8_t offline_need_qr_weights;
    uint8_t offline_need_sign_table;

    uint8_t power_need_imu_sign;
    uint8_t power_need_motor_online;
    uint8_t power_need_joint_zero_dir;
    uint8_t power_need_wheel_dir;
    uint8_t power_need_fk_leg_state;
    uint8_t power_need_jacobian_dir;
    uint8_t power_need_observer_speed;
    uint8_t power_need_lqr_response;

    uint8_t live_profile_on;
    uint8_t live_manual_on;
    uint8_t live_enabled;
    uint8_t live_control_stage;
    uint8_t live_mode;
    uint8_t live_status_valid;
    uint8_t live_state_valid;
    uint8_t live_all_mit_online;
    uint8_t live_imu_online;
    uint8_t live_sdlog_active;
    uint8_t live_lqr_debug_valid;
    uint8_t live_operation_mode;
    uint16_t live_fault_flags;
    uint16_t reserved1;

    fp32 lqr_wheel_scale;
    fp32 lqr_hip_scale;
    fp32 max_wheel_torque_nm;
    fp32 max_joint_torque_nm;

    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_foot_x_m;
    fp32 target_foot_y_m;
    fp32 target_leg_theta_deg;

    fp32 pitch_deg;
    fp32 pitch_gyro_deg_s;
    fp32 x_dot_mps;
    fp32 left_leg_length_m;
    fp32 right_leg_length_m;
    fp32 left_leg_theta_deg;
    fp32 right_leg_theta_deg;
    fp32 left_front_rel_deg;
    fp32 left_back_rel_deg;
    fp32 right_front_rel_deg;
    fp32 right_back_rel_deg;
    fp32 left_wheel_vel_deg_s;
    fp32 right_wheel_vel_deg_s;
    fp32 left_wheel_cmd_torque_nm;
    fp32 right_wheel_cmd_torque_nm;
    fp32 wheel_balance_cmd_nm;
    fp32 wheel_turn_cmd_nm;
    fp32 left_wheel_fb_torque_nm;
    fp32 right_wheel_fb_torque_nm;
    fp32 left_support_force_n;
    fp32 right_support_force_n;
    fp32 left_hip_torque_nm;
    fp32 right_hip_torque_nm;

    fp32 lqr_theta_err_deg;
    fp32 lqr_dtheta_deg_s;
    fp32 lqr_x_m;
    fp32 lqr_v_err_mps;
    fp32 lqr_pitch_err_deg;
    fp32 lqr_left_pitch_err_deg;
    fp32 lqr_pitch_gyro_deg_s;
    fp32 lqr_left_output_nm;
    fp32 lqr_right_output_nm;

    uint8_t run_capture_active;
    uint8_t run_capture_valid;
    uint16_t reserved2;
    uint32_t run_sample_count;
    fp32 run_target_v_avg_mps;
    fp32 run_target_v_abs_max_mps;
    fp32 run_target_yaw_rate_avg_radps;
    fp32 run_target_yaw_rate_abs_max_radps;
    fp32 run_x_dot_avg_mps;
    fp32 run_x_dot_min_mps;
    fp32 run_x_dot_max_mps;
    fp32 run_pitch_avg_deg;
    fp32 run_pitch_min_deg;
    fp32 run_pitch_max_deg;
    fp32 run_lqr_pitch_gyro_avg_deg_s;
    fp32 run_lqr_pitch_gyro_min_deg_s;
    fp32 run_lqr_pitch_gyro_max_deg_s;
    fp32 run_yaw_gyro_avg_deg_s;
    fp32 run_yaw_gyro_abs_max_deg_s;
    fp32 run_wheel_sum_avg_nm;
    fp32 run_wheel_sum_abs_max_nm;
    fp32 run_wheel_diff_avg_nm;
    fp32 run_wheel_diff_abs_max_nm;
    fp32 run_wheel_balance_avg_nm;
    fp32 run_wheel_balance_abs_max_nm;
    fp32 run_wheel_turn_avg_nm;
    fp32 run_wheel_turn_abs_max_nm;
    fp32 run_lqr_v_err_avg_mps;
    fp32 run_lqr_v_err_min_mps;
    fp32 run_lqr_v_err_max_mps;
    fp32 run_lqr_x_avg_m;
    fp32 run_lqr_x_min_m;
    fp32 run_lqr_x_max_m;
    fp32 run_lqr_pitch_err_avg_deg;
    fp32 run_lqr_pitch_err_min_deg;
    fp32 run_lqr_pitch_err_max_deg;
    fp32 run_lqr_left_pitch_err_avg_deg;
    fp32 run_lqr_left_pitch_err_min_deg;
    fp32 run_lqr_left_pitch_err_max_deg;
#endif

} WatchDiag;

typedef struct
{
    uint32_t heap_free;
    uint32_t heap_ever_free;
    uint32_t WatchUpdateEnterCount;
    uint32_t WatchUpdateCount;
    uint32_t WatchUpdateTickMs;
    uint32_t scheduler_state;
    uint32_t task_count;
    uint32_t current_task_handle;
    char current_task_name[16];

    uint8_t imu_task_stage;
    uint8_t WatchUpdateStage;
    uint16_t reserved0;
    uint32_t imu_task_enter_count;
    uint32_t imu_task_stage_tick_ms;
    uint32_t WatchUpdateStageTickMs;
    WatchTaskDiag task;
    WatchIrqDiag irq;

    // FreeRTOS uxTaskGetStackHighWaterMark() results (unit: words).
    uint32_t stack_default;
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    uint32_t stack_gimbal;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    uint32_t stack_chassis;
#endif
    uint32_t stack_detect;
    uint32_t stack_calibrate;

    // Fatal hooks (FreeRTOS stack overflow / malloc failed)
    // fatal_reason: 0 none, 1 stack_overflow, 2 malloc_failed
    uint32_t fatal_reason;
    uint32_t fatal_task_handle;
    char fatal_task_name[16];
} WatchRtos;

typedef struct
{
    WatchBootStage boot_stage;
    WatchBootStage boot_trace[4]; // [0] latest, [1] previous...
    uint32_t error_handler_count;
    WatchBootStage error_stage;
    uint32_t error_tick_ms;
    uint32_t error_ipsr;
    uint32_t error_arg0;
    uint32_t error_arg1;

    // Captured in HardFault_HandlerC (stm32f4xx_it.c) for post-mortem debugging.
    uint32_t hardfault_valid;
    uint32_t hardfault_r0;
    uint32_t hardfault_r1;
    uint32_t hardfault_r2;
    uint32_t hardfault_r3;
    uint32_t hardfault_r12;
    uint32_t hardfault_lr;
    uint32_t hardfault_pc;
    uint32_t hardfault_psr;
    uint32_t hardfault_exc_return;
    uint32_t hardfault_msp;
    uint32_t hardfault_psp;
    uint32_t hardfault_cfsr;
    uint32_t hardfault_hfsr;
    uint32_t hardfault_dfsr;
    uint32_t hardfault_afsr;
    uint32_t hardfault_mmfar;
    uint32_t hardfault_bfar;
    uint32_t hardfault_icsr;
    uint32_t hardfault_shcsr;
    uint32_t hardfault_control;
    uint32_t hardfault_stack_ptr;
    uint32_t hardfault_basic_ptr;
    uint32_t hardfault_stack_dump[16];
} WatchFault;

typedef struct
{
    uint32_t rc_uart_rx_event_count;
    uint32_t rc_uart_bad_size_count;
    uint32_t rc_uart_error_count;
    uint32_t rc_uart_last_error;
    uint32_t rc_uart_restart_count;
    uint32_t rc_uart_drop_count;
    uint32_t RcSbusFrameCount;
    uint32_t rc_set_source_count;
    uint16_t rc_uart_last_size;
    uint16_t rc_uart_last_event;

    uint32_t CanRxCount[3];
    uint32_t CanRxDropCount[3];
    uint32_t CanTxCount[3];
    uint32_t CanTxFailCount[3];
    uint16_t can_last_rx_id[3];
    uint16_t can_last_tx_id[3];
    uint8_t can_last_rx_dlc[3];
    uint8_t can_last_tx_dlc[3];
    uint8_t can_protocol_lec[3];
    uint8_t can_protocol_dlec[3];
    uint8_t can_bus_off[3];
    uint8_t CanTxErrorCount[3];
    uint8_t CanRxErrorCount[3];
} WatchComm;

typedef struct
{
    uint8_t active;
    uint8_t upper_online;
    uint8_t upper_limited;
    uint8_t yaw_online;
    uint8_t pitch_online;
    uint8_t imu_online;
    uint8_t follow_available;
    uint8_t fault_configured_mask;
    uint8_t fault_active_mask;
    uint8_t fault_blocking_mask;
    uint8_t fault_recovery_mask;
    uint8_t fault_inhibit_mask;
    uint8_t fault_hold_zero_mask;
    uint8_t fault_imu_required_mask;
    uint8_t fault_recovery_input_safe;
    uint16_t offline_mask;
    uint16_t required_offline_mask;
    uint16_t yaw_reason_mask;
    uint16_t yaw_upper_reason_mask;
    uint16_t pitch_reason_mask;
    uint16_t reserved1;
    uint32_t yaw_feedback_age_ms;
    uint32_t yaw_upper_feedback_age_ms;
    uint32_t pitch_feedback_age_ms;
    uint32_t imu_age_ms;
    uint32_t fault_inhibit_fail_count;
    uint32_t fault_release_fail_count;
    fp32 total_set_deg;
    fp32 big_angle_deg;
    fp32 big_set_deg;
    fp32 big_error_deg;
    fp32 upper_angle_deg;
    fp32 upper_set_deg;
    fp32 upper_error_deg;
    fp32 upper_gyro_dps;
    int16_t yaw_current;
    int16_t yaw_upper_current;
    uint16_t yaw_upper_ecd;
} WatchDualGimbal;

typedef struct
{
    uint8_t reserved0;
} WatchWheelLegServo;

typedef enum
{
    WATCH_UPDATE_STAGE_NONE = 0u,
    WATCH_UPDATE_STAGE_RC = 1u,
    WATCH_UPDATE_STAGE_NEWRC = 2u,
    WATCH_UPDATE_STAGE_COMM = 3u,
    WATCH_UPDATE_STAGE_RUNTIME = 4u,
    WATCH_UPDATE_STAGE_IMU = 5u,
    WATCH_UPDATE_STAGE_CHASSIS = 6u,
    WATCH_UPDATE_STAGE_GIMBAL = 7u,
    WATCH_UPDATE_STAGE_SHOOT = 8u,
    WATCH_UPDATE_STAGE_ARM = 9u,
    WATCH_UPDATE_STAGE_WHEELLEG = 10u,
    WATCH_UPDATE_STAGE_DIAG = 11u,
    WATCH_UPDATE_STAGE_RTOS = 12u,
    WATCH_UPDATE_STAGE_DONE = 13u,
} WatchUpdateStage;

typedef struct
{
    uint8_t id;
    uint8_t online;
    uint8_t cmd_active;
    uint8_t cmd_mode;
    uint8_t fb_bus;
    uint8_t fb_rx_dlc;
    uint8_t fb_rx_data0;
    uint8_t fb_rx_data0_low4;
    uint8_t fb_rx_data0_high4;
    uint8_t fb_motor_id;
    uint8_t fb_state;
    uint8_t applied_active;
    uint8_t applied_mode;
    uint8_t applied_drive_state;
    uint8_t applied_flags;
    uint8_t applied_bus;
    uint8_t applied_transport;
    uint8_t applied_protocol;
    uint8_t reserved0;
    uint16_t fb_rx_id;
    uint16_t applied_tx_id;
    uint16_t cmd_writer;
    uint16_t cmd_timeout_ms;
    uint32_t fb_rx_count;
    uint32_t fb_last_rx_tick_ms;
    uint32_t cmd_seq;
    uint32_t cmd_tick_ms;
    uint32_t applied_tick_ms;
    fp32 cmd_position_deg;
    fp32 cmd_velocity_deg_s;
    fp32 cmd_kp;
    fp32 cmd_kd;
    fp32 cmd_torque_nm;
    int16_t applied_current;
    int16_t reserved1;
    fp32 applied_position_deg;
    fp32 applied_velocity_deg_s;
    fp32 applied_kp;
    fp32 applied_kd;
    fp32 applied_torque_nm;
    fp32 fb_position_deg;
    fp32 fb_velocity_deg_s;
    fp32 fb_torque_nm;
    fp32 rel_position_deg;
    fp32 cmd_minus_fb_deg;
} WatchWheelLegMitMotor;

typedef struct
{
    uint8_t front_online;
    uint8_t back_online;
    uint8_t wheel_online;
    uint8_t reserved0;
    fp32 length_m;
    fp32 theta_deg;
    fp32 alpha_deg;
    fp32 support_force_n;
    fp32 wheel_torque_nm;
} WatchWheelLegMitLeg;

typedef struct
{
    uint8_t wheel_zero_valid;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    fp32 target_x_m;
    fp32 target_y_m;
    fp32 target_length_m;
    fp32 wheel_zero_deg;
    fp32 wheel_dx_m;
    fp32 wheel_comp_deg;
    fp32 wheel_target_deg;
} WatchWheelLegMitFootTest;

typedef struct
{
    uint8_t status_valid;
    uint8_t state_valid;
    uint8_t profile_on;
    uint8_t manual_on;
    uint8_t enabled;
    uint8_t mode;
    uint8_t last_mode;
    uint8_t health;
    uint8_t controller_active;
    uint8_t test_mode;
    uint8_t foot_test_phase;
    uint8_t foot_test_ik_ok;
    uint8_t input_chassis_switch;
    uint8_t enable_switch_pos;
    uint16_t fault_flags;
    uint16_t domain_fault_flags;
    uint8_t domain_member_count;
    uint8_t domain_online_mask;
    uint8_t domain_binding_valid;
    uint8_t recovery_input_safe;
    uint8_t domain_outputs_active;
    uint8_t domain_inhibit_complete;
    uint8_t domain_action;
    uint8_t domain_recovery_pending;
    uint8_t domain_fault_active;
    uint8_t domain_ever_faulted;
    uint8_t domain_device_count;
    uint8_t reserved_domain1;
    uint16_t active_controller_id;
    uint16_t lowcmd_last_reject_writer;
    uint16_t lowcmd_last_reject_owner;
    uint16_t lowcmd_emergency_writer;
    uint8_t lowcmd_emergency_active;
    uint8_t reserved0;
    uint32_t lowcmd_seq;
    uint32_t lowcmd_rejected_count;
    uint32_t lowcmd_emergency_stop_count;
    uint32_t lowcmd_last_reject_tick_ms;
    uint32_t domain_active_member_mask;
    uint32_t domain_blocking_member_mask;
    uint32_t domain_active_reason_mask;
    uint32_t domain_blocking_reason_mask;
    uint32_t domain_history_reason_mask;
    uint32_t domain_first_fault_ms;
    uint32_t domain_last_fault_ms;
    uint32_t domain_healthy_since_ms;
    uint32_t domain_stop_count;
    uint32_t domain_stop_fail_count;
    uint32_t domain_inhibit_fail_count;
    uint32_t domain_inhibit_release_fail_count;
    uint32_t domain_last_stop_tick_ms;
    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_foot_x_m;
    fp32 target_foot_y_m;
    fp32 target_leg_theta_deg;
    fp32 pitch_deg;
    fp32 x_dot_mps;
    WatchWheelLegMitLeg leg[2]; // 0:left, 1:right
    WatchWheelLegMitMotor joint[4]; // 0:LF, 1:LB, 2:RF, 3:RB
    WatchWheelLegMitMotor wheel[2]; // 0:left, 1:right
    WatchWheelLegMitFootTest foot_test[2]; // 0:left, 1:right
} WatchWheelLegMit;

typedef struct
{
    uint8_t enabled;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint8_t online;
    uint8_t last_mode;
    uint8_t motor_error;
    int8_t motor_temp;
    uint8_t last_tx_status;
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_frame_count;
    uint32_t rx_crc_fail_count;
    uint32_t rx_parse_error_count;
    uint32_t last_rx_tick_ms;
    fp32 cmd_speed_deg_s;
    fp32 cmd_kd;
    fp32 torque_nm;
    fp32 joint_speed_deg_s;
    fp32 joint_position_deg;
} WatchArmJ0Unitree;

typedef struct
{
    const char *name;
    uint16_t actuator_id;
    uint8_t role;
    uint8_t role_index;
    uint8_t enabled;
    uint8_t bus;
    uint8_t transport;
    uint8_t protocol;
    uint8_t control_mode;
    uint8_t cmd_caps;
    uint8_t model;
    uint8_t drive_state;
    uint8_t applied_mode;
    uint8_t applied_flags;
    uint8_t reserved0;
    int16_t applied_current;
    uint16_t reserved1;
    uint32_t applied_tick_ms;
    fp32 applied_torque_nm;
} WatchRuntimeMotor;

typedef struct
{
    const char *name;
    const char *module_name;
    uint8_t module;
    uint8_t kind;
    uint8_t create_state;
    uint8_t priority;
    uint16_t create_fail_count;
    uint16_t period_ms;
    uint16_t budget_us;
    uint16_t stack_words;
    uint16_t flags;
    uint8_t require_count;
    uint8_t provide_count;
    uint32_t thread_handle;
    uint32_t create_attempt_count;
} WatchRuntimeTaskModule;

typedef struct
{
    const char *name;
    uint16_t id;
    uint16_t period_ms;
    uint16_t phase_ms;
    uint8_t domain;
    uint8_t active;
    uint8_t priority;
    uint8_t input_count;
    uint8_t output_count;
} WatchRuntimeController;

typedef struct
{
    const char *active_name;
    uint16_t active_id;
    uint16_t pending_id;
    uint8_t domain;
    uint8_t active;
    uint8_t state;
    uint8_t pending_request;
    uint8_t last_reason;
    uint8_t last_result;
    uint32_t update_count;
    uint32_t transition_count;
    uint32_t reject_count;
} WatchRuntimeDomain;

typedef struct
{
    uint8_t motor_count;
    uint8_t motor_visible_count;
    uint8_t motor_enabled_count;
    uint8_t controller_count;
    uint8_t controller_visible_count;
    uint8_t active_controller_count;
    uint8_t task_module_count;
    uint8_t task_module_visible_count;
    uint8_t module_desc_count;
    uint8_t module_missing_desc_count;
    uint8_t module_required_count;
    uint8_t module_fast_count;
    uint8_t device_count;
    uint8_t entry_count;
    uint8_t entry_visible_count;
    uint8_t domain_count;
    uint8_t profile_kind;
    uint8_t BoardKind;
    uint8_t BoardCanBusCount;
    uint8_t BoardHasFpu;
    uint8_t rtProfCount;
    uint8_t RtProfActive_count;
    uint8_t RtProfOverBudget_count;
    uint8_t reserved0;
    uint32_t BoardCpuHz;
    uint32_t rtProfTotalOverrun;
    uint32_t rtProfMaxLastUs;
    uint32_t rtProfMaxBudgetUs;
    uint32_t rtProfMaxOverBudgetUs;
    uint32_t lowcmd_seq;
    uint32_t lowcmd_rejected_count;
    uint32_t lowcmd_emergency_stop_count;
    uint32_t lowcmd_inhibit_acquire_count;
    uint32_t lowcmd_inhibit_release_count;
    uint32_t lowcmd_inhibit_mask;
    uint32_t lowcmd_snapshot_retry_count;
    uint32_t lowcmd_snapshot_fallback_count;
    uint32_t lowcmd_last_reject_tick_ms;
    uint16_t lowcmd_last_reject_writer;
    uint16_t lowcmd_last_reject_owner;
    uint16_t lowcmd_emergency_writer;
    uint8_t lowcmd_emergency_active;
    uint8_t reserved1;
    uint32_t motor_feedback_conflict_count;
    uint32_t motor_feedback_table_full_count;
    uint16_t motor_feedback_conflict_id;
    uint8_t motor_feedback_conflict_bus;
    uint8_t motor_feedback_conflict_kept;
    uint8_t motor_feedback_conflict_dropped;
    uint8_t reserved2;
    uint32_t active_claim_mask;
    RuntimeInstanceRef entry[WATCH_RUNTIME_MAX_ENTRIES];
    WatchRuntimeTaskModule task_module[WATCH_RUNTIME_MAX_TASK_MODULES];
    WatchRuntimeMotor motor[WATCH_RUNTIME_MAX_MOTORS];
    WatchRuntimeController controller[WATCH_RUNTIME_MAX_CONTROLLERS];
    WatchRuntimeDomain domain[ControlDomainCount];
} WatchRuntime;

typedef enum
{
    WATCH_BLOCK_RC = 0u,
    WATCH_BLOCK_NEWRC,
    WATCH_BLOCK_RUNTIME,
    WATCH_BLOCK_LOCOMOTION_CLASSIC,
    WATCH_BLOCK_LOCOMOTION_WHEELLEG_SERVO,
    WATCH_BLOCK_LOCOMOTION_WHEELLEG_MIT,
    WATCH_BLOCK_GIMBAL_SINGLE,
    WATCH_BLOCK_GIMBAL_DUAL,
    WATCH_BLOCK_SHOOT_RM,
    WATCH_BLOCK_ARM_J0_UNITREE,
    WATCH_BLOCK_IMU,
    WATCH_BLOCK_DIAG,
    WATCH_BLOCK_RTOS,
    WATCH_BLOCK_FAULT,
    WATCH_BLOCK_COMM,
    WATCH_BLOCK_COUNT
} WatchBlockId;

typedef uint8_t (*WatchBlockActiveFn)(void);

typedef struct
{
    WatchBlockId id;
    const char *name;
    const void *data;
    uint32_t size;
    WatchBlockActiveFn is_active;
} WatchBlockDesc;

typedef struct
{
    WatchRc rc;
    WatchNewrc newrc;
#if WATCH_ENABLE_RUNTIME_COPY
    WatchRuntime runtime;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    WatchChassis chassis;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
    WatchWheelLegServo WheelLegServo;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    WatchWheelLegMit WheelLegMit;
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    WatchGimbal gimbal;
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
    WatchDualGimbal dual_gimbal;
#endif
#if WATCH_ENABLE_SHOOT_RM
    WatchShoot shoot;
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    WatchArmJ0Unitree ArmJ0Unitree;
#endif
    WatchImu imu;
    WatchDiag diag;
    WatchRtos rtos;
    WatchFault fault;
#if WATCH_ENABLE_COMM_COPY
    WatchComm comm;
#endif
} Watch;

extern Watch g_watch;
struct ManualInputState;
void WatchInit(void);
void WatchUpdate(void);
void WatchUpdateRcSnapshot(const struct ManualInputState *rc);
void WatchDiagSetBootStage(WatchBootStage stage);
void WatchDiagMarkErrorHandler(uint32_t tick_ms, uint32_t ipsr);
void WatchDiagSetErrorArgs(uint32_t arg0, uint32_t arg1);
void WatchDiagMarkFatal(uint32_t reason, uint32_t task_handle, const char *task_name);
void WatchTaskModuleCreateReset(void);
void WatchTaskModuleCreateResult(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state);
void WatchTaskBeat(WatchTaskId task_id);
void WatchTaskWait(WatchTaskId task_id);
void WatchTaskTimeout(WatchTaskId task_id);
void WatchTaskError(WatchTaskId task_id);
void WatchImuSetStage(WatchImuStage stage);
void WatchIrqHit(WatchIrqId irq_id);
const WatchBlockDesc *WatchGetBlockTable(uint32_t *count);
const WatchBlockDesc *WatchFindBlock(WatchBlockId id);
uint8_t WatchBlockIsActive(WatchBlockId id);
