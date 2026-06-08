/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "LowCmd.h"
#include "control_manager.h"
#include "config.h"
#include "runtime_instance.h"
#include "types.h"

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
#ifndef WATCH_RUNTIME_MAX_MOTORS
#define WATCH_RUNTIME_MAX_MOTORS MotorCount
#endif
#ifndef WATCH_RUNTIME_MAX_CONTROLLERS
#define WATCH_RUNTIME_MAX_CONTROLLERS CONTROL_MANAGER_MAX_CONTROLLERS
#endif
#ifndef WATCH_RUNTIME_MAX_TASK_MODULES
#define WATCH_RUNTIME_MAX_TASK_MODULES ROBOT_TASK_MODULE_MAX
#endif
#ifndef WATCH_RUNTIME_MAX_ENTRIES
#define WATCH_RUNTIME_MAX_ENTRIES 64u
#endif

// 为减少任务间耦合，本文件不直接依赖 chassis/gimbal/shoot 等头文件；
// mode 字段使用 watch_ 内部枚举，数值与对应模块枚举保持一致（用于调试观测）。

typedef enum
{
    WATCH_CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW = 0,
    WATCH_CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    WATCH_CHASSIS_VECTOR_NO_FOLLOW_YAW,
    WATCH_CHASSIS_VECTOR_RAW,
} watch_chassis_mode_e;

typedef enum
{
    WATCH_GIMBAL_MOTOR_RAW = 0,
    WATCH_GIMBAL_MOTOR_ENCONDE,
} watch_gimbal_motor_mode_e;

typedef enum
{
    WATCH_SHOOT_STOP = 0,
    WATCH_SHOOT_READY_FRIC,
    WATCH_SHOOT_READY_BULLET,
    WATCH_SHOOT_READY,
    WATCH_SHOOT_BULLET,
    WATCH_SHOOT_CONTINUE_BULLET,
    WATCH_SHOOT_DONE,
} watch_shoot_mode_e;

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
} watch_boot_stage_e;

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
} watch_task_id_e;

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
} watch_imu_stage_e;

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
} watch_irq_id_e;

typedef struct
{
    uint32_t beat_count;
    uint32_t last_tick_ms;
    uint32_t max_gap_ms;
    uint32_t wait_count;
    uint32_t timeout_count;
    uint32_t error_count;
} watch_task_diag_entry_t;

typedef struct
{
    watch_task_diag_entry_t default_task;
    watch_task_diag_entry_t detect_task;
    watch_task_diag_entry_t imu_task;
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    watch_task_diag_entry_t gimbal_control_task;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    watch_task_diag_entry_t chassis_control_task;
#endif
    watch_task_diag_entry_t can_feedback_rx_task;
    watch_task_diag_entry_t can_command_tx_task;
    watch_task_diag_entry_t rc_sbus_task;
    watch_task_diag_entry_t host_link_task;
    watch_task_diag_entry_t elrs_task;
#if WATCH_ENABLE_ARM_J0_UNITREE
    watch_task_diag_entry_t arm_task;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    watch_task_diag_entry_t wheelleg_mit_task;
#endif
} watch_task_diag_t;

typedef struct
{
    uint32_t hit_count;
    uint32_t last_tick_ms;
} watch_irq_diag_entry_t;

typedef struct
{
    watch_irq_diag_entry_t ist8310_exti;
    watch_irq_diag_entry_t imu_exti;
    watch_irq_diag_entry_t sd_exti;
    watch_irq_diag_entry_t can1_rx0;
    watch_irq_diag_entry_t can2_rx0;
    watch_irq_diag_entry_t usart1;
    watch_irq_diag_entry_t uart7;
    watch_irq_diag_entry_t uart8;
    watch_irq_diag_entry_t otg_fs;
    watch_irq_diag_entry_t tim6_dac;
    watch_irq_diag_entry_t dma_usart1_rx;
    watch_irq_diag_entry_t dma_spi5_tx;
    watch_irq_diag_entry_t dma_spi5_rx;
    watch_irq_diag_entry_t dma_sdio_tx;
} watch_irq_diag_t;

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
} watch_rc_t;

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
} watch_newrc_t;

typedef struct
{
    watch_chassis_mode_e mode;
    watch_chassis_mode_e last_mode;
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
} watch_chassis_t;

typedef struct
{
    watch_gimbal_motor_mode_e yaw_mode;
    watch_gimbal_motor_mode_e pitch_mode;
    fp32 yaw_angle_deg;
    fp32 yaw_set_deg;
    fp32 yaw_gyro_dps;
    int16_t yaw_current;
    int16_t yaw_rpm;
    int16_t yaw_current_fb;
    uint16_t yaw_ecd;
    uint8_t yaw_temp;
    fp32 pitch_angle_deg;
    fp32 pitch_set_deg;
    fp32 pitch_gyro_dps;
    int16_t pitch_current;
    int16_t pitch_rpm;
    int16_t pitch_current_fb;
    uint16_t pitch_ecd;
    uint8_t pitch_temp;
} watch_gimbal_t;

typedef struct
{
    watch_shoot_mode_e mode;
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
} watch_shoot_t;

typedef struct
{
    fp32 quat[4];      // wxyz
    fp32 gyro_dps[3];  // deg/s
    fp32 accel[3];     // m/s^2
    fp32 angle_deg[3]; // INS 原始欧拉角 yaw/roll/pitch in deg (see INS_task.h offsets); 未应用 PITCH_TURN/YAW_TURN
} watch_imu_t;

typedef struct
{
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

} watch_diag_t;

typedef struct
{
    uint32_t heap_free;
    uint32_t heap_ever_free;
    uint32_t watch_update_count;
    uint32_t watch_update_tick_ms;
    uint32_t scheduler_state;
    uint32_t task_count;
    uint32_t current_task_handle;
    char current_task_name[16];

    uint8_t imu_task_stage;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t imu_task_enter_count;
    uint32_t imu_task_stage_tick_ms;
    watch_task_diag_t task;
    watch_irq_diag_t irq;

    // FreeRTOS uxTaskGetStackHighWaterMark() results (unit: words).
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
} watch_rtos_t;

typedef struct
{
    watch_boot_stage_e boot_stage;
    watch_boot_stage_e boot_trace[4]; // [0] latest, [1] previous...
    uint32_t error_handler_count;
    watch_boot_stage_e error_stage;
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
} watch_fault_t;

typedef struct
{
    uint8_t reserved0;
} watch_dual_gimbal_t;

typedef struct
{
    uint8_t reserved0;
} watch_wheelleg_servo_t;

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
} watch_wheelleg_mit_motor_t;

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
} watch_wheelleg_mit_leg_t;

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
} watch_wheelleg_mit_foot_test_t;

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
    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_foot_x_m;
    fp32 target_foot_y_m;
    fp32 target_leg_theta_deg;
    fp32 pitch_deg;
    fp32 x_dot_mps;
    watch_wheelleg_mit_leg_t leg[2]; // 0:left, 1:right
    watch_wheelleg_mit_motor_t joint[4]; // 0:LF, 1:LB, 2:RF, 3:RB
    watch_wheelleg_mit_motor_t wheel[2]; // 0:left, 1:right
    watch_wheelleg_mit_foot_test_t foot_test[2]; // 0:left, 1:right
} watch_wheelleg_mit_t;

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
} watch_arm_j0_unitree_t;

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
} watch_runtime_motor_t;

typedef struct
{
    const char *name;
    uint8_t module;
    uint8_t create_state;
    uint16_t create_fail_count;
    uint32_t thread_handle;
    uint32_t create_attempt_count;
} watch_runtime_task_module_t;

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
} watch_runtime_controller_t;

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
} watch_runtime_domain_t;

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
    uint8_t device_count;
    uint8_t entry_count;
    uint8_t entry_visible_count;
    uint8_t domain_count;
    uint8_t profile_kind;
    uint8_t board_kind;
    uint8_t board_can_bus_count;
    uint8_t board_has_fpu;
    uint8_t rt_profiler_count;
    uint8_t rt_profiler_active_count;
    uint8_t rt_profiler_over_budget_count;
    uint8_t reserved0;
    uint32_t board_cpu_hz;
    uint32_t rt_profiler_total_overrun_count;
    uint32_t rt_profiler_max_last_us;
    uint32_t rt_profiler_max_budget_us;
    uint32_t rt_profiler_max_over_budget_us;
    uint32_t lowcmd_seq;
    uint32_t lowcmd_rejected_count;
    uint32_t lowcmd_emergency_stop_count;
    uint32_t lowcmd_last_reject_tick_ms;
    uint16_t lowcmd_last_reject_writer;
    uint16_t lowcmd_last_reject_owner;
    uint16_t lowcmd_emergency_writer;
    uint8_t lowcmd_emergency_active;
    uint8_t reserved1;
    uint32_t active_claim_mask;
    runtime_instance_ref_t entry[WATCH_RUNTIME_MAX_ENTRIES];
    watch_runtime_task_module_t task_module[WATCH_RUNTIME_MAX_TASK_MODULES];
    watch_runtime_motor_t motor[WATCH_RUNTIME_MAX_MOTORS];
    watch_runtime_controller_t controller[WATCH_RUNTIME_MAX_CONTROLLERS];
    watch_runtime_domain_t domain[CONTROL_DOMAIN__COUNT];
} watch_runtime_t;

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
    WATCH_BLOCK_COUNT
} watch_block_id_e;

typedef uint8_t (*watch_block_active_fn_t)(void);

typedef struct
{
    watch_block_id_e id;
    const char *name;
    const void *data;
    uint32_t size;
    watch_block_active_fn_t is_active;
} watch_block_desc_t;

typedef struct
{
    watch_rc_t rc;
    watch_newrc_t newrc;
    watch_runtime_t runtime;
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    watch_chassis_t chassis;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
    watch_wheelleg_servo_t wheelleg_servo;
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    watch_wheelleg_mit_t wheelleg_mit;
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    watch_gimbal_t gimbal;
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
    watch_dual_gimbal_t dual_gimbal;
#endif
#if WATCH_ENABLE_SHOOT_RM
    watch_shoot_t shoot;
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    watch_arm_j0_unitree_t arm_j0_unitree;
#endif
    watch_imu_t imu;
    watch_diag_t diag;
    watch_rtos_t rtos;
    watch_fault_t fault;
} watch_t;

extern watch_t g_watch;
void watch_init(void);
void watch_update(void);
void watch_diag_set_boot_stage(watch_boot_stage_e stage);
void watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr);
void watch_diag_set_error_args(uint32_t arg0, uint32_t arg1);
void watch_diag_mark_fatal(uint32_t reason, uint32_t task_handle, const char *task_name);
void watch_task_module_create_reset(void);
void watch_task_module_create_result(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state);
void watch_task_beat(watch_task_id_e task_id);
void watch_task_wait(watch_task_id_e task_id);
void watch_task_timeout(watch_task_id_e task_id);
void watch_task_error(watch_task_id_e task_id);
void watch_imu_set_stage(watch_imu_stage_e stage);
void watch_irq_hit(watch_irq_id_e irq_id);
const watch_block_desc_t *watch_get_block_table(uint32_t *count);
const watch_block_desc_t *watch_find_block(watch_block_id_e id);
uint8_t watch_block_is_active(watch_block_id_e id);
