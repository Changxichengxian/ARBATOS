/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "struct_typedef.h"
#include "rt_profiler.h"

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
} watch_task_id_e;

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
    watch_task_diag_entry_t gimbal_control_task;
    watch_task_diag_entry_t chassis_control_task;
    watch_task_diag_entry_t can_feedback_rx_task;
    watch_task_diag_entry_t can_command_tx_task;
    watch_task_diag_entry_t rc_sbus_task;
    watch_task_diag_entry_t host_link_task;
    watch_task_diag_entry_t elrs_task;
    watch_task_diag_entry_t arm_task;
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
    int16_t can1_0x200[4]; // motor1/motor2/pitch/trigger
    int16_t can1_0x1ff[4]; // motor205/yaw/motor207/motor208
    uint8_t flags[8];      // 0:dbus_lost 1:zero_force 2:yaw_offline 3:pitch_offline 4:trigger_offline 5:chassis_zeroed
    int16_t can1_1ff_status;
    uint32_t can1_err;
    uint32_t can2_err;
    uint8_t can1_tx_status;
    uint8_t can2_tx_status;
    uint8_t manual_active_source;
    uint8_t sd_mounted;
    uint8_t sdlog_active;
    uint8_t reserved0[3];
    uint32_t can1_rx_drop;
    uint32_t can2_rx_drop;
    uint32_t can1_tx_count;
    uint32_t can2_tx_count;
    uint32_t can1_tx_fail;
    uint32_t can2_tx_fail;
    uint32_t manual_sbus_frame_count;
    uint32_t manual_set_source_count;
    uint32_t sdlog_dropped;
    uint32_t sdlog_ring_used;
    uint32_t sdlog_ring_free;
    uint32_t sdlog_bytes_flushed;
    uint32_t sdlog_last_sync_ms;
    int32_t sdlog_last_error;
    uint32_t image_last_rx_tick_ms;
    uint32_t image_frame_count;
    uint32_t image_controller_frame_count;
    uint32_t image_client_frame_count;
    uint32_t image_vt13_frame_count;
    uint32_t image_crc_error_count;
    uint32_t image_parse_error_count;
    uint32_t image_restart_count;
    uint16_t image_last_cmd_id;
    uint8_t image_port_active;
    uint8_t image_last_range_mode;
    uint32_t watch_update_count;
    uint32_t watch_update_tick_ms;
    uint32_t scheduler_state;
    uint32_t task_count;
    uint32_t current_task_handle;
    char current_task_name[16];
    rt_profiler_stats_t rt_profiler[RT_PROFILER_COUNT];
    watch_task_diag_t task;
    watch_irq_diag_t irq;
    watch_boot_stage_e boot_stage;
    watch_boot_stage_e boot_trace[4]; // [0] 最新, [1] 上一次...
    uint32_t error_handler_count;
    watch_boot_stage_e error_stage;
    uint32_t error_tick_ms;
    uint32_t error_ipsr;
    uint32_t error_arg0;
    uint32_t error_arg1;
} watch_diag_t;

typedef struct
{
    uint32_t heap_free;
    uint32_t heap_ever_free;

    // FreeRTOS uxTaskGetStackHighWaterMark() results (unit: words).
    uint32_t stack_gimbal;
    uint32_t stack_chassis;
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
    uint8_t reserved0;
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
    fp32 cmd_speed_rad_s;
    fp32 cmd_kd;
    fp32 torque_nm;
    fp32 joint_speed_rad_s;
    fp32 joint_position_rad;
} watch_arm_j0_unitree_t;

typedef enum
{
    WATCH_BLOCK_RC = 0u,
    WATCH_BLOCK_NEWRC,
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
    watch_chassis_t chassis;
    watch_wheelleg_servo_t wheelleg_servo;
    watch_wheelleg_mit_t wheelleg_mit;
    watch_gimbal_t gimbal;
    watch_dual_gimbal_t dual_gimbal;
    watch_shoot_t shoot;
    watch_arm_j0_unitree_t arm_j0_unitree;
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
void watch_task_beat(watch_task_id_e task_id);
void watch_task_wait(watch_task_id_e task_id);
void watch_task_timeout(watch_task_id_e task_id);
void watch_task_error(watch_task_id_e task_id);
void watch_irq_hit(watch_irq_id_e irq_id);
const watch_block_desc_t *watch_get_block_table(uint32_t *count);
const watch_block_desc_t *watch_find_block(watch_block_id_e id);
uint8_t watch_block_is_active(watch_block_id_e id);
