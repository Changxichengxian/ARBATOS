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

// 为减少任务间耦合，本文件不直接依赖 chassis/gimbal/shoot 等头文件；
// mode 字段使用 app_watch_ 内部枚举，数值与对应模块枚举保持一致（用于调试观测）。

typedef enum
{
    APP_WATCH_CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW = 0,
    APP_WATCH_CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    APP_WATCH_CHASSIS_VECTOR_NO_FOLLOW_YAW,
    APP_WATCH_CHASSIS_VECTOR_RAW,
} app_watch_chassis_mode_e;

typedef enum
{
    APP_WATCH_GIMBAL_MOTOR_RAW = 0,
    APP_WATCH_GIMBAL_MOTOR_ENCONDE,
} app_watch_gimbal_motor_mode_e;

typedef enum
{
    APP_WATCH_SHOOT_STOP = 0,
    APP_WATCH_SHOOT_READY_FRIC,
    APP_WATCH_SHOOT_READY_BULLET,
    APP_WATCH_SHOOT_READY,
    APP_WATCH_SHOOT_BULLET,
    APP_WATCH_SHOOT_CONTINUE_BULLET,
    APP_WATCH_SHOOT_DONE,
} app_watch_shoot_mode_e;

typedef enum
{
    APP_WATCH_BOOT_STAGE_NONE = 0,
    APP_WATCH_BOOT_STAGE_HAL_INIT_DONE,
    APP_WATCH_BOOT_STAGE_SYS_CLOCK_OSC,
    APP_WATCH_BOOT_STAGE_SYS_CLOCK_BUS,
    APP_WATCH_BOOT_STAGE_GPIO_INIT,
    APP_WATCH_BOOT_STAGE_DMA_INIT,
    APP_WATCH_BOOT_STAGE_CAN1_INIT,
    APP_WATCH_BOOT_STAGE_CAN2_INIT,
    APP_WATCH_BOOT_STAGE_USART1_INIT,
    APP_WATCH_BOOT_STAGE_USART3_INIT,
    APP_WATCH_BOOT_STAGE_USART6_INIT,
    APP_WATCH_BOOT_STAGE_UART7_INIT,
    APP_WATCH_BOOT_STAGE_UART8_INIT,
    APP_WATCH_BOOT_STAGE_SPI5_INIT,
    APP_WATCH_BOOT_STAGE_SDIO_INIT,
    APP_WATCH_BOOT_STAGE_SDIO_CARD_INIT,
    APP_WATCH_BOOT_STAGE_SDIO_WIDE_BUS,
    APP_WATCH_BOOT_STAGE_SDIO_DMA_RX_INIT,
    APP_WATCH_BOOT_STAGE_SDIO_DMA_TX_INIT,
    APP_WATCH_BOOT_STAGE_TIM3_INIT,
    APP_WATCH_BOOT_STAGE_TIM12_INIT,
    APP_WATCH_BOOT_STAGE_CAN_FILTER_INIT,
    APP_WATCH_BOOT_STAGE_BUZZER_INIT,
    APP_WATCH_BOOT_STAGE_REMOTE_CONTROL_INIT,
    APP_WATCH_BOOT_STAGE_FREERTOS_INIT,
    APP_WATCH_BOOT_STAGE_SCHEDULER_START,
    APP_WATCH_BOOT_STAGE_DEFAULT_TASK_START,
    APP_WATCH_BOOT_STAGE_USB_DEVICE_USBD_INIT,
    APP_WATCH_BOOT_STAGE_USB_DEVICE_REGISTER_CLASS,
    APP_WATCH_BOOT_STAGE_USB_DEVICE_REGISTER_IF,
    APP_WATCH_BOOT_STAGE_USB_DEVICE_START,
    APP_WATCH_BOOT_STAGE_RUN,
} app_watch_boot_stage_e;

typedef enum
{
    APP_WATCH_TASK_DEFAULT = 0,
    APP_WATCH_TASK_DETECT,
    APP_WATCH_TASK_IMU,
    APP_WATCH_TASK_GIMBAL,
    APP_WATCH_TASK_CHASSIS,
    APP_WATCH_TASK_CAN_RX,
    APP_WATCH_TASK_CAN_TX,
    APP_WATCH_TASK_RC_SBUS,
    APP_WATCH_TASK_USB,
    APP_WATCH_TASK_UART1_ELRS,
} app_watch_task_id_e;

typedef enum
{
    APP_WATCH_IRQ_IST8310_EXTI = 0,
    APP_WATCH_IRQ_IMU_EXTI,
    APP_WATCH_IRQ_SD_EXTI,
    APP_WATCH_IRQ_CAN1_RX0,
    APP_WATCH_IRQ_CAN2_RX0,
    APP_WATCH_IRQ_USART1,
    APP_WATCH_IRQ_UART7,
    APP_WATCH_IRQ_UART8,
    APP_WATCH_IRQ_OTG_FS,
    APP_WATCH_IRQ_TIM6_DAC,
    APP_WATCH_IRQ_DMA_USART1_RX,
    APP_WATCH_IRQ_DMA_SPI5_TX,
    APP_WATCH_IRQ_DMA_SPI5_RX,
    APP_WATCH_IRQ_DMA_SDIO_TX,
} app_watch_irq_id_e;

typedef struct
{
    uint32_t beat_count;
    uint32_t last_tick_ms;
    uint32_t max_gap_ms;
    uint32_t wait_count;
    uint32_t timeout_count;
    uint32_t error_count;
} app_watch_task_diag_entry_t;

typedef struct
{
    app_watch_task_diag_entry_t default_task;
    app_watch_task_diag_entry_t detect_task;
    app_watch_task_diag_entry_t imu_task;
    app_watch_task_diag_entry_t gimbal_task;
    app_watch_task_diag_entry_t chassis_task;
    app_watch_task_diag_entry_t can_rx_task;
    app_watch_task_diag_entry_t can_tx_task;
    app_watch_task_diag_entry_t rc_sbus_task;
    app_watch_task_diag_entry_t usb_task;
    app_watch_task_diag_entry_t uart1_elrs_task;
} app_watch_task_diag_t;

typedef struct
{
    uint32_t hit_count;
    uint32_t last_tick_ms;
} app_watch_irq_diag_entry_t;

typedef struct
{
    app_watch_irq_diag_entry_t ist8310_exti;
    app_watch_irq_diag_entry_t imu_exti;
    app_watch_irq_diag_entry_t sd_exti;
    app_watch_irq_diag_entry_t can1_rx0;
    app_watch_irq_diag_entry_t can2_rx0;
    app_watch_irq_diag_entry_t usart1;
    app_watch_irq_diag_entry_t uart7;
    app_watch_irq_diag_entry_t uart8;
    app_watch_irq_diag_entry_t otg_fs;
    app_watch_irq_diag_entry_t tim6_dac;
    app_watch_irq_diag_entry_t dma_usart1_rx;
    app_watch_irq_diag_entry_t dma_spi5_tx;
    app_watch_irq_diag_entry_t dma_spi5_rx;
    app_watch_irq_diag_entry_t dma_sdio_tx;
} app_watch_irq_diag_t;

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
} app_watch_rc_t;

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
} app_watch_newrc_t;

typedef struct
{
    app_watch_chassis_mode_e mode;
    app_watch_chassis_mode_e last_mode;
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
} app_watch_chassis_t;

typedef struct
{
    app_watch_gimbal_motor_mode_e yaw_mode;
    app_watch_gimbal_motor_mode_e pitch_mode;
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
} app_watch_gimbal_t;

typedef struct
{
    app_watch_shoot_mode_e mode;
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
} app_watch_shoot_t;

typedef struct
{
    fp32 quat[4];      // wxyz
    fp32 gyro_dps[3];  // deg/s
    fp32 accel[3];     // m/s^2
    fp32 angle_deg[3]; // INS 原始欧拉角 yaw/roll/pitch in deg (see INS_task.h offsets); 未应用 PITCH_TURN/YAW_TURN
} app_watch_imu_t;

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
    app_watch_task_diag_t task;
    app_watch_irq_diag_t irq;
    app_watch_boot_stage_e boot_stage;
    app_watch_boot_stage_e boot_trace[4]; // [0] 最新, [1] 上一次...
    uint32_t error_handler_count;
    app_watch_boot_stage_e error_stage;
    uint32_t error_tick_ms;
    uint32_t error_ipsr;
    uint32_t error_arg0;
    uint32_t error_arg1;
} app_watch_diag_t;

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
} app_watch_rtos_t;

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
} app_watch_fault_t;

typedef struct
{
    app_watch_rc_t rc;
    app_watch_newrc_t newrc;
    app_watch_chassis_t chassis;
    app_watch_gimbal_t gimbal;
    app_watch_shoot_t shoot;
    app_watch_imu_t imu;
    app_watch_diag_t diag;
    app_watch_rtos_t rtos;
    app_watch_fault_t fault;
} app_watch_t;

extern app_watch_t g_app_watch;
void app_watch_init(void);
void app_watch_update(void);
void app_watch_diag_set_boot_stage(app_watch_boot_stage_e stage);
void app_watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr);
void app_watch_diag_set_error_args(uint32_t arg0, uint32_t arg1);
void app_watch_task_beat(app_watch_task_id_e task_id);
void app_watch_task_wait(app_watch_task_id_e task_id);
void app_watch_task_timeout(app_watch_task_id_e task_id);
void app_watch_task_error(app_watch_task_id_e task_id);
void app_watch_irq_hit(app_watch_irq_id_e irq_id);
