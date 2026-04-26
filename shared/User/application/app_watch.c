/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "app_watch.h"

#include <string.h>

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "mem_mang.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "sdcard.h"
#include "sdlog.h"
#include "shoot.h"
#include "usb_task.h"

app_watch_t g_app_watch;

static const RC_ctrl_t *rc_src;
static const fp32 *ins_quat_src;
static const fp32 *ins_angle_src;
static const fp32 *ins_gyro_src;
static const fp32 *ins_accel_src;

static void app_watch_copy_rc(void);
static void app_watch_copy_newrc(void);
static void app_watch_copy_imu(void);
static void app_watch_copy_chassis(void);
static void app_watch_copy_gimbal(void);
static void app_watch_copy_shoot(void);
static void app_watch_copy_diag(void);
static void app_watch_copy_rtos(void);
static void app_watch_diag_push_stage(app_watch_boot_stage_e stage);
static app_watch_task_diag_entry_t *app_watch_task_diag_get(app_watch_task_id_e task_id);
static app_watch_irq_diag_entry_t *app_watch_irq_diag_get(app_watch_irq_id_e irq_id);

static app_watch_task_diag_entry_t *app_watch_task_diag_get(app_watch_task_id_e task_id)
{
    switch (task_id)
    {
    case APP_WATCH_TASK_DEFAULT:
        return &g_app_watch.diag.task.default_task;
    case APP_WATCH_TASK_DETECT:
        return &g_app_watch.diag.task.detect_task;
    case APP_WATCH_TASK_IMU:
        return &g_app_watch.diag.task.imu_task;
    case APP_WATCH_TASK_GIMBAL:
        return &g_app_watch.diag.task.gimbal_task;
    case APP_WATCH_TASK_CHASSIS:
        return &g_app_watch.diag.task.chassis_task;
    case APP_WATCH_TASK_CAN_RX:
        return &g_app_watch.diag.task.can_rx_task;
    case APP_WATCH_TASK_CAN_TX:
        return &g_app_watch.diag.task.can_tx_task;
    case APP_WATCH_TASK_RC_SBUS:
        return &g_app_watch.diag.task.rc_sbus_task;
    case APP_WATCH_TASK_USB:
        return &g_app_watch.diag.task.usb_task;
    case APP_WATCH_TASK_UART1_ELRS:
        return &g_app_watch.diag.task.uart1_elrs_task;
    default:
        return NULL;
    }
}

static app_watch_irq_diag_entry_t *app_watch_irq_diag_get(app_watch_irq_id_e irq_id)
{
    switch (irq_id)
    {
    case APP_WATCH_IRQ_IST8310_EXTI:
        return &g_app_watch.diag.irq.ist8310_exti;
    case APP_WATCH_IRQ_IMU_EXTI:
        return &g_app_watch.diag.irq.imu_exti;
    case APP_WATCH_IRQ_SD_EXTI:
        return &g_app_watch.diag.irq.sd_exti;
    case APP_WATCH_IRQ_CAN1_RX0:
        return &g_app_watch.diag.irq.can1_rx0;
    case APP_WATCH_IRQ_CAN2_RX0:
        return &g_app_watch.diag.irq.can2_rx0;
    case APP_WATCH_IRQ_USART1:
        return &g_app_watch.diag.irq.usart1;
    case APP_WATCH_IRQ_UART7:
        return &g_app_watch.diag.irq.uart7;
    case APP_WATCH_IRQ_UART8:
        return &g_app_watch.diag.irq.uart8;
    case APP_WATCH_IRQ_OTG_FS:
        return &g_app_watch.diag.irq.otg_fs;
    case APP_WATCH_IRQ_TIM6_DAC:
        return &g_app_watch.diag.irq.tim6_dac;
    case APP_WATCH_IRQ_DMA_USART1_RX:
        return &g_app_watch.diag.irq.dma_usart1_rx;
    case APP_WATCH_IRQ_DMA_SPI5_TX:
        return &g_app_watch.diag.irq.dma_spi5_tx;
    case APP_WATCH_IRQ_DMA_SPI5_RX:
        return &g_app_watch.diag.irq.dma_spi5_rx;
    case APP_WATCH_IRQ_DMA_SDIO_TX:
        return &g_app_watch.diag.irq.dma_sdio_tx;
    default:
        return NULL;
    }
}

void app_watch_diag_set_boot_stage(app_watch_boot_stage_e stage)
{
    app_watch_diag_push_stage(stage);
}

void app_watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr)
{
    g_app_watch.diag.error_handler_count++;
    g_app_watch.diag.error_stage = g_app_watch.diag.boot_stage;
    g_app_watch.diag.error_tick_ms = tick_ms;
    g_app_watch.diag.error_ipsr = ipsr;
}

void app_watch_diag_set_error_args(uint32_t arg0, uint32_t arg1)
{
    g_app_watch.diag.error_arg0 = arg0;
    g_app_watch.diag.error_arg1 = arg1;
}

void app_watch_task_beat(app_watch_task_id_e task_id)
{
    const uint32_t now_ms = HAL_GetTick();
    app_watch_task_diag_entry_t *entry = app_watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (entry->last_tick_ms != 0u)
    {
        const uint32_t gap_ms = now_ms - entry->last_tick_ms;
        if (gap_ms > entry->max_gap_ms)
        {
            entry->max_gap_ms = gap_ms;
        }
    }
    entry->beat_count++;
    entry->last_tick_ms = now_ms;
    taskEXIT_CRITICAL();
}

void app_watch_task_wait(app_watch_task_id_e task_id)
{
    app_watch_task_diag_entry_t *entry = app_watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->wait_count++;
    taskEXIT_CRITICAL();
}

void app_watch_task_timeout(app_watch_task_id_e task_id)
{
    app_watch_task_diag_entry_t *entry = app_watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->timeout_count++;
    taskEXIT_CRITICAL();
}

void app_watch_task_error(app_watch_task_id_e task_id)
{
    app_watch_task_diag_entry_t *entry = app_watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->error_count++;
    taskEXIT_CRITICAL();
}

void app_watch_irq_hit(app_watch_irq_id_e irq_id)
{
    const uint32_t now_ms = HAL_GetTick();
    app_watch_irq_diag_entry_t *entry = app_watch_irq_diag_get(irq_id);
    if (entry == NULL)
    {
        return;
    }

    {
        UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
        entry->hit_count++;
        entry->last_tick_ms = now_ms;
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

void app_watch_init(void)
{
    memset(&g_app_watch, 0, sizeof(g_app_watch));

    rc_src = get_remote_control_point();
    ins_quat_src = get_INS_quat_point();
    ins_angle_src = get_INS_angle_point();
    ins_gyro_src = get_gyro_data_point();
    ins_accel_src = get_accel_data_point();

    app_watch_update();
}

void app_watch_update(void)
{
    app_watch_copy_rc();
    app_watch_copy_newrc();
    app_watch_copy_imu();
    app_watch_copy_chassis();
    app_watch_copy_gimbal();
    app_watch_copy_shoot();
    app_watch_copy_diag();
    app_watch_copy_rtos();
}

static void app_watch_diag_push_stage(app_watch_boot_stage_e stage)
{
    if (stage == APP_WATCH_BOOT_STAGE_NONE)
    {
        g_app_watch.diag.boot_stage = stage;
        return;
    }

    if (g_app_watch.diag.boot_stage == stage)
    {
        return;
    }

    for (uint32_t i = (uint32_t)(sizeof(g_app_watch.diag.boot_trace) / sizeof(g_app_watch.diag.boot_trace[0])) - 1u; i > 0u; i--)
    {
        g_app_watch.diag.boot_trace[i] = g_app_watch.diag.boot_trace[i - 1u];
    }
    g_app_watch.diag.boot_trace[0] = stage;
    g_app_watch.diag.boot_stage = stage;
}

static void app_watch_copy_rc(void)
{
    if (rc_src == NULL)
    {
        memset(&g_app_watch.rc, 0, sizeof(g_app_watch.rc));
        return;
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        g_app_watch.rc.ch[i] = rc_src->rc.ch[i];
    }
    g_app_watch.rc.s[0] = rc_src->rc.s[0];
    g_app_watch.rc.s[1] = rc_src->rc.s[1];

    g_app_watch.rc.mouse_x = rc_src->mouse.x;
    g_app_watch.rc.mouse_y = rc_src->mouse.y;
    g_app_watch.rc.mouse_z = rc_src->mouse.z;
    g_app_watch.rc.mouse_l = rc_src->mouse.press_l;
    g_app_watch.rc.mouse_r = rc_src->mouse.press_r;
    g_app_watch.rc.key = rc_src->key.v;
}

static void app_watch_copy_newrc(void)
{
    uart1_image_remote_state_t state = {0};

    if (!uart1_image_remote_get_state(&state))
    {
        memset(&g_app_watch.newrc, 0, sizeof(g_app_watch.newrc));
        return;
    }

    g_app_watch.newrc.valid = state.valid;
    g_app_watch.newrc.proto = state.proto;
    g_app_watch.newrc.range_mode = state.range_mode;
    for (uint8_t i = 0; i < 5; i++)
    {
        g_app_watch.newrc.raw_ch[i] = state.raw_ch[i];
    }
    for (uint8_t i = 0; i < 5; i++)
    {
        g_app_watch.newrc.ch[i] = state.ch[i];
    }
    g_app_watch.newrc.s[0] = state.s[0];
    g_app_watch.newrc.s[1] = state.s[1];
    g_app_watch.newrc.mouse_x = state.mouse_x;
    g_app_watch.newrc.mouse_y = state.mouse_y;
    g_app_watch.newrc.mouse_z = state.mouse_z;
    g_app_watch.newrc.mouse_l = state.mouse_l;
    g_app_watch.newrc.mouse_r = state.mouse_r;
    g_app_watch.newrc.mouse_mid = state.mouse_mid;
    g_app_watch.newrc.pause = state.pause;
    g_app_watch.newrc.btn_l = state.btn_l;
    g_app_watch.newrc.btn_r = state.btn_r;
    g_app_watch.newrc.trigger = state.trigger;
    g_app_watch.newrc.dial = state.dial;
    g_app_watch.newrc.key_value = state.key_value;
    g_app_watch.newrc.key_w = state.key_w;
    g_app_watch.newrc.key_s = state.key_s;
    g_app_watch.newrc.key_a = state.key_a;
    g_app_watch.newrc.key_d = state.key_d;
    g_app_watch.newrc.key_shift = state.key_shift;
    g_app_watch.newrc.key_ctrl = state.key_ctrl;
    g_app_watch.newrc.key_q = state.key_q;
    g_app_watch.newrc.key_e = state.key_e;
    g_app_watch.newrc.key_r = state.key_r;
    g_app_watch.newrc.key_f = state.key_f;
    g_app_watch.newrc.key_g = state.key_g;
    g_app_watch.newrc.key_z = state.key_z;
    g_app_watch.newrc.key_x = state.key_x;
    g_app_watch.newrc.key_c = state.key_c;
    g_app_watch.newrc.key_v = state.key_v;
    g_app_watch.newrc.key_b = state.key_b;
    g_app_watch.newrc.last_rx_tick_ms = state.last_rx_tick_ms;
}

static void app_watch_copy_imu(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (ins_quat_src != NULL)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            g_app_watch.imu.quat[i] = ins_quat_src[i];
        }
    }

    if (ins_angle_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_app_watch.imu.angle_deg[i] = ins_angle_src[i] * rad2deg;
        }
    }

    if (ins_gyro_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_app_watch.imu.gyro_dps[i] = ins_gyro_src[i] * rad2deg;
        }
    }

    if (ins_accel_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_app_watch.imu.accel[i] = ins_accel_src[i];
        }
    }
}

static void app_watch_copy_chassis(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    const chassis_move_t *chassis = get_chassis_move_point();
    if (chassis == NULL)
    {
        memset(&g_app_watch.chassis, 0, sizeof(g_app_watch.chassis));
        return;
    }

    g_app_watch.chassis.mode = (app_watch_chassis_mode_e)chassis->chassis_mode;
    g_app_watch.chassis.last_mode = (app_watch_chassis_mode_e)chassis->last_chassis_mode;

    g_app_watch.chassis.vx_set = chassis->vx_set;
    g_app_watch.chassis.vy_set = chassis->vy_set;
    g_app_watch.chassis.wz_set = chassis->wz_set;
    g_app_watch.chassis.vx = chassis->vx;
    g_app_watch.chassis.vy = chassis->vy;
    g_app_watch.chassis.wz = chassis->wz;
    g_app_watch.chassis.yaw_deg = chassis->chassis_yaw * rad2deg;

    for (uint8_t i = 0; i < 4; i++)
    {
        const chassis_motor_t *m = &chassis->motor_chassis[i];
        const motor_measure_t *mm = m->chassis_motor_measure;
        g_app_watch.chassis.motor_rpm[i] = mm ? mm->speed_rpm : 0;
        g_app_watch.chassis.motor_current[i] = m->give_current;
        g_app_watch.chassis.motor_speed_set[i] = m->speed_set;
        g_app_watch.chassis.motor_ecd[i] = mm ? mm->ecd : 0;
        g_app_watch.chassis.motor_temp[i] = mm ? mm->temperate : 0;
    }
}

static void app_watch_copy_gimbal(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    const gimbal_motor_t *yaw = get_yaw_motor_point();
    const gimbal_motor_t *pitch = get_pitch_motor_point();

    if (yaw)
    {
        g_app_watch.gimbal.yaw_mode = (app_watch_gimbal_motor_mode_e)yaw->gimbal_motor_mode;
        g_app_watch.gimbal.yaw_angle_deg = yaw->angle * rad2deg;
        g_app_watch.gimbal.yaw_set_deg = yaw->angle_set * rad2deg;
        g_app_watch.gimbal.yaw_gyro_dps = yaw->motor_gyro * rad2deg;
        g_app_watch.gimbal.yaw_current = yaw->given_current;

        const motor_measure_t *m = yaw->gimbal_motor_measure;
        g_app_watch.gimbal.yaw_rpm = m ? m->speed_rpm : 0;
        g_app_watch.gimbal.yaw_current_fb = m ? m->given_current : 0;
        g_app_watch.gimbal.yaw_ecd = m ? m->ecd : 0;
        g_app_watch.gimbal.yaw_temp = m ? m->temperate : 0;
    }
    else
    {
        g_app_watch.gimbal.yaw_mode = APP_WATCH_GIMBAL_MOTOR_RAW;
        g_app_watch.gimbal.yaw_angle_deg = 0.0f;
        g_app_watch.gimbal.yaw_set_deg = 0.0f;
        g_app_watch.gimbal.yaw_gyro_dps = 0.0f;
        g_app_watch.gimbal.yaw_current = 0;
        g_app_watch.gimbal.yaw_rpm = 0;
        g_app_watch.gimbal.yaw_current_fb = 0;
        g_app_watch.gimbal.yaw_ecd = 0;
        g_app_watch.gimbal.yaw_temp = 0;
    }

    if (pitch)
    {
        g_app_watch.gimbal.pitch_mode = (app_watch_gimbal_motor_mode_e)pitch->gimbal_motor_mode;
        g_app_watch.gimbal.pitch_angle_deg = pitch->angle * rad2deg;
        g_app_watch.gimbal.pitch_set_deg = pitch->angle_set * rad2deg;
        g_app_watch.gimbal.pitch_gyro_dps = pitch->motor_gyro * rad2deg;
        g_app_watch.gimbal.pitch_current = pitch->given_current;

        const motor_measure_t *m = pitch->gimbal_motor_measure;
        g_app_watch.gimbal.pitch_rpm = m ? m->speed_rpm : 0;
        g_app_watch.gimbal.pitch_current_fb = m ? m->given_current : 0;
        g_app_watch.gimbal.pitch_ecd = m ? m->ecd : 0;
        g_app_watch.gimbal.pitch_temp = m ? m->temperate : 0;
    }
    else
    {
        g_app_watch.gimbal.pitch_mode = APP_WATCH_GIMBAL_MOTOR_RAW;
        g_app_watch.gimbal.pitch_angle_deg = 0.0f;
        g_app_watch.gimbal.pitch_set_deg = 0.0f;
        g_app_watch.gimbal.pitch_gyro_dps = 0.0f;
        g_app_watch.gimbal.pitch_current = 0;
        g_app_watch.gimbal.pitch_rpm = 0;
        g_app_watch.gimbal.pitch_current_fb = 0;
        g_app_watch.gimbal.pitch_ecd = 0;
        g_app_watch.gimbal.pitch_temp = 0;
    }
}

static void app_watch_copy_shoot(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    const shoot_control_t *shoot = get_shoot_control_point();
    if (shoot == NULL)
    {
        memset(&g_app_watch.shoot, 0, sizeof(g_app_watch.shoot));
        return;
    }

    g_app_watch.shoot.mode = (app_watch_shoot_mode_e)shoot->shoot_mode;
    g_app_watch.shoot.fric_speed_set_rpm = (int16_t)shoot->fric_speed_set;
    for (uint8_t i = 0; i < 4; i++)
    {
        g_app_watch.shoot.fric_current_cmd[i] = actuator_cmd_get_friction_current_can2(i);
    }

    g_app_watch.shoot.trigger_angle_deg = shoot->angle * rad2deg;
    g_app_watch.shoot.trigger_set_deg = shoot->set_angle * rad2deg;
    g_app_watch.shoot.trigger_speed = shoot->speed;
    g_app_watch.shoot.trigger_speed_set = shoot->speed_set;
    g_app_watch.shoot.trigger_current = shoot->given_current;

    const motor_measure_t *trigger_meas = get_trigger_motor_measure_point();
    if (trigger_meas)
    {
        g_app_watch.shoot.trigger_rpm = trigger_meas->speed_rpm;
        g_app_watch.shoot.trigger_ecd = trigger_meas->ecd;
        g_app_watch.shoot.trigger_temp = trigger_meas->temperate;
    }
    else
    {
        g_app_watch.shoot.trigger_rpm = 0;
        g_app_watch.shoot.trigger_ecd = 0;
        g_app_watch.shoot.trigger_temp = 0;
    }

    g_app_watch.shoot.heat_limit = shoot->heat_limit;
    g_app_watch.shoot.heat = shoot->heat;

    for (uint8_t i = 0; i < 4; i++)
    {
        const motor_measure_t *fm = get_friction_motor_measure_point(i);
        if (fm)
        {
            g_app_watch.shoot.fric_rpm[i] = fm->speed_rpm;
            g_app_watch.shoot.fric_current[i] = fm->given_current;
            g_app_watch.shoot.fric_temp[i] = fm->temperate;
        }
        else
        {
            g_app_watch.shoot.fric_rpm[i] = 0;
            g_app_watch.shoot.fric_current[i] = 0;
            g_app_watch.shoot.fric_temp[i] = 0;
        }
    }
}

static void app_watch_copy_diag(void)
{
    sdlog_stats_t sd_stats = {0};
    sdlog_image_link_stats_t image_stats = {0};

    g_app_watch.diag.can1_0x200[0] = actuator_cmd_get_chassis_current_can1(0);        // 0x201
    g_app_watch.diag.can1_0x200[1] = actuator_cmd_get_chassis_current_can1(1);        // 0x202
    g_app_watch.diag.can1_0x200[2] = actuator_cmd_get_pitch_current_can1();           // 0x203
    g_app_watch.diag.can1_0x200[3] = actuator_cmd_get_trigger_current_can1();         // 0x204

    g_app_watch.diag.can1_0x1ff[0] = actuator_cmd_get_chassis_current_can1(3);        // 0x205
    g_app_watch.diag.can1_0x1ff[1] = actuator_cmd_get_yaw_current_can1();             // 0x206
    g_app_watch.diag.can1_0x1ff[2] = actuator_cmd_get_chassis_current_can1(2);        // 0x207
    g_app_watch.diag.can1_0x1ff[3] = 0;                                               // 0x208 reserved

    g_app_watch.diag.can1_1ff_status = (int16_t)CAN_get_last_1ff_status();
    g_app_watch.diag.can1_err = CAN_get_last_can1_error();
    g_app_watch.diag.can2_err = CAN_get_last_can2_error();
    g_app_watch.diag.can1_tx_status = bsp_can_get_last_tx_status(1u);
    g_app_watch.diag.can2_tx_status = bsp_can_get_last_tx_status(2u);
    g_app_watch.diag.manual_active_source = remote_control_get_active_source();
    g_app_watch.diag.sd_mounted = (uint8_t)sdcard_is_mounted();
    sdlog_get_stats(&sd_stats);
    g_app_watch.diag.sdlog_active = sd_stats.active;
    g_app_watch.diag.reserved0[0] = 0u;
    g_app_watch.diag.reserved0[1] = 0u;
    g_app_watch.diag.reserved0[2] = 0u;
    g_app_watch.diag.can1_rx_drop = CAN_get_can1_rx_drop_count();
    g_app_watch.diag.can2_rx_drop = CAN_get_can2_rx_drop_count();
    g_app_watch.diag.can1_tx_count = CAN_get_can1_tx_count();
    g_app_watch.diag.can2_tx_count = CAN_get_can2_tx_count();
    g_app_watch.diag.can1_tx_fail = CAN_get_can1_tx_fail_count();
    g_app_watch.diag.can2_tx_fail = CAN_get_can2_tx_fail_count();
    g_app_watch.diag.manual_sbus_frame_count = remote_control_get_sbus_frame_count();
    g_app_watch.diag.manual_set_source_count = remote_control_get_set_source_count();
    g_app_watch.diag.sdlog_dropped = sd_stats.dropped;
    g_app_watch.diag.sdlog_ring_used = sd_stats.ring_used;
    g_app_watch.diag.sdlog_ring_free = sd_stats.ring_free;
    g_app_watch.diag.sdlog_bytes_flushed = sd_stats.bytes_flushed;
    g_app_watch.diag.sdlog_last_sync_ms = sd_stats.last_sync_ms;
    g_app_watch.diag.sdlog_last_error = sd_stats.last_error;
    uart1_image_link_get_stats(&image_stats);
    g_app_watch.diag.image_last_rx_tick_ms = image_stats.last_rx_tick_ms;
    g_app_watch.diag.image_frame_count = image_stats.frame_count;
    g_app_watch.diag.image_controller_frame_count = image_stats.controller_frame_count;
    g_app_watch.diag.image_client_frame_count = image_stats.client_frame_count;
    g_app_watch.diag.image_vt13_frame_count = image_stats.vt13_frame_count;
    g_app_watch.diag.image_crc_error_count = image_stats.crc_error_count;
    g_app_watch.diag.image_parse_error_count = image_stats.parse_error_count;
    g_app_watch.diag.image_restart_count = image_stats.restart_count;
    g_app_watch.diag.image_last_cmd_id = image_stats.last_cmd_id;
    g_app_watch.diag.image_port_active = image_stats.port_active;
    g_app_watch.diag.image_last_range_mode = image_stats.last_range_mode;

    memset(g_app_watch.diag.flags, 0, sizeof(g_app_watch.diag.flags));
    g_app_watch.diag.flags[0] = (uint8_t)toe_is_error(DBUS_TOE);
    g_app_watch.diag.flags[1] = (uint8_t)(gimbal_behaviour_watch == GIMBAL_ZERO_FORCE);
    g_app_watch.diag.flags[2] = (uint8_t)toe_is_error(YAW_GIMBAL_MOTOR_TOE);
    g_app_watch.diag.flags[3] = (uint8_t)toe_is_error(PITCH_GIMBAL_MOTOR_TOE);
    g_app_watch.diag.flags[4] = (uint8_t)toe_is_error(TRIGGER_MOTOR_TOE);
    g_app_watch.diag.flags[5] = g_app_watch.diag.flags[0];
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared app_watch can link even if a target doesn't export them.
__weak uint32_t gimbal_high_water;
__weak uint32_t chassis_high_water;
__weak uint32_t detect_task_stack;
__weak uint32_t calibrate_task_stack;
#endif

static void app_watch_copy_rtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_app_watch.rtos.heap_free = heap_get_free();
    g_app_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_app_watch.diag.watch_update_count++;
    g_app_watch.diag.watch_update_tick_ms = HAL_GetTick();
    g_app_watch.diag.scheduler_state = (uint32_t)xTaskGetSchedulerState();
    g_app_watch.diag.task_count = (uint32_t)uxTaskGetNumberOfTasks();
    current_task = xTaskGetCurrentTaskHandle();
    g_app_watch.diag.current_task_handle = (uint32_t)current_task;
    current_name = pcTaskGetTaskName(NULL);
    memset(g_app_watch.diag.current_task_name, 0, sizeof(g_app_watch.diag.current_task_name));
    if (current_name != NULL)
    {
        (void)strncpy(g_app_watch.diag.current_task_name, current_name, sizeof(g_app_watch.diag.current_task_name) - 1u);
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
    g_app_watch.rtos.stack_gimbal = gimbal_high_water;
    g_app_watch.rtos.stack_chassis = chassis_high_water;
    g_app_watch.rtos.stack_detect = detect_task_stack;
    g_app_watch.rtos.stack_calibrate = calibrate_task_stack;
#else
    g_app_watch.rtos.stack_gimbal = 0u;
    g_app_watch.rtos.stack_chassis = 0u;
    g_app_watch.rtos.stack_detect = 0u;
    g_app_watch.rtos.stack_calibrate = 0u;
#endif
}
