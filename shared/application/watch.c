/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "watch.h"

#include <string.h>

#include "config.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "arm_task.h"
#include "battery_monitor_task.h"
#include "bsp_adc.h"
#include "chassis_state.h"
#include "control_input.h"
#include "gimbal_state.h"
#include "INS_task.h"
#include "bmi088driver.h"
#include "detect_task.h"
#include "mem_mang.h"
#include "manual_input.h"
#include "bsp_can.h"
#include "sdcard.h"
#include "sdlog.h"
#include "shoot_state.h"
#include "host_link_task.h"
#include "robot_task_profile.h"
#include "rt_profiler.h"
#include "wheelleg_mit_task.h"
#include "wheelleg_msg.h"

watch_t g_watch;

// Some targets do not compile arm_task.c at all. Keep watch linkable there and
// let real arm_task.c override this fallback when the Unitree executor is used.
__weak const arm_j0_unitree_state_t *arm_j0_unitree_get_state(void)
{
    return NULL;
}

__weak uint8_t wheelleg_mit_get_foot_test_phase(void)
{
    return 0u;
}

__weak uint8_t wheelleg_mit_get_foot_test_ik_ok(void)
{
    return 0u;
}

__weak void wheelleg_mit_get_foot_test_target(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    (void)side;
    if (x_m != NULL)
    {
        *x_m = 0.0f;
    }
    if (y_m != NULL)
    {
        *y_m = 0.0f;
    }
    if (length_m != NULL)
    {
        *length_m = 0.0f;
    }
}

__weak void wheelleg_mit_get_foot_test_wheel(uint8_t side,
                                             uint8_t *zero_valid,
                                             fp32 *zero_rad,
                                             fp32 *dx_m,
                                             fp32 *comp_rad,
                                             fp32 *target_rad)
{
    (void)side;
    if (zero_valid != NULL)
    {
        *zero_valid = 0u;
    }
    if (zero_rad != NULL)
    {
        *zero_rad = 0.0f;
    }
    if (dx_m != NULL)
    {
        *dx_m = 0.0f;
    }
    if (comp_rad != NULL)
    {
        *comp_rad = 0.0f;
    }
    if (target_rad != NULL)
    {
        *target_rad = 0.0f;
    }
}

__weak fp32 ins_get_imu_temperature_c(void)
{
    return 0.0f;
}

__weak uint16_t ins_get_imu_heater_pwm(void)
{
    return 0u;
}

__weak uint8_t ins_get_imu_heater_mode(void)
{
    return 0u;
}

__weak fp32 ins_get_imu_heater_pid_out(void)
{
    return 0.0f;
}

__weak void BMI088_get_diag(bmi088_diag_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->init_last_error = 0xEEu;
        out->init_fail_reg = 0xEEu;
        out->init_fail_expect = 0xEEu;
        out->init_fail_actual = 0xEEu;
        out->accel_chip_id = 0xEEu;
        out->gyro_chip_id = 0xEEu;
        out->gyro_read_chip_id = 0xEEu;
    }
}

__weak fp32 get_battery_voltage_cached(void)
{
    return 0.0f;
}

__weak fp32 get_battery_percentage_fp32(void)
{
    return 0.0f;
}

__weak uint8_t battery_monitor_is_low_alarm(void)
{
    return 0u;
}

__weak uint8_t bsp_adc_is_started(void)
{
    return 0u;
}

__weak uint16_t bsp_adc_get_raw(uint8_t index)
{
    (void)index;
    return 0u;
}

__weak fp32 bsp_adc_get_channel_voltage(uint8_t index)
{
    (void)index;
    return 0.0f;
}

__weak uint32_t bsp_adc_get_start_ok_count(void)
{
    return 0u;
}

__weak uint32_t bsp_adc_get_start_fail_count(void)
{
    return 0u;
}

static const manual_input_state_t *rc_src;
static const fp32 *ins_quat_src;
static const fp32 *ins_angle_src;
static const fp32 *ins_gyro_src;
static const fp32 *ins_accel_src;

static void watch_copy_rc(void);
static void watch_copy_newrc(void);
static void watch_copy_imu(void);
static void watch_copy_chassis(void);
static void watch_copy_gimbal(void);
static void watch_copy_shoot(void);
static void watch_copy_arm_j0_unitree(void);
static void watch_copy_wheelleg_mit(void);
static void watch_copy_diag(void);
static void watch_copy_rtos(void);
static void watch_diag_push_stage(watch_boot_stage_e stage);
static watch_task_diag_entry_t *watch_task_diag_get(watch_task_id_e task_id);
static watch_irq_diag_entry_t *watch_irq_diag_get(watch_irq_id_e irq_id);
static uint8_t watch_block_active_always(void);
static uint8_t watch_block_active_locomotion_classic(void);
static uint8_t watch_block_active_wheelleg_servo(void);
static uint8_t watch_block_active_wheelleg_mit(void);
static uint8_t watch_block_active_gimbal_single(void);
static uint8_t watch_block_active_gimbal_dual(void);
static uint8_t watch_block_active_arm(void);

static const watch_block_desc_t g_watch_blocks[] = {
    {WATCH_BLOCK_RC, "input.rc", &g_watch.rc, sizeof(g_watch.rc), watch_block_active_always},
    {WATCH_BLOCK_NEWRC, "input.newrc", &g_watch.newrc, sizeof(g_watch.newrc), watch_block_active_always},
    {WATCH_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", &g_watch.chassis, sizeof(g_watch.chassis), watch_block_active_locomotion_classic},
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.wheelleg_servo", &g_watch.wheelleg_servo, sizeof(g_watch.wheelleg_servo), watch_block_active_wheelleg_servo},
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.wheelleg_mit", &g_watch.wheelleg_mit, sizeof(g_watch.wheelleg_mit), watch_block_active_wheelleg_mit},
    {WATCH_BLOCK_GIMBAL_SINGLE, "gimbal.single", &g_watch.gimbal, sizeof(g_watch.gimbal), watch_block_active_gimbal_single},
    {WATCH_BLOCK_GIMBAL_DUAL, "gimbal.dual", &g_watch.dual_gimbal, sizeof(g_watch.dual_gimbal), watch_block_active_gimbal_dual},
    {WATCH_BLOCK_SHOOT_RM, "shoot.rm", &g_watch.shoot, sizeof(g_watch.shoot), watch_block_active_always},
    {WATCH_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", &g_watch.arm_j0_unitree, sizeof(g_watch.arm_j0_unitree), watch_block_active_arm},
    {WATCH_BLOCK_IMU, "common.imu", &g_watch.imu, sizeof(g_watch.imu), watch_block_active_always},
    {WATCH_BLOCK_DIAG, "common.diag", &g_watch.diag, sizeof(g_watch.diag), watch_block_active_always},
    {WATCH_BLOCK_RTOS, "common.rtos", &g_watch.rtos, sizeof(g_watch.rtos), watch_block_active_always},
    {WATCH_BLOCK_FAULT, "common.fault", &g_watch.fault, sizeof(g_watch.fault), watch_block_active_always},
};

static uint8_t watch_block_active_always(void)
{
    return 1u;
}

static uint8_t watch_block_active_locomotion_classic(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_CLASSIC_CHASSIS);
}

static uint8_t watch_block_active_wheelleg_servo(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_SERVO);
}

static uint8_t watch_block_active_wheelleg_mit(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_MIT);
}

static uint8_t watch_block_active_gimbal_single(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_SINGLE);
}

static uint8_t watch_block_active_gimbal_dual(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_DUAL);
}

static uint8_t watch_block_active_arm(void)
{
    return (uint8_t)(g_config.profile.arm_family != ARM_FAMILY_NONE);
}

static watch_task_diag_entry_t *watch_task_diag_get(watch_task_id_e task_id)
{
    switch (task_id)
    {
    case WATCH_TASK_DEFAULT:
        return &g_watch.diag.task.default_task;
    case WATCH_TASK_DETECT:
        return &g_watch.diag.task.detect_task;
    case WATCH_TASK_IMU:
        return &g_watch.diag.task.imu_task;
    case WATCH_TASK_GIMBAL_CONTROL:
        return &g_watch.diag.task.gimbal_control_task;
    case WATCH_TASK_CHASSIS_CONTROL:
        return &g_watch.diag.task.chassis_control_task;
    case WATCH_TASK_CAN_FEEDBACK_RX:
        return &g_watch.diag.task.can_feedback_rx_task;
    case WATCH_TASK_CAN_COMMAND_TX:
        return &g_watch.diag.task.can_command_tx_task;
    case WATCH_TASK_RC_SBUS:
        return &g_watch.diag.task.rc_sbus_task;
    case WATCH_TASK_HOST_LINK:
        return &g_watch.diag.task.host_link_task;
    case WATCH_TASK_ELRS:
        return &g_watch.diag.task.elrs_task;
    case WATCH_TASK_ARM:
        return &g_watch.diag.task.arm_task;
    case WATCH_TASK_WHEELLEG_MIT:
        return &g_watch.diag.task.wheelleg_mit_task;
    default:
        return NULL;
    }
}

static watch_irq_diag_entry_t *watch_irq_diag_get(watch_irq_id_e irq_id)
{
    switch (irq_id)
    {
    case WATCH_IRQ_IST8310_EXTI:
        return &g_watch.diag.irq.ist8310_exti;
    case WATCH_IRQ_IMU_EXTI:
        return &g_watch.diag.irq.imu_exti;
    case WATCH_IRQ_SD_EXTI:
        return &g_watch.diag.irq.sd_exti;
    case WATCH_IRQ_CAN1_RX0:
        return &g_watch.diag.irq.can1_rx0;
    case WATCH_IRQ_CAN2_RX0:
        return &g_watch.diag.irq.can2_rx0;
    case WATCH_IRQ_USART1:
        return &g_watch.diag.irq.usart1;
    case WATCH_IRQ_UART7:
        return &g_watch.diag.irq.uart7;
    case WATCH_IRQ_UART8:
        return &g_watch.diag.irq.uart8;
    case WATCH_IRQ_OTG_FS:
        return &g_watch.diag.irq.otg_fs;
    case WATCH_IRQ_TIM6_DAC:
        return &g_watch.diag.irq.tim6_dac;
    case WATCH_IRQ_DMA_USART1_RX:
        return &g_watch.diag.irq.dma_usart1_rx;
    case WATCH_IRQ_DMA_SPI5_TX:
        return &g_watch.diag.irq.dma_spi5_tx;
    case WATCH_IRQ_DMA_SPI5_RX:
        return &g_watch.diag.irq.dma_spi5_rx;
    case WATCH_IRQ_DMA_SDIO_TX:
        return &g_watch.diag.irq.dma_sdio_tx;
    default:
        return NULL;
    }
}

void watch_diag_set_boot_stage(watch_boot_stage_e stage)
{
    watch_diag_push_stage(stage);
}

void watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr)
{
    g_watch.diag.error_handler_count++;
    g_watch.diag.error_stage = g_watch.diag.boot_stage;
    g_watch.diag.error_tick_ms = tick_ms;
    g_watch.diag.error_ipsr = ipsr;
}

void watch_diag_set_error_args(uint32_t arg0, uint32_t arg1)
{
    g_watch.diag.error_arg0 = arg0;
    g_watch.diag.error_arg1 = arg1;
}

void watch_task_beat(watch_task_id_e task_id)
{
    const uint32_t now_ms = HAL_GetTick();
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    if (entry->last_tick_ms != 0u &&
        (uint32_t)(now_ms - entry->last_tick_ms) < (uint32_t)ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS)
    {
        return;
    }

    const uint64_t beat_start_us = rt_profiler_begin();
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
    rt_profiler_end(RT_PROFILER_WATCH_TASK_BEAT, beat_start_us);
}

void watch_task_wait(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->wait_count++;
    taskEXIT_CRITICAL();
}

void watch_task_timeout(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->timeout_count++;
    taskEXIT_CRITICAL();
}

void watch_task_error(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->error_count++;
    taskEXIT_CRITICAL();
}

void watch_imu_set_stage(watch_imu_stage_e stage)
{
    taskENTER_CRITICAL();
    g_watch.diag.imu_task_stage = (uint8_t)stage;
    g_watch.diag.imu_task_stage_tick_ms = HAL_GetTick();
    if (stage == WATCH_IMU_STAGE_ENTER)
    {
        g_watch.diag.imu_task_enter_count++;
    }
    taskEXIT_CRITICAL();
}

void watch_irq_hit(watch_irq_id_e irq_id)
{
    const uint32_t now_ms = HAL_GetTick();
    watch_irq_diag_entry_t *entry = watch_irq_diag_get(irq_id);
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

void watch_init(void)
{
    memset(&g_watch, 0, sizeof(g_watch));

    rc_src = get_remote_control_point();
    ins_quat_src = get_INS_quat_point();
    ins_angle_src = get_INS_angle_point();
    ins_gyro_src = get_gyro_data_point();
    ins_accel_src = get_accel_data_point();

    watch_update();
}

void watch_update(void)
{
    watch_copy_rc();
    watch_copy_newrc();
    watch_copy_imu();
    watch_copy_chassis();
    watch_copy_gimbal();
    watch_copy_shoot();
    watch_copy_arm_j0_unitree();
    watch_copy_wheelleg_mit();
    watch_copy_diag();
    watch_copy_rtos();
}

const watch_block_desc_t *watch_get_block_table(uint32_t *count)
{
    if (count != NULL)
    {
        *count = (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0]));
    }
    return g_watch_blocks;
}

const watch_block_desc_t *watch_find_block(watch_block_id_e id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        if (g_watch_blocks[i].id == id)
        {
            return &g_watch_blocks[i];
        }
    }
    return NULL;
}

uint8_t watch_block_is_active(watch_block_id_e id)
{
    const watch_block_desc_t *block = watch_find_block(id);
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

static void watch_diag_push_stage(watch_boot_stage_e stage)
{
    if (stage == WATCH_BOOT_STAGE_NONE)
    {
        g_watch.diag.boot_stage = stage;
        return;
    }

    if (g_watch.diag.boot_stage == stage)
    {
        return;
    }

    for (uint32_t i = (uint32_t)(sizeof(g_watch.diag.boot_trace) / sizeof(g_watch.diag.boot_trace[0])) - 1u; i > 0u; i--)
    {
        g_watch.diag.boot_trace[i] = g_watch.diag.boot_trace[i - 1u];
    }
    g_watch.diag.boot_trace[0] = stage;
    g_watch.diag.boot_stage = stage;
}

static void watch_copy_rc(void)
{
    if (rc_src == NULL)
    {
        memset(&g_watch.rc, 0, sizeof(g_watch.rc));
        return;
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.rc.ch[i] = rc_src->rc.ch[i];
    }
    g_watch.rc.s[0] = rc_src->rc.s[0];
    g_watch.rc.s[1] = rc_src->rc.s[1];

    g_watch.rc.mouse_x = rc_src->mouse.x;
    g_watch.rc.mouse_y = rc_src->mouse.y;
    g_watch.rc.mouse_z = rc_src->mouse.z;
    g_watch.rc.mouse_l = rc_src->mouse.press_l;
    g_watch.rc.mouse_r = rc_src->mouse.press_r;
    g_watch.rc.key = rc_src->key.v;
}

static void watch_copy_newrc(void)
{
    image_remote_state_t state = {0};

    if (!image_remote_get_state(&state))
    {
        memset(&g_watch.newrc, 0, sizeof(g_watch.newrc));
        return;
    }

    g_watch.newrc.valid = state.valid;
    g_watch.newrc.proto = state.proto;
    g_watch.newrc.range_mode = state.range_mode;
    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.newrc.raw_ch[i] = state.raw_ch[i];
    }
    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.newrc.ch[i] = state.ch[i];
    }
    g_watch.newrc.s[0] = state.s[0];
    g_watch.newrc.s[1] = state.s[1];
    g_watch.newrc.mouse_x = state.mouse_x;
    g_watch.newrc.mouse_y = state.mouse_y;
    g_watch.newrc.mouse_z = state.mouse_z;
    g_watch.newrc.mouse_l = state.mouse_l;
    g_watch.newrc.mouse_r = state.mouse_r;
    g_watch.newrc.mouse_mid = state.mouse_mid;
    g_watch.newrc.pause = state.pause;
    g_watch.newrc.btn_l = state.btn_l;
    g_watch.newrc.btn_r = state.btn_r;
    g_watch.newrc.trigger = state.trigger;
    g_watch.newrc.dial = state.dial;
    g_watch.newrc.key_value = state.key_value;
    g_watch.newrc.key_w = state.key_w;
    g_watch.newrc.key_s = state.key_s;
    g_watch.newrc.key_a = state.key_a;
    g_watch.newrc.key_d = state.key_d;
    g_watch.newrc.key_shift = state.key_shift;
    g_watch.newrc.key_ctrl = state.key_ctrl;
    g_watch.newrc.key_q = state.key_q;
    g_watch.newrc.key_e = state.key_e;
    g_watch.newrc.key_r = state.key_r;
    g_watch.newrc.key_f = state.key_f;
    g_watch.newrc.key_g = state.key_g;
    g_watch.newrc.key_z = state.key_z;
    g_watch.newrc.key_x = state.key_x;
    g_watch.newrc.key_c = state.key_c;
    g_watch.newrc.key_v = state.key_v;
    g_watch.newrc.key_b = state.key_b;
    g_watch.newrc.last_rx_tick_ms = state.last_rx_tick_ms;
}

static void watch_copy_imu(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (ins_quat_src != NULL)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            g_watch.imu.quat[i] = ins_quat_src[i];
        }
    }

    if (ins_angle_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.angle_deg[i] = ins_angle_src[i] * rad2deg;
        }
    }

    if (ins_gyro_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.gyro_dps[i] = ins_gyro_src[i] * rad2deg;
        }
    }

    if (ins_accel_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.accel[i] = ins_accel_src[i];
        }
    }
}

static void watch_copy_chassis(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (!watch_block_active_locomotion_classic())
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    chassis_state_t chassis;
    if (chassis_state_read(&chassis) == 0u || chassis.valid == 0u)
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    g_watch.chassis.mode = (watch_chassis_mode_e)chassis.mode;
    g_watch.chassis.last_mode = (watch_chassis_mode_e)chassis.last_mode;

    g_watch.chassis.vx_set = chassis.vx_set;
    g_watch.chassis.vy_set = chassis.vy_set;
    g_watch.chassis.wz_set = chassis.wz_set;
    g_watch.chassis.vx = chassis.vx;
    g_watch.chassis.vy = chassis.vy;
    g_watch.chassis.wz = chassis.wz;
    g_watch.chassis.yaw_deg = chassis.chassis_yaw * rad2deg;

    for (uint8_t i = 0; i < 4u && i < CHASSIS_STATE_MOTOR_COUNT; i++)
    {
        const chassis_motor_state_t *m = &chassis.motor[i];
        g_watch.chassis.motor_rpm[i] = (m->measure.valid != 0u) ? m->measure.speed_rpm : 0;
        g_watch.chassis.motor_current[i] = m->give_current;
        g_watch.chassis.motor_speed_set[i] = m->speed_set;
        g_watch.chassis.motor_ecd[i] = (m->measure.valid != 0u) ? m->measure.ecd : 0;
        g_watch.chassis.motor_temp[i] = (m->measure.valid != 0u) ? m->measure.temperature : 0;
    }
}

static void watch_copy_gimbal(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (!watch_block_active_gimbal_single())
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    gimbal_state_t gimbal;
    if (gimbal_state_read(&gimbal) == 0u || gimbal.valid == 0u)
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    const gimbal_motor_state_t *yaw = &gimbal.yaw;
    const gimbal_motor_state_t *pitch = &gimbal.pitch;

    if (yaw->valid != 0u)
    {
        g_watch.gimbal.yaw_mode = (watch_gimbal_motor_mode_e)yaw->motor_mode;
        g_watch.gimbal.yaw_angle_deg = yaw->angle * rad2deg;
        g_watch.gimbal.yaw_set_deg = yaw->angle_set * rad2deg;
        g_watch.gimbal.yaw_gyro_dps = yaw->motor_gyro * rad2deg;
        g_watch.gimbal.yaw_current = yaw->given_current;

        g_watch.gimbal.yaw_rpm = (yaw->measure.valid != 0u) ? yaw->measure.speed_rpm : 0;
        g_watch.gimbal.yaw_current_fb = (yaw->measure.valid != 0u) ? yaw->measure.given_current : 0;
        g_watch.gimbal.yaw_ecd = (yaw->measure.valid != 0u) ? yaw->measure.ecd : 0;
        g_watch.gimbal.yaw_temp = (yaw->measure.valid != 0u) ? yaw->measure.temperature : 0;
    }
    else
    {
        g_watch.gimbal.yaw_mode = WATCH_GIMBAL_MOTOR_RAW;
        g_watch.gimbal.yaw_angle_deg = 0.0f;
        g_watch.gimbal.yaw_set_deg = 0.0f;
        g_watch.gimbal.yaw_gyro_dps = 0.0f;
        g_watch.gimbal.yaw_current = 0;
        g_watch.gimbal.yaw_rpm = 0;
        g_watch.gimbal.yaw_current_fb = 0;
        g_watch.gimbal.yaw_ecd = 0;
        g_watch.gimbal.yaw_temp = 0;
    }

    if (pitch->valid != 0u)
    {
        g_watch.gimbal.pitch_mode = (watch_gimbal_motor_mode_e)pitch->motor_mode;
        g_watch.gimbal.pitch_angle_deg = pitch->angle * rad2deg;
        g_watch.gimbal.pitch_set_deg = pitch->angle_set * rad2deg;
        g_watch.gimbal.pitch_gyro_dps = pitch->motor_gyro * rad2deg;
        g_watch.gimbal.pitch_current = pitch->given_current;

        g_watch.gimbal.pitch_rpm = (pitch->measure.valid != 0u) ? pitch->measure.speed_rpm : 0;
        g_watch.gimbal.pitch_current_fb = (pitch->measure.valid != 0u) ? pitch->measure.given_current : 0;
        g_watch.gimbal.pitch_ecd = (pitch->measure.valid != 0u) ? pitch->measure.ecd : 0;
        g_watch.gimbal.pitch_temp = (pitch->measure.valid != 0u) ? pitch->measure.temperature : 0;
    }
    else
    {
        g_watch.gimbal.pitch_mode = WATCH_GIMBAL_MOTOR_RAW;
        g_watch.gimbal.pitch_angle_deg = 0.0f;
        g_watch.gimbal.pitch_set_deg = 0.0f;
        g_watch.gimbal.pitch_gyro_dps = 0.0f;
        g_watch.gimbal.pitch_current = 0;
        g_watch.gimbal.pitch_rpm = 0;
        g_watch.gimbal.pitch_current_fb = 0;
        g_watch.gimbal.pitch_ecd = 0;
        g_watch.gimbal.pitch_temp = 0;
    }
}

static void watch_copy_shoot(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    shoot_state_t shoot;
    if (shoot_state_read(&shoot) == 0u || shoot.valid == 0u)
    {
        memset(&g_watch.shoot, 0, sizeof(g_watch.shoot));
        return;
    }

    g_watch.shoot.mode = (watch_shoot_mode_e)shoot.mode;
    g_watch.shoot.fric_speed_set_rpm = (int16_t)shoot.fric_speed_set;
    for (uint8_t i = 0; i < 4; i++)
    {
        g_watch.shoot.fric_current_cmd[i] = actuator_cmd_get_friction_current(i);
    }

    g_watch.shoot.trigger_angle_deg = shoot.angle * rad2deg;
    g_watch.shoot.trigger_set_deg = shoot.set_angle * rad2deg;
    g_watch.shoot.trigger_speed = shoot.speed;
    g_watch.shoot.trigger_speed_set = shoot.speed_set;
    g_watch.shoot.trigger_current = shoot.given_current;

    const motor_measure_t *trigger_meas = get_trigger_motor_measure_point();
    if (trigger_meas)
    {
        g_watch.shoot.trigger_rpm = trigger_meas->speed_rpm;
        g_watch.shoot.trigger_ecd = trigger_meas->ecd;
        g_watch.shoot.trigger_temp = trigger_meas->temperate;
    }
    else
    {
        g_watch.shoot.trigger_rpm = 0;
        g_watch.shoot.trigger_ecd = 0;
        g_watch.shoot.trigger_temp = 0;
    }

    g_watch.shoot.heat_limit = shoot.heat_limit;
    g_watch.shoot.heat = shoot.heat;

    for (uint8_t i = 0; i < 4; i++)
    {
        const motor_measure_t *fm = get_friction_motor_measure_point(i);
        if (fm)
        {
            g_watch.shoot.fric_rpm[i] = fm->speed_rpm;
            g_watch.shoot.fric_current[i] = fm->given_current;
            g_watch.shoot.fric_temp[i] = fm->temperate;
        }
        else
        {
            g_watch.shoot.fric_rpm[i] = 0;
            g_watch.shoot.fric_current[i] = 0;
            g_watch.shoot.fric_temp[i] = 0;
        }
    }
}

static void watch_copy_arm_j0_unitree(void)
{
    const arm_j0_unitree_state_t *state = NULL;

    if (!watch_block_active_arm())
    {
        memset(&g_watch.arm_j0_unitree, 0, sizeof(g_watch.arm_j0_unitree));
        return;
    }

    state = arm_j0_unitree_get_state();
    if (state == NULL)
    {
        memset(&g_watch.arm_j0_unitree, 0, sizeof(g_watch.arm_j0_unitree));
        return;
    }

    g_watch.arm_j0_unitree.enabled = state->enabled;
    g_watch.arm_j0_unitree.rs485_port = state->rs485_port;
    g_watch.arm_j0_unitree.motor_id = state->motor_id;
    g_watch.arm_j0_unitree.online = state->online;
    g_watch.arm_j0_unitree.last_mode = state->last_mode;
    g_watch.arm_j0_unitree.motor_error = state->motor_error;
    g_watch.arm_j0_unitree.motor_temp = state->motor_temp;
    g_watch.arm_j0_unitree.last_tx_status = state->last_tx_status;
    g_watch.arm_j0_unitree.tx_count = state->tx_count;
    g_watch.arm_j0_unitree.tx_fail_count = state->tx_fail_count;
    g_watch.arm_j0_unitree.rx_frame_count = state->rx_frame_count;
    g_watch.arm_j0_unitree.rx_crc_fail_count = state->rx_crc_fail_count;
    g_watch.arm_j0_unitree.rx_parse_error_count = state->rx_parse_error_count;
    g_watch.arm_j0_unitree.last_rx_tick_ms = state->last_rx_tick_ms;
    g_watch.arm_j0_unitree.cmd_speed_rad_s = state->cmd_output_speed_rad_s;
    g_watch.arm_j0_unitree.cmd_kd = state->cmd_output_kd;
    g_watch.arm_j0_unitree.torque_nm = state->torque_nm;
    g_watch.arm_j0_unitree.joint_speed_rad_s = state->joint_speed_rad_s;
    g_watch.arm_j0_unitree.joint_position_rad = state->joint_position_rad;
}

static fp32 watch_wheelleg_dir_sign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static fp32 watch_rad_to_deg(fp32 rad)
{
    return rad * 57.29577951308232f;
}

static void watch_copy_wheelleg_mit_motor(watch_wheelleg_mit_motor_t *out,
                                          uint8_t raw_id,
                                          fp32 zero_rad,
                                          int8_t dir,
                                          uint8_t use_rel_position)
{
    actuator_cmd_t cmd;
    actuator_feedback_t fb;
    actuator_id_e id;

    if (out == NULL)
    {
        return;
    }
    out->id = raw_id;
    if ((uint32_t)raw_id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return;
    }

    id = (actuator_id_e)raw_id;
    if (actuator_cmd_get_copy(id, &cmd) != 0u)
    {
        out->cmd_active = cmd.active;
        out->cmd_mode = cmd.mode;
        out->cmd_position_rad = cmd.position;
        out->cmd_position_deg = watch_rad_to_deg(cmd.position);
        out->cmd_velocity_rad_s = cmd.velocity;
        out->cmd_kp = cmd.kp;
        out->cmd_kd = cmd.kd;
        out->cmd_torque_nm = cmd.torque;
    }

    if (actuator_feedback_get_copy(id, &fb) != 0u)
    {
        out->online = fb.online;
        out->fb_bus = fb.bus;
        out->fb_rx_dlc = fb.rx_dlc;
        out->fb_rx_data0 = fb.rx_data0;
        out->fb_rx_data0_low4 = (uint8_t)(fb.rx_data0 & 0x0Fu);
        out->fb_rx_data0_high4 = (uint8_t)(fb.rx_data0 >> 4);
        out->fb_motor_id = fb.motor_id;
        out->fb_state = fb.state;
        out->fb_rx_id = fb.rx_id;
        out->fb_rx_count = fb.rx_count;
        out->fb_last_rx_tick_ms = fb.last_rx_tick;
        out->fb_position_rad = fb.position;
        out->fb_position_deg = watch_rad_to_deg(fb.position);
        out->fb_velocity_rad_s = fb.velocity;
        out->fb_torque_nm = fb.torque;
        if (use_rel_position != 0u)
        {
            out->rel_position_rad = (fb.position - zero_rad) * watch_wheelleg_dir_sign(dir);
            out->rel_position_deg = watch_rad_to_deg(out->rel_position_rad);
        }
        if (out->cmd_active != 0u)
        {
            out->cmd_minus_fb_rad = out->cmd_position_rad - fb.position;
            out->cmd_minus_fb_deg = watch_rad_to_deg(out->cmd_minus_fb_rad);
        }
    }
}

static void watch_copy_wheelleg_mit_foot_test(uint8_t side)
{
    watch_wheelleg_mit_foot_test_t *out;

    if (side >= (uint8_t)WHEELLEG_SIDE_COUNT)
    {
        return;
    }

    out = &g_watch.wheelleg_mit.foot_test[side];
    wheelleg_mit_get_foot_test_target(side,
                                      &out->target_x_m,
                                      &out->target_y_m,
                                      &out->target_length_m);
    wheelleg_mit_get_foot_test_wheel(side,
                                     &out->wheel_zero_valid,
                                     &out->wheel_zero_rad,
                                     &out->wheel_dx_m,
                                     &out->wheel_comp_rad,
                                     &out->wheel_target_rad);
    out->wheel_zero_deg = watch_rad_to_deg(out->wheel_zero_rad);
    out->wheel_comp_deg = watch_rad_to_deg(out->wheel_comp_rad);
    out->wheel_target_deg = watch_rad_to_deg(out->wheel_target_rad);
}

static void watch_copy_wheelleg_mit(void)
{
    const uint8_t profile_on = watch_block_active_wheelleg_mit();
    const uint8_t chassis_sw = input_switch(INPUT_SW_CHASSIS_MODE);
    const uint8_t test_mode = g_config.test.mode;
    wheelleg_status_t status;
    wheelleg_state_t state;
    uint8_t status_valid;
    uint8_t state_valid;

    memset(&g_watch.wheelleg_mit, 0, sizeof(g_watch.wheelleg_mit));
    if (profile_on == 0u)
    {
        return;
    }

    status_valid = wheelleg_status_read(&status);
    state_valid = wheelleg_state_read(&state);

    g_watch.wheelleg_mit.status_valid = status_valid;
    g_watch.wheelleg_mit.state_valid = state_valid;
    g_watch.wheelleg_mit.profile_on = profile_on;
    g_watch.wheelleg_mit.input_chassis_switch = chassis_sw;
    g_watch.wheelleg_mit.enable_switch_pos = g_config.wheelleg_mit.enable_switch_pos;
    g_watch.wheelleg_mit.manual_on =
        control_input_switch_is_pos(chassis_sw, g_config.wheelleg_mit.enable_switch_pos);
    g_watch.wheelleg_mit.test_mode = test_mode;
    g_watch.wheelleg_mit.foot_test_phase = wheelleg_mit_get_foot_test_phase();
    g_watch.wheelleg_mit.foot_test_ik_ok = wheelleg_mit_get_foot_test_ik_ok();
    watch_copy_wheelleg_mit_foot_test((uint8_t)WHEELLEG_SIDE_LEFT);
    watch_copy_wheelleg_mit_foot_test((uint8_t)WHEELLEG_SIDE_RIGHT);

    if (status_valid != 0u)
    {
        g_watch.wheelleg_mit.mode = status.mode;
        g_watch.wheelleg_mit.last_mode = status.last_mode;
        g_watch.wheelleg_mit.fault_flags = status.fault_flags;
        g_watch.wheelleg_mit.health = status.health;
        g_watch.wheelleg_mit.controller_active = status.controller_active;
        g_watch.wheelleg_mit.active_controller_id = status.active_controller_id;
        g_watch.wheelleg_mit.target_v_mps = status.target_v_mps;
        g_watch.wheelleg_mit.target_leg_length_m = status.target_leg_length_m;
        g_watch.wheelleg_mit.target_foot_x_m = status.target_foot_x_m;
        g_watch.wheelleg_mit.target_leg_theta_rad = status.target_leg_theta_rad;
        g_watch.wheelleg_mit.target_leg_theta_deg = watch_rad_to_deg(status.target_leg_theta_rad);
        g_watch.wheelleg_mit.pitch_rad = status.pitch_rad;
        g_watch.wheelleg_mit.pitch_deg = watch_rad_to_deg(status.pitch_rad);
        g_watch.wheelleg_mit.x_dot_mps = status.x_dot_mps;
        g_watch.wheelleg_mit.enabled =
            (uint8_t)(g_watch.wheelleg_mit.manual_on != 0u &&
                      (status.fault_flags == WHEELLEG_FAULT_NONE ||
                       status.controller_active != 0u));

        g_watch.wheelleg_mit.leg[0].length_m = status.leg_length_m[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].length_m = status.leg_length_m[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].theta_rad = status.leg_theta_rad[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].theta_rad = status.leg_theta_rad[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.wheelleg_mit.leg[1].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.wheelleg_mit.leg[0].support_force_n = status.support_force_n[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].support_force_n = status.support_force_n[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_RIGHT];
    }

    if (state_valid != 0u)
    {
        g_watch.wheelleg_mit.leg[0].front_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[0];
        g_watch.wheelleg_mit.leg[0].back_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[1];
        g_watch.wheelleg_mit.leg[1].front_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[0];
        g_watch.wheelleg_mit.leg[1].back_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[1];
        g_watch.wheelleg_mit.leg[0].wheel_online = state.wheel_online[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].wheel_online = state.wheel_online[WHEELLEG_SIDE_RIGHT];
    }

    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[0],
                                  g_config.wheelleg_mit.left_front_actuator,
                                  g_config.wheelleg_mit.left_front_zero_rad,
                                  g_config.wheelleg_mit.left_front_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[1],
                                  g_config.wheelleg_mit.left_back_actuator,
                                  g_config.wheelleg_mit.left_back_zero_rad,
                                  g_config.wheelleg_mit.left_back_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[2],
                                  g_config.wheelleg_mit.right_front_actuator,
                                  g_config.wheelleg_mit.right_front_zero_rad,
                                  g_config.wheelleg_mit.right_front_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[3],
                                  g_config.wheelleg_mit.right_back_actuator,
                                  g_config.wheelleg_mit.right_back_zero_rad,
                                  g_config.wheelleg_mit.right_back_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.wheel[0],
                                  g_config.wheelleg_mit.left_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.wheel[1],
                                  g_config.wheelleg_mit.right_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
}

static void watch_copy_diag(void)
{
    bmi088_diag_t bmi088_diag = {0};
    wheelleg_debug_t wheelleg_debug = {0};
    sdlog_stats_t sd_stats = {0};
    static uint32_t can_rate_last_tick_ms;
    static uint32_t can_rate_last_rx_count;
    static uint32_t can_rate_last_tx_count;
    static uint32_t can_rate_last_tx_fail_count;
    static uint32_t can_rate_last_rx_drop_count;
    static uint32_t can1_rx_fps;
    static uint32_t can1_tx_fps;
    static uint32_t can1_tx_fail_fps;
    static uint32_t can1_rx_drop_fps;
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t can1_rx_count = CAN_get_can1_rx_count();
    const uint32_t can1_tx_count = CAN_get_can1_tx_count();
    const uint32_t can1_tx_fail_count = CAN_get_can1_tx_fail_count();
    const uint32_t can1_rx_drop_count = CAN_get_can1_rx_drop_count();
    const uint8_t wheelleg_debug_valid =
        (robot_profile_is_wheelleg_mit() != 0u) ? wheelleg_debug_read(&wheelleg_debug) : 0u;
    uint32_t elapsed_ms;

    g_watch.diag.can1_1ff_status = (int16_t)CAN_get_last_1ff_status();
    g_watch.diag.can1_err = CAN_get_last_can1_error();
    g_watch.diag.can2_err = CAN_get_last_can2_error();
    g_watch.diag.can1_tx_status = bsp_can_get_last_tx_status(1u);
    g_watch.diag.can2_tx_status = bsp_can_get_last_tx_status(2u);
    if (can_rate_last_tick_ms == 0u)
    {
        can_rate_last_tick_ms = now_ms;
        can_rate_last_rx_count = can1_rx_count;
        can_rate_last_tx_count = can1_tx_count;
        can_rate_last_tx_fail_count = can1_tx_fail_count;
        can_rate_last_rx_drop_count = can1_rx_drop_count;
    }
    elapsed_ms = now_ms - can_rate_last_tick_ms;
    if (elapsed_ms >= 1000u)
    {
        can1_rx_fps = ((can1_rx_count - can_rate_last_rx_count) * 1000u) / elapsed_ms;
        can1_tx_fps = ((can1_tx_count - can_rate_last_tx_count) * 1000u) / elapsed_ms;
        can1_tx_fail_fps = ((can1_tx_fail_count - can_rate_last_tx_fail_count) * 1000u) / elapsed_ms;
        can1_rx_drop_fps = ((can1_rx_drop_count - can_rate_last_rx_drop_count) * 1000u) / elapsed_ms;
        can_rate_last_tick_ms = now_ms;
        can_rate_last_rx_count = can1_rx_count;
        can_rate_last_tx_count = can1_tx_count;
        can_rate_last_tx_fail_count = can1_tx_fail_count;
        can_rate_last_rx_drop_count = can1_rx_drop_count;
    }

    g_watch.diag.can1_rx_count = can1_rx_count;
    g_watch.diag.can1_tx_count = can1_tx_count;
    g_watch.diag.can1_tx_fail_count = can1_tx_fail_count;
    g_watch.diag.can1_rx_drop_count = can1_rx_drop_count;
    g_watch.diag.can1_rx_fps = can1_rx_fps;
    g_watch.diag.can1_tx_fps = can1_tx_fps;
    g_watch.diag.can1_tx_fail_fps = can1_tx_fail_fps;
    g_watch.diag.can1_rx_drop_fps = can1_rx_drop_fps;
    g_watch.diag.can_rx_pending = bsp_can_rx_pending();
    g_watch.diag.can1_last_rx_id = CAN_get_can1_last_rx_id();
    g_watch.diag.can1_last_tx_id = CAN_get_can1_last_tx_id();
    g_watch.diag.can1_last_rx_dlc = CAN_get_can1_last_rx_dlc();
    g_watch.diag.can1_last_tx_dlc = CAN_get_can1_last_tx_dlc();
    g_watch.diag.can1_tx_error_count = CAN_get_can1_tx_error_count();
    g_watch.diag.can1_rx_error_count = CAN_get_can1_rx_error_count();
    g_watch.diag.manual_active_source = remote_control_get_active_source();
    g_watch.diag.sd_mounted = (uint8_t)sdcard_is_mounted();
    sdlog_get_stats(&sd_stats);
    g_watch.diag.sdlog_active = sd_stats.active;
    g_watch.diag.manual_sbus_frame_count = remote_control_get_sbus_frame_count();
    g_watch.diag.manual_set_source_count = remote_control_get_set_source_count();
    g_watch.diag.battery_voltage_v = get_battery_voltage_cached();
    g_watch.diag.battery_percent = get_battery_percentage_fp32() * 100.0f;
    g_watch.diag.battery_low_alarm = battery_monitor_is_low_alarm();
    g_watch.diag.adc_started = bsp_adc_is_started();
    g_watch.diag.adc_raw0 = bsp_adc_get_raw(0u);
    g_watch.diag.adc_raw1 = bsp_adc_get_raw(1u);
    g_watch.diag.adc_voltage0_v = bsp_adc_get_channel_voltage(0u);
    g_watch.diag.adc_voltage1_v = bsp_adc_get_channel_voltage(1u);
    g_watch.diag.adc_start_ok_count = bsp_adc_get_start_ok_count();
    g_watch.diag.adc_start_fail_count = bsp_adc_get_start_fail_count();

    BMI088_get_diag(&bmi088_diag);
    g_watch.diag.imu_bmi088_init_error = bmi088_diag.init_last_error;
    g_watch.diag.imu_bmi088_fail_reg = bmi088_diag.init_fail_reg;
    g_watch.diag.imu_bmi088_fail_expect = bmi088_diag.init_fail_expect;
    g_watch.diag.imu_bmi088_fail_actual = bmi088_diag.init_fail_actual;
    g_watch.diag.imu_accel_chip_id = bmi088_diag.accel_chip_id;
    g_watch.diag.imu_gyro_chip_id = bmi088_diag.gyro_chip_id;
    g_watch.diag.imu_gyro_read_chip_id = bmi088_diag.gyro_read_chip_id;
    g_watch.diag.imu_gyro_read_ok = bmi088_diag.gyro_read_ok;
    g_watch.diag.imu_bmi088_init_count = bmi088_diag.init_count;
    g_watch.diag.imu_bmi088_init_error_count = bmi088_diag.init_error_count;
    g_watch.diag.imu_bmi088_read_count = bmi088_diag.read_count;
    g_watch.diag.imu_gyro_read_bad_count = bmi088_diag.gyro_read_bad_count;
    g_watch.diag.imu_gyro_boot_calibrating = (uint8_t)ins_is_gyro_boot_calibrating();
    g_watch.diag.imu_gyro_boot_result = (uint8_t)ins_get_gyro_boot_initial_result();
    g_watch.diag.imu_temp_c = ins_get_imu_temperature_c();
    g_watch.diag.imu_heater_pwm = ins_get_imu_heater_pwm();
    g_watch.diag.imu_heater_mode = ins_get_imu_heater_mode();
    g_watch.diag.imu_heater_pid_out = ins_get_imu_heater_pid_out();

    g_watch.diag.wheelleg_lqr_valid = wheelleg_debug_valid;
    if (wheelleg_debug_valid != 0u)
    {
        g_watch.diag.wheelleg_lqr_theta_err_rad = wheelleg_debug.lqr.error[0];
        g_watch.diag.wheelleg_lqr_dtheta_radps = wheelleg_debug.lqr.error[1];
        g_watch.diag.wheelleg_lqr_x_m = wheelleg_debug.lqr.error[2];
        g_watch.diag.wheelleg_lqr_v_err_mps = wheelleg_debug.lqr.error[3];
        g_watch.diag.wheelleg_lqr_pitch_rad = wheelleg_debug.lqr.error[4];
        g_watch.diag.wheelleg_lqr_pitch_gyro_radps = wheelleg_debug.lqr.error[5];
        g_watch.diag.wheelleg_lqr_right_torque_nm = wheelleg_debug.lqr.output[0];
        g_watch.diag.wheelleg_lqr_left_torque_nm = wheelleg_debug.lqr.output[1];
    }
    else
    {
        g_watch.diag.wheelleg_lqr_theta_err_rad = 0.0f;
        g_watch.diag.wheelleg_lqr_dtheta_radps = 0.0f;
        g_watch.diag.wheelleg_lqr_x_m = 0.0f;
        g_watch.diag.wheelleg_lqr_v_err_mps = 0.0f;
        g_watch.diag.wheelleg_lqr_pitch_rad = 0.0f;
        g_watch.diag.wheelleg_lqr_pitch_gyro_radps = 0.0f;
        g_watch.diag.wheelleg_lqr_right_torque_nm = 0.0f;
        g_watch.diag.wheelleg_lqr_left_torque_nm = 0.0f;
    }
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared watch can link even if a target doesn't export them.
__weak uint32_t gimbal_high_water;
__weak uint32_t chassis_high_water;
__weak uint32_t detect_task_stack;
__weak uint32_t calibrate_task_stack;
#endif

static void watch_copy_rtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_watch.rtos.heap_free = heap_get_free();
    g_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_watch.diag.watch_update_count++;
    g_watch.diag.watch_update_tick_ms = HAL_GetTick();
    g_watch.diag.scheduler_state = (uint32_t)xTaskGetSchedulerState();
    g_watch.diag.task_count = (uint32_t)uxTaskGetNumberOfTasks();
    current_task = xTaskGetCurrentTaskHandle();
    g_watch.diag.current_task_handle = (uint32_t)current_task;
    current_name = pcTaskGetTaskName(NULL);
    memset(g_watch.diag.current_task_name, 0, sizeof(g_watch.diag.current_task_name));
    if (current_name != NULL)
    {
        (void)strncpy(g_watch.diag.current_task_name, current_name, sizeof(g_watch.diag.current_task_name) - 1u);
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
    g_watch.rtos.stack_gimbal = gimbal_high_water;
    g_watch.rtos.stack_chassis = chassis_high_water;
    g_watch.rtos.stack_detect = detect_task_stack;
    g_watch.rtos.stack_calibrate = calibrate_task_stack;
#else
    g_watch.rtos.stack_gimbal = 0u;
    g_watch.rtos.stack_chassis = 0u;
    g_watch.rtos.stack_detect = 0u;
    g_watch.rtos.stack_calibrate = 0u;
#endif
}
