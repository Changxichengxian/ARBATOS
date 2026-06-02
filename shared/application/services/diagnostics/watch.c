/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
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
#include "LowCmd.h"
#include "arm_task.h"
#include "battery_monitor_task.h"
#include "bsp_adc.h"
#include "chassis_state.h"
#include "control_manager.h"
#include "control_input.h"
#include "gimbal_state.h"
#include "INS_task.h"
#include "bmi088driver.h"
#include "detect_task.h"
#include "mem_mang.h"
#include "manual_input.h"
#include "bsp_can.h"
#include "motor_instance.h"
#include "sdcard.h"
#include "sdlog.h"
#include "shoot_state.h"
#include "host_link_task.h"
#include "robot_device_config.h"
#include "robot_task_profile.h"
#include "rt_profiler.h"
#include "wheelleg_mit_task.h"
#include "wheelleg_msg.h"

watch_t g_watch;

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
typedef struct
{
    uint8_t active;
    uint8_t valid;
    uint16_t reserved0;
    uint32_t sample_count;
    fp32 target_v_sum;
    fp32 target_v_abs_max;
    fp32 target_yaw_rate_sum;
    fp32 target_yaw_rate_abs_max;
    fp32 x_dot_sum;
    fp32 x_dot_min;
    fp32 x_dot_max;
    fp32 pitch_sum;
    fp32 pitch_min;
    fp32 pitch_max;
    fp32 lqr_pitch_gyro_sum;
    fp32 lqr_pitch_gyro_min;
    fp32 lqr_pitch_gyro_max;
    fp32 yaw_gyro_sum;
    fp32 yaw_gyro_abs_max;
    fp32 wheel_sum_sum;
    fp32 wheel_sum_abs_max;
    fp32 wheel_diff_sum;
    fp32 wheel_diff_abs_max;
    fp32 lqr_v_err_sum;
    fp32 lqr_v_err_min;
    fp32 lqr_v_err_max;
    fp32 lqr_x_sum;
    fp32 lqr_x_min;
    fp32 lqr_x_max;
    fp32 lqr_pitch_err_sum;
    fp32 lqr_pitch_err_min;
    fp32 lqr_pitch_err_max;
    fp32 lqr_left_pitch_err_sum;
    fp32 lqr_left_pitch_err_min;
    fp32 lqr_left_pitch_err_max;
} watch_wheelleg_run_capture_t;

static watch_wheelleg_run_capture_t s_wheelleg_run_capture;
#endif

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
static void watch_copy_runtime(void);
static void watch_copy_imu(void);
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static void watch_copy_chassis(void);
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
static void watch_copy_gimbal(void);
#endif
#if WATCH_ENABLE_SHOOT_RM
static void watch_copy_shoot(void);
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
static void watch_copy_arm_j0_unitree(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static void watch_copy_wheelleg_mit(void);
#endif
static void watch_copy_diag(void);
static void watch_copy_rtos(void);
static void watch_diag_push_stage(watch_boot_stage_e stage);
static void watch_runtime_add_entry(const char *name,
                                    runtime_instance_kind_e kind,
                                    runtime_instance_state_e state,
                                    uint16_t source_id,
                                    uint16_t source_index,
                                    uint16_t parent_index);
static runtime_instance_state_e watch_runtime_device_state(const robot_config_device_t *device);
static watch_task_diag_entry_t *watch_task_diag_get(watch_task_id_e task_id);
static watch_irq_diag_entry_t *watch_irq_diag_get(watch_irq_id_e irq_id);
static uint8_t watch_block_active_always(void);
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static uint8_t watch_block_active_locomotion_classic(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
static uint8_t watch_block_active_wheelleg_servo(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static uint8_t watch_block_active_wheelleg_mit(void);
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
static uint8_t watch_block_active_gimbal_single(void);
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
static uint8_t watch_block_active_gimbal_dual(void);
#endif
#if WATCH_ENABLE_SHOOT_RM
static uint8_t watch_block_active_shoot_rm(void);
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
static uint8_t watch_block_active_arm(void);
#endif

static const watch_block_desc_t g_watch_blocks[] = {
    {WATCH_BLOCK_RC, "input.rc", &g_watch.rc, sizeof(g_watch.rc), watch_block_active_always},
    {WATCH_BLOCK_NEWRC, "input.newrc", &g_watch.newrc, sizeof(g_watch.newrc), watch_block_active_always},
    {WATCH_BLOCK_RUNTIME, "runtime.instances", &g_watch.runtime, sizeof(g_watch.runtime), watch_block_active_always},
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    {WATCH_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", &g_watch.chassis, sizeof(g_watch.chassis), watch_block_active_locomotion_classic},
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.wheelleg_servo", &g_watch.wheelleg_servo, sizeof(g_watch.wheelleg_servo), watch_block_active_wheelleg_servo},
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.wheelleg_mit", &g_watch.wheelleg_mit, sizeof(g_watch.wheelleg_mit), watch_block_active_wheelleg_mit},
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    {WATCH_BLOCK_GIMBAL_SINGLE, "gimbal.single", &g_watch.gimbal, sizeof(g_watch.gimbal), watch_block_active_gimbal_single},
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
    {WATCH_BLOCK_GIMBAL_DUAL, "gimbal.dual", &g_watch.dual_gimbal, sizeof(g_watch.dual_gimbal), watch_block_active_gimbal_dual},
#endif
#if WATCH_ENABLE_SHOOT_RM
    {WATCH_BLOCK_SHOOT_RM, "shoot.rm", &g_watch.shoot, sizeof(g_watch.shoot), watch_block_active_shoot_rm},
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    {WATCH_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", &g_watch.arm_j0_unitree, sizeof(g_watch.arm_j0_unitree), watch_block_active_arm},
#endif
    {WATCH_BLOCK_IMU, "common.imu", &g_watch.imu, sizeof(g_watch.imu), watch_block_active_always},
    {WATCH_BLOCK_DIAG, "common.diag", &g_watch.diag, sizeof(g_watch.diag), watch_block_active_always},
    {WATCH_BLOCK_RTOS, "common.rtos", &g_watch.rtos, sizeof(g_watch.rtos), watch_block_active_always},
    {WATCH_BLOCK_FAULT, "common.fault", &g_watch.fault, sizeof(g_watch.fault), watch_block_active_always},
};

static watch_block_desc_t g_watch_active_blocks[WATCH_BLOCK_COUNT];

static uint8_t watch_block_active_always(void)
{
    return 1u;
}

static fp32 watch_rad_to_deg(fp32 rad)
{
    return rad * 57.29577951308232f;
}

static fp32 watch_abs_fp32(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static uint8_t watch_block_active_locomotion_classic(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
static uint8_t watch_block_active_wheelleg_servo(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static uint8_t watch_block_active_wheelleg_mit(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static uint8_t watch_block_active_gimbal_single(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
}
#endif

#if WATCH_ENABLE_GIMBAL_DUAL
static uint8_t watch_block_active_gimbal_dual(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}
#endif

#if WATCH_ENABLE_SHOOT_RM
static uint8_t watch_block_active_shoot_rm(void)
{
    if (g_config.motor.trigger.can_id != 0u)
    {
        return 1u;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.motor.friction[i].can_id != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}
#endif

#if WATCH_ENABLE_ARM_J0_UNITREE
static uint8_t watch_block_active_arm(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_ARM);
}
#endif

static watch_task_diag_entry_t *watch_task_diag_get(watch_task_id_e task_id)
{
    switch (task_id)
    {
    case WATCH_TASK_DEFAULT:
        return &g_watch.rtos.task.default_task;
    case WATCH_TASK_DETECT:
        return &g_watch.rtos.task.detect_task;
    case WATCH_TASK_IMU:
        return &g_watch.rtos.task.imu_task;
    case WATCH_TASK_GIMBAL_CONTROL:
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
        return &g_watch.rtos.task.gimbal_control_task;
#else
        return NULL;
#endif
    case WATCH_TASK_CHASSIS_CONTROL:
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
        return &g_watch.rtos.task.chassis_control_task;
#else
        return NULL;
#endif
    case WATCH_TASK_CAN_FEEDBACK_RX:
        return &g_watch.rtos.task.can_feedback_rx_task;
    case WATCH_TASK_CAN_COMMAND_TX:
        return &g_watch.rtos.task.can_command_tx_task;
    case WATCH_TASK_RC_SBUS:
        return &g_watch.rtos.task.rc_sbus_task;
    case WATCH_TASK_HOST_LINK:
        return &g_watch.rtos.task.host_link_task;
    case WATCH_TASK_ELRS:
        return &g_watch.rtos.task.elrs_task;
    case WATCH_TASK_ARM:
#if WATCH_ENABLE_ARM_J0_UNITREE
        return &g_watch.rtos.task.arm_task;
#else
        return NULL;
#endif
    case WATCH_TASK_WHEELLEG_MIT:
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
        return &g_watch.rtos.task.wheelleg_mit_task;
#else
        return NULL;
#endif
    default:
        return NULL;
    }
}

static watch_irq_diag_entry_t *watch_irq_diag_get(watch_irq_id_e irq_id)
{
    switch (irq_id)
    {
    case WATCH_IRQ_IST8310_EXTI:
        return &g_watch.rtos.irq.ist8310_exti;
    case WATCH_IRQ_IMU_EXTI:
        return &g_watch.rtos.irq.imu_exti;
    case WATCH_IRQ_SD_EXTI:
        return &g_watch.rtos.irq.sd_exti;
    case WATCH_IRQ_CAN1_RX0:
        return &g_watch.rtos.irq.can1_rx0;
    case WATCH_IRQ_CAN2_RX0:
        return &g_watch.rtos.irq.can2_rx0;
    case WATCH_IRQ_USART1:
        return &g_watch.rtos.irq.usart1;
    case WATCH_IRQ_UART7:
        return &g_watch.rtos.irq.uart7;
    case WATCH_IRQ_UART8:
        return &g_watch.rtos.irq.uart8;
    case WATCH_IRQ_OTG_FS:
        return &g_watch.rtos.irq.otg_fs;
    case WATCH_IRQ_TIM6_DAC:
        return &g_watch.rtos.irq.tim6_dac;
    case WATCH_IRQ_DMA_USART1_RX:
        return &g_watch.rtos.irq.dma_usart1_rx;
    case WATCH_IRQ_DMA_SPI5_TX:
        return &g_watch.rtos.irq.dma_spi5_tx;
    case WATCH_IRQ_DMA_SPI5_RX:
        return &g_watch.rtos.irq.dma_spi5_rx;
    case WATCH_IRQ_DMA_SDIO_TX:
        return &g_watch.rtos.irq.dma_sdio_tx;
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
    g_watch.fault.error_handler_count++;
    g_watch.fault.error_stage = g_watch.fault.boot_stage;
    g_watch.fault.error_tick_ms = tick_ms;
    g_watch.fault.error_ipsr = ipsr;
}

void watch_diag_set_error_args(uint32_t arg0, uint32_t arg1)
{
    g_watch.fault.error_arg0 = arg0;
    g_watch.fault.error_arg1 = arg1;
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
    g_watch.rtos.imu_task_stage = (uint8_t)stage;
    g_watch.rtos.imu_task_stage_tick_ms = HAL_GetTick();
    if (stage == WATCH_IMU_STAGE_ENTER)
    {
        g_watch.rtos.imu_task_enter_count++;
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
    watch_copy_runtime();
    watch_copy_imu();
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    watch_copy_chassis();
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    watch_copy_gimbal();
#endif
#if WATCH_ENABLE_SHOOT_RM
    watch_copy_shoot();
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    watch_copy_arm_j0_unitree();
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    watch_copy_wheelleg_mit();
#endif
    watch_copy_diag();
    watch_copy_rtos();
}

const watch_block_desc_t *watch_get_block_table(uint32_t *count)
{
    uint32_t active_count = 0u;
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const watch_block_desc_t *block = &g_watch_blocks[i];
        if (block->is_active == NULL || block->is_active())
        {
            g_watch_active_blocks[active_count++] = *block;
        }
    }

    if (count != NULL)
    {
        *count = active_count;
    }
    return g_watch_active_blocks;
}

const watch_block_desc_t *watch_find_block(watch_block_id_e id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const watch_block_desc_t *block = &g_watch_blocks[i];
        if (block->id == id &&
            (block->is_active == NULL || block->is_active()))
        {
            return block;
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

static void watch_runtime_add_entry(const char *name,
                                    runtime_instance_kind_e kind,
                                    runtime_instance_state_e state,
                                    uint16_t source_id,
                                    uint16_t source_index,
                                    uint16_t parent_index)
{
    const uint8_t index = g_watch.runtime.entry_visible_count;

    if (g_watch.runtime.entry_count < 0xFFu)
    {
        g_watch.runtime.entry_count++;
    }

    if (index >= (uint8_t)(sizeof(g_watch.runtime.entry) / sizeof(g_watch.runtime.entry[0])))
    {
        return;
    }

    g_watch.runtime.entry[index] = runtime_instance_ref_make(name,
                                                            kind,
                                                            state,
                                                            source_id,
                                                            source_index,
                                                            parent_index);
    g_watch.runtime.entry_visible_count++;
}

static runtime_instance_state_e watch_runtime_device_state(const robot_config_device_t *device)
{
    if (device == NULL)
    {
        return RUNTIME_INSTANCE_STATE_UNKNOWN;
    }

    switch ((robot_config_device_kind_e)device->kind)
    {
    case ROBOT_CONFIG_DEVICE_KIND_MOTOR:
    {
        const motor_instance_t *inst = motor_instance_find_by_actuator((MotorId)device->source_id);
        return (inst != NULL && motor_instance_enabled(inst) != 0u) ?
                   RUNTIME_INSTANCE_STATE_ENABLED :
                   RUNTIME_INSTANCE_STATE_DISABLED;
    }
    default:
        return RUNTIME_INSTANCE_STATE_ENABLED;
    }
}

static void watch_diag_push_stage(watch_boot_stage_e stage)
{
    if (stage == WATCH_BOOT_STAGE_NONE)
    {
        g_watch.fault.boot_stage = stage;
        return;
    }

    if (g_watch.fault.boot_stage == stage)
    {
        return;
    }

    for (uint32_t i = (uint32_t)(sizeof(g_watch.fault.boot_trace) / sizeof(g_watch.fault.boot_trace[0])) - 1u; i > 0u; i--)
    {
        g_watch.fault.boot_trace[i] = g_watch.fault.boot_trace[i - 1u];
    }
    g_watch.fault.boot_trace[0] = stage;
    g_watch.fault.boot_stage = stage;
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

static void watch_copy_runtime(void)
{
    uint8_t motor_count;
    uint8_t motor_visible_count;
    uint8_t controller_count;
    uint8_t controller_visible_count;
    uint8_t task_module_count;
    uint8_t task_module_visible_count;
    uint8_t device_count;
    rt_profiler_summary_t profiler_summary;

    memset(&g_watch.runtime, 0, sizeof(g_watch.runtime));
    rt_profiler_get_summary(&profiler_summary);
    g_watch.runtime.profile_kind = (uint8_t)robot_profile_kind();
    g_watch.runtime.board_kind = (uint8_t)robot_board_kind();
    g_watch.runtime.board_can_bus_count = robot_board_can_bus_count();
    g_watch.runtime.board_has_fpu = robot_board_has_fpu();
    g_watch.runtime.board_cpu_hz = robot_board_cpu_hz();
    g_watch.runtime.rt_profiler_count = profiler_summary.total_count;
    g_watch.runtime.rt_profiler_active_count = profiler_summary.active_count;
    g_watch.runtime.rt_profiler_over_budget_count = profiler_summary.over_budget_count;
    g_watch.runtime.rt_profiler_total_overrun_count = profiler_summary.total_overrun_count;
    g_watch.runtime.rt_profiler_max_last_us = profiler_summary.max_last_us;
    g_watch.runtime.rt_profiler_max_budget_us = profiler_summary.max_budget_us;
    g_watch.runtime.rt_profiler_max_over_budget_us = profiler_summary.max_over_budget_us;

    task_module_count = robot_profile_module_count();
    task_module_visible_count = task_module_count;
    if (task_module_visible_count > (uint8_t)(sizeof(g_watch.runtime.task_module) / sizeof(g_watch.runtime.task_module[0])))
    {
        task_module_visible_count = (uint8_t)(sizeof(g_watch.runtime.task_module) / sizeof(g_watch.runtime.task_module[0]));
    }

    g_watch.runtime.task_module_count = task_module_count;
    g_watch.runtime.task_module_visible_count = task_module_visible_count;
    for (uint8_t i = 0u; i < task_module_visible_count; i++)
    {
        const robot_task_module_id_t module = robot_profile_module_id_at(i);
        watch_runtime_task_module_t *dst = &g_watch.runtime.task_module[i];

        dst->module = (uint8_t)module;
        dst->name = robot_profile_module_name(module);
        watch_runtime_add_entry(dst->name,
                                RUNTIME_INSTANCE_KIND_TASK,
                                RUNTIME_INSTANCE_STATE_ENABLED,
                                (uint16_t)module,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }

    device_count = robot_config_device_count();
    g_watch.runtime.device_count = device_count;
    for (uint8_t i = 0u; i < device_count; i++)
    {
        robot_config_device_t device;

        if (robot_config_device_get(i, &device) == 0u)
        {
            continue;
        }

        watch_runtime_add_entry(device.name,
                                RUNTIME_INSTANCE_KIND_DEVICE,
                                watch_runtime_device_state(&device),
                                device.source_id,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }

    motor_count = motor_instance_count();
    motor_visible_count = motor_count;
    if (motor_visible_count > (uint8_t)(sizeof(g_watch.runtime.motor) / sizeof(g_watch.runtime.motor[0])))
    {
        motor_visible_count = (uint8_t)(sizeof(g_watch.runtime.motor) / sizeof(g_watch.runtime.motor[0]));
    }

    g_watch.runtime.motor_count = motor_count;
    g_watch.runtime.motor_visible_count = motor_visible_count;
    for (uint8_t i = 0u; i < motor_visible_count; i++)
    {
        const motor_instance_t *inst = motor_instance_get(i);
        watch_runtime_motor_t *dst = &g_watch.runtime.motor[i];
        MotorId actuator_id;

        if (inst == NULL)
        {
            continue;
        }

        actuator_id = motor_instance_actuator_id(inst);
        dst->name = motor_instance_name(inst);
        dst->actuator_id = (uint16_t)actuator_id;
        dst->role = (uint8_t)inst->role;
        dst->role_index = inst->role_index;
        dst->enabled = motor_instance_enabled(inst);
        dst->bus = motor_instance_bus(inst);
        dst->transport = motor_instance_transport_id(actuator_id);
        dst->protocol = motor_instance_protocol_id(actuator_id);
        dst->control_mode = motor_instance_control_mode_id(actuator_id);
        dst->cmd_caps = motor_instance_cmd_caps_id(actuator_id);
        dst->model = motor_instance_model_id(actuator_id);
        if (dst->enabled != 0u)
        {
            g_watch.runtime.motor_enabled_count++;
        }
    }

    controller_count = control_manager_registered_count();
    controller_visible_count = controller_count;
    if (controller_visible_count > (uint8_t)(sizeof(g_watch.runtime.controller) / sizeof(g_watch.runtime.controller[0])))
    {
        controller_visible_count = (uint8_t)(sizeof(g_watch.runtime.controller) / sizeof(g_watch.runtime.controller[0]));
    }

    g_watch.runtime.controller_count = controller_count;
    g_watch.runtime.controller_visible_count = controller_visible_count;
    g_watch.runtime.active_claim_mask = control_manager_active_claim_mask();
    for (uint8_t i = 0u; i < controller_visible_count; i++)
    {
        const control_controller_t *controller = control_manager_get_registered(i);
        watch_runtime_controller_t *dst = &g_watch.runtime.controller[i];

        if (controller == NULL)
        {
            continue;
        }

        dst->name = controller->name;
        dst->id = controller->id;
        dst->period_ms = controller->meta.period_ms;
        dst->phase_ms = controller->meta.phase_ms;
        dst->domain = (uint8_t)controller->domain;
        dst->active = control_manager_is_active(controller->id);
        dst->priority = controller->meta.priority;
        dst->input_count = controller->meta.input_count;
        dst->output_count = controller->meta.output_count;
        watch_runtime_add_entry(dst->name,
                                RUNTIME_INSTANCE_KIND_CONTROLLER,
                                (dst->active != 0u) ?
                                    RUNTIME_INSTANCE_STATE_ACTIVE :
                                    RUNTIME_INSTANCE_STATE_ENABLED,
                                dst->id,
                                i,
                                (uint16_t)controller->domain);
    }

    g_watch.runtime.domain_count = (uint8_t)CONTROL_DOMAIN__COUNT;
    for (uint8_t i = 0u; i < (uint8_t)CONTROL_DOMAIN__COUNT; i++)
    {
        control_domain_status_t status = {0};
        watch_runtime_domain_t *dst = &g_watch.runtime.domain[i];

        if (control_manager_get_domain_status((control_domain_e)i, &status) != CONTROL_RESULT_OK)
        {
            continue;
        }

        dst->active_name = status.active_name;
        dst->active_id = status.active_id;
        dst->pending_id = status.pending_id;
        dst->domain = (uint8_t)status.domain;
        dst->active = status.active;
        dst->state = (uint8_t)status.state;
        dst->pending_request = (uint8_t)status.pending_request;
        dst->last_reason = (uint8_t)status.last_reason;
        dst->last_result = (uint8_t)status.last_result;
        dst->update_count = status.update_count;
        dst->transition_count = status.transition_count;
        dst->reject_count = status.reject_count;
        if (dst->active != 0u)
        {
            g_watch.runtime.active_controller_count++;
        }
        watch_runtime_add_entry(control_domain_name(status.domain),
                                RUNTIME_INSTANCE_KIND_GROUP,
                                (status.state == CONTROL_STATE_FAULT) ?
                                    RUNTIME_INSTANCE_STATE_FAULT :
                                    ((dst->active != 0u) ?
                                         RUNTIME_INSTANCE_STATE_ACTIVE :
                                         RUNTIME_INSTANCE_STATE_DISABLED),
                                (uint16_t)status.domain,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }
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

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
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
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
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
#endif

#if WATCH_ENABLE_SHOOT_RM
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
        g_watch.shoot.fric_current_cmd[i] =
            LowCmdGetCurrent(MotorIdRange(Motor8, i, 4u));
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
#endif

#if WATCH_ENABLE_ARM_J0_UNITREE
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
    g_watch.arm_j0_unitree.cmd_speed_deg_s = watch_rad_to_deg(state->cmd_output_speed_rad_s);
    g_watch.arm_j0_unitree.cmd_kd = state->cmd_output_kd;
    g_watch.arm_j0_unitree.torque_nm = state->torque_nm;
    g_watch.arm_j0_unitree.joint_speed_deg_s = watch_rad_to_deg(state->joint_speed_rad_s);
    g_watch.arm_j0_unitree.joint_position_deg = watch_rad_to_deg(state->joint_position_rad);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static fp32 watch_wheelleg_dir_sign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static void watch_copy_wheelleg_mit_motor(watch_wheelleg_mit_motor_t *out,
                                          uint8_t raw_id,
                                          fp32 zero_rad,
                                          int8_t dir,
                                          uint8_t use_rel_position)
{
    MotorCmd cmd;
    MotorState fb;
    MotorId id;

    if (out == NULL)
    {
        return;
    }
    out->id = raw_id;
    if ((uint32_t)raw_id >= (uint32_t)MotorCount)
    {
        return;
    }

    id = (MotorId)raw_id;
    if (LowCmdGetMotor(id, &cmd) != 0u)
    {
        out->cmd_active = cmd.active;
        out->cmd_mode = cmd.mode;
        out->cmd_position_deg = watch_rad_to_deg(cmd.q);
        out->cmd_velocity_deg_s = watch_rad_to_deg(cmd.dq);
        out->cmd_kp = cmd.kp;
        out->cmd_kd = cmd.kd;
        out->cmd_torque_nm = cmd.tau;
    }

    if (LowStateGetMotor(id, &fb) != 0u)
    {
        out->online = fb.online;
        out->fb_bus = fb.bus;
        out->fb_rx_dlc = fb.rxDlc;
        out->fb_rx_data0 = fb.rxData0;
        out->fb_rx_data0_low4 = (uint8_t)(fb.rxData0 & 0x0Fu);
        out->fb_rx_data0_high4 = (uint8_t)(fb.rxData0 >> 4);
        out->fb_motor_id = fb.motorId;
        out->fb_state = fb.state;
        out->fb_rx_id = fb.rxId;
        out->fb_rx_count = fb.rxCount;
        out->fb_last_rx_tick_ms = fb.lastRxTick;
        out->fb_position_deg = watch_rad_to_deg(fb.q);
        out->fb_velocity_deg_s = watch_rad_to_deg(fb.dq);
        out->fb_torque_nm = fb.tauEst;
        if (use_rel_position != 0u)
        {
            const fp32 rel_position_rad = (fb.q - zero_rad) * watch_wheelleg_dir_sign(dir);
            out->rel_position_deg = watch_rad_to_deg(rel_position_rad);
        }
        if (out->cmd_active != 0u)
        {
            out->cmd_minus_fb_deg = watch_rad_to_deg(cmd.q - fb.q);
        }
    }
}

static void watch_copy_wheelleg_mit_foot_test(uint8_t side)
{
    watch_wheelleg_mit_foot_test_t *out;
    fp32 wheel_zero_rad = 0.0f;
    fp32 wheel_comp_rad = 0.0f;
    fp32 wheel_target_rad = 0.0f;

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
                                     &wheel_zero_rad,
                                     &out->wheel_dx_m,
                                     &wheel_comp_rad,
                                     &wheel_target_rad);
    out->wheel_zero_deg = watch_rad_to_deg(wheel_zero_rad);
    out->wheel_comp_deg = watch_rad_to_deg(wheel_comp_rad);
    out->wheel_target_deg = watch_rad_to_deg(wheel_target_rad);
}

static uint8_t watch_wheelleg_manual_enabled_by_switch(uint8_t chassis_sw)
{
    return (uint8_t)(control_input_switch_is_pos(chassis_sw,
                                                 g_config.manual_input.semantics.chassis_safe_pos) == 0u);
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
    g_watch.wheelleg_mit.manual_on = watch_wheelleg_manual_enabled_by_switch(chassis_sw);
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
        g_watch.wheelleg_mit.target_yaw_rate_radps = status.target_yaw_rate_radps;
        g_watch.wheelleg_mit.target_leg_length_m = status.target_leg_length_m;
        g_watch.wheelleg_mit.target_foot_x_m = status.target_foot_x_m;
        g_watch.wheelleg_mit.target_foot_y_m = status.target_foot_y_m;
        g_watch.wheelleg_mit.target_leg_theta_deg = watch_rad_to_deg(status.target_leg_theta_rad);
        g_watch.wheelleg_mit.pitch_deg = watch_rad_to_deg(status.pitch_rad);
        g_watch.wheelleg_mit.x_dot_mps = status.x_dot_mps;
        g_watch.wheelleg_mit.enabled =
            (uint8_t)(g_watch.wheelleg_mit.manual_on != 0u &&
                      (status.fault_flags == WHEELLEG_FAULT_NONE ||
                       status.controller_active != 0u));

        g_watch.wheelleg_mit.leg[0].length_m = status.leg_length_m[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].length_m = status.leg_length_m[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.wheelleg_mit.leg[1].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.wheelleg_mit.leg[0].alpha_deg = watch_rad_to_deg(status.leg_alpha_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.wheelleg_mit.leg[1].alpha_deg = watch_rad_to_deg(status.leg_alpha_rad[WHEELLEG_SIDE_RIGHT]);
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

static void watch_wheelleg_run_capture_reset(void)
{
    memset(&s_wheelleg_run_capture, 0, sizeof(s_wheelleg_run_capture));
}

static void watch_wheelleg_run_capture_range(fp32 value, fp32 *min_value, fp32 *max_value, uint8_t first)
{
    if (first != 0u)
    {
        *min_value = value;
        *max_value = value;
        return;
    }
    if (value < *min_value)
    {
        *min_value = value;
    }
    if (value > *max_value)
    {
        *max_value = value;
    }
}

static void watch_wheelleg_run_capture_update(void)
{
    const uint8_t active =
        (uint8_t)((g_watch.wheelleg_mit.profile_on != 0u &&
                   g_watch.wheelleg_mit.status_valid != 0u &&
                   g_watch.wheelleg_mit.controller_active != 0u)
                      ? 1u
                      : 0u);
    watch_wheelleg_run_capture_t *capture = &s_wheelleg_run_capture;
    uint8_t first_sample;
    fp32 target_v;
    fp32 target_yaw_rate;
    fp32 x_dot;
    fp32 pitch;
    fp32 lqr_pitch_gyro;
    fp32 yaw_gyro;
    fp32 wheel_sum;
    fp32 wheel_diff;
    fp32 lqr_v_err;
    fp32 lqr_x;
    fp32 lqr_pitch_err;
    fp32 lqr_left_pitch_err;

    if (active == 0u)
    {
        capture->active = 0u;
        return;
    }

    if (capture->active == 0u)
    {
        watch_wheelleg_run_capture_reset();
        capture->active = 1u;
    }

    first_sample = (capture->sample_count == 0u) ? 1u : 0u;
    target_v = g_watch.diag.target_v_mps;
    target_yaw_rate = g_watch.diag.target_yaw_rate_radps;
    x_dot = g_watch.diag.x_dot_mps;
    pitch = g_watch.diag.pitch_deg;
    lqr_pitch_gyro = g_watch.diag.lqr_pitch_gyro_deg_s;
    yaw_gyro = g_watch.imu.gyro_dps[INS_GYRO_Z_ADDRESS_OFFSET];
    wheel_sum = g_watch.diag.left_wheel_cmd_torque_nm + g_watch.diag.right_wheel_cmd_torque_nm;
    wheel_diff = g_watch.diag.right_wheel_cmd_torque_nm - g_watch.diag.left_wheel_cmd_torque_nm;
    lqr_v_err = g_watch.diag.lqr_v_err_mps;
    lqr_x = g_watch.diag.lqr_x_m;
    lqr_pitch_err = g_watch.diag.lqr_pitch_err_deg;
    lqr_left_pitch_err = g_watch.diag.lqr_left_pitch_err_deg;

    capture->sample_count++;
    capture->valid = 1u;
    capture->target_v_sum += target_v;
    if (watch_abs_fp32(target_v) > capture->target_v_abs_max)
    {
        capture->target_v_abs_max = watch_abs_fp32(target_v);
    }
    capture->target_yaw_rate_sum += target_yaw_rate;
    if (watch_abs_fp32(target_yaw_rate) > capture->target_yaw_rate_abs_max)
    {
        capture->target_yaw_rate_abs_max = watch_abs_fp32(target_yaw_rate);
    }
    capture->x_dot_sum += x_dot;
    watch_wheelleg_run_capture_range(x_dot, &capture->x_dot_min, &capture->x_dot_max, first_sample);
    capture->pitch_sum += pitch;
    watch_wheelleg_run_capture_range(pitch, &capture->pitch_min, &capture->pitch_max, first_sample);
    capture->lqr_pitch_gyro_sum += lqr_pitch_gyro;
    watch_wheelleg_run_capture_range(lqr_pitch_gyro,
                                     &capture->lqr_pitch_gyro_min,
                                     &capture->lqr_pitch_gyro_max,
                                     first_sample);
    capture->yaw_gyro_sum += yaw_gyro;
    if (watch_abs_fp32(yaw_gyro) > capture->yaw_gyro_abs_max)
    {
        capture->yaw_gyro_abs_max = watch_abs_fp32(yaw_gyro);
    }
    capture->wheel_sum_sum += wheel_sum;
    if (watch_abs_fp32(wheel_sum) > capture->wheel_sum_abs_max)
    {
        capture->wheel_sum_abs_max = watch_abs_fp32(wheel_sum);
    }
    capture->wheel_diff_sum += wheel_diff;
    if (watch_abs_fp32(wheel_diff) > capture->wheel_diff_abs_max)
    {
        capture->wheel_diff_abs_max = watch_abs_fp32(wheel_diff);
    }
    capture->lqr_v_err_sum += lqr_v_err;
    watch_wheelleg_run_capture_range(lqr_v_err,
                                     &capture->lqr_v_err_min,
                                     &capture->lqr_v_err_max,
                                     first_sample);
    capture->lqr_x_sum += lqr_x;
    watch_wheelleg_run_capture_range(lqr_x,
                                     &capture->lqr_x_min,
                                     &capture->lqr_x_max,
                                     first_sample);
    capture->lqr_pitch_err_sum += lqr_pitch_err;
    watch_wheelleg_run_capture_range(lqr_pitch_err,
                                     &capture->lqr_pitch_err_min,
                                     &capture->lqr_pitch_err_max,
                                     first_sample);
    capture->lqr_left_pitch_err_sum += lqr_left_pitch_err;
    watch_wheelleg_run_capture_range(lqr_left_pitch_err,
                                     &capture->lqr_left_pitch_err_min,
                                     &capture->lqr_left_pitch_err_max,
                                     first_sample);
}

static void watch_wheelleg_run_capture_copy(void)
{
    const watch_wheelleg_run_capture_t *capture = &s_wheelleg_run_capture;
    fp32 inv_count;

    g_watch.diag.run_capture_active = capture->active;
    g_watch.diag.run_capture_valid = capture->valid;
    g_watch.diag.run_sample_count = capture->sample_count;

    if (capture->valid == 0u || capture->sample_count == 0u)
    {
        return;
    }

    inv_count = 1.0f / (fp32)capture->sample_count;
    g_watch.diag.run_target_v_avg_mps = capture->target_v_sum * inv_count;
    g_watch.diag.run_target_v_abs_max_mps = capture->target_v_abs_max;
    g_watch.diag.run_target_yaw_rate_avg_radps = capture->target_yaw_rate_sum * inv_count;
    g_watch.diag.run_target_yaw_rate_abs_max_radps = capture->target_yaw_rate_abs_max;
    g_watch.diag.run_x_dot_avg_mps = capture->x_dot_sum * inv_count;
    g_watch.diag.run_x_dot_min_mps = capture->x_dot_min;
    g_watch.diag.run_x_dot_max_mps = capture->x_dot_max;
    g_watch.diag.run_pitch_avg_deg = capture->pitch_sum * inv_count;
    g_watch.diag.run_pitch_min_deg = capture->pitch_min;
    g_watch.diag.run_pitch_max_deg = capture->pitch_max;
    g_watch.diag.run_lqr_pitch_gyro_avg_deg_s = capture->lqr_pitch_gyro_sum * inv_count;
    g_watch.diag.run_lqr_pitch_gyro_min_deg_s = capture->lqr_pitch_gyro_min;
    g_watch.diag.run_lqr_pitch_gyro_max_deg_s = capture->lqr_pitch_gyro_max;
    g_watch.diag.run_yaw_gyro_avg_deg_s = capture->yaw_gyro_sum * inv_count;
    g_watch.diag.run_yaw_gyro_abs_max_deg_s = capture->yaw_gyro_abs_max;
    g_watch.diag.run_wheel_sum_avg_nm = capture->wheel_sum_sum * inv_count;
    g_watch.diag.run_wheel_sum_abs_max_nm = capture->wheel_sum_abs_max;
    g_watch.diag.run_wheel_diff_avg_nm = capture->wheel_diff_sum * inv_count;
    g_watch.diag.run_wheel_diff_abs_max_nm = capture->wheel_diff_abs_max;
    g_watch.diag.run_wheel_balance_avg_nm = g_watch.diag.run_wheel_diff_avg_nm;
    g_watch.diag.run_wheel_balance_abs_max_nm = g_watch.diag.run_wheel_diff_abs_max_nm;
    g_watch.diag.run_wheel_turn_avg_nm = g_watch.diag.run_wheel_sum_avg_nm;
    g_watch.diag.run_wheel_turn_abs_max_nm = g_watch.diag.run_wheel_sum_abs_max_nm;
    g_watch.diag.run_lqr_v_err_avg_mps = capture->lqr_v_err_sum * inv_count;
    g_watch.diag.run_lqr_v_err_min_mps = capture->lqr_v_err_min;
    g_watch.diag.run_lqr_v_err_max_mps = capture->lqr_v_err_max;
    g_watch.diag.run_lqr_x_avg_m = capture->lqr_x_sum * inv_count;
    g_watch.diag.run_lqr_x_min_m = capture->lqr_x_min;
    g_watch.diag.run_lqr_x_max_m = capture->lqr_x_max;
    g_watch.diag.run_lqr_pitch_err_avg_deg = capture->lqr_pitch_err_sum * inv_count;
    g_watch.diag.run_lqr_pitch_err_min_deg = capture->lqr_pitch_err_min;
    g_watch.diag.run_lqr_pitch_err_max_deg = capture->lqr_pitch_err_max;
    g_watch.diag.run_lqr_left_pitch_err_avg_deg = capture->lqr_left_pitch_err_sum * inv_count;
    g_watch.diag.run_lqr_left_pitch_err_min_deg = capture->lqr_left_pitch_err_min;
    g_watch.diag.run_lqr_left_pitch_err_max_deg = capture->lqr_left_pitch_err_max;
}
#endif

static void watch_copy_diag(void)
{
    sdlog_stats_t sd_stats = {0};
    memset(&g_watch.diag, 0, sizeof(g_watch.diag));

    g_watch.diag.offline_need_geometry = 1u;
    g_watch.diag.offline_need_mass_inertia = 1u;
    g_watch.diag.offline_need_com_inertia = 1u;
    g_watch.diag.offline_need_motor_limits = 0u;
    g_watch.diag.offline_need_lqr_model = 1u;
    g_watch.diag.offline_need_stage_k = 1u;
    g_watch.diag.offline_need_qr_weights = 1u;
    g_watch.diag.offline_need_sign_table = 0u;

    g_watch.diag.power_need_imu_sign = 0u;
    g_watch.diag.power_need_motor_online = 0u;
    g_watch.diag.power_need_joint_zero_dir = 0u;
    g_watch.diag.power_need_wheel_dir = 0u;
    g_watch.diag.power_need_fk_leg_state = 1u;
    g_watch.diag.power_need_jacobian_dir = 1u;
    g_watch.diag.power_need_observer_speed = 1u;
    g_watch.diag.power_need_lqr_response = 1u;

    sdlog_get_stats(&sd_stats);
    g_watch.diag.live_sdlog_active = sd_stats.active;
    g_watch.diag.live_imu_online =
        (uint8_t)((toe_is_error(BOARD_GYRO_TOE) == 0u && toe_is_error(BOARD_ACCEL_TOE) == 0u) ? 1u : 0u);
    g_watch.diag.pitch_deg = g_watch.imu.angle_deg[INS_PITCH_ADDRESS_OFFSET];
    g_watch.diag.pitch_gyro_deg_s = g_watch.imu.gyro_dps[INS_GYRO_Y_ADDRESS_OFFSET];

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    {
        wheelleg_debug_t wheelleg_debug = {0};
        const uint8_t wheelleg_debug_valid =
            (robot_profile_is_wheelleg_mit() != 0u) ? wheelleg_debug_read(&wheelleg_debug) : 0u;

        g_watch.diag.live_profile_on = g_watch.wheelleg_mit.profile_on;
        g_watch.diag.live_manual_on = g_watch.wheelleg_mit.manual_on;
        g_watch.diag.live_enabled = g_watch.wheelleg_mit.enabled;
        g_watch.diag.live_control_stage = (uint8_t)g_config.wheelleg_mit.control_stage;
        g_watch.diag.live_mode = g_watch.wheelleg_mit.mode;
        g_watch.diag.live_status_valid = g_watch.wheelleg_mit.status_valid;
        g_watch.diag.live_state_valid = g_watch.wheelleg_mit.state_valid;
        g_watch.diag.live_fault_flags = g_watch.wheelleg_mit.fault_flags;
        g_watch.diag.live_lqr_debug_valid = wheelleg_debug_valid;
        g_watch.diag.live_all_mit_online =
            (uint8_t)((g_watch.wheelleg_mit.joint[0].online != 0u &&
                       g_watch.wheelleg_mit.joint[1].online != 0u &&
                       g_watch.wheelleg_mit.joint[2].online != 0u &&
                       g_watch.wheelleg_mit.joint[3].online != 0u &&
                       g_watch.wheelleg_mit.wheel[0].online != 0u &&
                       g_watch.wheelleg_mit.wheel[1].online != 0u)
                          ? 1u
                          : 0u);

        g_watch.diag.lqr_wheel_scale = g_config.wheelleg_mit.lqr_wheel_torque_scale;
        g_watch.diag.lqr_hip_scale = g_config.wheelleg_mit.lqr_hip_torque_scale;
        g_watch.diag.max_wheel_torque_nm = g_config.wheelleg_mit.max_wheel_torque_nm;
        g_watch.diag.max_joint_torque_nm = g_config.wheelleg_mit.max_joint_torque_nm;

        g_watch.diag.target_v_mps = g_watch.wheelleg_mit.target_v_mps;
        g_watch.diag.target_yaw_rate_radps = g_watch.wheelleg_mit.target_yaw_rate_radps;
        g_watch.diag.target_leg_length_m = g_watch.wheelleg_mit.target_leg_length_m;
        g_watch.diag.target_foot_x_m = g_watch.wheelleg_mit.target_foot_x_m;
        g_watch.diag.target_foot_y_m = g_watch.wheelleg_mit.target_foot_y_m;
        g_watch.diag.target_leg_theta_deg = g_watch.wheelleg_mit.target_leg_theta_deg;

        if (g_watch.wheelleg_mit.status_valid != 0u)
        {
            g_watch.diag.pitch_deg = g_watch.wheelleg_mit.pitch_deg;
        }
        g_watch.diag.x_dot_mps = g_watch.wheelleg_mit.x_dot_mps;
        g_watch.diag.left_leg_length_m = g_watch.wheelleg_mit.leg[0].length_m;
        g_watch.diag.right_leg_length_m = g_watch.wheelleg_mit.leg[1].length_m;
        g_watch.diag.left_leg_theta_deg = g_watch.wheelleg_mit.leg[0].theta_deg;
        g_watch.diag.right_leg_theta_deg = g_watch.wheelleg_mit.leg[1].theta_deg;
        g_watch.diag.left_front_rel_deg = g_watch.wheelleg_mit.joint[0].rel_position_deg;
        g_watch.diag.left_back_rel_deg = g_watch.wheelleg_mit.joint[1].rel_position_deg;
        g_watch.diag.right_front_rel_deg = g_watch.wheelleg_mit.joint[2].rel_position_deg;
        g_watch.diag.right_back_rel_deg = g_watch.wheelleg_mit.joint[3].rel_position_deg;
        g_watch.diag.left_wheel_vel_deg_s = g_watch.wheelleg_mit.wheel[0].fb_velocity_deg_s;
        g_watch.diag.right_wheel_vel_deg_s = g_watch.wheelleg_mit.wheel[1].fb_velocity_deg_s;
        g_watch.diag.left_wheel_cmd_torque_nm = g_watch.wheelleg_mit.wheel[0].cmd_torque_nm;
        g_watch.diag.right_wheel_cmd_torque_nm = g_watch.wheelleg_mit.wheel[1].cmd_torque_nm;
        g_watch.diag.wheel_balance_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm - g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.wheel_turn_cmd_nm =
            g_watch.diag.right_wheel_cmd_torque_nm + g_watch.diag.left_wheel_cmd_torque_nm;
        g_watch.diag.left_wheel_fb_torque_nm = g_watch.wheelleg_mit.wheel[0].fb_torque_nm;
        g_watch.diag.right_wheel_fb_torque_nm = g_watch.wheelleg_mit.wheel[1].fb_torque_nm;
        g_watch.diag.left_support_force_n = g_watch.wheelleg_mit.leg[0].support_force_n;
        g_watch.diag.right_support_force_n = g_watch.wheelleg_mit.leg[1].support_force_n;
        g_watch.diag.lqr_pitch_err_deg =
            g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_right_rad);
        g_watch.diag.lqr_left_pitch_err_deg =
            -g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_left_rad);

        if (wheelleg_debug_valid != 0u)
        {
            g_watch.diag.left_hip_torque_nm = wheelleg_debug.lqr.output[3];
            g_watch.diag.right_hip_torque_nm = wheelleg_debug.lqr.output[2];
            g_watch.diag.lqr_theta_err_deg = watch_rad_to_deg(wheelleg_debug.lqr.error[0]);
            g_watch.diag.lqr_dtheta_deg_s = watch_rad_to_deg(wheelleg_debug.lqr.error[1]);
            g_watch.diag.lqr_x_m = wheelleg_debug.lqr.error[2];
            g_watch.diag.lqr_v_err_mps = wheelleg_debug.lqr.error[3];
            g_watch.diag.lqr_pitch_err_deg = watch_rad_to_deg(wheelleg_debug.lqr.error[4]);
            g_watch.diag.lqr_left_pitch_err_deg =
                -g_watch.diag.pitch_deg - watch_rad_to_deg(g_config.wheelleg_mit.pitch_balance_offset_left_rad);
            g_watch.diag.lqr_pitch_gyro_deg_s = watch_rad_to_deg(wheelleg_debug.lqr.error[5]);
            g_watch.diag.lqr_right_output_nm = wheelleg_debug.lqr.output[0];
            g_watch.diag.lqr_left_output_nm = wheelleg_debug.lqr.output[1];
        }

        watch_wheelleg_run_capture_update();
        watch_wheelleg_run_capture_copy();
    }
#endif
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared watch can link even if a target doesn't export them.
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
__weak uint32_t gimbal_high_water;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
__weak uint32_t chassis_high_water;
#endif
__weak uint32_t detect_task_stack;
__weak uint32_t calibrate_task_stack;
#endif

static void watch_copy_rtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_watch.rtos.heap_free = heap_get_free();
    g_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_watch.rtos.watch_update_count++;
    g_watch.rtos.watch_update_tick_ms = HAL_GetTick();
    g_watch.rtos.scheduler_state = (uint32_t)xTaskGetSchedulerState();
    g_watch.rtos.task_count = (uint32_t)uxTaskGetNumberOfTasks();
    current_task = xTaskGetCurrentTaskHandle();
    g_watch.rtos.current_task_handle = (uint32_t)current_task;
    current_name = pcTaskGetTaskName(NULL);
    memset(g_watch.rtos.current_task_name, 0, sizeof(g_watch.rtos.current_task_name));
    if (current_name != NULL)
    {
        (void)strncpy(g_watch.rtos.current_task_name, current_name, sizeof(g_watch.rtos.current_task_name) - 1u);
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = gimbal_high_water;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = chassis_high_water;
#endif
    g_watch.rtos.stack_detect = detect_task_stack;
    g_watch.rtos.stack_calibrate = calibrate_task_stack;
#else
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = 0u;
#endif
    g_watch.rtos.stack_detect = 0u;
    g_watch.rtos.stack_calibrate = 0u;
#endif
}
