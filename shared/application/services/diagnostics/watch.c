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
#include "bsp_rc.h"
#include "motor_instance.h"
#include "sdcard.h"
#include "sdlog.h"
#include "shoot_state.h"
#include "host_link_task.h"
#include "robot_device_config.h"
#include "robot_task_profile.h"
#include "robot_mode.h"
#include "rt_profiler.h"
#include "wheelleg_mit_task.h"
#include "wheelleg_msg.h"

watch_t g_watch;

typedef struct
{
    uint8_t used;
    uint8_t module;
    uint8_t create_state;
    uint8_t reserved0;
    uint16_t create_fail_count;
    uint16_t reserved1;
    uint32_t thread_handle;
    uint32_t create_attempt_count;
    const char *name;
} watch_task_module_create_slot_t;

static watch_task_module_create_slot_t s_task_module_create[WATCH_RUNTIME_MAX_TASK_MODULES];

static const char *const s_watch_friction_motor_names[4u] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static MotorId s_watch_friction_motor_ids[4u] = {MotorCount, MotorCount, MotorCount, MotorCount};
static uint8_t s_watch_friction_motor_ids_ready = 0u;

static void watch_prepare_motor_ids(void)
{
    if (s_watch_friction_motor_ids_ready != 0u)
    {
        return;
    }

    (void)motor_instance_resolve_actuator_ids(s_watch_friction_motor_names,
                                              4u,
                                              s_watch_friction_motor_ids,
                                              4u);
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (s_watch_friction_motor_ids[i] == MotorCount)
        {
            s_watch_friction_motor_ids[i] = MotorIdRange(Motor8, i, 4u);
        }
    }
    s_watch_friction_motor_ids_ready = 1u;
}

static MotorId watch_friction_motor_id(uint8_t index)
{
    index = (uint8_t)(index & 0x03u);
    if (s_watch_friction_motor_ids_ready == 0u)
    {
        return MotorIdRange(Motor8, index, 4u);
    }

    return s_watch_friction_motor_ids[index];
}

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

static manual_input_state_t rc_snapshot;
static const manual_input_state_t *rc_src;
static const fp32 *ins_quat_src;
static const fp32 *ins_angle_src;
static const fp32 *ins_gyro_src;
static const fp32 *ins_accel_src;

static void watch_copy_rc(void);
static void watch_copy_newrc(void);
#if WATCH_ENABLE_COMM_COPY
static void watch_copy_comm(void);
#endif
#if WATCH_ENABLE_RUNTIME_COPY
static void watch_copy_runtime(void);
#endif
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
#if WATCH_ENABLE_RUNTIME_COPY
static void watch_runtime_add_entry(const char *name,
                                    runtime_instance_kind_e kind,
                                    runtime_instance_state_e state,
                                    uint16_t source_id,
                                    uint16_t source_index,
                                    uint16_t parent_index);
static runtime_instance_state_e watch_runtime_device_state(const robot_config_device_t *device);
static const watch_task_module_create_slot_t *watch_task_module_create_find(uint8_t module);
#endif
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
#if WATCH_ENABLE_RUNTIME_COPY
    {WATCH_BLOCK_RUNTIME, "runtime.instances", &g_watch.runtime, sizeof(g_watch.runtime), watch_block_active_always},
#endif
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
#if WATCH_ENABLE_COMM_COPY
    {WATCH_BLOCK_COMM, "common.comm", &g_watch.comm, sizeof(g_watch.comm), watch_block_active_always},
#endif
};

static watch_block_desc_t g_watch_active_blocks[WATCH_BLOCK_COUNT];

#include "watch_core_helpers.inc"

#include "watch_runtime_copy.inc"

#include "watch_state_copy.inc"

#include "watch_wheelleg_copy.inc"

#include "watch_diag_copy.inc"
