/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_task.h"
#include "manual_input.h"

#include "FreeRTOS.h"
#include "task.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "motor_config.h"
#include "arm_motor_table.h"

#include "arm_motion.h"
#include "can_mit_motor_driver.h"
#include "unitree_motor_driver.h"

#include <string.h>

#define ARM_J0_INDEX 0u
#define ARM_J0_CURRENT_DEFAULT 2000

volatile uint8_t g_arm_deadman_hold_ctrl = 0u;
volatile fp32 g_arm_key_speed_scale = 1.0f;
volatile fp32 g_arm_key_kd = 1.0f;
volatile int16_t g_arm_j0_current = ARM_J0_CURRENT_DEFAULT;

static volatile uint8_t g_arm_can1_yaw_override_active = 0u;
static uint8_t g_arm_mit_armed = 0u;
static arm_motor_feedback_t g_arm_feedback[ARM_MOTOR_COUNT];
static can_mit_motor_feedback_t g_arm_mit_feedback[ARM_MOTOR_COUNT];
static arm_j0_unitree_state_t g_arm_j0_unitree_state;
static uint32_t g_arm_j0_unitree_last_step_tick_ms = 0u;
static fp32 g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
static fp32 g_arm_j0_unitree_cmd_output_kd = 0.0f;

static fp32 arm_motion_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max);
static uint8_t arm_key_active(const arm_motor_entry_t *entry, uint16_t key_mask);
static uint8_t arm_any_keys_active(uint16_t key_mask);
static uint8_t arm_any_mit_keys_active(uint16_t key_mask);
static uint16_t arm_mit_std_id(const arm_motor_entry_t *entry);
static const can_mit_motor_limits_t *arm_mit_limits(const arm_motor_entry_t *entry);
static void arm_copy_mit_feedback(uint8_t index);
static void arm_send_mit_stop_all(void);
static void arm_refresh_j0_feedback(void);
static fp32 arm_j0_unitree_ratio_safe(const arm_j0_unitree_config_t *cfg);
static fp32 arm_j0_unitree_output_to_rotor_speed(const arm_j0_unitree_config_t *cfg, fp32 output_speed_rad_s);
static fp32 arm_j0_unitree_output_to_rotor_kd(const arm_j0_unitree_config_t *cfg, fp32 output_kd);
static void arm_build_j0_unitree_config(unitree_motor_config_t *out, const arm_j0_unitree_config_t *cfg);
static void arm_sync_j0_unitree_state(void);
static void arm_step_j0_unitree(uint8_t move_key, uint8_t reverse, uint8_t ctrl_held);
static void arm_step_j0(const arm_motor_entry_t *entry, uint16_t key_mask);
static void arm_step_mit(uint16_t key_mask);

static fp32 arm_motion_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    if (x > x_max)
    {
        return x_max;
    }
    return x;
}

static uint8_t arm_key_active(const arm_motor_entry_t *entry, uint16_t key_mask)
{
    if (entry == NULL)
    {
        return 0u;
    }

    return ((key_mask & entry->key_mask) != 0u) ? 1u : 0u;
}

static uint8_t arm_any_keys_active(uint16_t key_mask)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        if (arm_key_active(&g_arm_motor_table[i], key_mask) != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t arm_any_mit_keys_active(uint16_t key_mask)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }
        if (arm_key_active(entry, key_mask) != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint16_t arm_mit_std_id(const arm_motor_entry_t *entry)
{
    if (entry == NULL)
    {
        return 0u;
    }

    return motor_cfg_can_id(&entry->node);
}

static const can_mit_motor_limits_t *arm_mit_limits(const arm_motor_entry_t *entry)
{
    if (entry == NULL)
    {
        return NULL;
    }

    return motor_cfg_mit_limits(&entry->node);
}

static void arm_copy_mit_feedback(uint8_t index)
{
    const can_mit_motor_feedback_t *src;
    arm_motor_feedback_t *dst;

    if (index >= ARM_MOTOR_COUNT)
    {
        return;
    }

    src = &g_arm_mit_feedback[index];
    dst = &g_arm_feedback[index];

    dst->online = src->online;
    dst->rx_dlc = src->rx_dlc;
    dst->rx_id = src->rx_id;
    dst->rx_count = src->rx_count;
    dst->last_rx_tick = src->last_rx_tick;
    dst->position = src->position;
    dst->velocity = src->velocity;
    dst->torque = src->torque;
}

static void arm_send_mit_stop_all(void)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }

        can_mit_motor_send_stop(entry->bus, arm_mit_std_id(entry), arm_mit_limits(entry));
    }
}

static fp32 arm_j0_unitree_ratio_safe(const arm_j0_unitree_config_t *cfg)
{
    if (cfg == NULL || cfg->reduction_ratio <= 0.0f)
    {
        return 1.0f;
    }

    return cfg->reduction_ratio;
}

static fp32 arm_j0_unitree_output_to_rotor_speed(const arm_j0_unitree_config_t *cfg, fp32 output_speed_rad_s)
{
    return output_speed_rad_s * arm_j0_unitree_ratio_safe(cfg);
}

static fp32 arm_j0_unitree_output_to_rotor_kd(const arm_j0_unitree_config_t *cfg, fp32 output_kd)
{
    const fp32 ratio = arm_j0_unitree_ratio_safe(cfg);

    return output_kd / (ratio * ratio);
}

static void arm_build_j0_unitree_config(unitree_motor_config_t *out, const arm_j0_unitree_config_t *cfg)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (cfg == NULL)
    {
        return;
    }

    out->enable = cfg->enable;
    out->rs485_port = cfg->rs485_port;
    out->motor_id = cfg->motor_id;
    out->baudrate = cfg->baudrate;
    out->rx_timeout_ms = cfg->rx_timeout_ms;
}

static void arm_sync_j0_unitree_state(void)
{
    const arm_j0_unitree_config_t *cfg = &g_config.arm_j0_unitree;
    const unitree_motor_state_t *state = unitree_motor_get_state();

    if (state == NULL)
    {
        (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
        return;
    }

    g_arm_j0_unitree_state.enabled = cfg->enable;
    g_arm_j0_unitree_state.rs485_port = cfg->rs485_port;
    g_arm_j0_unitree_state.motor_id = (state->motor_id != 0u) ? state->motor_id : cfg->motor_id;
    g_arm_j0_unitree_state.online = state->online;
    g_arm_j0_unitree_state.last_mode = state->last_mode;
    g_arm_j0_unitree_state.motor_error = state->motor_error;
    g_arm_j0_unitree_state.motor_temp = state->motor_temp;
    g_arm_j0_unitree_state.last_tx_status = state->last_tx_status;
    g_arm_j0_unitree_state.tx_count = state->tx_count;
    g_arm_j0_unitree_state.tx_fail_count = state->tx_fail_count;
    g_arm_j0_unitree_state.rx_frame_count = state->rx_frame_count;
    g_arm_j0_unitree_state.rx_crc_fail_count = state->rx_crc_fail_count;
    g_arm_j0_unitree_state.rx_parse_error_count = state->rx_parse_error_count;
    g_arm_j0_unitree_state.last_rx_tick_ms = state->last_rx_tick_ms;
    g_arm_j0_unitree_state.cmd_output_speed_rad_s = g_arm_j0_unitree_cmd_output_speed_rad_s;
    g_arm_j0_unitree_state.cmd_output_kd = g_arm_j0_unitree_cmd_output_kd;
    g_arm_j0_unitree_state.torque_nm = state->torque_nm;
    g_arm_j0_unitree_state.joint_speed_rad_s = state->joint_speed_rad_s;
    g_arm_j0_unitree_state.joint_position_rad = state->joint_position_rad;
}

static void arm_step_j0_unitree(uint8_t move_key, uint8_t reverse, uint8_t ctrl_held)
{
    const arm_j0_unitree_config_t *cfg = &g_config.arm_j0_unitree;
    const uint16_t period_ms = (cfg->control_period_ms == 0u) ? 5u : cfg->control_period_ms;
    const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t command_enabled = 0u;
    unitree_motor_config_t driver_cfg;
    unitree_motor_cmd_t cmd = {0};

    arm_build_j0_unitree_config(&driver_cfg, cfg);
    unitree_motor_refresh(&driver_cfg);

    if (cfg->enable == 0u)
    {
        unitree_motor_stop();
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
        g_arm_j0_unitree_last_step_tick_ms = now_ms;
        arm_sync_j0_unitree_state();
        return;
    }

    if ((g_arm_j0_unitree_last_step_tick_ms != 0u) &&
        ((now_ms - g_arm_j0_unitree_last_step_tick_ms) < period_ms))
    {
        arm_sync_j0_unitree_state();
        return;
    }

    g_arm_j0_unitree_last_step_tick_ms = now_ms;

    if (unitree_motor_configure(&driver_cfg) == 0u)
    {
        arm_sync_j0_unitree_state();
        return;
    }

    if (g_arm_deadman_hold_ctrl != 0u && ctrl_held == 0u)
    {
        cmd.kd = 0.0f;
    }
    else if (move_key != 0u)
    {
        const fp32 dir = (reverse != 0u) ? -1.0f : 1.0f;
        const fp32 output_speed = dir * cfg->key_speed_rad_s;

        cmd.speed_rad_s = arm_j0_unitree_output_to_rotor_speed(cfg, output_speed);
        cmd.kd = arm_j0_unitree_output_to_rotor_kd(cfg, cfg->drive_kd);
        g_arm_j0_unitree_cmd_output_speed_rad_s = output_speed;
        g_arm_j0_unitree_cmd_output_kd = cfg->drive_kd;
        command_enabled = 1u;
    }
    else
    {
        cmd.kd = arm_j0_unitree_output_to_rotor_kd(cfg, cfg->hold_kd);
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = cfg->hold_kd;
        command_enabled = 1u;
    }

    if (command_enabled == 0u)
    {
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
    }

    (void)unitree_motor_send_cmd(&driver_cfg, &cmd);
    arm_sync_j0_unitree_state();
}

static void arm_refresh_j0_feedback(void)
{
    arm_motor_feedback_t *feedback = &g_arm_feedback[ARM_J0_INDEX];

    if (g_config.arm_j0_unitree.enable != 0u)
    {
        const arm_j0_unitree_state_t *state;

        arm_sync_j0_unitree_state();
        state = &g_arm_j0_unitree_state;

        feedback->online = state->online;
        feedback->rx_dlc = 0u;
        feedback->rx_id = state->motor_id;
        feedback->rx_count = state->rx_frame_count;
        feedback->last_rx_tick = state->last_rx_tick_ms;
        feedback->position = state->joint_position_rad;
        feedback->velocity = state->joint_speed_rad_s;
        feedback->torque = state->torque_nm;
        return;
    }

    {
        const motor_measure_t *measure = get_yaw_gimbal_motor_measure_point();

        if (measure == NULL)
        {
            (void)memset(feedback, 0, sizeof(*feedback));
            return;
        }

        feedback->online = ((measure->ecd != 0u) ||
                            (measure->speed_rpm != 0) ||
                            (measure->given_current != 0) ||
                            (measure->temperate != 0u)) ? 1u : 0u;
        feedback->rx_dlc = 8u;
        feedback->rx_id = CAN_YAW_MOTOR_ID;
        feedback->rx_count = 0u;
        feedback->last_rx_tick = 0u;
        feedback->position = (fp32)measure->ecd;
        feedback->velocity = (fp32)measure->speed_rpm;
        feedback->torque = (fp32)measure->given_current;
    }
}

static void arm_step_j0(const arm_motor_entry_t *entry, uint16_t key_mask)
{
    const uint8_t ctrl_held = ((key_mask & KEY_PRESSED_OFFSET_CTRL) != 0u) ? 1u : 0u;
    const uint8_t reverse = ((key_mask & KEY_PRESSED_OFFSET_SHIFT) != 0u) ? 1u : 0u;
    const uint8_t move_key = arm_key_active(entry, key_mask);

    arm_step_j0_unitree(move_key, reverse, ctrl_held);

    if (g_config.arm_j0_unitree.enable != 0u)
    {
        g_arm_can1_yaw_override_active = 0u;
        actuator_cmd_set_yaw_current_can1(0);
        return;
    }

    if (g_arm_deadman_hold_ctrl != 0u && ctrl_held == 0u)
    {
        g_arm_can1_yaw_override_active = 0u;
        actuator_cmd_set_yaw_current_can1(0);
        return;
    }

    if (move_key != 0u)
    {
        const int16_t current_abs = (g_arm_j0_current >= 0) ? g_arm_j0_current : (int16_t)(-g_arm_j0_current);
        int16_t current = (reverse != 0u) ? (int16_t)(-current_abs) : current_abs;

        current = motor_cfg_limit_current_node(&g_config.motor.yaw, current);
        g_arm_can1_yaw_override_active = 1u;
        actuator_cmd_set_yaw_current_can1(current);
    }
    else
    {
        g_arm_can1_yaw_override_active = 0u;
        actuator_cmd_set_yaw_current_can1(0);
    }
}

static void arm_step_mit(uint16_t key_mask)
{
    const uint8_t ctrl_held = ((key_mask & KEY_PRESSED_OFFSET_CTRL) != 0u) ? 1u : 0u;
    const uint8_t reverse = ((key_mask & KEY_PRESSED_OFFSET_SHIFT) != 0u) ? 1u : 0u;
    const uint8_t any_keys = arm_any_keys_active(key_mask);
    const uint8_t dm_active = arm_any_mit_keys_active(key_mask);
    const uint8_t deadman = (g_arm_deadman_hold_ctrl != 0u) ?
        (((ctrl_held != 0u) && (any_keys != 0u)) ? 1u : 0u) :
        any_keys;
    uint32_t i;

    if (deadman == 0u)
    {
        if (g_arm_mit_armed != 0u)
        {
            arm_send_mit_stop_all();
        }

        g_arm_mit_armed = 0u;
        return;
    }

    if (dm_active != 0u && g_arm_mit_armed == 0u)
    {
        for (i = 0u; i < ARM_MOTOR_COUNT; i++)
        {
            const arm_motor_entry_t *entry = &g_arm_motor_table[i];

            if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
            {
                continue;
            }
            if (arm_mit_limits(entry) == NULL)
            {
                continue;
            }

            can_mit_motor_send_enable(entry->bus, arm_mit_std_id(entry));
        }
        g_arm_mit_armed = 1u;
    }
    else if (dm_active == 0u && g_arm_mit_armed != 0u)
    {
        arm_send_mit_stop_all();
        g_arm_mit_armed = 0u;
    }

    if (g_arm_mit_armed == 0u)
    {
        return;
    }

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        can_mit_motor_cmd_t cmd = {0};
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];
        const can_mit_motor_limits_t *limits;

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }

        limits = arm_mit_limits(entry);
        if (limits == NULL)
        {
            continue;
        }

        if (arm_key_active(entry, key_mask) != 0u)
        {
            const fp32 dir = (reverse != 0u) ? -1.0f : 1.0f;
            cmd.velocity = dir * (fp32)entry->direction * entry->key_speed_rad_s * g_arm_key_speed_scale;
            cmd.kd = arm_motion_clamp_fp32(g_arm_key_kd, 0.0f, limits->kd_max);
        }

        can_mit_motor_send_cmd(entry->bus, arm_mit_std_id(entry), limits, &cmd);
    }
}

void arm_motion_init(void)
{
    (void)memset(g_arm_feedback, 0, sizeof(g_arm_feedback));
    (void)memset(g_arm_mit_feedback, 0, sizeof(g_arm_mit_feedback));
    (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
    g_arm_can1_yaw_override_active = 0u;
    g_arm_mit_armed = 0u;
    g_arm_j0_unitree_last_step_tick_ms = 0u;
    g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
    g_arm_j0_unitree_cmd_output_kd = 0.0f;
    actuator_cmd_set_yaw_current_can1(0);
    unitree_motor_driver_init();
    arm_sync_j0_unitree_state();
    arm_refresh_j0_feedback();
}

void arm_motion_step_manual(uint16_t key_mask)
{
    arm_step_j0(&g_arm_motor_table[ARM_J0_INDEX], key_mask);
    arm_refresh_j0_feedback();
    arm_step_mit(key_mask);
}

const arm_motor_feedback_t *arm_motion_get_feedback(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return NULL;
    }

    return &g_arm_feedback[index];
}

uint8_t arm_motion_can1_yaw_override_active(void)
{
    return g_arm_can1_yaw_override_active;
}

uint8_t arm_motion_process_can_feedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }
        if (entry->bus != bus)
        {
            continue;
        }
        if (motor_cfg_feedback_id(&entry->node) != std_id)
        {
            continue;
        }

        if (can_mit_motor_update_feedback(std_id,
                                          entry->node.can_id,
                                          arm_mit_limits(entry),
                                          dlc,
                                          data,
                                          &g_arm_mit_feedback[i]) != 0u)
        {
            arm_copy_mit_feedback((uint8_t)i);
            return 1u;
        }
        return 0u;
    }

    return 0u;
}

const arm_j0_unitree_state_t *arm_motion_get_j0_unitree_state(void)
{
    arm_sync_j0_unitree_state();
    return &g_arm_j0_unitree_state;
}
