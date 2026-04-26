/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_task.h"

#include "cmsis_os.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "bsp_can.h"
#include "motor_config.h"
#include "remote_control.h"
#include "arm_motor_table.h"

typedef struct
{
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
} arm_mit_cmd_t;

#define ARM_J0_KEY_MASK KEY_PRESSED_OFFSET_G
#define ARM_J0_CURRENT_DEFAULT 2000

volatile uint8_t g_arm_deadman_hold_ctrl = 0u;
volatile fp32 g_arm_key_speed_scale = 1.0f;
volatile fp32 g_arm_key_kd = 1.0f;
volatile int16_t g_arm_j0_current = ARM_J0_CURRENT_DEFAULT;

static volatile uint8_t g_arm_can1_yaw_override_active = 0u;
static arm_motor_feedback_t g_arm_feedback[ARM_MOTOR_COUNT];

static uint32_t arm_float_to_uint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 arm_uint_to_float(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 arm_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max);
static uint8_t arm_any_dm_keys_active(uint16_t key_mask);
static uint8_t arm_any_keys_active(uint16_t key_mask);
static void arm_send_mit_cmd(uint8_t bus,
                             uint16_t std_id,
                             const arm_mit_cmd_t *cmd,
                             const arm_mit_limits_t *limits);
static void arm_send_mit_enable(uint8_t bus, uint16_t std_id);
static void arm_send_stop_all(void);
static void arm_update_feedback(uint8_t index, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

static uint32_t arm_float_to_uint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const fp32 span = x_max - x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    fp32 x_clamped = arm_clamp_fp32(x, x_min, x_max);

    if (span <= 0.0f)
    {
        return 0u;
    }

    return (uint32_t)(((x_clamped - x_min) * (fp32)max_int) / span + 0.5f);
}

static fp32 arm_uint_to_float(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const uint32_t max_int = (1u << bits) - 1u;
    const fp32 span = x_max - x_min;

    if (max_int == 0u || span <= 0.0f)
    {
        return x_min;
    }

    return ((fp32)x_int) * span / (fp32)max_int + x_min;
}

static fp32 arm_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
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

static uint8_t arm_any_dm_keys_active(uint16_t key_mask)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        if ((key_mask & g_arm_motor_table[i].key_mask) != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t arm_any_keys_active(uint16_t key_mask)
{
    if ((key_mask & ARM_J0_KEY_MASK) != 0u)
    {
        return 1u;
    }

    return arm_any_dm_keys_active(key_mask);
}

static void arm_send_mit_cmd(uint8_t bus,
                             uint16_t std_id,
                             const arm_mit_cmd_t *cmd,
                             const arm_mit_limits_t *limits)
{
    uint8_t data[8];
    uint32_t p_int;
    uint32_t v_int;
    uint32_t kp_int;
    uint32_t kd_int;
    uint32_t t_int;

    if (cmd == NULL || limits == NULL || std_id == 0u)
    {
        return;
    }

    p_int = arm_float_to_uint(cmd->position, -limits->position_max, limits->position_max, 16u);
    v_int = arm_float_to_uint(cmd->velocity, -limits->velocity_max, limits->velocity_max, 12u);
    kp_int = arm_float_to_uint(cmd->kp, 0.0f, limits->kp_max, 12u);
    kd_int = arm_float_to_uint(cmd->kd, 0.0f, limits->kd_max, 12u);
    t_int = arm_float_to_uint(cmd->torque, -limits->torque_max, limits->torque_max, 12u);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)p_int;
    data[2] = (uint8_t)(v_int >> 4);
    data[3] = (uint8_t)(((v_int & 0x0Fu) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)kp_int;
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0Fu) << 4) | (t_int >> 8));
    data[7] = (uint8_t)t_int;

    (void)bsp_can_tx(bus, std_id, data, 8u);
}

static void arm_send_mit_enable(uint8_t bus, uint16_t std_id)
{
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFCu};

    if (std_id == 0u)
    {
        return;
    }

    (void)bsp_can_tx(bus, std_id, data, 8u);
}

static void arm_send_stop_all(void)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        arm_mit_cmd_t stop_cmd = {0};
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];
        const uint16_t std_id = motor_cfg_can_id(&entry->node);

        arm_send_mit_cmd(entry->bus, std_id, &stop_cmd, &entry->mit_limits);
    }
}

static void arm_update_feedback(uint8_t index, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint8_t payload_off = 0u;
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;
    const arm_motor_entry_t *entry;
    arm_motor_feedback_t *feedback;

    if (index >= ARM_MOTOR_COUNT || data == NULL)
    {
        return;
    }

    entry = &g_arm_motor_table[index];
    feedback = &g_arm_feedback[index];

    if (dlc >= 6u && (data[0] == entry->node.can_id || data[0] == (uint8_t)std_id))
    {
        payload_off = 1u;
    }
    else if (dlc < 5u)
    {
        return;
    }

    p_int = ((uint32_t)data[payload_off + 0u] << 8) | (uint32_t)data[payload_off + 1u];
    v_int = ((uint32_t)data[payload_off + 2u] << 4) | ((uint32_t)data[payload_off + 3u] >> 4);
    t_int = (((uint32_t)data[payload_off + 3u] & 0x0Fu) << 8) | (uint32_t)data[payload_off + 4u];

    feedback->online = 1u;
    feedback->rx_dlc = dlc;
    feedback->rx_id = std_id;
    feedback->rx_count++;
    feedback->last_rx_tick = xTaskGetTickCount();
    feedback->position = arm_uint_to_float(p_int,
                                           -entry->mit_limits.position_max,
                                           entry->mit_limits.position_max,
                                           16u);
    feedback->velocity = arm_uint_to_float(v_int,
                                           -entry->mit_limits.velocity_max,
                                           entry->mit_limits.velocity_max,
                                           12u);
    feedback->torque = arm_uint_to_float(t_int,
                                         -entry->mit_limits.torque_max,
                                         entry->mit_limits.torque_max,
                                         12u);
}

const arm_motor_feedback_t *arm_get_feedback(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return (const arm_motor_feedback_t *)0;
    }

    return &g_arm_feedback[index];
}

uint8_t can_tx_allow_can1_yaw_override(void)
{
    return g_arm_can1_yaw_override_active;
}

uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];

        if (entry->bus != bus)
        {
            continue;
        }
        if (motor_cfg_feedback_id(&entry->node) != std_id)
        {
            continue;
        }

        arm_update_feedback((uint8_t)i, std_id, dlc, data);
        return 1u;
    }

    return 0u;
}

void arm_task(void const *argument)
{
    const RC_ctrl_t *rc = NULL;
    uint8_t was_mit_armed = 0u;

    (void)argument;
    rc = get_remote_control_point();

    for (;;)
    {
        uint16_t key_mask = 0u;
        uint8_t deadman = 0u;
        uint8_t dm_active = 0u;
        uint8_t reverse = 0u;
        uint32_t i;

        if (rc != NULL)
        {
            key_mask = rc->key.v;
        }

        dm_active = arm_any_dm_keys_active(key_mask);
        deadman = (g_arm_deadman_hold_ctrl != 0u) ?
            ((((key_mask & KEY_PRESSED_OFFSET_CTRL) != 0u) && (arm_any_keys_active(key_mask) != 0u)) ? 1u : 0u) :
            arm_any_keys_active(key_mask);
        reverse = ((key_mask & KEY_PRESSED_OFFSET_SHIFT) != 0u) ? 1u : 0u;

        if (deadman == 0u)
        {
            g_arm_can1_yaw_override_active = 0u;
            actuator_cmd_set_yaw_current_can1(0);

            if (was_mit_armed != 0u)
            {
                arm_send_stop_all();
            }

            was_mit_armed = 0u;
            osDelay(5u);
            continue;
        }

        if ((key_mask & ARM_J0_KEY_MASK) != 0u)
        {
            const int16_t current_abs = (g_arm_j0_current >= 0) ? g_arm_j0_current : (int16_t)(-g_arm_j0_current);
            const int16_t current = (reverse != 0u) ? (int16_t)(-current_abs) : current_abs;

            g_arm_can1_yaw_override_active = 1u;
            actuator_cmd_set_yaw_current_can1(current);
        }
        else
        {
            g_arm_can1_yaw_override_active = 0u;
            actuator_cmd_set_yaw_current_can1(0);
        }

        if (dm_active != 0u && was_mit_armed == 0u)
        {
            for (i = 0u; i < ARM_MOTOR_COUNT; i++)
            {
                const arm_motor_entry_t *entry = &g_arm_motor_table[i];
                const uint16_t std_id = motor_cfg_can_id(&entry->node);

                arm_send_mit_enable(entry->bus, std_id);
            }
            was_mit_armed = 1u;
        }
        else if (dm_active == 0u && was_mit_armed != 0u)
        {
            arm_send_stop_all();
            was_mit_armed = 0u;
        }

        if (was_mit_armed != 0u)
        {
            for (i = 0u; i < ARM_MOTOR_COUNT; i++)
            {
                arm_mit_cmd_t cmd = {0};
                const arm_motor_entry_t *entry = &g_arm_motor_table[i];
                const uint16_t std_id = motor_cfg_can_id(&entry->node);

                if ((key_mask & entry->key_mask) != 0u)
                {
                    const fp32 dir = (reverse != 0u) ? -1.0f : 1.0f;
                    cmd.velocity = dir * (fp32)entry->direction * entry->key_speed_rad_s * g_arm_key_speed_scale;
                    cmd.kd = arm_clamp_fp32(g_arm_key_kd, 0.0f, entry->mit_limits.kd_max);
                }

                arm_send_mit_cmd(entry->bus, std_id, &cmd, &entry->mit_limits);
            }
        }

        osDelay(5u);
    }
}
