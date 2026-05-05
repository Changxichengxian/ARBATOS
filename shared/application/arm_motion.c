/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：按键判断、关节装配表读取、MIT/Unitree 协议辅助函数。
 * - 中段：MIT 反馈同步、停止命令、J0 Unitree 状态同步。
 * - 后段：手动步进各关节，处理 CAN/RS485 反馈，并给 CAN 发送任务补特殊轴。
 * - 入口：arm_motion_step_manual() 每周期执行手动控制。
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
#include "mit_motor.h"
#include "unitree_motor_driver.h"

#include <string.h>

#define ARM_J0_INDEX 0u
#define ARM_J0_CURRENT_DEFAULT 2000

volatile uint8_t g_arm_deadman_hold_ctrl = 0u;
volatile fp32 g_arm_key_speed_scale = 1.0f;
volatile fp32 g_arm_key_kd = 1.0f;
volatile int16_t g_arm_j0_current = ARM_J0_CURRENT_DEFAULT;

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
static const arm_motor_entry_t *arm_j0_entry(void);
static const motor_node_param_t *arm_entry_node(const arm_motor_entry_t *entry);
static uint8_t arm_entry_can_bus(const arm_motor_entry_t *entry);
static uint8_t arm_entry_is_unitree_rs485(const arm_motor_entry_t *entry);
static uint8_t arm_j0_unitree_enabled(const arm_motor_entry_t *entry);
static uint16_t arm_mit_std_id(const arm_motor_entry_t *entry);
static const can_mit_motor_limits_t *arm_mit_limits(const arm_motor_entry_t *entry);
static void arm_copy_mit_feedback(uint8_t index);
static void arm_clear_mit_actuator_cmds(void);
static void arm_send_mit_stop_all(void);
static void arm_send_can_mit_from_actuator(const arm_motor_entry_t *entry,
                                           actuator_id_e actuator_id,
                                           const can_mit_motor_limits_t *limits);
static void arm_refresh_j0_feedback(void);
static fp32 arm_j0_unitree_ratio_safe(const arm_motor_entry_t *entry, const arm_j0_unitree_config_t *cfg);
static fp32 arm_j0_unitree_output_to_rotor_position(const arm_motor_entry_t *entry,
                                                    const arm_j0_unitree_config_t *cfg,
                                                    fp32 output_position_rad);
static fp32 arm_j0_unitree_output_to_rotor_speed(const arm_motor_entry_t *entry,
                                                 const arm_j0_unitree_config_t *cfg,
                                                 fp32 output_speed_rad_s);
static fp32 arm_j0_unitree_output_to_rotor_torque(const arm_motor_entry_t *entry,
                                                  const arm_j0_unitree_config_t *cfg,
                                                  fp32 output_torque_nm);
static fp32 arm_j0_unitree_output_to_rotor_kp(const arm_motor_entry_t *entry,
                                              const arm_j0_unitree_config_t *cfg,
                                              fp32 output_kp);
static fp32 arm_j0_unitree_output_to_rotor_kd(const arm_motor_entry_t *entry,
                                              const arm_j0_unitree_config_t *cfg,
                                              fp32 output_kd);
static void arm_build_j0_unitree_config(unitree_motor_config_t *out,
                                        const arm_motor_entry_t *entry,
                                        const arm_j0_unitree_config_t *cfg);
static uint8_t arm_build_j0_mit_cmd_from_actuator(mit_motor_cmd_t *out,
                                                  fp32 *output_speed_rad_s,
                                                  fp32 *output_kd);
static void arm_j0_unitree_cmd_from_mit(const arm_motor_entry_t *entry,
                                        const arm_j0_unitree_config_t *cfg,
                                        const mit_motor_cmd_t *src,
                                        unitree_motor_cmd_t *out);
static void arm_update_j0_actuator_feedback_from_unitree(void);
static void arm_write_j0_unitree_cmd(const arm_motor_entry_t *entry,
                                     uint8_t move_key,
                                     uint8_t reverse,
                                     uint8_t ctrl_held);
static void arm_sync_j0_unitree_state(void);
static void arm_step_j0_unitree(const arm_motor_entry_t *entry);
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

static const arm_motor_entry_t *arm_j0_entry(void)
{
    return &g_arm_motor_table[ARM_J0_INDEX];
}

static const motor_node_param_t *arm_entry_node(const arm_motor_entry_t *entry)
{
    uint32_t i;

    if (entry == NULL)
    {
        return NULL;
    }

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        if (entry == &g_arm_motor_table[i])
        {
            return &g_config.motor.arm[i];
        }
    }

    return NULL;
}

static uint8_t arm_entry_can_bus(const arm_motor_entry_t *entry)
{
    if (entry == NULL)
    {
        return 0u;
    }
    return motor_cfg_can_bus(entry->fallback_bus, arm_entry_node(entry));
}

static uint8_t arm_entry_is_unitree_rs485(const arm_motor_entry_t *entry)
{
    const motor_node_param_t *node = arm_entry_node(entry);

    if (entry == NULL)
    {
        return 0u;
    }
    if (motor_cfg_transport(node) != MOTOR_TRANSPORT_RS485)
    {
        return 0u;
    }
    return (motor_cfg_protocol(node) == MOTOR_PROTOCOL_UNITREE_RS485) ? 1u : 0u;
}

static uint8_t arm_j0_unitree_enabled(const arm_motor_entry_t *entry)
{
    return (arm_entry_is_unitree_rs485(entry) != 0u) ? 1u : 0u;
}

static uint16_t arm_mit_std_id(const arm_motor_entry_t *entry)
{
    const motor_node_param_t *node = arm_entry_node(entry);

    if (entry == NULL)
    {
        return 0u;
    }

    return motor_cfg_can_id(node);
}

static const can_mit_motor_limits_t *arm_mit_limits(const arm_motor_entry_t *entry)
{
    const motor_node_param_t *node = arm_entry_node(entry);

    if (entry == NULL)
    {
        return NULL;
    }

    return motor_cfg_mit_limits(node);
}

static void arm_copy_mit_feedback(uint8_t index)
{
    const can_mit_motor_feedback_t *src;
    arm_motor_feedback_t *dst;
    actuator_feedback_t fb;
    actuator_id_e actuator_id;
    const arm_motor_entry_t *entry;

    if (index >= ARM_MOTOR_COUNT)
    {
        return;
    }

    entry = &g_arm_motor_table[index];
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

    actuator_id = actuator_id_arm_joint(index);
    if ((uint32_t)actuator_id < (uint32_t)ACTUATOR_ID__COUNT)
    {
        (void)memset(&fb, 0, sizeof(fb));
        fb.online = src->online;
        fb.bus = arm_entry_can_bus(entry);
        fb.rx_dlc = src->rx_dlc;
        fb.transport = (uint8_t)ACTUATOR_TRANSPORT_CAN;
        fb.rx_id = src->rx_id;
        fb.rx_count = src->rx_count;
        fb.last_rx_tick = src->last_rx_tick;
        fb.position = src->position;
        fb.velocity = src->velocity;
        fb.torque = src->torque;
        actuator_feedback_update(actuator_id, &fb);
    }
}

static void arm_clear_mit_actuator_cmds(void)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        if (g_arm_motor_table[i].driver == ARM_MOTOR_DRIVER_CAN_MIT)
        {
            actuator_cmd_set_speed(actuator_id_arm_joint((uint8_t)i), 0.0f, 0.0f, 0.0f);
        }
    }
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

        can_mit_motor_send_stop(arm_entry_can_bus(entry), arm_mit_std_id(entry), arm_mit_limits(entry));
    }
}

static void arm_send_can_mit_from_actuator(const arm_motor_entry_t *entry,
                                           actuator_id_e actuator_id,
                                           const can_mit_motor_limits_t *limits)
{
    actuator_cmd_t src;
    mit_motor_cmd_t cmd;

    if (entry == NULL || limits == NULL)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    if (actuator_cmd_get_copy(actuator_id, &src) != 0u && src.active != 0u)
    {
        switch ((actuator_cmd_mode_e)src.mode)
        {
        case ACTUATOR_CMD_MODE_STATE_TORQUE:
        case ACTUATOR_CMD_MODE_POS_VEL:
        case ACTUATOR_CMD_MODE_FORCE_POS:
            cmd.position = src.position;
            cmd.velocity = src.velocity;
            cmd.kp = src.kp;
            cmd.kd = src.kd;
            cmd.torque = src.torque;
            break;
        case ACTUATOR_CMD_MODE_SPEED:
            cmd.velocity = src.velocity;
            cmd.kd = src.kd;
            cmd.torque = src.torque;
            break;
        case ACTUATOR_CMD_MODE_CURRENT:
        default:
            cmd.torque = src.torque;
            break;
        }
    }

    can_mit_motor_send_cmd(arm_entry_can_bus(entry), arm_mit_std_id(entry), limits, &cmd);
}

static fp32 arm_j0_unitree_ratio_safe(const arm_motor_entry_t *entry, const arm_j0_unitree_config_t *cfg)
{
    if (entry != NULL)
    {
        const motor_node_param_t *node = arm_entry_node(entry);
        const motor_model_param_t *model = (node != NULL) ? motor_cfg_model(node->model) : NULL;

        if (model != NULL && model->reduction_ratio > 0.0f)
        {
            return model->reduction_ratio;
        }
    }
    (void)cfg;

    return 1.0f;
}

static fp32 arm_j0_unitree_output_to_rotor_position(const arm_motor_entry_t *entry,
                                                    const arm_j0_unitree_config_t *cfg,
                                                    fp32 output_position_rad)
{
    return output_position_rad * arm_j0_unitree_ratio_safe(entry, cfg);
}

static fp32 arm_j0_unitree_output_to_rotor_speed(const arm_motor_entry_t *entry,
                                                 const arm_j0_unitree_config_t *cfg,
                                                 fp32 output_speed_rad_s)
{
    return output_speed_rad_s * arm_j0_unitree_ratio_safe(entry, cfg);
}

static fp32 arm_j0_unitree_output_to_rotor_torque(const arm_motor_entry_t *entry,
                                                  const arm_j0_unitree_config_t *cfg,
                                                  fp32 output_torque_nm)
{
    return output_torque_nm / arm_j0_unitree_ratio_safe(entry, cfg);
}

static fp32 arm_j0_unitree_output_to_rotor_kp(const arm_motor_entry_t *entry,
                                              const arm_j0_unitree_config_t *cfg,
                                              fp32 output_kp)
{
    const fp32 ratio = arm_j0_unitree_ratio_safe(entry, cfg);

    if (ratio <= 0.0f)
    {
        return output_kp;
    }

    return output_kp / (ratio * ratio);
}

static fp32 arm_j0_unitree_output_to_rotor_kd(const arm_motor_entry_t *entry,
                                              const arm_j0_unitree_config_t *cfg,
                                              fp32 output_kd)
{
    const fp32 ratio = arm_j0_unitree_ratio_safe(entry, cfg);

    if (ratio <= 0.0f)
    {
        return output_kd;
    }

    return output_kd / (ratio * ratio);
}

static uint8_t arm_build_j0_mit_cmd_from_actuator(mit_motor_cmd_t *out,
                                                  fp32 *output_speed_rad_s,
                                                  fp32 *output_kd)
{
    actuator_cmd_t src;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    if (output_speed_rad_s != NULL)
    {
        *output_speed_rad_s = 0.0f;
    }
    if (output_kd != NULL)
    {
        *output_kd = 0.0f;
    }

    if (actuator_cmd_get_copy(ACTUATOR_ID_ARM_J0, &src) == 0u || src.active == 0u)
    {
        return 0u;
    }

    switch ((actuator_cmd_mode_e)src.mode)
    {
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
    case ACTUATOR_CMD_MODE_POS_VEL:
    case ACTUATOR_CMD_MODE_FORCE_POS:
        out->position = src.position;
        out->velocity = src.velocity;
        out->kp = src.kp;
        out->kd = src.kd;
        out->torque = src.torque;
        break;
    case ACTUATOR_CMD_MODE_SPEED:
        out->velocity = src.velocity;
        out->kd = src.kd;
        out->torque = src.torque;
        break;
    case ACTUATOR_CMD_MODE_CURRENT:
    default:
        out->torque = src.torque;
        break;
    }

    if (output_speed_rad_s != NULL)
    {
        *output_speed_rad_s = src.velocity;
    }
    if (output_kd != NULL)
    {
        *output_kd = src.kd;
    }
    return 1u;
}

static void arm_j0_unitree_cmd_from_mit(const arm_motor_entry_t *entry,
                                        const arm_j0_unitree_config_t *cfg,
                                        const mit_motor_cmd_t *src,
                                        unitree_motor_cmd_t *out)
{
    if (src == NULL || out == NULL)
    {
        return;
    }

    out->position_rad = arm_j0_unitree_output_to_rotor_position(entry, cfg, src->position);
    out->speed_rad_s = arm_j0_unitree_output_to_rotor_speed(entry, cfg, src->velocity);
    out->kp = arm_j0_unitree_output_to_rotor_kp(entry, cfg, src->kp);
    out->kd = arm_j0_unitree_output_to_rotor_kd(entry, cfg, src->kd);
    out->torque_nm = arm_j0_unitree_output_to_rotor_torque(entry, cfg, src->torque);
}

static void arm_build_j0_unitree_config(unitree_motor_config_t *out,
                                        const arm_motor_entry_t *entry,
                                        const arm_j0_unitree_config_t *cfg)
{
    const motor_node_param_t *node = arm_entry_node(entry);

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (cfg == NULL)
    {
        return;
    }

    out->enable = arm_j0_unitree_enabled(entry);
    out->rs485_port = (node != NULL) ? node->rs485_port : 0u;
    out->motor_id = motor_cfg_node_id(node);
    out->baudrate = (node != NULL) ? node->baudrate : 0u;
    out->rx_timeout_ms = (node != NULL) ? node->rx_timeout_ms : 0u;
}

static void arm_update_j0_actuator_feedback_from_unitree(void)
{
    actuator_feedback_t fb;

    (void)memset(&fb, 0, sizeof(fb));
    fb.online = g_arm_j0_unitree_state.online;
    fb.bus = g_arm_j0_unitree_state.rs485_port;
    fb.transport = (uint8_t)ACTUATOR_TRANSPORT_RS485;
    fb.rx_id = g_arm_j0_unitree_state.motor_id;
    fb.rx_count = g_arm_j0_unitree_state.rx_frame_count;
    fb.last_rx_tick = g_arm_j0_unitree_state.last_rx_tick_ms;
    fb.position = g_arm_j0_unitree_state.joint_position_rad;
    fb.velocity = g_arm_j0_unitree_state.joint_speed_rad_s;
    fb.torque = g_arm_j0_unitree_state.torque_nm;
    fb.temperature = (uint8_t)g_arm_j0_unitree_state.motor_temp;
    actuator_feedback_update(ACTUATOR_ID_ARM_J0, &fb);
}

static void arm_sync_j0_unitree_state(void)
{
    const arm_motor_entry_t *entry = arm_j0_entry();
    const arm_j0_unitree_config_t *cfg = &g_config.arm_j0_unitree;
    const unitree_motor_state_t *state = unitree_motor_get_state();
    unitree_motor_config_t driver_cfg;

    if (state == NULL)
    {
        (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
        arm_update_j0_actuator_feedback_from_unitree();
        return;
    }

    arm_build_j0_unitree_config(&driver_cfg, entry, cfg);

    g_arm_j0_unitree_state.enabled = driver_cfg.enable;
    g_arm_j0_unitree_state.rs485_port = driver_cfg.rs485_port;
    g_arm_j0_unitree_state.motor_id = (state->motor_id != 0u) ? state->motor_id : driver_cfg.motor_id;
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
    arm_update_j0_actuator_feedback_from_unitree();
}

static void arm_write_j0_unitree_cmd(const arm_motor_entry_t *entry,
                                     uint8_t move_key,
                                     uint8_t reverse,
                                     uint8_t ctrl_held)
{
    const arm_j0_unitree_config_t *cfg = &g_config.arm_j0_unitree;

    if (arm_j0_unitree_enabled(entry) == 0u)
    {
        actuator_cmd_set_speed(ACTUATOR_ID_ARM_J0, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (g_arm_deadman_hold_ctrl != 0u && ctrl_held == 0u)
    {
        actuator_cmd_set_speed(ACTUATOR_ID_ARM_J0, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (move_key != 0u)
    {
        const fp32 dir = (reverse != 0u) ? -1.0f : 1.0f;
        actuator_cmd_set_speed(ACTUATOR_ID_ARM_J0, dir * cfg->key_speed_rad_s, cfg->drive_kd, 0.0f);
    }
    else
    {
        actuator_cmd_set_speed(ACTUATOR_ID_ARM_J0, 0.0f, cfg->hold_kd, 0.0f);
    }
}

static void arm_step_j0_unitree(const arm_motor_entry_t *entry)
{
    const arm_j0_unitree_config_t *cfg = &g_config.arm_j0_unitree;
    const uint16_t period_ms = (cfg->control_period_ms == 0u) ? 5u : cfg->control_period_ms;
    const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unitree_motor_config_t driver_cfg;
    mit_motor_cmd_t mit_cmd = {0};
    unitree_motor_cmd_t cmd = {0};
    fp32 output_speed = 0.0f;
    fp32 output_kd = 0.0f;

    arm_build_j0_unitree_config(&driver_cfg, entry, cfg);
    unitree_motor_refresh(&driver_cfg);

    if (driver_cfg.enable == 0u)
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

    if (arm_build_j0_mit_cmd_from_actuator(&mit_cmd, &output_speed, &output_kd) == 0u)
    {
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
        arm_sync_j0_unitree_state();
        return;
    }

    arm_j0_unitree_cmd_from_mit(entry, cfg, &mit_cmd, &cmd);
    g_arm_j0_unitree_cmd_output_speed_rad_s = output_speed;
    g_arm_j0_unitree_cmd_output_kd = output_kd;
    (void)unitree_motor_send_cmd(&driver_cfg, &cmd);
    arm_sync_j0_unitree_state();
}

static void arm_refresh_j0_feedback(void)
{
    const arm_motor_entry_t *entry = arm_j0_entry();
    arm_motor_feedback_t *feedback = &g_arm_feedback[ARM_J0_INDEX];

    if (arm_j0_unitree_enabled(entry) != 0u)
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
        feedback->rx_id = motor_cfg_feedback_id(arm_entry_node(entry));
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

    arm_write_j0_unitree_cmd(entry, move_key, reverse, ctrl_held);
    arm_step_j0_unitree(entry);

    if (arm_j0_unitree_enabled(entry) != 0u)
    {
        actuator_cmd_set_current(ACTUATOR_ID_ARM_J0, 0);
        return;
    }

    if (g_arm_deadman_hold_ctrl != 0u && ctrl_held == 0u)
    {
        actuator_cmd_set_current(ACTUATOR_ID_ARM_J0, 0);
        return;
    }

    if (move_key != 0u)
    {
        const int16_t current_abs = (g_arm_j0_current >= 0) ? g_arm_j0_current : (int16_t)(-g_arm_j0_current);
        int16_t current = (reverse != 0u) ? (int16_t)(-current_abs) : current_abs;

        current = motor_cfg_limit_current_node(arm_entry_node(entry), current);
        actuator_cmd_set_current(ACTUATOR_ID_ARM_J0, current);
    }
    else
    {
        actuator_cmd_set_current(ACTUATOR_ID_ARM_J0, 0);
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

        arm_clear_mit_actuator_cmds();
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

            can_mit_motor_send_enable(arm_entry_can_bus(entry), arm_mit_std_id(entry));
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
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];
        const can_mit_motor_limits_t *limits;
        const actuator_id_e actuator_id = actuator_id_arm_joint((uint8_t)i);

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
            const fp32 velocity = dir * (fp32)entry->direction * entry->key_speed_rad_s * g_arm_key_speed_scale;
            const fp32 kd = arm_motion_clamp_fp32(g_arm_key_kd, 0.0f, limits->kd_max);
            actuator_cmd_set_speed(actuator_id, velocity, kd, 0.0f);
        }
        else
        {
            actuator_cmd_set_speed(actuator_id, 0.0f, 0.0f, 0.0f);
        }

        arm_send_can_mit_from_actuator(entry, actuator_id, limits);
    }
}

void arm_motion_init(void)
{
    (void)memset(g_arm_feedback, 0, sizeof(g_arm_feedback));
    (void)memset(g_arm_mit_feedback, 0, sizeof(g_arm_mit_feedback));
    (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
    g_arm_mit_armed = 0u;
    g_arm_j0_unitree_last_step_tick_ms = 0u;
    g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
    g_arm_j0_unitree_cmd_output_kd = 0.0f;
    actuator_cmd_set_current(ACTUATOR_ID_ARM_J0, 0);
    arm_clear_mit_actuator_cmds();
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

uint8_t arm_motion_process_can_feedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const arm_motor_entry_t *entry = &g_arm_motor_table[i];
        const motor_node_param_t *node = arm_entry_node(entry);

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }
        if (arm_entry_can_bus(entry) != bus)
        {
            continue;
        }
        if (motor_cfg_feedback_id(node) != std_id)
        {
            continue;
        }

        if (can_mit_motor_update_feedback(std_id,
                                          motor_cfg_node_id(node),
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
