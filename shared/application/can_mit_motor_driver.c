/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "can_mit_motor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
static uint32_t can_mit_motor_float_to_uint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 can_mit_motor_uint_to_float(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 can_mit_motor_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max);

static uint32_t can_mit_motor_float_to_uint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const fp32 span = x_max - x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    fp32 x_clamped = can_mit_motor_clamp_fp32(x, x_min, x_max);

    if (span <= 0.0f)
    {
        return 0u;
    }

    return (uint32_t)(((x_clamped - x_min) * (fp32)max_int) / span + 0.5f);
}

static fp32 can_mit_motor_uint_to_float(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const uint32_t max_int = (1u << bits) - 1u;
    const fp32 span = x_max - x_min;

    if (max_int == 0u || span <= 0.0f)
    {
        return x_min;
    }

    return ((fp32)x_int) * span / (fp32)max_int + x_min;
}

static fp32 can_mit_motor_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
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

void can_mit_motor_send_cmd(uint8_t bus,
                            uint16_t std_id,
                            const can_mit_motor_limits_t *limits,
                            const can_mit_motor_cmd_t *cmd)
{
    uint8_t data[8];
    uint32_t p_int;
    uint32_t v_int;
    uint32_t kp_int;
    uint32_t kd_int;
    uint32_t t_int;

    if (limits == NULL || cmd == NULL || std_id == 0u)
    {
        return;
    }

    p_int = can_mit_motor_float_to_uint(cmd->position, -limits->position_max, limits->position_max, 16u);
    v_int = can_mit_motor_float_to_uint(cmd->velocity, -limits->velocity_max, limits->velocity_max, 12u);
    kp_int = can_mit_motor_float_to_uint(cmd->kp, 0.0f, limits->kp_max, 12u);
    kd_int = can_mit_motor_float_to_uint(cmd->kd, 0.0f, limits->kd_max, 12u);
    t_int = can_mit_motor_float_to_uint(cmd->torque, -limits->torque_max, limits->torque_max, 12u);

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

void can_mit_motor_send_enable(uint8_t bus, uint16_t std_id)
{
    static const uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFCu};

    if (std_id == 0u)
    {
        return;
    }

    (void)bsp_can_tx(bus, std_id, data, 8u);
}

void can_mit_motor_send_stop(uint8_t bus, uint16_t std_id, const can_mit_motor_limits_t *limits)
{
    can_mit_motor_cmd_t stop_cmd = {0};

    can_mit_motor_send_cmd(bus, std_id, limits, &stop_cmd);
}

uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                      uint8_t motor_id,
                                      const can_mit_motor_limits_t *limits,
                                      uint8_t dlc,
                                      const uint8_t data[8],
                                      can_mit_motor_feedback_t *feedback)
{
    uint8_t payload_off = 0u;
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;

    if (limits == NULL || data == NULL || feedback == NULL)
    {
        return 0u;
    }

    if (dlc >= 6u && (data[0] == motor_id || data[0] == (uint8_t)std_id))
    {
        payload_off = 1u;
    }
    else if (dlc < 5u)
    {
        return 0u;
    }

    p_int = ((uint32_t)data[payload_off + 0u] << 8) | (uint32_t)data[payload_off + 1u];
    v_int = ((uint32_t)data[payload_off + 2u] << 4) | ((uint32_t)data[payload_off + 3u] >> 4);
    t_int = (((uint32_t)data[payload_off + 3u] & 0x0Fu) << 8) | (uint32_t)data[payload_off + 4u];

    feedback->online = 1u;
    feedback->rx_dlc = dlc;
    feedback->rx_id = std_id;
    feedback->rx_count++;
    feedback->last_rx_tick = xTaskGetTickCount();
    feedback->position = can_mit_motor_uint_to_float(p_int,
                                                     -limits->position_max,
                                                     limits->position_max,
                                                     16u);
    feedback->velocity = can_mit_motor_uint_to_float(v_int,
                                                     -limits->velocity_max,
                                                     limits->velocity_max,
                                                     12u);
    feedback->torque = can_mit_motor_uint_to_float(t_int,
                                                   -limits->torque_max,
                                                   limits->torque_max,
                                                   12u);
    return 1u;
}
