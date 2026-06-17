/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CanMitMotorDriver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BspCan.h"
static uint32_t CanMitMotorFloatToUint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 CanMitMotorUintToFloat(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);
static fp32 CanMitMotorClampFp32(fp32 x, fp32 x_min, fp32 x_max);

static uint32_t CanMitMotorFloatToUint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const fp32 span = x_max - x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    fp32 x_clamped = CanMitMotorClampFp32(x, x_min, x_max);

    if (span <= 0.0f)
    {
        return 0u;
    }

    return (uint32_t)(((x_clamped - x_min) * (fp32)max_int) / span + 0.5f);
}

static fp32 CanMitMotorUintToFloat(uint32_t x_int, fp32 x_min, fp32 x_max, uint8_t bits)
{
    const uint32_t max_int = (1u << bits) - 1u;
    const fp32 span = x_max - x_min;

    if (max_int == 0u || span <= 0.0f)
    {
        return x_min;
    }

    return ((fp32)x_int) * span / (fp32)max_int + x_min;
}

static fp32 CanMitMotorClampFp32(fp32 x, fp32 x_min, fp32 x_max)
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

int CanMitMotorSendCmd(uint8_t bus,
                           uint16_t std_id,
                           const CanMitMotorLimits *limits,
                           const CanMitMotorCmd *cmd)
{
    uint8_t data[8];
    uint32_t p_int;
    uint32_t v_int;
    uint32_t kp_int;
    uint32_t kd_int;
    uint32_t t_int;

    if (limits == NULL || cmd == NULL || std_id == 0u)
    {
        return -1;
    }

    p_int = CanMitMotorFloatToUint(cmd->position, -limits->position_max, limits->position_max, 16u);
    v_int = CanMitMotorFloatToUint(cmd->velocity, -limits->velocity_max, limits->velocity_max, 12u);
    kp_int = CanMitMotorFloatToUint(cmd->kp, 0.0f, limits->kp_max, 12u);
    kd_int = CanMitMotorFloatToUint(cmd->kd, 0.0f, limits->kd_max, 12u);
    t_int = CanMitMotorFloatToUint(cmd->torque, -limits->torque_max, limits->torque_max, 12u);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)p_int;
    data[2] = (uint8_t)(v_int >> 4);
    data[3] = (uint8_t)(((v_int & 0x0Fu) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)kp_int;
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0Fu) << 4) | (t_int >> 8));
    data[7] = (uint8_t)t_int;

    return BspCanTx(bus, std_id, data, 8u);
}

int CanMitMotorSendEnable(uint8_t bus, uint16_t std_id)
{
    static const uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFCu};

    if (std_id == 0u)
    {
        return -1;
    }

    return BspCanTx(bus, std_id, data, 8u);
}

int CanMitMotorSendDisable(uint8_t bus, uint16_t std_id)
{
    static const uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFDu};

    if (std_id == 0u)
    {
        return -1;
    }

    return BspCanTx(bus, std_id, data, 8u);
}

int CanMitMotorSendStop(uint8_t bus, uint16_t std_id, const CanMitMotorLimits *limits)
{
    CanMitMotorCmd stop_cmd = {0};

    return CanMitMotorSendCmd(bus, std_id, limits, &stop_cmd);
}

uint8_t CanMitMotorUpdateFeedback(uint16_t std_id,
                                      uint8_t motor_id,
                                      const CanMitMotorLimits *limits,
                                      uint8_t dlc,
                                      const uint8_t data[8],
                                      CanMitMotorFeedback *feedback)
{
    uint8_t payload_off = 0u;
    uint8_t feedback_motor_id = 0u;
    uint8_t feedback_state = 0u;
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;

    if (limits == NULL || data == NULL || feedback == NULL)
    {
        return 0u;
    }

    if (dlc >= 6u)
    {
        const uint8_t low_id = (uint8_t)(data[0] & 0x0Fu);
        if (data[0] == motor_id || data[0] == (uint8_t)std_id)
        {
            payload_off = 1u;
            feedback_motor_id = data[0];
        }
        else if (low_id == motor_id || low_id == (uint8_t)(std_id & 0x0Fu))
        {
            payload_off = 1u;
            feedback_motor_id = low_id;
            feedback_state = (uint8_t)(data[0] >> 4);
        }
    }
    if (payload_off == 0u)
    {
        if (dlc < 5u)
        {
            return 0u;
        }
        feedback_motor_id = motor_id;
    }

    if ((uint8_t)(payload_off + 4u) >= dlc)
    {
        return 0u;
    }

    p_int = ((uint32_t)data[payload_off + 0u] << 8) | (uint32_t)data[payload_off + 1u];
    v_int = ((uint32_t)data[payload_off + 2u] << 4) | ((uint32_t)data[payload_off + 3u] >> 4);
    t_int = (((uint32_t)data[payload_off + 3u] & 0x0Fu) << 8) | (uint32_t)data[payload_off + 4u];

    feedback->online = 1u;
    feedback->rx_dlc = dlc;
    feedback->rx_id = std_id;
    feedback->motor_id = feedback_motor_id;
    feedback->state = feedback_state;
    feedback->mos_temperature = (dlc >= 8u && payload_off == 1u) ? data[6] : 0u;
    feedback->coil_temperature = (dlc >= 8u && payload_off == 1u) ? data[7] : 0u;
    feedback->rx_count++;
    feedback->last_rx_tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    feedback->position = CanMitMotorUintToFloat(p_int,
                                                     -limits->position_max,
                                                     limits->position_max,
                                                     16u);
    feedback->velocity = CanMitMotorUintToFloat(v_int,
                                                     -limits->velocity_max,
                                                     limits->velocity_max,
                                                     12u);
    feedback->torque = CanMitMotorUintToFloat(t_int,
                                                   -limits->torque_max,
                                                   limits->torque_max,
                                                   12u);
    return 1u;
}
