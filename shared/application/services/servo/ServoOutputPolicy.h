/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SERVO_OUTPUT_POLICY_H
#define SERVO_OUTPUT_POLICY_H

#include <stddef.h>
#include "LowCmd.h"
#include "ServoConfig.h"

static inline uint8_t ServoChannelValid(const ServoChannelConfig *channel)
{
    return (uint8_t)(channel != NULL && channel->enabled <= 1u && channel->port < 4u &&
                     channel->inputMode <= 1u && channel->inputChannel < 5u && channel->invert <= 1u &&
                     channel->minUs >= 500u && channel->maxUs <= 2500u &&
                     channel->minUs < channel->centerUs && channel->centerUs < channel->maxUs &&
                     channel->stepUs >= 1u && channel->stepUs <= 200u);
}

static inline uint8_t ServoConfigValid(const ServoConfig *config)
{
    uint8_t ports = 0u;

    if (config == NULL || config->configured != 1u) return 0u;
    for (uint8_t i = 0u; i < 4u; i++) {
        const ServoChannelConfig *channel = &config->channels[i];
        if (channel->enabled == 0u) continue;
        if (ServoChannelValid(channel) == 0u || (ports & (1u << channel->port)) != 0u) return 0u;
        ports |= (uint8_t)(1u << channel->port);
    }
    return 1u;
}

// 统一输入范围为 -660..660；端点外夹紧，反向不会越过脉宽上下限。
static inline uint16_t ServoPulseFromAxis(const ServoChannelConfig *channel, int16_t axis)
{
    int32_t value = axis;
    int32_t pulse;

    if (ServoChannelValid(channel) == 0u) return 0u;
    if (value > 660) value = 660;
    if (value < -660) value = -660;
    if (channel->invert != 0u) value = -value;
    pulse = channel->centerUs;
    pulse += value >= 0 ? value * (channel->maxUs - channel->centerUs) / 660 :
                         value * (channel->centerUs - channel->minUs) / 660;
    return (uint16_t)pulse;
}

static inline fp32 ServoPositionFromAxis(int16_t axis)
{
    int32_t value = axis;

    if (value > 660) value = 660;
    if (value < -660) value = -660;
    return (fp32)value / 660.0f;
}

/*
 * PWM 舵机槽的 MotorModePosVel.q 是 [-1, 1] 归一化位置，0 为中点。
 * 这里不把 q 解释为 rad；真实物理角度必须由具体舵机行程标定后另行换算。
 */
static inline uint16_t ServoPulseFromPositionBound(const ServoChannelConfig *channel,
                                                   fp32 position)
{
    fp32 value = position;
    fp32 pulse;

    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    if (channel->invert != 0u) value = -value;
    pulse = (fp32)channel->centerUs;
    pulse += value >= 0.0f ? value * (fp32)(channel->maxUs - channel->centerUs) :
                            value * (fp32)(channel->centerUs - channel->minUs);
    return (uint16_t)(pulse + 0.5f);
}

static inline uint16_t ServoPulseFromPosition(const ServoChannelConfig *channel, fp32 position)
{
    if (ServoChannelValid(channel) == 0u || position != position) return 0u;
    return ServoPulseFromPositionBound(channel, position);
}

static inline fp32 ServoPositionFromPulseBound(const ServoChannelConfig *channel,
                                               uint16_t pulse)
{
    fp32 value;

    if (pulse < channel->minUs) pulse = channel->minUs;
    if (pulse > channel->maxUs) pulse = channel->maxUs;
    if (pulse >= channel->centerUs) {
        value = (fp32)(pulse - channel->centerUs) /
                (fp32)(channel->maxUs - channel->centerUs);
    } else {
        value = -(fp32)(channel->centerUs - pulse) /
                 (fp32)(channel->centerUs - channel->minUs);
    }
    return (channel->invert != 0u) ? -value : value;
}

static inline fp32 ServoPositionFromPulse(const ServoChannelConfig *channel, uint16_t pulse)
{
    if (ServoChannelValid(channel) == 0u) return 0.0f;
    return ServoPositionFromPulseBound(channel, pulse);
}

static inline fp32 ServoPositionStep(const ServoChannelConfig *channel,
                                     fp32 previous,
                                     int8_t direction)
{
    int32_t pulse;

    if (ServoChannelValid(channel) == 0u) return 0.0f;
    if (previous != previous) previous = 0.0f;
    pulse = ServoPulseFromPositionBound(channel, previous);
    pulse += (int32_t)direction * channel->stepUs *
             (channel->invert != 0u ? -1 : 1);
    if (pulse < channel->minUs) pulse = channel->minUs;
    if (pulse > channel->maxUs) pulse = channel->maxUs;

    return ServoPositionFromPulseBound(channel, (uint16_t)pulse);
}

static inline uint8_t ServoCommandPulseBound(const ServoChannelConfig *channel,
                                             MotorId id,
                                             const MotorCmd *cmd,
                                             const ControlOutputStamp *owner,
                                             uint32_t nowMs,
                                             uint8_t outputLocked,
                                             uint16_t *pulse)
{
    uint16_t timeout;

    if (pulse == NULL) return 0u;
    *pulse = 0u;
    if (outputLocked != 0u || cmd == NULL || owner == NULL ||
        cmd->active == 0u || cmd->mode != (uint8_t)MotorModePosVel ||
        cmd->q != cmd->q)
    {
        return 0u;
    }
    timeout = (cmd->timeoutMs != 0u) ? cmd->timeoutMs : (uint16_t)LOWCMD_DEFAULT_TIMEOUT_MS;
    if ((uint32_t)(nowMs - cmd->tick) > (uint32_t)timeout ||
        LowCmdOutputSnapshotAuthorized(id, cmd, owner) == 0u)
    {
        return 0u;
    }
    *pulse = ServoPulseFromPositionBound(channel, cmd->q);
    return (uint8_t)(*pulse != 0u);
}

static inline uint8_t ServoCommandPulse(const ServoChannelConfig *channel,
                                        MotorId id,
                                        const MotorCmd *cmd,
                                        const ControlOutputStamp *owner,
                                        uint32_t nowMs,
                                        uint8_t outputLocked,
                                        uint16_t *pulse)
{
    if (ServoChannelValid(channel) == 0u)
    {
        if (pulse != NULL) *pulse = 0u;
        return 0u;
    }
    return ServoCommandPulseBound(channel, id, cmd, owner, nowMs, outputLocked, pulse);
}

static inline uint16_t ServoPulseStep(const ServoChannelConfig *channel, uint16_t previous, int8_t direction)
{
    int32_t pulse;

    if (ServoChannelValid(channel) == 0u) return 0u;
    pulse = (int32_t)previous + (int32_t)direction * channel->stepUs * (channel->invert != 0u ? -1 : 1);
    if (pulse < channel->minUs) pulse = channel->minUs;
    if (pulse > channel->maxUs) pulse = channel->maxUs;
    return (uint16_t)pulse;
}

#endif
