/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SERVO_OUTPUT_POLICY_H
#define SERVO_OUTPUT_POLICY_H

#include <stddef.h>
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
