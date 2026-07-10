/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANUAL_INPUT_CRSF_H
#define MANUAL_INPUT_CRSF_H

#include <stdint.h>

#include "ManualInput.h"

#define MANUAL_INPUT_CRSF_CHANNEL_COUNT 16u
#define MANUAL_INPUT_CRSF_VALUE_MIN     172u
#define MANUAL_INPUT_CRSF_VALUE_MID     992u
#define MANUAL_INPUT_CRSF_VALUE_MAX     1811u

static inline uint8_t ManualInputCrsfAxisChannel(const input_config_t *config,
                                                  uint8_t index)
{
    uint8_t channel;

    if (config == NULL || index >= 5u)
    {
        return MANUAL_INPUT_CRSF_CHANNEL_COUNT;
    }
    channel = config->ElrsChMap[index];
    return channel;
}

static inline uint8_t ManualInputCrsfSwitchChannel(const input_config_t *config,
                                                    uint8_t index)
{
    uint8_t channel;

    if (config == NULL || index >= 2u)
    {
        return MANUAL_INPUT_CRSF_CHANNEL_COUNT;
    }
    channel = config->ElrsSwMap[index];
    return channel;
}

/*
 * 项目控制语义只接受 CRSF 标准 +/-100% 范围。只检查实际映射到控制量的通道，
 * 避免未使用 AUX 的接收机占位值误伤仍然有效的控制帧。
 */
static inline uint8_t ManualInputCrsfMappedValuesValid(
    const uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
    const input_config_t *config)
{
    if (raw == NULL || config == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 5u; i++)
    {
        const uint8_t channel = ManualInputCrsfAxisChannel(config, i);
        uint16_t value;

        if (channel >= MANUAL_INPUT_CRSF_CHANNEL_COUNT)
        {
            return 0u;
        }
        value = raw[channel];
        if (value < MANUAL_INPUT_CRSF_VALUE_MIN ||
            value > MANUAL_INPUT_CRSF_VALUE_MAX)
        {
            return 0u;
        }
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        const uint8_t channel = ManualInputCrsfSwitchChannel(config, i);
        uint16_t value;

        if (channel >= MANUAL_INPUT_CRSF_CHANNEL_COUNT)
        {
            return 0u;
        }
        value = raw[channel];
        if (value < MANUAL_INPUT_CRSF_VALUE_MIN ||
            value > MANUAL_INPUT_CRSF_VALUE_MAX)
        {
            return 0u;
        }
    }
    return 1u;
}

static inline int16_t ManualInputCrsfMapAxis(uint16_t value)
{
    if (value < MANUAL_INPUT_CRSF_VALUE_MIN)
    {
        value = MANUAL_INPUT_CRSF_VALUE_MIN;
    }
    if (value > MANUAL_INPUT_CRSF_VALUE_MAX)
    {
        value = MANUAL_INPUT_CRSF_VALUE_MAX;
    }

    if (value >= MANUAL_INPUT_CRSF_VALUE_MID)
    {
        const uint16_t delta = (uint16_t)(value - MANUAL_INPUT_CRSF_VALUE_MID);
        const uint16_t denom =
            (uint16_t)(MANUAL_INPUT_CRSF_VALUE_MAX - MANUAL_INPUT_CRSF_VALUE_MID);
        return (int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) +
                          (denom / 2u)) /
                         denom);
    }

    const uint16_t delta = (uint16_t)(MANUAL_INPUT_CRSF_VALUE_MID - value);
    const uint16_t denom =
        (uint16_t)(MANUAL_INPUT_CRSF_VALUE_MID - MANUAL_INPUT_CRSF_VALUE_MIN);
    return (int16_t)(-((int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) +
                                  (denom / 2u)) /
                                 denom)));
}

static inline uint8_t ManualInputCrsfMapSwitch(uint16_t value)
{
    const uint16_t threshold_down =
        (uint16_t)((MANUAL_INPUT_CRSF_VALUE_MIN + MANUAL_INPUT_CRSF_VALUE_MID) / 2u);
    const uint16_t threshold_up =
        (uint16_t)((MANUAL_INPUT_CRSF_VALUE_MID + MANUAL_INPUT_CRSF_VALUE_MAX) / 2u);

    if (value <= threshold_down)
    {
        return RC_SW_DOWN;
    }
    if (value >= threshold_up)
    {
        return RC_SW_UP;
    }
    return RC_SW_MID;
}

static inline uint8_t ManualInputCrsfDecode(
    const uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
    const input_config_t *config,
    ManualInputState *out)
{
    if (raw == NULL || config == NULL || out == NULL ||
        ManualInputCrsfMappedValuesValid(raw, config) == 0u)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 5u; i++)
    {
        const uint8_t channel = ManualInputCrsfAxisChannel(config, i);
        out->rc.ch[i] = ManualInputCrsfMapAxis(raw[channel]);
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        const uint8_t channel = ManualInputCrsfSwitchChannel(config, i);
        out->rc.s[i] = (char)ManualInputCrsfMapSwitch(raw[channel]);
    }
    return 1u;
}

#endif
