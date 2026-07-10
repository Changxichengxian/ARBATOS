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

static inline void ManualInputCrsfDecode(
    const uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
    const input_config_t *config,
    ManualInputState *out)
{
    if (raw == NULL || config == NULL || out == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < 5u; i++)
    {
        uint8_t channel = config->ElrsChMap[i];
        if (channel >= MANUAL_INPUT_CRSF_CHANNEL_COUNT)
        {
            channel = (i < 4u) ? i : 6u;
        }
        out->rc.ch[i] = ManualInputCrsfMapAxis(raw[channel]);
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        uint8_t channel = config->ElrsSwMap[i];
        if (channel >= MANUAL_INPUT_CRSF_CHANNEL_COUNT)
        {
            channel = (uint8_t)(4u + i);
        }
        out->rc.s[i] = (char)ManualInputCrsfMapSwitch(raw[channel]);
    }
}

#endif
