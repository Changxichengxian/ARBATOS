/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MANUAL_INPUT_DBUS_H
#define MANUAL_INPUT_DBUS_H

#include <stddef.h>
#include <stdint.h>

#define MANUAL_INPUT_DBUS_FRAME_LENGTH       18u
#define MANUAL_INPUT_DBUS_CHANNEL_COUNT      5u
#define MANUAL_INPUT_DBUS_CHANNEL_OFFSET     1024u
#define MANUAL_INPUT_DBUS_CHANNEL_ERROR_ABS  700u
#define MANUAL_INPUT_DBUS_SWITCH_UP          1u
#define MANUAL_INPUT_DBUS_SWITCH_DOWN        2u
#define MANUAL_INPUT_DBUS_SWITCH_MID         3u

typedef struct
{
    uint16_t channel[MANUAL_INPUT_DBUS_CHANNEL_COUNT];
    uint8_t sw[2];
    int16_t mouse[3];
    uint8_t mouseButton[2];
    uint16_t key;
} ManualInputDbusData;

static __inline uint8_t ManualInputDbusDecode(const uint8_t *frame, ManualInputDbusData *out)
{
    ManualInputDbusData data;

    if (frame == NULL || out == NULL)
    {
        return 0u;
    }

    data.channel[0] = (uint16_t)(((uint16_t)frame[0] | ((uint16_t)frame[1] << 8)) & 0x07FFu);
    data.channel[1] = (uint16_t)((((uint16_t)frame[1] >> 3) | ((uint16_t)frame[2] << 5)) & 0x07FFu);
    data.channel[2] = (uint16_t)((((uint16_t)frame[2] >> 6) |
                                  ((uint16_t)frame[3] << 2) |
                                  ((uint16_t)frame[4] << 10)) & 0x07FFu);
    data.channel[3] = (uint16_t)((((uint16_t)frame[4] >> 1) | ((uint16_t)frame[5] << 7)) & 0x07FFu);
    data.channel[4] = (uint16_t)((uint16_t)frame[16] | ((uint16_t)frame[17] << 8));
    data.sw[0] = (uint8_t)((frame[5] >> 4) & 0x03u);
    data.sw[1] = (uint8_t)((frame[5] >> 6) & 0x03u);
    data.mouse[0] = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    data.mouse[1] = (int16_t)((uint16_t)frame[8] | ((uint16_t)frame[9] << 8));
    data.mouse[2] = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    data.mouseButton[0] = frame[12];
    data.mouseButton[1] = frame[13];
    data.key = (uint16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));

    *out = data;
    return 1u;
}

static __inline uint8_t ManualInputDbusValid(const ManualInputDbusData *data)
{
    uint8_t i;

    if (data == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < (uint8_t)MANUAL_INPUT_DBUS_CHANNEL_COUNT; i++)
    {
        const int32_t centered = (int32_t)data->channel[i] - (int32_t)MANUAL_INPUT_DBUS_CHANNEL_OFFSET;
        if (centered < -(int32_t)MANUAL_INPUT_DBUS_CHANNEL_ERROR_ABS ||
            centered > (int32_t)MANUAL_INPUT_DBUS_CHANNEL_ERROR_ABS)
        {
            return 0u;
        }
    }

    for (i = 0u; i < 2u; i++)
    {
        if (data->sw[i] != MANUAL_INPUT_DBUS_SWITCH_UP &&
            data->sw[i] != MANUAL_INPUT_DBUS_SWITCH_MID &&
            data->sw[i] != MANUAL_INPUT_DBUS_SWITCH_DOWN)
        {
            return 0u;
        }
        if (data->mouseButton[i] > 1u)
        {
            return 0u;
        }
    }

    return 1u;
}

#endif
