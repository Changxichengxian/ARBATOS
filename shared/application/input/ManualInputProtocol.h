/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANUAL_INPUT_PROTOCOL_H
#define MANUAL_INPUT_PROTOCOL_H

typedef enum
{
    MANUAL_INPUT_PROTOCOL_NONE = 0u,
    MANUAL_INPUT_PROTOCOL_DBUS = 1u,
    MANUAL_INPUT_PROTOCOL_CRSF = 2u,
    MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM = 3u,
    MANUAL_INPUT_PROTOCOL_IMAGE_VT13 = 4u,
} ManualInputProtocol;

#endif
