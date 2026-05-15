/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈舜 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef APP_PUBSUB_H
#define APP_PUBSUB_H

#include "struct_typedef.h"

typedef enum
{
    APP_TOPIC_GIMBAL_STATE = 0,
    APP_TOPIC_CHASSIS_STATE,
    APP_TOPIC_SHOOT_STATE,
    APP_TOPIC_WHEELLEG_CMD,
    APP_TOPIC_WHEELLEG_STATE,
    APP_TOPIC_WHEELLEG_STATUS,
    APP_TOPIC_WHEELLEG_DEBUG,
    APP_TOPIC_ARM_STATUS,
    APP_TOPIC__COUNT,
} app_topic_id_e;

#define APP_PUBSUB_MAX_PAYLOAD_BYTES 512u

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
} app_topic_status_t;

uint8_t app_pubsub_publish(app_topic_id_e topic, const void *payload, uint16_t size);
uint8_t app_pubsub_copy(app_topic_id_e topic, void *out, uint16_t size);
app_topic_status_t app_pubsub_status(app_topic_id_e topic);

#endif
