/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈舜 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "app_pubsub.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
    uint8_t payload[APP_PUBSUB_MAX_PAYLOAD_BYTES];
} app_topic_slot_t;

static app_topic_slot_t s_topics[APP_TOPIC__COUNT];

static uint8_t app_topic_is_valid(app_topic_id_e topic)
{
    return ((uint32_t)topic < (uint32_t)APP_TOPIC__COUNT) ? 1u : 0u;
}

uint8_t app_pubsub_publish(app_topic_id_e topic, const void *payload, uint16_t size)
{
    if (app_topic_is_valid(topic) == 0u || payload == NULL || size == 0u || size > APP_PUBSUB_MAX_PAYLOAD_BYTES)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    memcpy(s_topics[topic].payload, payload, size);
    s_topics[topic].size = size;
    s_topics[topic].seq++;
    s_topics[topic].valid = 1u;
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t app_pubsub_copy(app_topic_id_e topic, void *out, uint16_t size)
{
    if (app_topic_is_valid(topic) == 0u || out == NULL || size == 0u)
    {
        return 0u;
    }

    uint8_t ok = 0u;
    taskENTER_CRITICAL();
    if (s_topics[topic].valid != 0u && s_topics[topic].size == size)
    {
        memcpy(out, s_topics[topic].payload, size);
        ok = 1u;
    }
    taskEXIT_CRITICAL();
    return ok;
}

app_topic_status_t app_pubsub_status(app_topic_id_e topic)
{
    app_topic_status_t status = {0};
    if (app_topic_is_valid(topic) == 0u)
    {
        return status;
    }

    taskENTER_CRITICAL();
    status.valid = s_topics[topic].valid;
    status.size = s_topics[topic].size;
    status.seq = s_topics[topic].seq;
    taskEXIT_CRITICAL();
    return status;
}
