/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "external_motion_intent.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define EXTERNAL_MOTION_DEFAULT_TIMEOUT_MS 200u

static external_motion_intent_t external_motion_intent_latest;
static volatile bool external_motion_intent_valid = false;
static volatile uint32_t external_motion_intent_tick_ms = 0u;

void external_motion_intent_clear(void)
{
    taskENTER_CRITICAL();
    memset(&external_motion_intent_latest, 0, sizeof(external_motion_intent_latest));
    external_motion_intent_valid = false;
    external_motion_intent_tick_ms = 0u;
    taskEXIT_CRITICAL();
}

void external_motion_intent_write_from_isr(const external_motion_intent_t *intent)
{
    if (intent == NULL)
    {
        return;
    }

    UBaseType_t saved_status = taskENTER_CRITICAL_FROM_ISR();
    external_motion_intent_latest = *intent;
    external_motion_intent_valid = true;
    external_motion_intent_tick_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    taskEXIT_CRITICAL_FROM_ISR(saved_status);
}

bool external_motion_intent_read_latest(external_motion_intent_t *out, uint32_t *age_ms)
{
    external_motion_intent_t intent;
    bool valid = false;
    uint32_t rx_tick_ms = 0u;

    taskENTER_CRITICAL();
    valid = external_motion_intent_valid;
    intent = external_motion_intent_latest;
    rx_tick_ms = external_motion_intent_tick_ms;
    taskEXIT_CRITICAL();

    if (!valid || intent.mode == (uint8_t)EXTERNAL_MOTION_MODE_IDLE)
    {
        return false;
    }
    if (intent.mode > (uint8_t)EXTERNAL_MOTION_MODE_STOP ||
        intent.frame > (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        return false;
    }

    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const uint32_t age = now_ms - rx_tick_ms;
    const uint32_t timeout_ms = (intent.timeout_ms != 0u)
                                    ? (uint32_t)intent.timeout_ms
                                    : (uint32_t)EXTERNAL_MOTION_DEFAULT_TIMEOUT_MS;
    if (age > timeout_ms)
    {
        return false;
    }

    if (out != NULL)
    {
        *out = intent;
    }
    if (age_ms != NULL)
    {
        *age_ms = age;
    }
    return true;
}
