/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "PowerMeter.h"

#include <string.h>

#if defined(POWER_METER_HOST_TEST)
#define PowerMeterEnterCritical() ((void)0)
#define PowerMeterExitCritical() ((void)0)
#else
#include "FreeRTOS.h"
#include "task.h"
#define PowerMeterEnterCritical() taskENTER_CRITICAL()
#define PowerMeterExitCritical() taskEXIT_CRITICAL()
#endif

typedef struct
{
    uint8_t enable;
    uint8_t canBus;
    uint16_t canId;
    uint16_t freshTimeoutMs;
    uint16_t queueHead;
    uint16_t queueTail;
    PowerMeterSample snapshot;
    PowerMeterSample queue[POWER_METER_QUEUE_RING_CAPACITY];
    PowerMeterStats stats;
} PowerMeterStore;

static PowerMeterStore s_powerMeter;

void PowerMeterInit(uint8_t enable, uint8_t canBus, uint16_t canId, uint16_t freshTimeoutMs)
{
    PowerMeterEnterCritical();
    (void)memset(&s_powerMeter, 0, sizeof(s_powerMeter));
    if (enable != 0u && canBus >= 1u && canBus <= 3u && canId <= 0x7FFu && freshTimeoutMs != 0u)
    {
        s_powerMeter.enable = 1u;
        s_powerMeter.canBus = canBus;
        s_powerMeter.canId = canId;
        s_powerMeter.freshTimeoutMs = freshTimeoutMs;
    }
    PowerMeterExitCritical();
}

void PowerMeterCanRx(uint8_t bus,
                     uint16_t stdId,
                     uint8_t dlc,
                     uint8_t flags,
                     const uint8_t data[8],
                     uint32_t rxTickMs)
{
    PowerMeterSample sample;
    uint16_t next;

    if (data == NULL)
    {
        return;
    }

    PowerMeterEnterCritical();
    if (s_powerMeter.enable == 0u)
    {
        PowerMeterExitCritical();
        return;
    }
    if (flags != 0u)
    {
        s_powerMeter.stats.rejectTypeCount++;
        PowerMeterExitCritical();
        return;
    }
    if (bus != s_powerMeter.canBus)
    {
        s_powerMeter.stats.rejectBusCount++;
        PowerMeterExitCritical();
        return;
    }
    if (stdId != s_powerMeter.canId)
    {
        s_powerMeter.stats.rejectIdCount++;
        PowerMeterExitCritical();
        return;
    }
    if (dlc != 8u)
    {
        s_powerMeter.stats.rejectDlcCount++;
        PowerMeterExitCritical();
        return;
    }

    sample.rawVoltage = (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
    sample.rawCurrent = (uint16_t)data[2] | ((uint16_t)data[3] << 8u);
    sample.voltageV = (float)sample.rawVoltage / 100.0f;
    sample.currentA = (float)sample.rawCurrent / 100.0f;
    sample.powerW = sample.voltageV * sample.currentA;
    sample.rxTickMs = rxTickMs;
    sample.sequence = ++s_powerMeter.stats.validFrameCount;
    s_powerMeter.snapshot = sample;
    next = (uint16_t)((s_powerMeter.queueHead + 1u) % POWER_METER_QUEUE_RING_CAPACITY);
    if (next == s_powerMeter.queueTail)
    {
        s_powerMeter.stats.queueDropCount++;
        PowerMeterExitCritical();
        return;
    }

    s_powerMeter.queue[s_powerMeter.queueHead] = sample;
    s_powerMeter.queueHead = next;
    s_powerMeter.stats.queuedFrameCount++;
    PowerMeterExitCritical();
}

uint8_t PowerMeterReadSnapshot(PowerMeterSample *out, uint32_t nowMs)
{
    uint8_t fresh;

    if (out == NULL)
    {
        return 0u;
    }

    PowerMeterEnterCritical();
    fresh = (uint8_t)(s_powerMeter.enable != 0u &&
                      s_powerMeter.snapshot.sequence != 0u &&
                      (uint32_t)(nowMs - s_powerMeter.snapshot.rxTickMs) <= s_powerMeter.freshTimeoutMs);
    *out = s_powerMeter.snapshot;
    PowerMeterExitCritical();
    return fresh;
}

uint8_t PowerMeterIsFresh(uint32_t nowMs)
{
    PowerMeterSample sample;

    return PowerMeterReadSnapshot(&sample, nowMs);
}

uint16_t PowerMeterPopBatch(PowerMeterSample *out, uint16_t capacity)
{
    uint16_t count = 0u;

    if (out == NULL || capacity == 0u)
    {
        return 0u;
    }

    while (count < capacity)
    {
        PowerMeterEnterCritical();
        if (s_powerMeter.queueTail == s_powerMeter.queueHead)
        {
            PowerMeterExitCritical();
            break;
        }
        out[count] = s_powerMeter.queue[s_powerMeter.queueTail];
        s_powerMeter.queueTail = (uint16_t)((s_powerMeter.queueTail + 1u) % POWER_METER_QUEUE_RING_CAPACITY);
        PowerMeterExitCritical();
        count++;
    }
    return count;
}

void PowerMeterGetStats(PowerMeterStats *out)
{
    if (out == NULL)
    {
        return;
    }

    PowerMeterEnterCritical();
    *out = s_powerMeter.stats;
    PowerMeterExitCritical();
}
