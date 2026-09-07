/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef POWER_METER_H
#define POWER_METER_H

#include <stdint.h>

#define POWER_METER_QUEUE_CAPACITY 256u
#define POWER_METER_QUEUE_RING_CAPACITY (POWER_METER_QUEUE_CAPACITY + 1u)

/* 与 BspCanFrame.flags 的 FD/BRS 位一致；扩展帧和远程帧会在 BspCan 接收层前丢弃。 */
#define POWER_METER_FRAME_FLAG_FD 0x01u
#define POWER_METER_FRAME_FLAG_BRS 0x02u
#define POWER_METER_FRAME_FLAG_EXTENDED 0x04u
#define POWER_METER_FRAME_FLAG_REMOTE 0x08u

typedef struct
{
    uint16_t rawVoltage;
    uint16_t rawCurrent;
    float voltageV;
    float currentA;
    float powerW;
    uint32_t rxTickMs;
    uint32_t sequence;
} PowerMeterSample;

typedef struct
{
    uint32_t validFrameCount;
    uint32_t queuedFrameCount;
    uint32_t queueDropCount;
    uint32_t rejectBusCount;
    uint32_t rejectIdCount;
    uint32_t rejectDlcCount;
    uint32_t rejectTypeCount;
} PowerMeterStats;

void PowerMeterInit(uint8_t enable, uint8_t canBus, uint16_t canId, uint16_t freshTimeoutMs);
void PowerMeterCanRx(uint8_t bus,
                     uint16_t stdId,
                     uint8_t dlc,
                     uint8_t flags,
                     const uint8_t data[8],
                     uint32_t rxTickMs);
uint8_t PowerMeterReadSnapshot(PowerMeterSample *out, uint32_t nowMs);
uint8_t PowerMeterIsFresh(uint32_t nowMs);
uint16_t PowerMeterPopBatch(PowerMeterSample *out, uint16_t capacity);
void PowerMeterGetStats(PowerMeterStats *out);

#endif
