#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "PowerMeter.h"

static void Feed(uint8_t bus, uint16_t id, uint8_t dlc, uint8_t flags, uint16_t voltage, uint16_t current, uint32_t tick)
{
    const uint8_t data[8] = {
        (uint8_t)voltage, (uint8_t)(voltage >> 8u),
        (uint8_t)current, (uint8_t)(current >> 8u),
        0xA5u, 0x5Au, 0xCCu, 0x33u,
    };

    PowerMeterCanRx(bus, id, dlc, flags, data, tick);
}

int main(void)
{
    PowerMeterSample sample;
    PowerMeterStats stats;

    PowerMeterInit(1u, 1u, 0x212u, 20u);
    Feed(1u, 0x212u, 8u, 0u, 2450u, 65535u, 100u);
    assert(PowerMeterReadSnapshot(&sample, 120u));
    assert(sample.rawVoltage == 2450u && sample.rawCurrent == 65535u);
    assert(fabsf(sample.voltageV - 24.5f) < 0.0001f);
    assert(fabsf(sample.currentA - 655.35f) < 0.0001f);
    assert(fabsf(sample.powerW - 16056.075f) < 0.01f && sample.sequence == 1u);
    assert(!PowerMeterIsFresh(121u));

    Feed(2u, 0x212u, 8u, 0u, 1u, 1u, 101u);
    Feed(1u, 0x213u, 8u, 0u, 1u, 1u, 101u);
    Feed(1u, 0x212u, 7u, 0u, 1u, 1u, 101u);
    Feed(1u, 0x212u, 8u, POWER_METER_FRAME_FLAG_EXTENDED, 1u, 1u, 101u);
    Feed(1u, 0x212u, 8u, POWER_METER_FRAME_FLAG_REMOTE, 1u, 1u, 101u);
    PowerMeterGetStats(&stats);
    assert(stats.validFrameCount == 1u);
    assert(stats.rejectBusCount == 1u && stats.rejectIdCount == 1u);
    assert(stats.rejectDlcCount == 1u && stats.rejectTypeCount == 2u);

    PowerMeterInit(1u, 1u, 0x212u, 20u);
    for (uint16_t i = 0u; i <= POWER_METER_QUEUE_CAPACITY; i++)
    {
        Feed(1u, 0x212u, 8u, 0u, i, i, i);
    }
    PowerMeterGetStats(&stats);
    assert(stats.validFrameCount == POWER_METER_QUEUE_CAPACITY + 1u);
    assert(stats.queuedFrameCount == POWER_METER_QUEUE_CAPACITY);
    assert(stats.queueDropCount == 1u);
    assert(PowerMeterPopBatch(&sample, 1u) == 1u && sample.sequence == 1u);
    puts("PASS: CAN power meter conversion, rejection, freshness and queue overflow");
    return 0;
}
