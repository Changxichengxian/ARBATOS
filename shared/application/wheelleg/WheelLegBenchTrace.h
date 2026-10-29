#ifndef WHEELLEG_BENCH_TRACE_H
#define WHEELLEG_BENCH_TRACE_H

#include "WheelLegBench.h"
#include <stdint.h>

/* 0x0059：每控制帧和停机首帧共用布局；tick 保留采样时刻而非重试写入时刻。 */
typedef struct
{
    uint8_t version, kind, computed, reserved;
    uint32_t sequence, tick, stopSequence, stopTick, source, detail;
    uint32_t faults, dropped, lostEvents, phase;
    float value;
    WheelLegBenchInput input;
    WheelLegBenchOutput output;
    /* speed, yawRate, height, rollTarget, supportDelta, rollBias, parkSpeed, acceleration */
    float control[8];
} WheelLegBenchTrace;

_Static_assert(sizeof(WheelLegBenchTrace) == 272, "bench trace layout changed");

typedef struct
{
    WheelLegBenchTrace records[4];
    uint32_t first, count, lost;
} WheelLegBenchTraceQueue;

/* 写卡暂忙时保留原始事件，后续帧优先重试；满队列不覆盖更早的首因。 */
static inline int WheelLegBenchTracePush(WheelLegBenchTraceQueue *queue,
                                        const WheelLegBenchTrace *record)
{
    if (queue->count >= 4u) {
        queue->lost++;
        return 0;
    }
    queue->records[(queue->first + queue->count) % 4u] = *record;
    queue->count++;
    return 1;
}

static inline void WheelLegBenchTracePop(WheelLegBenchTraceQueue *queue)
{
    if (queue->count != 0u) {
        queue->first = (queue->first + 1u) % 4u;
        queue->count--;
    }
}

#endif
