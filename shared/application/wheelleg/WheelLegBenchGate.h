#ifndef WHEELLEG_BENCH_GATE_H
#define WHEELLEG_BENCH_GATE_H

#include <stdint.h>

#define WHEELLEG_BENCH_ARM_KEY 0x42454e43u
extern volatile uint32_t WheelLegBenchArm;
extern volatile uint32_t WheelLegBenchUntilMs;
extern volatile uint32_t WheelLegBenchTest;
extern volatile uint32_t WheelLegBenchTestStopMs;
extern volatile uint32_t WheelLegBenchPhase;

static inline int WheelLegBenchTxOpen(uint32_t now)
{
    return WheelLegBenchArm == WHEELLEG_BENCH_ARM_KEY &&
           (WheelLegBenchTest == 1u || WheelLegBenchTest == 2u ||
            (WheelLegBenchTest == 3u && WheelLegBenchPhase >= 1u && WheelLegBenchPhase <= 3u)) &&
           (WheelLegBenchTest != 1u || (int32_t)(WheelLegBenchTestStopMs - now) > 0) &&
           (int32_t)(WheelLegBenchUntilMs - now) > 0 &&
           (uint32_t)(WheelLegBenchUntilMs - now) <= 15u;
}

static inline int WheelLegBenchFrameAllowed(uint8_t bus, uint16_t id,
                                          const uint8_t *data, uint8_t length,
                                          uint32_t now)
{
    if (bus != 1u || id < 1u || id > 6u || length != 8u || data == 0) return 0;
    if (WheelLegBenchTxOpen(now) &&
        (WheelLegBenchTest == 2u ||
         (WheelLegBenchTest == 3u && WheelLegBenchPhase == 3u) || id <= 4u)) return 1;
    /* 未解锁时只允许失能帧，兼作失能状态的反馈查询。 */
    for (unsigned i = 0; i < 7; i++) if (data[i] != 0xffu) return 0;
    return data[7] == 0xfdu;
}

#endif
