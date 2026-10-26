#include <assert.h>
#include <stdio.h>
#include "WheelLegBenchGate.h"
volatile uint32_t WheelLegBenchArm;
volatile uint32_t WheelLegBenchUntilMs;
volatile uint32_t WheelLegBenchTest = 2u;
volatile uint32_t WheelLegBenchTestStopMs;
volatile uint32_t WheelLegBenchPhase;
volatile uint32_t WheelLegBenchWheels;
int main(void)
{
    uint8_t safeSeen = 0;
    assert(!WheelLegBenchRemoteReady(&safeSeen, 1, 0, 0)); /* 上电中档不能启动。 */
    assert(!WheelLegBenchRemoteReady(&safeSeen, 0, 1, 0));
    assert(WheelLegBenchRemoteReady(&safeSeen, 1, 0, 0));
    assert(!WheelLegBenchRemoteReady(&safeSeen, 0, 0, 1)); /* 掉线后中档恢复仍不能启动。 */
    assert(!WheelLegBenchRemoteReady(&safeSeen, 1, 0, 0));
    assert(!WheelLegBenchRemoteReady(&safeSeen, 0, 1, 0));
    assert(WheelLegBenchRemoteReady(&safeSeen, 1, 0, 0));
    assert(!WheelLegBenchRemoteReady(&safeSeen, 1, 0, 1)); /* 运行故障须回安全档。 */
    assert(!WheelLegBenchRemoteReady(&safeSeen, 1, 0, 0));
    uint8_t frame[8] = {255,255,255,255,255,255,255,253};
    for (unsigned id = 1; id <= 6; id++) assert(WheelLegBenchFrameAllowed(1, id, frame, 8, 10));
    frame[7] = 252;
    assert(!WheelLegBenchFrameAllowed(1, 1, frame, 8, 10));
    WheelLegBenchArm = WHEELLEG_BENCH_ARM_KEY;
    assert(!WheelLegBenchTxOpen(10));
    WheelLegBenchUntilMs = 25;
    assert(WheelLegBenchFrameAllowed(1, 1, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(2, 1, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 7, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 1, frame, 7, 10));
    assert(!WheelLegBenchTxOpen(25));
    assert(!WheelLegBenchTxOpen(26));
    WheelLegBenchUntilMs = 25;
    WheelLegBenchTest = 1;
    WheelLegBenchTestStopMs = 20;
    assert(WheelLegBenchFrameAllowed(1, 4, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 5, frame, 8, 10));
    assert(!WheelLegBenchTxOpen(20));
    WheelLegBenchTest = 3;
    WheelLegBenchPhase = 0;
    assert(!WheelLegBenchTxOpen(10));
    WheelLegBenchPhase = 1;
    assert(WheelLegBenchFrameAllowed(1, 4, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 5, frame, 8, 10));
    WheelLegBenchPhase = 2;
    assert(!WheelLegBenchFrameAllowed(1, 6, frame, 8, 10));
    WheelLegBenchPhase = 3;
    assert(!WheelLegBenchFrameAllowed(1, 5, frame, 8, 10));
    WheelLegBenchWheels = WHEELLEG_BENCH_WHEELS_KEY;
    assert(WheelLegBenchFrameAllowed(1, 5, frame, 8, 10));
    WheelLegBenchPhase = 4;
    assert(WheelLegBenchFrameAllowed(1, 4, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 5, frame, 8, 10));
    assert(!WheelLegBenchFrameAllowed(1, 6, frame, 8, 10));
    WheelLegBenchTest = 2;
    WheelLegBenchUntilMs = 8;
    assert(WheelLegBenchTxOpen(0xfffffff9u));
    WheelLegBenchArm = 0;
    assert(!WheelLegBenchTxOpen(0xfffffff9u));
    puts("bench gate: boot lock, disable-only, expiry, bus/ID bounds and wrap passed");
    return 0;
}
