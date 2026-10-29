#include "WheelLegBenchTrace.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    WheelLegBenchTraceQueue queue = {0};
    WheelLegBenchTrace current = {.version=1, .kind=1, .tick=101, .source=4, .detail=12};
    current.input.wheelRate[1] = 37.5f;
    assert(WheelLegBenchTracePush(&queue, &current));
    /* 模拟事件发生后连续状态被清零，以及写卡暂时无法接收。 */
    current.input.wheelRate[1] = 0;
    current.tick = 200;
    for (unsigned i = 1; i < 4; i++) {
        current.stopSequence = i;
        assert(WheelLegBenchTracePush(&queue, &current));
    }
    assert(!WheelLegBenchTracePush(&queue, &current) && queue.lost == 1);
    assert(queue.records[queue.first].tick == 101);
    assert(queue.records[queue.first].source == 4 && queue.records[queue.first].detail == 12);
    assert(queue.records[queue.first].input.wheelRate[1] == 37.5f);
    WheelLegBenchTracePop(&queue);
    for (unsigned i = 1; i < 4; i++) {
        assert(queue.records[queue.first].stopSequence == i);
        WheelLegBenchTracePop(&queue);
    }
    assert(queue.count == 0);
    current.tick = 999;
    assert(WheelLegBenchTracePush(&queue, &current));
    assert(queue.records[queue.first].tick == 999);
    puts("bench stop trace: capture preserved, retry queue and explicit overflow passed");
    return 0;
}
