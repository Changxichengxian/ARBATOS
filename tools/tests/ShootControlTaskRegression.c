/* 实际发射任务循环：无需云台线程，输入失效、安全档与生命周期锁定必须逐帧生效。 */
#include <setjmp.h>
#include <stdio.h>
#include "TaskPlatform.h"
#include "ShootControlTask.h"

static jmp_buf s_done;
static unsigned s_frame;
static unsigned s_prepared;
static unsigned s_steps;
static unsigned s_failures;
static unsigned s_startDelay;

void ShootCtrlPrepare(void) { s_prepared++; }
TickType_t xTaskGetTickCount(void) { return 100u + s_startDelay + s_frame * 3u; }
uint32_t BspTimeGetTickMs(void) { return xTaskGetTickCount(); }
uint16_t RobotProfileShootControlPeriodMs(void) { return 3u; }
uint64_t RtProfBegin(void) { return 0u; }
void RtProfEnd(RtProfId id, uint64_t started) { (void)id; (void)started; }
void WatchTaskBeat(WatchTaskId id) { (void)id; }
uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t position) { return raw == position; }
uint8_t RobotLifecycleOutputAllowed(void) { return s_frame != 5u; }

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    out->online = s_frame != 6u;
    out->dataValid = s_frame != 3u;
    out->sourceTimeoutMs = 100u;
    out->sourceAgeMs = s_frame == 2u ? 101u : 0u;
    out->semantics.GimbalSafePos = 1u;
    out->control.sw[0] = s_frame == 4u ? 1u : 2u;
    return s_frame != 1u;
}

ControlResult ShootCtrlStep(const ShootCtrlInput *input, ShootCtrlOutput *output)
{
    const uint8_t expectedSafe = s_frame > 0u && s_frame < 7u;
    if (input->forceSafe != expectedSafe || input->periodMs != 3u ||
        input->tickMs != xTaskGetTickCount() || s_prepared != 1u ||
        ((input->manualInput == NULL) != (s_frame == 1u))) {
        s_failures++;
    }
    output->triggerCurrent = 0;
    s_steps++;
    return ControlResultOk;
}

void vTaskDelayUntil(TickType_t *lastWake, TickType_t ticks)
{
    if (ticks != 3u) {
        s_failures++;
    }
    *lastWake += ticks;
    if (++s_frame == 8u) {
        longjmp(s_done, 1);
    }
}

void vTaskDelay(TickType_t ticks)
{
    if (ticks != 201u || s_prepared != 0u || s_startDelay != 0u) {
        s_failures++;
    }
    s_startDelay += ticks;
}

int main(void)
{
    if (setjmp(s_done) == 0) {
        ShootControlTask(NULL);
    }
    if (s_failures != 0u || s_steps != 8u || s_startDelay != 201u) {
        fprintf(stderr, "FAIL: shoot task boundaries (%u failures, %u steps)\n", s_failures, s_steps);
        return 1;
    }
    puts("PASS: independent shoot task input and lifecycle regression");
    return 0;
}
