"""执行真实板凳保持函数，核对锁定、失联拒绝和重新进入行为。"""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "local/build/host-tests"


def chunk(path, start, end):
    text = (ROOT / path).read_text(encoding="utf-8")
    return text[text.index(start):text.index(end)]


PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef float fp32;
typedef unsigned MotorId;
#define WHEELLEG_SIDE_COUNT 2u
#define WHEELLEG_BENCH_TARGET_FOOT_X_M 0.0f
#define WHEELLEG_DETACHED_JOINT_TEST_KP 2.0f
#define WHEELLEG_DETACHED_JOINT_TEST_KD 0.16f
#define portTICK_PERIOD_MS 1u
typedef struct { fp32 q; uint32_t tick; uint8_t online; } MotorState;
typedef struct { MotorId front, back; } WheelLegActuatorMap;
typedef struct { fp32 foot_test_kp, foot_test_kd; } WheelLegMitConfig;
typedef struct { int unused; } WheelLegCoreTargetSmooth;
static struct { WheelLegMitConfig WheelLegMit; } g_config;
static struct {
    WheelLegActuatorMap actuator[2];
    MotorState front_fb[2], back_fb[2];
    fp32 bench_hold_front_rad[2], bench_hold_back_rad[2];
    uint8_t bench_hold_front_valid[2], bench_hold_back_valid[2];
    uint8_t bench_hold_pose_valid, auto_leg_stage;
    uint32_t auto_leg_stage_tick_ms;
    fp32 bench_hold_target_leg_m, bench_hold_target_foot_x_m;
} s_wheelleg;
static fp32 targets[4];
static unsigned count;
static uint32_t tick = 1000u;
static uint32_t xTaskGetTickCount(void) { return tick; }
static fp32 WheelLegBenchDefaultLegM(void) { return 0.1f; }
static uint8_t WheelLegFeedbackFresh(MotorId id, const MotorState *fb, uint32_t now)
{
    (void)id;
    return fb->online && now - fb->tick <= 20u;
}
static void WheelLegCoreTargetSmoothClear(WheelLegCoreTargetSmooth *p) { p->unused = 0; }
static void WheelLegTargetSmoothToState(const WheelLegCoreTargetSmooth *p) { (void)p; }
static uint8_t WheelLegPlanStateCmd(MotorId id, fp32 q, fp32 dq, fp32 kp, fp32 kd, fp32 tau)
{
    assert(id < 4u && dq == 0.0f && tau == 0.0f);
    assert(kp == 2.0f && kd == 0.16f);
    targets[id] = q;
    count++;
    return 1u;
}
'''

TESTS = r'''
static void Prepare(void)
{
    memset(&s_wheelleg, 0, sizeof(s_wheelleg));
    count = 0u;
    for (unsigned s = 0; s < 2; s++) {
        s_wheelleg.actuator[s] = (WheelLegActuatorMap){2u * s, 2u * s + 1u};
        s_wheelleg.front_fb[s] = (MotorState){0.3f + s, tick, 1u};
        s_wheelleg.back_fb[s] = (MotorState){-0.7f - s, tick, 1u};
    }
}
int main(void)
{
    Prepare();
    assert(WheelLegBenchHoldApplyJoints() == 4u && count == 4u);
    assert(targets[0] == 0.3f && targets[1] == -0.7f);
    assert(targets[2] == 1.3f && targets[3] == -1.7f);
    for (unsigned s = 0; s < 2; s++) {
        s_wheelleg.front_fb[s].q += 0.2f;
        s_wheelleg.back_fb[s].q -= 0.2f;
    }
    WheelLegBenchHoldUpdatePoseTarget();
    count = 0u;
    assert(WheelLegBenchHoldApplyJoints() == 4u && count == 4u);
    assert(targets[0] == 0.3f && targets[3] == -1.7f);
    /* 使用生产重置函数，下一次使能必须采用新站姿。 */
    WheelLegTargetSmoothReset();
    assert(!s_wheelleg.bench_hold_pose_valid);
    assert(!s_wheelleg.bench_hold_front_valid[0] && !s_wheelleg.bench_hold_back_valid[1]);
    assert(WheelLegBenchHoldApplyJoints() == 4u);
    assert(targets[0] == s_wheelleg.front_fb[0].q);
    assert(targets[3] == s_wheelleg.back_fb[1].q);
    /* 任意一个关节失联、过期或异常，都不能产生部分保持命令。 */
    for (unsigned joint = 0; joint < 4; joint++) {
        for (unsigned fault = 0; fault < 4; fault++) {
            Prepare();
            MotorState *fb = joint % 2u ? &s_wheelleg.back_fb[joint / 2u]
                                       : &s_wheelleg.front_fb[joint / 2u];
            if (fault == 0) { fb->online = 0u; }
            if (fault == 1) { fb->tick = tick - 21u; }
            if (fault == 2) { fb->q = NAN; }
            if (fault == 3) { fb->q = INFINITY; }
            assert(WheelLegBenchHoldApplyJoints() == 0u);
            assert(count == 0u && !s_wheelleg.bench_hold_pose_valid);
            for (unsigned s = 0; s < 2; s++) {
                assert(!s_wheelleg.bench_hold_front_valid[s]);
                assert(!s_wheelleg.bench_hold_back_valid[s]);
            }
        }
    }
    puts("bench hold: capture, fixed target, re-entry and 16 invalid-feedback cases passed");
    return 0;
}
'''


def main():
    runtime = chunk("shared/application/wheelleg/WheelLegMitRuntimeHelpers.inc",
                    "static uint8_t WheelLegBenchHoldCapture(void)",
                    "static MotorId WheelLegSingleTestActuator(void)")
    reset = chunk("shared/application/wheelleg/WheelLegMitMathHelpers.inc",
                  "static void WheelLegTargetSmoothReset(void)",
                  "static void WheelLegBalanceStateReset(void)")
    BUILD.mkdir(parents=True, exist_ok=True)
    source = BUILD / "wheelleg-bench-hold.c"
    output = BUILD / "wheelleg-bench-hold.exe"
    source.write_text(PREFIX + reset + runtime + TESTS, encoding="utf-8")
    zig = shutil.which("zig")
    if not zig:
        raise RuntimeError("zig is required")
    subprocess.run([zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(output)], check=True)
    subprocess.run([str(output)], check=True)


if __name__ == "__main__":
    main()
