/* 经典底盘单帧快照策略和生产采集时序回归。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ChassisSnapshotPolicy.h"
#include "MotorHealth.h"

#define CHASSIS_MOTOR_COUNT CHASSIS_SNAPSHOT_MOTOR_COUNT

typedef struct
{
    uint32_t tick_ms;
    MotorHealthResult motor_health[CHASSIS_MOTOR_COUNT];
    motor_measure_t motor_measure[CHASSIS_MOTOR_COUNT];
} TestChassisSnapshot;

static MotorState TestFeedback[MotorCount];
static MotorState s_chassisMotorReadFeedback[CHASSIS_MOTOR_COUNT];
static uint32_t TestNowMs;
static uint32_t TestReadAdvanceMs;
static uint8_t TestReadOk;
static uint8_t TestReadCount;
static uint8_t TestClockCount;
static uint8_t TestClockBeforeRead;

uint32_t BspTimeGetTickMs(void)
{
    TestClockCount++;
    if (TestReadCount == 0u)
    {
        TestClockBeforeRead = 1u;
    }
    return TestNowMs;
}

uint8_t LowStateGetMotorMany(const MotorId *ids, MotorState *out, uint8_t count)
{
    TestReadCount++;
    TestNowMs += TestReadAdvanceMs;
    if (TestReadOk == 0u)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = TestFeedback[ids[i]];
    }
    return 1u;
}

static uint32_t ChassisMotorFeedbackTimeoutMs(uint8_t axis)
{
    (void)axis;
    return 20u;
}

#include "ChassisSnapshotUnavailableUnderTest.inc"

static void TestCaptureMotorBatch(TestChassisSnapshot *snapshot,
                                  const MotorId axisId[CHASSIS_MOTOR_COUNT],
                                  const uint8_t configured[CHASSIS_MOTOR_COUNT],
                                  const uint8_t bound[CHASSIS_MOTOR_COUNT])
{
    ChassisMotorReadPlan readPlan;
    uint8_t feedbackReadOk;
    uint32_t feedbackNowMs = 0u;

#include "ChassisSnapshotBatchUnderTest.inc"
    /* 允许编译旧生产段，时序断言应在旧段运行时失败。 */
    (void)feedbackNowMs;
}

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    (void)id;
    (void)out;
    return 0u;
}

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static int TestGimbalSnapshot(void)
{
    GimbalState state;
    ChassisGimbalSnapshot snapshot;

    (void)memset(&state, 0, sizeof(state));
    state.valid = 1u;
    state.online = 1u;
    state.ChassisStop = 1u;
    state.follow_available = 1u;
    state.turnaround_active = 1u;
    state.turnaround_frame_valid = 1u;
    state.turnaround_follow_offset_rad = 0.25f;
    state.turnaround_frame_yaw_relative = -0.75f;
    state.yaw.valid = 1u;
    state.yaw.measure.ecd = 1234u;
    state.pitch.valid = 1u;
    state.pitch.angle = 0.5f;

    ChassisGimbalSnapshotBuild(&snapshot, &state, 1u);
    if (!TestCheck(snapshot.valid != 0u && snapshot.online != 0u,
                   "有效/在线状态未复制") ||
        !TestCheck(snapshot.turnaroundActive != 0u && snapshot.frameValid != 0u,
                   "掉头状态未复制") ||
        !TestCheck(snapshot.chassisStop != 0u && snapshot.followAvailable != 0u,
                   "停机/跟随许可未复制") ||
        !TestCheck(snapshot.followOffsetRad == 0.25f && snapshot.frameYawRelative == -0.75f,
                   "掉头参考角未复制") ||
        !TestCheck(snapshot.yaw.measure.ecd == 1234u && snapshot.pitch.angle == 0.5f,
                   "云台轴反馈未复制"))
    {
        return 0;
    }

    ChassisGimbalSnapshotBuild(&snapshot, &state, 0u);
    return TestCheck(snapshot.valid == 0u && snapshot.online == 0u &&
                         snapshot.turnaroundActive == 0u && snapshot.yaw.valid == 0u,
                     "陈旧读取必须清空整份云台快照");
}

static int TestCompactMotorPlan(void)
{
    const MotorId ids[CHASSIS_SNAPSHOT_MOTOR_COUNT] = {
        Motor1,
        Motor3,
        (MotorId)MotorCount,
        Motor7,
    };
    const uint8_t configured[CHASSIS_SNAPSHOT_MOTOR_COUNT] = {1u, 0u, 1u, 1u};
    const uint8_t bound[CHASSIS_SNAPSHOT_MOTOR_COUNT] = {1u, 1u, 1u, 0u};
    ChassisMotorReadPlan plan;

    ChassisMotorReadPlanBuild(ids, configured, bound, &plan);
    return TestCheck(plan.count == 1u && plan.id[0] == Motor1 && plan.axis[0] == 0u,
                     "批量读取计划没有紧凑排除空洞") &&
           TestCheck(plan.unconfiguredMask == 0x02u,
                     "未配置轴原因掩码错误") &&
           TestCheck(plan.invalidMask == 0x0Cu,
                     "无效或未绑定轴原因掩码错误");
}

static int TestFeedbackConversionAndHealth(void)
{
    MotorState feedback;
    MotorHealthResult health;
    motor_measure_t measure;

    (void)memset(&feedback, 0, sizeof(feedback));
    feedback.rxCount = 8u;
    feedback.lastRxTick = 100u;
    feedback.driveState = (uint8_t)MotorDriveStateEnabled;
    feedback.ecd = 2000u;
    feedback.lastEcd = 1990u;
    feedback.speedRpm = -321;
    feedback.current = 456;
    feedback.temperature = 62u;

    (void)MotorHealthEval(Motor1, &feedback, 130u, 20u, &health);
    ChassisMotorMeasureFromState(&measure, &health.feedback, health.feedbackValid);
    if (!TestCheck(health.healthy == 0u &&
                       (health.reasonMask & MOTOR_HEALTH_REASON_TIMEOUT) != 0u,
                   "超时反馈应进入逐轴故障") ||
        !TestCheck(measure.ecd == 2000u && measure.last_ecd == 1990 &&
                       measure.speed_rpm == -321 && measure.given_current == 456 &&
                       measure.temperate == 62u,
                   "超时但已成功复制的同帧反馈不应被换成旧指针或提前清零"))
    {
        return 0;
    }

    ChassisMotorMeasureFromState(&measure, &feedback, 0u);
    return TestCheck(measure.ecd == 0u && measure.speed_rpm == 0 &&
                         measure.given_current == 0 && measure.temperate == 0u,
                     "批量读取失败必须安全零化反馈值");
}

static void TestResetBatch(TestChassisSnapshot *snapshot, uint32_t frameMs,
                           uint32_t advanceMs, uint32_t feedbackMs)
{
    (void)memset(snapshot, 0, sizeof(*snapshot));
    (void)memset(TestFeedback, 0, sizeof(TestFeedback));
    snapshot->tick_ms = frameMs;
    TestNowMs = frameMs;
    TestReadAdvanceMs = advanceMs;
    TestReadOk = 1u;
    TestReadCount = 0u;
    TestClockCount = 0u;
    TestClockBeforeRead = 0u;
    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        TestFeedback[i].rxCount = 8u;
        TestFeedback[i].lastRxTick = feedbackMs;
        TestFeedback[i].driveState = (uint8_t)MotorDriveStateEnabled;
        TestFeedback[i].speedRpm = (int16_t)(100 + i);
    }
}

static int TestProductionCaptureTiming(void)
{
    const MotorId ids[CHASSIS_MOTOR_COUNT] = {Motor0, Motor1, Motor2, Motor3};
    const uint8_t configured[CHASSIS_MOTOR_COUNT] = {1u, 1u, 1u, 1u};
    const uint8_t bound[CHASSIS_MOTOR_COUNT] = {1u, 1u, 1u, 1u};
    TestChassisSnapshot snapshot;
    const uint32_t frameMs[] = {100u, UINT32_MAX - 1u};
    const uint32_t feedbackMs[] = {102u, 0u};
    const char *timingMessage[] = {
        "生产采集段跨两毫秒后误报反馈超时",
        "生产采集段跨两毫秒并自然回绕后误报反馈超时",
    };
    int timingOk = 1;

    for (uint8_t caseIndex = 0u; caseIndex < 2u; caseIndex++)
    {
        TestResetBatch(&snapshot, frameMs[caseIndex], 2u, feedbackMs[caseIndex]);
        TestCaptureMotorBatch(&snapshot, ids, configured, bound);
        int batchHealthy = 1;
        for (uint8_t axis = 0u; axis < CHASSIS_MOTOR_COUNT; axis++)
        {
            if (snapshot.motor_health[axis].healthy == 0u ||
                snapshot.motor_health[axis].ageMs != 0u)
            {
                batchHealthy = 0;
            }
        }
        if (!TestCheck(batchHealthy, timingMessage[caseIndex]) ||
            !TestCheck(snapshot.tick_ms == frameMs[caseIndex],
                       "健康判断补取时间不应改写控制帧时间") ||
            !TestCheck(TestReadCount == 1u && TestClockCount == 1u &&
                           TestClockBeforeRead == 0u,
                       "整批反馈必须读取完成后只采一次健康判断时间"))
        {
            timingOk = 0;
        }
    }
    if (timingOk == 0) return 0;

    TestResetBatch(&snapshot, 100u, 2u, 102u);
    TestFeedback[Motor2].lastRxTick = 81u;
    TestCaptureMotorBatch(&snapshot, ids, configured, bound);
    if (!TestCheck(snapshot.motor_health[2].healthy == 0u &&
                       snapshot.motor_health[2].ageMs == 21u &&
                       snapshot.motor_health[2].reasonMask == MOTOR_HEALTH_REASON_TIMEOUT &&
                       snapshot.motor_health[0].healthy != 0u &&
                       snapshot.motor_health[1].healthy != 0u &&
                       snapshot.motor_health[3].healthy != 0u,
                   "补取时间必须继续隔离真实陈旧轴且保留健康轴")) return 0;

    TestResetBatch(&snapshot, 100u, 2u, 102u);
    TestReadOk = 0u;
    TestCaptureMotorBatch(&snapshot, ids, configured, bound);
    for (uint8_t axis = 0u; axis < CHASSIS_MOTOR_COUNT; axis++)
    {
        if (!TestCheck(snapshot.motor_health[axis].healthy == 0u &&
                           snapshot.motor_health[axis].reasonMask ==
                               (MOTOR_HEALTH_REASON_NO_FEEDBACK | MOTOR_HEALTH_REASON_READ_FAILED) &&
                           snapshot.motor_measure[axis].speed_rpm == 0,
                       "生产批量读取失败必须保持故障并清零反馈")) return 0;
    }

    TestResetBatch(&snapshot, 100u, 2u, 102u);
    TestFeedback[Motor1].rxCount = 0u;
    TestCaptureMotorBatch(&snapshot, ids, configured, bound);
    return TestCheck(snapshot.motor_health[1].healthy == 0u &&
                         snapshot.motor_health[1].reasonMask == MOTOR_HEALTH_REASON_NO_FEEDBACK,
                     "从未收到反馈的轴不能因补取时间变健康");
}

static int TestProductionCaptureHoles(void)
{
    const MotorId ids[CHASSIS_MOTOR_COUNT] = {Motor0, Motor1, (MotorId)MotorCount, Motor3};
    const uint8_t configured[CHASSIS_MOTOR_COUNT] = {1u, 0u, 1u, 1u};
    const uint8_t bound[CHASSIS_MOTOR_COUNT] = {1u, 1u, 1u, 0u};
    TestChassisSnapshot snapshot;

    TestResetBatch(&snapshot, 100u, 2u, 102u);
    TestCaptureMotorBatch(&snapshot, ids, configured, bound);
    return TestCheck(snapshot.motor_health[0].healthy != 0u &&
                         snapshot.motor_measure[0].speed_rpm == 100,
                     "紧凑读取不能错配剩余健康轴") &&
           TestCheck(snapshot.motor_health[1].healthy == 0u &&
                         snapshot.motor_health[1].reasonMask == MOTOR_HEALTH_REASON_NO_FEEDBACK,
                     "未配置轴必须保留无反馈故障") &&
           TestCheck(snapshot.motor_health[2].healthy == 0u &&
                         snapshot.motor_health[2].reasonMask == MOTOR_HEALTH_REASON_INVALID_ID &&
                         snapshot.motor_health[3].healthy == 0u &&
                         snapshot.motor_health[3].reasonMask == MOTOR_HEALTH_REASON_INVALID_ID,
                     "无效编号和绑定空洞必须继续隔离");
}

int main(void)
{
    if (!TestGimbalSnapshot() ||
        !TestCompactMotorPlan() ||
        !TestFeedbackConversionAndHealth() ||
        !TestProductionCaptureTiming() ||
        !TestProductionCaptureHoles())
    {
        return 1;
    }

    (void)puts("PASS: Chassis snapshot policy host regression");
    return 0;
}
