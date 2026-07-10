/* 经典底盘单帧输入快照纯策略回归。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ChassisSnapshotPolicy.h"
#include "MotorHealth.h"

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

int main(void)
{
    if (!TestGimbalSnapshot() ||
        !TestCompactMotorPlan() ||
        !TestFeedbackConversionAndHealth())
    {
        return 1;
    }

    (void)puts("PASS: Chassis snapshot policy host regression");
    return 0;
}
