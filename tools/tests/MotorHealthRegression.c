/*
 * MotorHealth 主机回归。测试直接注入反馈，生产读取入口使用内存桩替代 LowState。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LowCmd.h"

static MotorState s_feedback[MotorCount];
static uint8_t s_readAllowed[MotorCount];

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    if ((uint32_t)id >= (uint32_t)MotorCount || out == NULL || s_readAllowed[id] == 0u)
    {
        return 0u;
    }

    *out = s_feedback[id];
    return 1u;
}

#include "MotorHealth.c"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static MotorState TestFeedback(uint32_t rxCount,
                               uint32_t lastRxTick,
                               MotorDriveState driveState,
                               uint8_t online)
{
    MotorState feedback;

    (void)memset(&feedback, 0, sizeof(feedback));
    feedback.rxCount = rxCount;
    feedback.lastRxTick = lastRxTick;
    feedback.driveState = (uint8_t)driveState;
    feedback.online = online;
    return feedback;
}

static int TestPureEvaluation(void)
{
    MotorHealthResult result;
    MotorState feedback = TestFeedback(3u, 100u, MotorDriveStateEnabled, 0u);

    if (!TestCheck(MotorHealthEval(Motor0, &feedback, 110u, 20u, &result) != 0u,
                   "健康反馈判定失败") ||
        !TestCheck(result.healthy != 0u && result.fresh != 0u && result.ageMs == 10u,
                   "新鲜反馈应健康") ||
        !TestCheck(result.feedbackValid != 0u && result.feedback.rxCount == 3u,
                   "判定结果应保留完整反馈快照"))
    {
        return 0;
    }

    feedback.online = 1u;
    feedback.lastRxTick = 89u;
    if (!TestCheck(MotorHealthEval(Motor0, &feedback, 110u, 20u, &result) != 0u,
                   "超时反馈判定失败") ||
        !TestCheck(result.healthy == 0u && result.ageMs == 21u &&
                       (result.reasonMask & MOTOR_HEALTH_REASON_TIMEOUT) != 0u,
                   "online 粘住时仍应按时间判离线"))
    {
        return 0;
    }

    feedback = TestFeedback(0u, 110u, MotorDriveStateUnknown, 1u);
    if (!TestCheck(MotorHealthEval(Motor1, &feedback, 110u, 20u, &result) != 0u,
                   "无反馈判定失败") ||
        !TestCheck(result.healthy == 0u && result.ageMs == MOTOR_HEALTH_AGE_UNKNOWN &&
                       (result.reasonMask & MOTOR_HEALTH_REASON_NO_FEEDBACK) != 0u,
                   "rxCount 为零不能因 online 置位而健康"))
    {
        return 0;
    }

    feedback = TestFeedback(1u, 100u, MotorDriveStateOffline, 1u);
    (void)MotorHealthEval(Motor2, &feedback, 105u, 20u, &result);
    if (!TestCheck(result.fresh != 0u && result.healthy == 0u &&
                       (result.reasonMask & MOTOR_HEALTH_REASON_DRIVE_OFFLINE) != 0u,
                   "新鲜反馈中的 Offline 驱动状态仍应判故障"))
    {
        return 0;
    }

    feedback.driveState = (uint8_t)MotorDriveStateFault;
    (void)MotorHealthEval(Motor2, &feedback, 105u, 20u, &result);
    if (!TestCheck((result.reasonMask & MOTOR_HEALTH_REASON_DRIVE_FAULT) != 0u,
                   "驱动 Fault 状态未进入原因位"))
    {
        return 0;
    }

    feedback = TestFeedback(1u, 101u, MotorDriveStateEnabled, 1u);
    (void)MotorHealthEval(Motor3, &feedback, 100u, 10u, &result);
    if (!TestCheck(result.healthy != 0u && result.ageMs == 0u,
                   "先采时间、后读到下一毫秒反馈时不应下溢成超时"))
    {
        return 0;
    }

    feedback = TestFeedback(1u, 102u, MotorDriveStateEnabled, 1u);
    (void)MotorHealthEval(Motor3, &feedback, 100u, 10u, &result);
    if (!TestCheck(result.healthy == 0u &&
                       (result.reasonMask & MOTOR_HEALTH_REASON_TIMEOUT) != 0u,
                   "超过一毫秒的未来时间不能被静默夹成健康"))
    {
        return 0;
    }

    feedback = TestFeedback(1u, 0u, MotorDriveStateEnabled, 1u);
    (void)MotorHealthEval(Motor3, &feedback, 0x90000000u, 10u, &result);
    if (!TestCheck(result.healthy == 0u && result.ageMs == 0x90000000u,
                   "真实长期陈旧反馈不能因差值高位为一而变成健康"))
    {
        return 0;
    }

    feedback = TestFeedback(1u, UINT32_MAX - 5u, MotorDriveStateDisabled, 0u);
    (void)MotorHealthEval(Motor3, &feedback, 4u, 10u, &result);
    return TestCheck(result.healthy != 0u && result.ageMs == 10u,
                     "毫秒计数回绕后新鲜度计算错误");
}

static int TestReadAndBatch(void)
{
    const MotorId ids[] = {Motor2, Motor5, Motor7, Motor9};
    MotorHealthResult result;
    MotorHealthBatch batch;

    (void)memset(s_feedback, 0, sizeof(s_feedback));
    (void)memset(s_readAllowed, 0, sizeof(s_readAllowed));

    s_feedback[Motor2] = TestFeedback(2u, 190u, MotorDriveStateEnabled, 0u);
    s_feedback[Motor5] = TestFeedback(2u, 170u, MotorDriveStateEnabled, 1u);
    s_feedback[Motor7] = TestFeedback(2u, 195u, MotorDriveStateFault, 1u);
    s_readAllowed[Motor2] = 1u;
    s_readAllowed[Motor5] = 1u;
    s_readAllowed[Motor7] = 1u;

    if (!TestCheck(MotorHealthRead(Motor2, 200u, 20u, &result) != 0u && result.healthy != 0u,
                   "生产单轴读取入口未使用 LowState 快照") ||
        !TestCheck(MotorHealthRead((MotorId)MotorCount, 200u, 20u, &result) == 0u &&
                       (result.reasonMask & MOTOR_HEALTH_REASON_INVALID_ID) != 0u,
                   "非法 MotorId 应拒绝读取"))
    {
        return 0;
    }

    if (!TestCheck(MotorHealthReadMany(ids, 4u, 200u, 20u, &batch) == 0u,
                   "批量中有读取失败时返回值应报告失败") ||
        !TestCheck(batch.count == 4u && batch.faultMask == 0x0Eu,
                   "批量 fault mask 应按输入位置标出超时、驱动故障和读取失败") ||
        !TestCheck(batch.item[0].feedback.rxCount == 2u && batch.item[0].healthy != 0u,
                   "批量结果未保留健康轴反馈") ||
        !TestCheck((batch.item[1].reasonMask & MOTOR_HEALTH_REASON_TIMEOUT) != 0u,
                   "批量超时原因缺失") ||
        !TestCheck((batch.item[2].reasonMask & MOTOR_HEALTH_REASON_DRIVE_FAULT) != 0u,
                   "批量驱动故障原因缺失") ||
        !TestCheck((batch.item[3].reasonMask & MOTOR_HEALTH_REASON_READ_FAILED) != 0u,
                   "批量读取失败原因缺失"))
    {
        return 0;
    }

    return TestCheck(MotorHealthReadMany(ids,
                                         (uint8_t)(MOTOR_HEALTH_MAX_MOTORS + 1u),
                                         200u,
                                         20u,
                                         &batch) == 0u && batch.count == 0u,
                     "超过固定容量的批量查询必须拒绝");
}

static int TestInjectedBatch(void)
{
    const MotorId ids[] = {Motor1, Motor4, Motor8};
    MotorState feedback[3];
    MotorHealthBatch batch;

    feedback[0] = TestFeedback(1u, 50u, MotorDriveStateReady, 0u);
    feedback[1] = TestFeedback(1u, 40u, MotorDriveStateEnabled, 1u);
    feedback[2] = TestFeedback(0u, 50u, MotorDriveStateUnknown, 1u);

    if (!TestCheck(MotorHealthEvalMany(ids, feedback, 3u, 50u, 5u, &batch) != 0u,
                   "纯批量判定拒绝了有效注入") ||
        !TestCheck(batch.faultMask == 0x06u,
                   "纯批量判定 fault mask 错误"))
    {
        return 0;
    }

    return TestCheck(batch.item[0].healthy != 0u &&
                         (batch.item[1].reasonMask & MOTOR_HEALTH_REASON_TIMEOUT) != 0u &&
                         (batch.item[2].reasonMask & MOTOR_HEALTH_REASON_NO_FEEDBACK) != 0u,
                     "纯批量判定原因不完整");
}

int main(void)
{
    if (!TestPureEvaluation() || !TestReadAndBatch() || !TestInjectedBatch())
    {
        return 1;
    }

    (void)puts("PASS: MotorHealth host regression");
    return 0;
}
