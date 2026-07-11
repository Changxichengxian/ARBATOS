/*
 * WheelLeg 六轴输出计划主机回归。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "WheelLegOutputPlan.h"

typedef struct
{
    MotorCmd cmd;
    ControlOutputStamp owner;
    uint32_t seq;
    uint8_t written;
} TestPublishedAxis;

static TestPublishedAxis s_published[WHEELLEG_OUTPUT_AXIS_COUNT];
static uint8_t s_available_mask;
static uint8_t s_inhibit_mask;
static uint8_t s_publish_calls;
static uint32_t s_seq;

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static ControlOutputPermit TestPermit(uint32_t cycle)
{
    ControlOutputPermit permit;

    (void)memset(&permit, 0, sizeof(permit));
    permit.stamp.authorityEpoch = 7u;
    permit.stamp.cycleSeq = cycle;
    permit.stamp.controllerId = 11u;
    permit.stamp.domain = 4u;
    permit.stamp.valid = 1u;
    permit.actuatorMask = 0x0003F000u;
    return permit;
}

static void TestResetPublish(void)
{
    (void)memset(s_published, 0, sizeof(s_published));
    s_available_mask = WHEELLEG_OUTPUT_ALL_MASK;
    s_inhibit_mask = 0u;
    s_publish_calls = 0u;
}

static uint8_t TestPublishStrict(const MotorId *ids,
                                 const MotorCmd *cmds,
                                 uint8_t count,
                                 const ControlOutputPermit *permit)
{
    uint8_t i;

    if (ids == NULL || cmds == NULL || permit == NULL ||
        count != WHEELLEG_OUTPUT_AXIS_COUNT)
    {
        return 0u;
    }

    s_publish_calls++;
    for (i = 0u; i < count; i++)
    {
        const uint8_t bit = (uint8_t)(1u << i);

        if ((s_available_mask & bit) == 0u || (s_inhibit_mask & bit) != 0u ||
            ids[i] != (MotorId)((uint32_t)Motor12 + i))
        {
            return 0u;
        }
    }

    s_seq++;
    for (i = 0u; i < count; i++)
    {
        s_published[i].cmd = cmds[i];
        s_published[i].owner = permit->stamp;
        s_published[i].seq = s_seq;
        s_published[i].written = 1u;
    }
    return 1u;
}

static int TestWrittenShareStampAndSeq(const ControlOutputPermit *permit,
                                       uint8_t expected_count)
{
    uint32_t seq = 0u;
    uint8_t count = 0u;
    uint8_t i;

    for (i = 0u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        if (s_published[i].written == 0u)
        {
            continue;
        }
        if (seq == 0u)
        {
            seq = s_published[i].seq;
        }
        if (s_published[i].seq != seq ||
            ControlOutputStampEqual(&s_published[i].owner, &permit->stamp) == 0u)
        {
            return 0;
        }
        count++;
    }
    return count == expected_count;
}

static void TestIds(MotorId ids[WHEELLEG_OUTPUT_AXIS_COUNT])
{
    uint8_t i;

    for (i = 0u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        ids[i] = (MotorId)((uint32_t)Motor12 + i);
    }
}

static int TestFillSixStateTorque(WheelLegOutputPlan *plan, fp32 base)
{
    uint8_t i;

    for (i = 0u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        if (WheelLegOutputPlanSetStateTorque(plan,
                                            (WheelLegOutputAxis)i,
                                            base + (fp32)i,
                                            0.0f,
                                            2.0f,
                                            0.2f,
                                            0.1f * (fp32)i) == 0u)
        {
            return 0;
        }
    }
    return plan->commandMask == WHEELLEG_OUTPUT_ALL_MASK;
}

static int TestFillMode(WheelLegOutputPlan *plan, uint8_t mode)
{
    uint8_t i;

    for (i = 0u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        const uint8_t wheel =
            (uint8_t)(i == (uint8_t)WheelLegOutputLeftWheel ||
                      i == (uint8_t)WheelLegOutputRightWheel);
        fp32 position = 0.0f;
        fp32 kp = 0.0f;
        fp32 kd = 0.0f;
        fp32 torque = 0.2f + 0.1f * (fp32)i;

        if (mode == 0u && wheel == 0u)
        {
            position = 1.0f + (fp32)i;
            kp = 2.0f;
            kd = 0.2f;
            torque = 0.0f;
        }
        else if (mode == 1u && wheel == 0u)
        {
            position = 20.0f + (fp32)i;
            kp = 3.0f;
            kd = 0.3f;
            torque = 0.0f;
        }

        if (WheelLegOutputPlanSetStateTorque(plan,
                                            (WheelLegOutputAxis)i,
                                            position,
                                            0.0f,
                                            kp,
                                            kd,
                                            torque) == 0u)
        {
            return 0;
        }
    }
    return plan->commandMask == WHEELLEG_OUTPUT_ALL_MASK;
}

static int TestThreeModesOneBatch(void)
{
    MotorId ids[WHEELLEG_OUTPUT_AXIS_COUNT];
    WheelLegOutputPlan plan;
    ControlOutputPermit permit;
    uint8_t mode;

    TestIds(ids);
    for (mode = 0u; mode < 3u; mode++)
    {
        permit = TestPermit((uint32_t)mode + 1u);
        TestResetPublish();
        if (!TestCheck(WheelLegOutputPlanBegin(&plan, ids) != 0u, "三模式 Begin 应成功") ||
            !TestCheck(TestFillMode(&plan, mode) != 0,
                       "Bench/Position/VMC 都必须覆盖六轴") ||
            !TestCheck(WheelLegOutputPlanCommit(&plan, &permit, TestPublishStrict) != 0u,
                       "完整六轴计划应一次发布六轴") ||
            !TestCheck(s_publish_calls == 1u, "每种模式只能调用一次批量发布") ||
            !TestCheck(TestWrittenShareStampAndSeq(&permit, 6u),
                       "左右六轴必须共享同一 seq 和 stamp"))
        {
            return 0;
        }
        if (!TestCheck((mode == 0u && s_published[WheelLegOutputLeftFront].cmd.kp == 2.0f &&
                        s_published[WheelLegOutputLeftWheel].cmd.kp == 0.0f) ||
                       (mode == 1u && s_published[WheelLegOutputLeftFront].cmd.q == 20.0f &&
                        s_published[WheelLegOutputLeftFront].cmd.kp == 3.0f) ||
                       (mode == 2u && s_published[WheelLegOutputLeftFront].cmd.q == 0.0f &&
                        s_published[WheelLegOutputLeftFront].cmd.kp == 0.0f &&
                        s_published[WheelLegOutputLeftFront].cmd.tau != 0.0f),
                       "三种模式的六轴计划内容不能混淆"))
        {
            return 0;
        }
    }
    return 1;
}

static int TestMissingOrInhibitedAxisRejectsWholeBatch(void)
{
    MotorId ids[WHEELLEG_OUTPUT_AXIS_COUNT];
    WheelLegOutputPlan plan;
    ControlOutputPermit permit = TestPermit(20u);
    uint32_t seq_before;

    TestIds(ids);
    TestResetPublish();
    seq_before = s_seq;
    s_available_mask &= (uint8_t)~(1u << WheelLegOutputRightFront);
    if (!TestCheck(WheelLegOutputPlanBegin(&plan, ids) != 0u, "缺实例场景 Begin 应成功") ||
        !TestCheck(TestFillSixStateTorque(&plan, 20.0f) != 0, "缺实例前仍应生成完整六槽计划") ||
        !TestCheck(WheelLegOutputPlanCommit(&plan, &permit, TestPublishStrict) == 0u,
                   "单轴缺实例必须拒绝整个六轴批次") ||
        !TestCheck(s_publish_calls == 1u && s_seq == seq_before,
                   "缺实例不得产生新命令序号") ||
        !TestCheck(TestWrittenShareStampAndSeq(&permit, 0u),
                   "缺实例不得部分写入健康轴"))
    {
        return 0;
    }

    permit = TestPermit(21u);
    TestResetPublish();
    seq_before = s_seq;
    s_inhibit_mask = (uint8_t)(1u << WheelLegOutputLeftWheel);
    if (!TestCheck(WheelLegOutputPlanBegin(&plan, ids) != 0u, "禁写场景 Begin 应成功") ||
        !TestCheck(TestFillSixStateTorque(&plan, 30.0f) != 0, "禁写前仍应生成完整六槽计划") ||
        !TestCheck(WheelLegOutputPlanCommit(&plan, &permit, TestPublishStrict) == 0u,
                   "单轴禁写必须拒绝整个六轴批次") ||
        !TestCheck(s_publish_calls == 1u && s_seq == seq_before,
                   "禁写拒绝不得产生新命令序号") ||
        !TestCheck(TestWrittenShareStampAndSeq(&permit, 0u),
                   "禁写拒绝不得部分写入其他轴") ||
        !TestCheck(WheelLegOutputPlanCommit(&plan, &permit, TestPublishStrict) == 0u,
                   "同一计划不能二次发布") ||
        !TestCheck(s_publish_calls == 1u, "二次发布必须在调用下层前拒绝"))
    {
        return 0;
    }
    return 1;
}

static int TestUnusedAxesHaveExplicitSafeCommand(void)
{
    MotorId ids[WHEELLEG_OUTPUT_AXIS_COUNT];
    WheelLegOutputPlan plan;
    ControlOutputPermit permit = TestPermit(30u);
    uint8_t i;

    TestIds(ids);
    TestResetPublish();
    if (!TestCheck(WheelLegOutputPlanBegin(&plan, ids) != 0u, "安全替代 Begin 应成功") ||
        !TestCheck(WheelLegOutputPlanSetStateTorque(&plan,
                                                   WheelLegOutputLeftFront,
                                                   1.0f,
                                                   0.0f,
                                                   1.0f,
                                                   0.1f,
                                                   0.0f) != 0u,
                   "单轴测试命令应进入计划") ||
        !TestCheck(WheelLegOutputPlanCommit(&plan, &permit, TestPublishStrict) != 0u,
                   "测试模式也应一次提交完整六槽计划"))
    {
        return 0;
    }

    for (i = 1u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        if (!TestCheck(s_published[i].cmd.active != 0u &&
                       s_published[i].cmd.mode == (uint8_t)MotorModeDisable,
                       "未参与测试的轴必须带显式 Disable"))
        {
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    if (!TestThreeModesOneBatch() ||
        !TestMissingOrInhibitedAxisRejectsWholeBatch() ||
        !TestUnusedAxesHaveExplicitSafeCommand())
    {
        return 1;
    }

    (void)printf("WheelLeg output plan regression passed.\n");
    return 0;
}
