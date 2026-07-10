/* 机械臂逐轴禁写、恢复空帧与无动作保护位图回归。 */

#include <stdint.h>
#include <stdio.h>

#include "ArmFaultPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

int main(void)
{
    const uint32_t configured = 0x3Fu;
    const uint32_t eligible = 0x3Eu;
    const uint32_t joint2 = 1u << 2;
    ArmFaultInhibitPlan plan;

    plan = ArmFaultInhibitPlanMake(configured, joint2, eligible, 0u, 0u);
    if (!TestCheck(plan.desiredMask == joint2 &&
                   plan.acquireMask == joint2 &&
                   plan.releaseMask == 0u &&
                   plan.holdZeroMask == joint2,
                   "单轴故障只能禁写并保持该轴零输出")) return 1;

    plan = ArmFaultInhibitPlanMake(configured, 0u, eligible, 1u, 0u);
    if (!TestCheck(plan.desiredMask == eligible && plan.holdZeroMask == eligible,
                   "无 deadman/动作时应禁写全部已配置 MIT 轴，不含 J0")) return 1;

    plan = ArmFaultInhibitPlanMake(configured, 0u, eligible, 0u, joint2);
    if (!TestCheck(plan.desiredMask == 0u &&
                   plan.releaseMask == joint2 &&
                   plan.holdZeroMask == joint2,
                   "恢复帧应先释放但仍保持该轴零输出")) return 1;

    plan = ArmFaultInhibitPlanMake(configured, 0u, eligible, 0u, 0u);
    if (!TestCheck(plan.releaseMask == 0u && plan.holdZeroMask == 0u,
                   "释放后的下一帧才允许恢复输出")) return 1;

    plan = ArmFaultInhibitPlanMake(0x03u, 0x24u, eligible, 0u, 0u);
    if (!TestCheck(plan.desiredMask == 0u,
                   "未配置关节不得进入禁写计划")) return 1;

    (void)puts("PASS: Arm fault policy regression");
    return 0;
}
