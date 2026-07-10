/* Shoot trigger/摩擦轮域禁写与恢复空帧回归。 */

#include <stdint.h>
#include <stdio.h>

#include "ShootFaultPolicy.h"

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
    const uint8_t trigger = 1u << 0;
    const uint8_t configured = 0x1Fu;
    ShootFaultInhibitPlan plan;

    plan = ShootFaultInhibitPlanMake(configured, trigger, 1u, 0u, 0u);
    if (!TestCheck(plan.desiredMask == trigger &&
                   plan.acquireMask == trigger &&
                   plan.holdZeroMask == trigger,
                   "trigger 隔离只能禁写 trigger")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, trigger, 0u, 1u, trigger);
    if (!TestCheck(plan.desiredMask == configured &&
                   plan.releaseMask == 0u &&
                   plan.holdZeroMask == configured,
                   "摩擦轮域停机应禁写全部已配置 Shoot 电机")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, trigger, 0u, 0u, trigger);
    if (!TestCheck(plan.desiredMask == 0u &&
                   plan.releaseMask == trigger &&
                   plan.holdZeroMask == trigger,
                   "trigger 恢复帧应释放但仍保持零输出")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, trigger, 1u, 0u, configured);
    if (!TestCheck(plan.desiredMask == trigger &&
                   plan.releaseMask == (uint8_t)(configured & (uint8_t)~trigger) &&
                   plan.holdZeroMask == configured,
                   "域恢复但 trigger 仍故障时，只能释放摩擦轮并保持整帧安全")) return 1;

    plan = ShootFaultInhibitPlanMake(0x0Eu, trigger, 1u, 0u, 0u);
    if (!TestCheck(plan.desiredMask == 0u,
                   "未配置 trigger 不得加锁")) return 1;

    if (!TestCheck(ShootGimbalStateBlocksFire(0u, 1u, 1u) != 0u &&
                       ShootGimbalStateBlocksFire(1u, 0u, 1u) != 0u &&
                       ShootGimbalStateBlocksFire(1u, 1u, 0u) != 0u &&
                       ShootGimbalStateBlocksFire(1u, 1u, 1u) == 0u,
                   "云台状态缺失、过期、无效或禁止射击都必须阻断输出")) return 1;

    (void)puts("PASS: Shoot fault inhibit policy regression");
    return 0;
}
