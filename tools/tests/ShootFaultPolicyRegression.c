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
    const uint8_t friction1 = 1u << 2;
    const uint8_t frictionMask = 0x1Eu;
    const uint8_t configured = 0x1Fu;
    ShootFaultInhibitPlan plan;

    plan = ShootFaultInhibitPlanMake(configured, trigger, 0u);
    if (!TestCheck(plan.desiredMask == trigger &&
                   plan.acquireMask == trigger &&
                   plan.holdZeroMask == trigger,
                   "trigger 隔离只能禁写 trigger")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, friction1, trigger);
    if (!TestCheck(plan.desiredMask == friction1 &&
                   plan.acquireMask == friction1 &&
                   plan.releaseMask == trigger &&
                   plan.holdZeroMask == (uint8_t)(trigger | friction1),
                   "单个摩擦轮故障只能隔离该轴，不能停掉整个 Shoot 域")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, 0u, trigger);
    if (!TestCheck(plan.desiredMask == 0u &&
                   plan.releaseMask == trigger &&
                   plan.holdZeroMask == trigger,
                   "trigger 恢复帧应释放但仍保持零输出")) return 1;

    plan = ShootFaultInhibitPlanMake(configured, trigger, configured);
    if (!TestCheck(plan.desiredMask == trigger &&
                   plan.releaseMask == (uint8_t)(configured & (uint8_t)~trigger) &&
                   plan.holdZeroMask == configured,
                   "其他轴恢复但 trigger 仍故障时，只能释放健康轴并保持整帧安全")) return 1;

    plan = ShootFaultInhibitPlanMake(0x0Eu, trigger, 0u);
    if (!TestCheck(plan.desiredMask == 0u,
                   "未配置 trigger 不得加锁")) return 1;

    if (!TestCheck(ShootFrictionFaultBlocksTrigger(friction1, frictionMask) != 0u &&
                       ShootFrictionFaultBlocksTrigger(trigger, frictionMask) == 0u &&
                       ShootFrictionFaultBlocksTrigger(0u, frictionMask) == 0u,
                   "任一摩擦轮故障必须阻止拨弹，但 trigger 自身故障不伪造摩擦轮故障")) return 1;

    if (!TestCheck(ShootGimbalGateResolve(0u, 1u, 0u, 1u) == ShootGimbalGateStopDomain &&
                       ShootGimbalGateResolve(1u, 0u, 0u, 1u) == ShootGimbalGateStopDomain &&
                       ShootGimbalGateResolve(1u, 1u, 1u, 1u) == ShootGimbalGateStopDomain,
                   "云台状态缺失、过期、无效或 ShootStop 必须停止整域")) return 1;

    if (!TestCheck(ShootGimbalGateResolve(1u, 1u, 0u, 0u) == ShootGimbalGateTriggerOnly &&
                       ShootGimbalGateResolve(1u, 1u, 0u, 1u) == ShootGimbalGateRun,
                   "IMU 降级的 fire_allowed=0 只能禁止拨弹并保留摩擦轮预热")) return 1;

    (void)puts("PASS: Shoot fault inhibit policy regression");
    return 0;
}
