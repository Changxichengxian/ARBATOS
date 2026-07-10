/* 云台/经典底盘逐轴禁写、IMU 影响范围和恢复空帧回归。 */

#include <stdint.h>
#include <stdio.h>

#include "GimbalFaultPolicy.h"
#include "MotorAxisFaultPolicy.h"

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
    const uint32_t allGimbal = GIMBAL_FAULT_MASK_YAW |
                               GIMBAL_FAULT_MASK_YAW_UPPER |
                               GIMBAL_FAULT_MASK_PITCH;
    const uint32_t wheel2 = 1u << 2;
    MotorAxisFaultInhibitPlan plan;
    uint32_t mask;

    plan = MotorAxisFaultInhibitPlanMake(0x0Fu, wheel2, 0u);
    if (!TestCheck(plan.desiredMask == wheel2 &&
                   plan.acquireMask == wheel2 &&
                   plan.releaseMask == 0u &&
                   plan.holdZeroMask == wheel2,
                   "单个底盘轮故障只能禁写该轮")) return 1;

    plan = MotorAxisFaultInhibitPlanMake(0x0Fu, 0u, wheel2);
    if (!TestCheck(plan.desiredMask == 0u &&
                   plan.releaseMask == wheel2 &&
                   plan.holdZeroMask == wheel2,
                   "恢复帧释放禁写后仍必须保持该轴零输出")) return 1;

    plan = MotorAxisFaultInhibitPlanMake(0x03u, 0x0Cu, 0u);
    if (!TestCheck(plan.desiredMask == 0u,
                   "未配置轴不得进入禁写计划")) return 1;

    plan = MotorAxisFaultInhibitPlanMake(0u, 0u, wheel2);
    if (!TestCheck(plan.releaseMask == wheel2 && plan.holdZeroMask == wheel2,
                   "故障后运行时关闭配置也必须释放旧禁写并保持恢复帧为零")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_NORMAL,
                                  allGimbal,
                                  0u,
                                  0u);
    if (!TestCheck(mask == (GIMBAL_FAULT_MASK_YAW | GIMBAL_FAULT_MASK_PITCH),
                   "普通单 yaw 云台的 IMU 故障只影响 yaw 和 pitch")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_GIMBAL_DUAL,
                                  allGimbal,
                                  0u,
                                  1u);
    if (!TestCheck(mask == allGimbal,
                   "双 yaw 补偿启用时三个输出轴都依赖 IMU")) return 1;

    mask = GimbalFaultAimAxisMask(ROBOT_RUN_VARIANT_GIMBAL_DUAL,
                                  allGimbal,
                                  0u);
    if (!TestCheck(mask == (GIMBAL_FAULT_MASK_YAW | GIMBAL_FAULT_MASK_PITCH),
                   "双 yaw 射击就绪不得被辅助 yaw 单轴故障扩大")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY,
                                  allGimbal,
                                  0u,
                                  1u);
    if (!TestCheck(mask == GIMBAL_FAULT_MASK_YAW,
                   "yaw-only 的 IMU 故障不得连带 pitch 和辅助 yaw")) return 1;

    mask = GimbalFaultAimAxisMask(ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY,
                                  allGimbal,
                                  0u);
    if (!TestCheck(mask == GIMBAL_FAULT_MASK_YAW,
                   "yaw-only 射击就绪不得要求 pitch")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY,
                                  allGimbal,
                                  1u,
                                  1u);
    if (!TestCheck(mask == GIMBAL_FAULT_MASK_YAW_UPPER,
                   "上 yaw 接管时只禁当前控制 yaw")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY,
                                  allGimbal,
                                  0u,
                                  1u);
    if (!TestCheck(mask == GIMBAL_FAULT_MASK_PITCH,
                   "pitch-only 的 IMU 故障不得连带 yaw")) return 1;

    mask = GimbalFaultImuAxisMask(ROBOT_RUN_VARIANT_SHOOT_COMBO,
                                  allGimbal,
                                  0u,
                                  1u);
    if (!TestCheck(mask == 0u,
                   "不运行云台输出的变体不得因 IMU 故障占用云台轴")) return 1;

    (void)puts("PASS: motor axis fault policy regression");
    return 0;
}
