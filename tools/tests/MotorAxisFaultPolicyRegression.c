/* 云台/经典底盘逐轴禁写、IMU 影响范围和恢复空帧回归。 */

#include <stdint.h>
#include <stdio.h>

#include "ChassisPowerLimiter.h"
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

static void ApplyHoldZeroMask(fp32 currents[4], uint32_t holdZeroMask)
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (MotorAxisFaultMustHoldZero(holdZeroMask, i) != 0u)
        {
            currents[i] = 0.0f;
        }
    }
}

int main(void)
{
    const uint32_t allGimbal = GIMBAL_FAULT_MASK_YAW |
                               GIMBAL_FAULT_MASK_YAW_UPPER |
                               GIMBAL_FAULT_MASK_PITCH;
    const uint32_t wheel2 = 1u << 2;
    MotorAxisFaultInhibitPlan plan;
    uint32_t mask;
    fp32 contaminatedCurrents[4] = {16000.0f, 4000.0f, -4000.0f, 4000.0f};
    fp32 isolatedCurrents[4] = {16000.0f, 4000.0f, -4000.0f, 4000.0f};
    fp32 modelContaminatedCurrents[4] = {16000.0f, 4000.0f, -4000.0f, 4000.0f};
    fp32 modelIsolatedCurrents[4] = {16000.0f, 4000.0f, -4000.0f, 4000.0f};
    fp32 modelAbsentCurrents[4] = {16000.0f, 4000.0f, -4000.0f, 4000.0f};
    motor_node_param_t modelNodes[4] = {0};
    motor_node_param_t modelAbsentNodes[4] = {0};
    const int16_t wheelRpm[4] = {6000, 0, 0, 0};
    fp32 isolatedPower = 0.0f;
    fp32 absentPower = 0.0f;
    fp32 absentScale;
    fp32 scale;

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

    scale = ChassisPowerLimiterScaleCurrents(contaminatedCurrents, 12000.0f, NULL);
    if (!TestCheck(scale < 1.0f && contaminatedCurrents[1] < 4000.0f,
                   "回归前提：故障轴饱和请求会压缩健康轴")) return 1;

    plan = MotorAxisFaultInhibitPlanMake(0x0Fu, 1u << 0, 0u);
    ApplyHoldZeroMask(isolatedCurrents, plan.holdZeroMask);
    scale = ChassisPowerLimiterScaleCurrents(isolatedCurrents, 12000.0f, NULL);
    if (!TestCheck(scale == 1.0f &&
                   isolatedCurrents[0] == 0.0f &&
                   isolatedCurrents[1] == 4000.0f &&
                   isolatedCurrents[2] == -4000.0f &&
                   isolatedCurrents[3] == 4000.0f,
                   "故障轴必须在共享功率限制前清零，健康轴不得被压缩")) return 1;

    for (uint8_t i = 0u; i < 4u; i++)
    {
        modelNodes[i].model = MOTOR_MODEL_3508;
        modelNodes[i].can_id = (uint8_t)(i + 1u);
    }
    modelNodes[0].model = MOTOR_MODEL_3510;
    if (!TestCheck(ChassisPowerLimiterIsPowerModelReady(modelNodes, 0x0Fu) == 0u &&
                   ChassisPowerLimiterIsPowerModelReady(modelNodes, 0x0Eu) != 0u,
                   "未启用的不支持模型不得拖累健康轴，启用后必须判定模型不可用")) return 1;

    modelNodes[0].model = MOTOR_MODEL_3508;
    ApplyHoldZeroMask(modelContaminatedCurrents, plan.holdZeroMask);
    scale = ChassisPowerLimiterScaleCurrentsByPowerModel(modelContaminatedCurrents,
                                                          modelNodes,
                                                          wheelRpm,
                                                          0x0Fu,
                                                          18.0f,
                                                          NULL);
    if (!TestCheck(scale < 1.0f && modelContaminatedCurrents[1] < 4000.0f,
                   "回归前提：仅清零故障轴电流仍会让其空载模型压缩健康轴")) return 1;

    scale = ChassisPowerLimiterScaleCurrentsByPowerModel(modelIsolatedCurrents,
                                                          modelNodes,
                                                          wheelRpm,
                                                          0x0Eu,
                                                          18.0f,
                                                          &isolatedPower);
    if (!TestCheck(scale == 1.0f &&
                   modelIsolatedCurrents[0] == 0.0f &&
                   modelIsolatedCurrents[1] == 4000.0f &&
                   modelIsolatedCurrents[2] == -4000.0f &&
                   modelIsolatedCurrents[3] == 4000.0f,
                   "在线功率模型必须排除故障轴，健康轴不得被压缩")) return 1;

    for (uint8_t i = 0u; i < 4u; i++)
    {
        modelAbsentNodes[i] = modelNodes[i];
    }
    modelAbsentNodes[0].can_id = 0u;
    absentScale = ChassisPowerLimiterScaleCurrentsByPowerModel(modelAbsentCurrents,
                                                                modelAbsentNodes,
                                                                wheelRpm,
                                                                0x0Fu,
                                                                18.0f,
                                                                &absentPower);
    if (!TestCheck(absentScale == scale &&
                   absentPower == isolatedPower &&
                   modelAbsentCurrents[0] == modelIsolatedCurrents[0] &&
                   modelAbsentCurrents[1] == modelIsolatedCurrents[1] &&
                   modelAbsentCurrents[2] == modelIsolatedCurrents[2] &&
                   modelAbsentCurrents[3] == modelIsolatedCurrents[3],
                   "未启用轴的饱和请求和高转速必须与该轴不存在完全等价")) return 1;

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

    if (!TestCheck(GimbalFaultFrameCurrent(GIMBAL_FAULT_MASK_YAW,
                                            GIMBAL_FAULT_MASK_YAW,
                                            6000) == 0 &&
                   GimbalFaultFrameCurrent(GIMBAL_FAULT_MASK_PITCH,
                                            GIMBAL_FAULT_MASK_YAW,
                                            6000) == 6000,
                   "反馈切换或恢复帧必须实际发布零值，不能跳过可靠轴写入")) return 1;

    (void)puts("PASS: motor axis fault policy regression");
    return 0;
}
