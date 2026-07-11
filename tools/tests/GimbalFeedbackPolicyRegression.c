/* 云台 IMU/编码器反馈切换、逐轴降级和编码器范围回归。 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "GimbalFaultPolicy.h"
#include "GimbalFeedbackPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static int TestFloatNear(float actual, float expected, float tolerance,
                         const char *message)
{
    return TestCheck(fabsf(actual - expected) <= tolerance, message);
}

static int TestFiveRuntimeVariants(void)
{
    const uint32_t allAxes = GIMBAL_FAULT_MASK_YAW |
                             GIMBAL_FAULT_MASK_YAW_UPPER |
                             GIMBAL_FAULT_MASK_PITCH;
    const struct
    {
        robot_run_variant_e variant;
        uint8_t yawControlIsUpper;
        uint8_t dualYawOutputActive;
        uint32_t expected;
    } cases[] = {
        {ROBOT_RUN_VARIANT_NORMAL, 0u, 0u,
         GIMBAL_FAULT_MASK_YAW | GIMBAL_FAULT_MASK_PITCH},
        {ROBOT_RUN_VARIANT_GIMBAL_DUAL, 0u, 1u, allAxes},
        {ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY, 0u, 1u,
         GIMBAL_FAULT_MASK_YAW},
        {ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY, 0u, 1u,
         GIMBAL_FAULT_MASK_YAW},
        {ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY, 0u, 1u,
         GIMBAL_FAULT_MASK_PITCH},
    };

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(cases) / sizeof(cases[0])); i++)
    {
        const uint32_t actual = GimbalFaultImuAxisMask(
            cases[i].variant,
            allAxes,
            cases[i].yawControlIsUpper,
            cases[i].dualYawOutputActive);

        if (!TestCheck(actual == cases[i].expected,
                       "五种运行变体必须只声明实际依赖 IMU 的轴")) return 0;
    }
    return 1;
}

static int TestFeedbackTransitions(void)
{
    GimbalFeedbackPolicyState state;
    GimbalFeedbackPolicyOutput out;
    const uint32_t required = GIMBAL_FAULT_MASK_YAW |
                              GIMBAL_FAULT_MASK_YAW_UPPER |
                              GIMBAL_FAULT_MASK_PITCH;
    const uint32_t fallback = GIMBAL_FAULT_MASK_YAW |
                              GIMBAL_FAULT_MASK_YAW_UPPER;

    GimbalFeedbackPolicyInit(&state);
    out = GimbalFeedbackPolicyStep(&state, 100u, 1u, 0u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL &&
                   out.encoderDegradedMask == 0u &&
                   out.feedbackBlockMask == 0u,
                   "IMU 正常时不得启用编码器降级")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 101u, 0u, 0u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED &&
                   out.modeChanged != 0u &&
                   out.transitionPending != 0u &&
                   out.encoderDegradedMask == fallback &&
                   out.feedbackBlockMask == GIMBAL_FAULT_MASK_PITCH &&
                   out.transitionZeroMask == required,
                   "IMU 超时必须立即逐轴降级并让不可替代轴阻断")) return 0;

    GimbalFeedbackPolicyConsumeTransition(&state);
    out = GimbalFeedbackPolicyStep(&state, 200u, 1u, 1u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED &&
                   out.recoveryPending != 0u &&
                   out.transitionZeroMask == 0u,
                   "IMU 刚恢复时仍需等待连续稳定时间")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 399u, 1u, 1u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED,
                   "IMU 稳定不足 200ms 不得切回")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 400u, 1u, 0u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED &&
                   out.recoveryPending != 0u,
                   "未进入云台安全档不得切回 IMU")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 401u, 1u, 1u, required, fallback);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL &&
                   out.modeChanged != 0u &&
                   out.transitionPending != 0u &&
                   out.transitionZeroMask == required,
                   "IMU 连续稳定且安全档确认后才能切回并强制零帧")) return 0;

    GimbalFeedbackPolicyConsumeTransition(&state);
    out = GimbalFeedbackPolicyStep(&state, 402u, 1u, 1u, required, fallback);
    if (!TestCheck(out.transitionZeroMask == 0u,
                   "反馈切换零输出只能保持一帧")) return 0;
    return 1;
}

static int TestRouteChangesAndRetry(void)
{
    GimbalFeedbackPolicyState state;
    GimbalFeedbackPolicyOutput out;
    const uint32_t yaw = GIMBAL_FAULT_MASK_YAW;
    const uint32_t yawPitch = yaw | GIMBAL_FAULT_MASK_PITCH;

    GimbalFeedbackPolicyInit(&state);
    out = GimbalFeedbackPolicyStep(&state, 0u, 0u, 0u, yaw, yaw);
    if (!TestCheck(out.transitionPending == 0u,
                   "首次建立反馈路由不应伪造一次运行中切换")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 1u, 0u, 0u, yawPitch, yaw);
    if (!TestCheck(out.transitionPending != 0u &&
                   out.transitionZeroMask == yawPitch &&
                   out.encoderDegradedMask == yaw &&
                   out.feedbackBlockMask == GIMBAL_FAULT_MASK_PITCH,
                   "降级模式下运行变体改变必须重置新旧轴并重新计算阻断")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 2u, 0u, 0u, yawPitch, yaw);
    if (!TestCheck(out.transitionPending != 0u &&
                   out.transitionZeroMask == yawPitch,
                   "零命令未确认发布前切换请求必须保持等待")) return 0;

    GimbalFeedbackPolicyConsumeTransition(&state);
    out = GimbalFeedbackPolicyStep(&state, 3u, 0u, 0u, yawPitch, yaw);
    if (!TestCheck(out.transitionPending == 0u && out.transitionZeroMask == 0u,
                   "只有发布方确认后才能清除切换等待")) return 0;

    out = GimbalFeedbackPolicyStep(&state, 4u, 0u, 0u, yawPitch, yawPitch);
    if (!TestCheck(out.transitionPending != 0u &&
                   out.encoderDegradedMask == yawPitch &&
                   out.feedbackBlockMask == 0u,
                   "模式不变但编码器替代能力变化时也必须重置反馈路由")) return 0;
    return 1;
}

static int TestRecoveryContinuity(void)
{
    GimbalFeedbackPolicyState state;
    GimbalFeedbackPolicyOutput out;

    GimbalFeedbackPolicyInit(&state);
    (void)GimbalFeedbackPolicyStep(&state, 0u, 0u, 0u,
                                   GIMBAL_FAULT_MASK_YAW,
                                   GIMBAL_FAULT_MASK_YAW);
    out = GimbalFeedbackPolicyStep(&state, 10u, 1u, 1u,
                                   GIMBAL_FAULT_MASK_YAW,
                                   GIMBAL_FAULT_MASK_YAW);
    if (!TestCheck(out.recoveryPending != 0u,
                   "降级启动时必须报告恢复等待")) return 0;
    (void)GimbalFeedbackPolicyStep(&state, 150u, 0u, 1u,
                                   GIMBAL_FAULT_MASK_YAW,
                                   GIMBAL_FAULT_MASK_YAW);
    out = GimbalFeedbackPolicyStep(&state, 160u, 1u, 1u,
                                   GIMBAL_FAULT_MASK_YAW,
                                   GIMBAL_FAULT_MASK_YAW);
    out = GimbalFeedbackPolicyStep(&state, 359u, 1u, 1u,
                                   GIMBAL_FAULT_MASK_YAW,
                                   GIMBAL_FAULT_MASK_YAW);
    if (!TestCheck(out.mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED,
                   "恢复计时中再次掉线必须重新累计 200ms")) return 0;
    return 1;
}

static int TestEncoderRanges(void)
{
    if (!TestCheck(GimbalEncoderRelativeCount(100u, 8100u, 8192u) == 192,
                   "8192 编码器正向跨零换算错误")) return 0;
    if (!TestCheck(GimbalEncoderRelativeCount(8100u, 100u, 8192u) == -192,
                   "8192 编码器反向跨零换算错误")) return 0;
    if (!TestCheck(GimbalEncoderRelativeCount(120u, 16200u, 16384u) == 304,
                   "16384 编码器跨零换算错误")) return 0;
    if (!TestFloatNear(GimbalEncoderAngleRad(4096u, 0u, 8192u),
                       3.14159265359f, 0.00001f,
                       "8192 编码器半圈角错误")) return 0;
    if (!TestFloatNear(GimbalEncoderAngleRad(8192u, 0u, 16384u),
                       3.14159265359f, 0.00001f,
                       "16384 编码器半圈角错误")) return 0;
    if (!TestCheck(GimbalEncoderNormalizeCount(-1, 8192u) == 8191u &&
                   GimbalEncoderNormalizeCount(16385, 16384u) == 1u,
                   "编码器计数归一化必须覆盖负数和 16384 范围")) return 0;
    return 1;
}

static int TestZeroReceiptBarrier(void)
{
    GimbalFeedbackZeroBarrier barrier;

    GimbalFeedbackZeroBarrierInit(&barrier);
    if (!TestCheck(GimbalFeedbackZeroBarrierMatches(&barrier, 7u) == 0u,
                   "未发布零命令时不得伪造发送屏障")) return 0;

    GimbalFeedbackZeroBarrierBegin(&barrier, 7u);
    if (!TestCheck(GimbalFeedbackZeroBarrierAdd(&barrier,
                                                 GIMBAL_FAULT_MASK_YAW,
                                                 3u,
                                                 101u,
                                                 500u) != 0u &&
                       GimbalFeedbackZeroBarrierExcludeSafe(
                           &barrier,
                           GIMBAL_FAULT_MASK_PITCH) != 0u &&
                       GimbalFeedbackZeroBarrierMatches(&barrier, 7u) != 0u &&
                       barrier.waitMask == GIMBAL_FAULT_MASK_YAW &&
                       barrier.safeInhibitMask == GIMBAL_FAULT_MASK_PITCH &&
                       barrier.motorId[0] == 3u &&
                       barrier.cmdSeq[0] == 101u &&
                       barrier.cmdTick[0] == 500u,
                   "零命令屏障必须保存切换代次、物理轴和精确命令身份")) return 0;

    if (!TestCheck(GimbalFeedbackZeroBarrierAdd(&barrier,
                                                 GIMBAL_FAULT_MASK_YAW |
                                                     GIMBAL_FAULT_MASK_PITCH,
                                                 4u,
                                                 102u,
                                                 501u) == 0u &&
                       GimbalFeedbackZeroBarrierExcludeSafe(&barrier, 0x08u) == 0u,
                   "发送屏障必须拒绝多轴合并位和未知轴")) return 0;

    GimbalFeedbackZeroBarrierBegin(&barrier, 8u);
    return TestCheck(GimbalFeedbackZeroBarrierMatches(&barrier, 7u) == 0u &&
                         GimbalFeedbackZeroBarrierMatches(&barrier, 8u) != 0u &&
                         barrier.waitMask == 0u &&
                         barrier.safeInhibitMask == 0u,
                     "新的反馈切换必须作废旧发送屏障");
}

static int TestRouteZeroDebt(void)
{
    GimbalFeedbackRouteDebt debt;

    GimbalFeedbackRouteDebtInit(&debt);
    if (!TestCheck(GimbalFeedbackRouteDebtMask(&debt) == 0u,
                   "初始状态不得存在重新放行欠账")) return 0;
    if (!TestCheck(GimbalFeedbackRouteDebtHold(&debt,
                                               GIMBAL_FAULT_MASK_PITCH,
                                               5u,
                                               103u,
                                               502u) != 0u &&
                       debt.blockedMask == GIMBAL_FAULT_MASK_PITCH &&
                       debt.publishMask == 0u &&
                       debt.waitMask == 0u &&
                       debt.motorId[2] == 5u &&
                       debt.cmdSeq[2] == 103u &&
                       debt.cmdTick[2] == 502u,
                   "运行模式禁发轴必须独立保存精确零命令")) return 0;
    if (!TestCheck(GimbalFeedbackRouteDebtRequestPublish(
                       &debt,
                       GIMBAL_FAULT_MASK_PITCH,
                       5u) != 0u &&
                       debt.blockedMask == 0u &&
                       debt.publishMask == GIMBAL_FAULT_MASK_PITCH &&
                       debt.waitMask == 0u,
                   "重新放行时必须先进入零命令重发状态")) return 0;
    if (!TestCheck(GimbalFeedbackRouteDebtWait(&debt,
                                               GIMBAL_FAULT_MASK_PITCH,
                                               5u,
                                               104u,
                                               503u) != 0u &&
                       debt.blockedMask == 0u &&
                       debt.publishMask == 0u &&
                       debt.waitMask == GIMBAL_FAULT_MASK_PITCH &&
                       debt.cmdSeq[2] == 104u &&
                       debt.cmdTick[2] == 503u,
                   "补发零命令后必须等待新命令身份的发送回执")) return 0;
    if (!TestCheck(GimbalFeedbackRouteDebtComplete(
                       &debt,
                       GIMBAL_FAULT_MASK_PITCH) != 0u &&
                       GimbalFeedbackRouteDebtMask(&debt) == 0u &&
                       debt.motorId[2] == 0u &&
                       debt.cmdSeq[2] == 0u &&
                       debt.cmdTick[2] == 0u,
                   "只有匹配回执到达后才能清除重新放行欠账")) return 0;
    return TestCheck(GimbalFeedbackRouteDebtHold(
                         &debt,
                         GIMBAL_FAULT_MASK_YAW | GIMBAL_FAULT_MASK_PITCH,
                         3u,
                         105u,
                         504u) == 0u,
                     "重新放行欠账必须拒绝多轴合并位");
}

int main(void)
{
    if (!TestFiveRuntimeVariants()) return 1;
    if (!TestFeedbackTransitions()) return 1;
    if (!TestRouteChangesAndRetry()) return 1;
    if (!TestRecoveryContinuity()) return 1;
    if (!TestEncoderRanges()) return 1;
    if (!TestZeroReceiptBarrier()) return 1;
    if (!TestRouteZeroDebt()) return 1;

    (void)puts("PASS: gimbal feedback policy regression");
    return 0;
}
