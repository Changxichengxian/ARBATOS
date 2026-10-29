/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 固件抽取前控制顺序的对照实现；覆盖分腿开关、默认/覆盖 LQR 行和拒绝路径。
 */

#include "WheelLegVmc.h"

#include <assert.h>
#include <math.h>
#include <string.h>

typedef struct
{
    fp32 thetaErr, dTheta, xErr, vErr, pitchErr, gyroY;
} RefSide;

static void RefFill(uint8_t side, const WheelLegVmcConfig *cfg,
                    const WheelLegVmcInput *in, const WheelLegCoreLegCalc leg[2],
                    fp32 targetTheta, RefSide *out)
{
    const fp32 sign = (side == WHEELLEG_SIDE_LEFT) ? -1.0f : 1.0f;
    out->thetaErr = leg[side].theta - targetTheta;
    out->dTheta = leg[side].d_theta;
    out->xErr = sign * WheelLegCoreLqrXError(in->xM, in->targetV,
                   in->targetYawRate, 0.05f, 1.0e-4f);
    out->vErr = sign * (in->vMps - in->targetV);
    out->pitchErr = sign * in->pitch -
                    (cfg->pitchOffsetRad[side] + sign * in->pitchAutoOffsetRad);
    out->gyroY = sign * in->gyroY;
}

static uint8_t RefStep(const WheelLegVmcConfig *cfg, const WheelLegVmcInput *in,
                       WheelLegVmcState *state, WheelLegCoreLegCalc leg[2], fp32 wheel[2])
{
    fp32 kRight[12], kLeft[12], turnT, rollF0, splitTp;
    fp32 wheelScale, hipScale, targetTheta, cosTheta;
    RefSide right, left;
    uint8_t i;

    targetTheta = WheelLegCoreTargetThetaFromFootX(in->targetFootX, in->targetLeg);
    wheelScale = WheelLegCoreClamp(cfg->wheelScale <= 0.0f ? 0.18f : cfg->wheelScale, 0.0f, 2.0f);
    hipScale = in->useLqrHip ? WheelLegCoreClamp(cfg->hipScale <= 0.0f ? 0.18f : cfg->hipScale, 0.0f, 2.0f) : 0.0f;
    WheelLegVmcEvalLqr(cfg, leg[WHEELLEG_SIDE_RIGHT].length, kRight);
    WheelLegVmcEvalLqr(cfg, leg[WHEELLEG_SIDE_LEFT].length, kLeft);
    if (!state->yawInited)
    {
        state->yawSet = in->yaw;
        state->lastYaw = in->yaw;
        state->yawInited = 1u;
    }
    state->yawSet += in->targetYawRate * in->dt;
    turnT = WheelLegCoreTurnTorque(state->yawSet, in->yaw, in->gyroZ,
                                  cfg->turnKp, cfg->turnKd, cfg->turnMax);
    rollF0 = WheelLegCoreRollForce(0.0f, in->roll, in->gyroX,
                                  cfg->rollKp, cfg->rollKd, cfg->rollMax);
    if (in->useLqrHip)
    {
        splitTp = WheelLegCorePidCalc(&state->splitPid,
                  leg[WHEELLEG_SIDE_RIGHT].theta + leg[WHEELLEG_SIDE_LEFT].theta,
                  2.0f * targetTheta);
    }
    else
    {
        splitTp = 0.0f;
        WheelLegCorePidClear(&state->splitPid);
    }
    RefFill(WHEELLEG_SIDE_RIGHT, cfg, in, leg, targetTheta, &right);
    RefFill(WHEELLEG_SIDE_LEFT, cfg, in, leg, targetTheta, &left);

    wheel[WHEELLEG_SIDE_RIGHT] = WheelLegCoreLqrWheelOutput(kRight, right.thetaErr,
        right.dTheta, right.xErr, right.vErr, right.pitchErr, right.gyroY);
    leg[WHEELLEG_SIDE_RIGHT].tp = 0.0f;
    if (in->useLqrHip)
    {
        leg[WHEELLEG_SIDE_RIGHT].tp = hipScale * WheelLegCoreLqrHipOutput(kRight,
            right.thetaErr, right.dTheta, right.xErr, right.vErr, right.pitchErr,
            right.gyroY, splitTp);
    }
    wheel[WHEELLEG_SIDE_LEFT] = WheelLegCoreLqrWheelOutput(kLeft, left.thetaErr,
        left.dTheta, left.xErr, left.vErr, left.pitchErr, left.gyroY);
    leg[WHEELLEG_SIDE_LEFT].tp = 0.0f;
    if (in->useLqrHip)
    {
        leg[WHEELLEG_SIDE_LEFT].tp = hipScale * WheelLegCoreLqrHipOutput(kLeft,
            left.thetaErr, left.dTheta, left.xErr, left.vErr, left.pitchErr,
            left.gyroY, splitTp);
    }

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        cosTheta = cosf(leg[i].theta);
        if (WheelLegCoreAbs(cosTheta) < 0.1f)
        {
            cosTheta = cosTheta >= 0.0f ? 0.1f : -0.1f;
        }
        leg[i].f0 = cfg->supportBiasN / cosTheta +
                    WheelLegCorePidCalc(&state->legPid[i], leg[i].length, in->targetLeg);
        leg[i].f0 += (i == WHEELLEG_SIDE_RIGHT) ? rollF0 : -rollF0;
        leg[i].f0 = WheelLegCoreClamp(leg[i].f0, -cfg->maxSupportN, cfg->maxSupportN);
        leg[i].tp = WheelLegCoreClamp(leg[i].tp, -cfg->maxJointNm, cfg->maxJointNm);
        wheel[i] = WheelLegCoreClamp(wheel[i] * wheelScale - turnT,
                                    -cfg->maxWheelNm, cfg->maxWheelNm);
        leg[i].contact = (leg[i].f0 * cosf(leg[i].theta) > 3.0f) ? 1u : 0u;
        if (!WheelLegCoreCalcVmc(&leg[i]))
        {
            return 0u;
        }
        leg[i].joint_torque[0] = WheelLegCoreClamp(leg[i].joint_torque[0],
                                                   -cfg->maxJointNm, cfg->maxJointNm);
        leg[i].joint_torque[1] = WheelLegCoreClamp(leg[i].joint_torque[1],
                                                   -cfg->maxJointNm, cfg->maxJointNm);
    }
    return 1u;
}

static void Near(fp32 a, fp32 b)
{
    assert(fabsf(a - b) < 1.0e-6f);
}

static void TestCommonForward(void)
{
    WheelLegVmcConfig cfg = {0};
    WheelLegVmcInput in = {0};
    WheelLegVmcState state = {0};
    WheelLegVmcState baselineState;
    WheelLegCoreLegCalc leg[2] = {0};
    WheelLegCoreLegCalc baselineLeg[2];
    fp32 wheel[2];
    fp32 noTurn[2];
    fp32 tinyPoly[12][4] = {{0}};
    unsigned i;

    cfg.commonForward = 1u;
    cfg.minLegM = 0.06f;
    cfg.maxLegM = 0.12f;
    cfg.wheelScale = 0.18f;
    cfg.hipScale = 0.18f;
    cfg.maxSupportN = 60.0f;
    cfg.maxJointNm = 10.0f;
    cfg.maxWheelNm = 100.0f;
    cfg.turnKp = 0.6f;
    cfg.turnMax = 1.0f;
    cfg.pitchOffsetRad[0] = 0.0f;
    cfg.pitchOffsetRad[1] = 0.0f;
    in.dt = 0.003f;
    in.pitch = 0.06f;
    in.gyroY = 0.02f;
    in.xM = 0.01f;
    in.vMps = 0.02f;
    in.targetV = 0.015f;
    in.targetLeg = 0.09f;
    in.useLqrHip = 1u;
    state.yawInited = 1u;
    state.yawSet = 0.2f;
    for (i = 0u; i < 2u; i++)
    {
        leg[i].l1 = 0.05f;
        leg[i].l4 = 0.05f;
        leg[i].length = 0.09f;
        leg[i].theta = 0.03f;
        leg[i].phi0 = 0.2f;
        leg[i].phi1 = -0.6f;
        leg[i].phi2 = -0.3f;
        leg[i].phi3 = 0.5f;
        leg[i].phi4 = 0.9f;
    }
    cfg.turnKp = 0.0f;
    baselineState = state;
    memcpy(baselineLeg, leg, sizeof(leg));
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, noTurn) == 1u);
    Near(noTurn[0], noTurn[1]);
    Near(leg[0].tp, leg[1].tp);
    cfg.turnKp = 0.6f;
    state = baselineState;
    memcpy(leg, baselineLeg, sizeof(leg));
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.yawSet, 0.15f);
    Near(wheel[WHEELLEG_SIDE_RIGHT] - noTurn[WHEELLEG_SIDE_RIGHT], 0.09f);
    Near(wheel[WHEELLEG_SIDE_LEFT] - noTurn[WHEELLEG_SIDE_LEFT], -0.09f);

    /* 非零但极小的行关闭 LQR 本体，仅观察左右分腿虚力矩方向。 */
    for (i = 0u; i < 12u; i++)
    {
        tinyPoly[i][3] = 1.0e-9f;
    }
    cfg.lqrPoly = tinyPoly;
    cfg.turnKp = 0.0f;
    WheelLegCorePidConfigure(&state.splitPid, 30.0f, 0.0f, 0.0f, 5.0f, 0.0f);
    leg[WHEELLEG_SIDE_RIGHT].theta = 0.10f;
    leg[WHEELLEG_SIDE_LEFT].theta = 0.02f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    assert(leg[WHEELLEG_SIDE_RIGHT].tp > 2.39f);
    assert(leg[WHEELLEG_SIDE_LEFT].tp < -2.39f);
    Near(leg[WHEELLEG_SIDE_RIGHT].tp + leg[WHEELLEG_SIDE_LEFT].tp, 0.0f);

    in.useLqrHip = 0u;
    in.roll = 0.1f;
    cfg.rollKp = 25.0f;
    cfg.rollMax = 10.0f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(leg[WHEELLEG_SIDE_RIGHT].f0, 2.5f);
    Near(leg[WHEELLEG_SIDE_LEFT].f0, -2.5f);

    /* 行驶时参考位置跟车，松杆后只对松杆后的位移施加位置反馈。 */
    in.roll = 0.0f;
    tinyPoly[2][3] = 1.0f;
    in.targetV = 0.1f;
    in.xM = 1.0f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.holdXM, 1.0f);
    in.targetV = 0.0f;
    in.xM = 1.0f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    assert(fabsf(wheel[0]) < 1.0e-6f);
    for (i = 0u; i < 200u && state.wasDriving != 0u; i++)
    {
        assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    }
    assert(state.wasDriving == 0u);
    in.xM = 1.02f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(wheel[WHEELLEG_SIDE_RIGHT], 0.02f);
    Near(wheel[WHEELLEG_SIDE_LEFT], 0.02f);

    /* 左右腿角速度相反时只让分腿环处理，不把它当整体平衡误差。 */
    tinyPoly[0][3] = 2.0f;
    tinyPoly[1][3] = 3.0f;
    tinyPoly[2][3] = 1.0e-9f;
    leg[WHEELLEG_SIDE_RIGHT].theta = 0.10f;
    leg[WHEELLEG_SIDE_LEFT].theta = -0.10f;
    leg[WHEELLEG_SIDE_RIGHT].d_theta = 0.20f;
    leg[WHEELLEG_SIDE_LEFT].d_theta = -0.20f;
    in.xM = state.holdXM;
    in.pitch = 0.0f;
    in.gyroY = 0.0f;
    in.vMps = 0.0f;
    in.pitchAutoOffsetRad = 0.0f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    assert(fabsf(wheel[0]) < 1.0e-6f && fabsf(wheel[1]) < 1.0e-6f);

    /* 偏航目标即使累计超过一圈，也始终围绕实测角限幅并保持转向力矩符号。 */
    cfg.turnKp = 0.6f;
    cfg.turnKd = 0.08f;
    in.targetYawRate = 3.6f;
    state.yawSet = 7.0f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.yawSet, 0.15f);
    Near(state.yawRate, 0.012f);
    Near(wheel[WHEELLEG_SIDE_RIGHT] - wheel[WHEELLEG_SIDE_LEFT], 0.18192f);
    in.gyroY = NAN;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 0u);
}

static void TestObserve(void)
{
    WheelLegVmcObserveState state = {0};
    WheelLegVmcObserveState rateState = {0};
    WheelLegCoreLegCalc leg[2] = {0};
    fp32 wheelRate[2] = {10.0f, 10.0f};
    fp32 rodRate[2] = {10.0f, 10.0f};
    fp32 x = 0.0f, v = 0.0f;
    fp32 rateX = 0.0f, rateV = 0.0f;

    leg[0].phi2 = 3.13f;
    leg[1].phi2 = 3.13f;
    assert(WheelLegVmcObserve(&state, leg, wheelRate, 0.5f, 0.03f, 1.0f,
                              0.01f, &x, &v) == 1u);
    Near(v, 0.315f);
    Near(x, 0.00315f);
    wheelRate[0] = wheelRate[1] = 0.0f;
    leg[0].phi2 = leg[1].phi2 = -3.13f;
    assert(WheelLegVmcObserve(&state, leg, wheelRate, 0.0f, 0.03f, 1.0f,
                              0.01f, &x, &v) == 1u);
    assert(v > 0.06f && v < 0.08f);
    assert(WheelLegVmcObserve(&state, leg, wheelRate, 0.0f, 0.03f, 1.0f,
                              0.0f, &x, &v) == 0u);
    assert(WheelLegVmcObserveRates(&rateState, leg, wheelRate, rodRate,
                                   0.0f, 0.03f, 1.0f, 0.01f,
                                   &rateX, &rateV) == 1u);
    Near(rateV, 0.3f);
    leg[0].phi2 = leg[1].phi2 = 2.0f;
    rodRate[0] = rodRate[1] = 0.0f;
    assert(WheelLegVmcObserveRates(&rateState, leg, wheelRate, rodRate,
                                   0.0f, 0.03f, 1.0f, 0.01f,
                                   &rateX, &rateV) == 1u);
    Near(rateV, 0.0f);
}

static void TestKinematicsRates(void)
{
    const WheelLegCoreGeometry geo = {0.05f, 0.105f, 0.105f, 0.05f, 0.07f};
    const fp32 q[3][2] = {{1.2f, -1.0f}, {0.1f, -0.1f}, {0.5f, -0.4f}};
    const fp32 frontRate = 0.7f;
    const fp32 backRate = -0.4f;
    const fp32 gyroY = 0.08f;
    const fp32 eps = 1.0e-4f;
    unsigned i;

    for (i = 0u; i < 3u; i++)
    {
        WheelLegCoreLegCalc leg = {0}, plus = {0}, minus = {0};
        fp32 rodRate;
        assert(WheelLegVmcKinematics(&geo, &leg, q[i][0], q[i][1],
                                     frontRate, backRate, 0.03f, gyroY,
                                     0.003f, &rodRate) == 1u);
        assert(WheelLegCoreCalcKinematics(&geo, &plus,
                    q[i][0] + eps * frontRate, q[i][1] + eps * backRate,
                    0.03f, gyroY, 0u, 0.003f) == 1u);
        assert(WheelLegCoreCalcKinematics(&geo, &minus,
                    q[i][0] - eps * frontRate, q[i][1] - eps * backRate,
                    0.03f, gyroY, 0u, 0.003f) == 1u);
        assert(fabsf(leg.d_length - (plus.length - minus.length) / (2.0f * eps)) < 0.002f);
        assert(fabsf(leg.d_alpha - WheelLegCoreWrapPi(plus.alpha - minus.alpha) /
                     (2.0f * eps)) < 0.003f);
        assert(fabsf(rodRate - WheelLegCoreWrapPi(plus.phi2 - minus.phi2) /
                     (2.0f * eps)) < 0.003f);
        Near(leg.d_theta, leg.d_alpha - gyroY);
        Near(leg.dd_length, 0.0f);
        Near(leg.dd_theta, 0.0f);
    }
}

static void TestParkHold(void)
{
    WheelLegVmcConfig cfg = {0};
    WheelLegVmcInput in = {0};
    WheelLegVmcState state = {0};
    WheelLegCoreLegCalc leg[2] = {0};
    fp32 poly[12][4] = {{0}};
    fp32 wheel[2];
    unsigned i;

    for (i = 0u; i < 12u; i++) poly[i][3] = 1.0e-9f;
    poly[3][3] = 1.0f;
    cfg.lqrPoly = poly;
    cfg.commonForward = 1u;
    cfg.minLegM = 0.06f;
    cfg.maxLegM = 0.12f;
    cfg.maxSupportN = 60.0f;
    cfg.maxJointNm = 3.0f;
    cfg.maxWheelNm = 1.0f;
    in.dt = 0.003f;
    in.targetLeg = 0.09f;
    for (i = 0u; i < 2u; i++)
    {
        leg[i].length = 0.09f;
        leg[i].l1 = leg[i].l4 = 0.05f;
        leg[i].phi0 = 0.2f;
        leg[i].phi1 = -0.6f;
        leg[i].phi2 = -0.3f;
        leg[i].phi3 = 0.5f;
        leg[i].phi4 = 0.9f;
    }
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.holdXM, 0.0f);
    in.xM = 0.06f;
    in.vMps = 0.04f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.parkIntegral, 0.00018f);
    Near(state.parkSpeed, -0.0009f);
    Near(wheel[0], 0.0409f);
    Near(wheel[1], 0.0409f);
    for (i = 0u; i < 400u; i++)
    {
        assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    }
    assert(state.parkSpeed < -0.07f && state.parkSpeed > -0.15f);
    in.targetV = 0.1f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.parkIntegral, 0.0f);
    Near(state.parkSpeed, 0.0f);
    Near(state.holdXM, 0.06f);
    in.targetV = 0.0f;
    in.targetYawRate = 0.2f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.parkIntegral, 0.0f);
    Near(state.parkSpeed, 0.0f);
    in.targetYawRate = 0.0f;
    in.yaw = 0.2f;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    for (i = 0u; i < 300u && state.wasDriving != 0u; i++)
    {
        assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    }
    assert(state.wasDriving == 0u);
    Near(state.yawSet, in.yaw);

    cfg.trimEnable = 1u;
    cfg.trimGain = 0.08f;
    cfg.trimMaxRad = 0.18f;
    cfg.trimRateRadps = 0.015f;
    cfg.trimVDeadbandMps = 0.015f;
    cfg.trimLpf = 0.03f;
    in.vMps = 0.10f;
    for (i = 0u; i < 100u; i++)
    {
        assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    }
    assert(state.trimVFiltered > 0.09f && state.trimVFiltered < 0.10f);
    assert(state.pitchAutoOffsetRad > 0.0f && state.pitchAutoOffsetRad < 0.18f);
    in.targetV = 0.1f;
    const fp32 offset = state.pitchAutoOffsetRad;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.trimVFiltered, 0.0f);
    Near(state.pitchAutoOffsetRad, offset);
    cfg.trimEnable = 0u;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    Near(state.pitchAutoOffsetRad, 0.0f);
}

static void TestVirtualWork(void)
{
    const WheelLegCoreGeometry geo = {0.05f, 0.105f, 0.105f, 0.05f, 0.07f};
    WheelLegCoreLegCalc center = {0}, front = {0}, back = {0};
    const fp32 delta = 1.0e-4f;
    fp32 dLengthFront, dLengthBack, dThetaFront, dThetaBack;
    fp32 predictedFront, predictedBack;

    assert(WheelLegCoreCalcKinematics(&geo, &center, 0.1f, -0.1f,
                                      0.0f, 0.0f, 0u, 0.003f) == 1u);
    assert(WheelLegCoreCalcKinematics(&geo, &front, 0.1f + delta, -0.1f,
                                      0.0f, 0.0f, 0u, 0.003f) == 1u);
    assert(WheelLegCoreCalcKinematics(&geo, &back, 0.1f, -0.1f + delta,
                                      0.0f, 0.0f, 0u, 0.003f) == 1u);
    dLengthFront = (front.length - center.length) / delta;
    dLengthBack = (back.length - center.length) / delta;
    dThetaFront = (front.theta - center.theta) / delta;
    dThetaBack = (back.theta - center.theta) / delta;
    center.f0 = 7.0f;
    center.tp = -0.8f;
    assert(WheelLegCoreCalcVmc(&center) == 1u);
    /* Core 的 tp 对应 -dtheta；hip 控制量转为 tp 时也取负号。 */
    predictedFront = center.f0 * dLengthFront - center.tp * dThetaFront;
    predictedBack = center.f0 * dLengthBack - center.tp * dThetaBack;
    assert(fabsf(center.joint_torque[0] - predictedFront) < 0.02f);
    assert(fabsf(center.joint_torque[1] - predictedBack) < 0.02f);
}

static void TestMotionSlew(void)
{
    WheelLegVmcConfig cfg = {0};
    WheelLegVmcInput in = {0};
    WheelLegVmcState state = {0};
    WheelLegCoreLegCalc leg[2] = {0};
    fp32 poly[12][4] = {{0}};
    fp32 wheel[2];
    fp32 lastSpeed, lastAccel, lastYawRate;
    unsigned i;

    for (i = 0u; i < 12u; i++) poly[i][3] = 1.0e-9f;
    cfg.lqrPoly = poly;
    cfg.commonForward = 1u;
    cfg.minLegM = 0.06f;
    cfg.maxLegM = 0.12f;
    cfg.maxSupportN = 60.0f;
    cfg.maxJointNm = 3.0f;
    cfg.maxWheelNm = 1.0f;
    in.dt = 0.003f;
    in.targetLeg = 0.12f;
    in.targetV = 0.7f;
    in.targetYawRate = 3.6f;
    for (i = 0u; i < 2u; i++)
    {
        leg[i].length = 0.12f;
        leg[i].l1 = leg[i].l4 = 0.05f;
        leg[i].phi0 = 0.2f;
        leg[i].phi1 = -0.6f;
        leg[i].phi2 = -0.3f;
        leg[i].phi3 = 0.5f;
        leg[i].phi4 = 0.9f;
    }
    for (i = 0u; i < 300u; i++)
    {
        lastSpeed = state.motionSpeed;
        lastAccel = state.motionAccel;
        lastYawRate = state.yawRate;
        in.vMps = lastSpeed;
        assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
        assert(fabsf(state.motionAccel - lastAccel) <= 0.018001f);
        assert(fabsf(state.motionAccel) <= 0.500001f);
        assert(fabsf(state.motionSpeed - lastSpeed) <= 0.001501f);
        assert(fabsf(state.yawRate - lastYawRate) <= 0.012001f);
    }
    assert(state.motionSpeed > 0.3f);
    in.targetV = -0.7f;
    in.targetYawRate = 0.0f;
    lastSpeed = state.motionSpeed;
    lastAccel = state.motionAccel;
    lastYawRate = state.yawRate;
    in.vMps = lastSpeed;
    assert(WheelLegVmcStep(&cfg, &in, &state, leg, wheel) == 1u);
    assert(fabsf(state.motionSpeed - lastSpeed) <= 0.001501f);
    Near(state.motionAccel, lastAccel - 0.018f);
    Near(state.yawRate, lastYawRate - 0.012f);
    assert(state.wasDriving != 0u);
    Near(state.parkSpeed, 0.0f);
}

int main(void)
{
    WheelLegVmcConfig cfg = {0};
    WheelLegVmcInput in = {0};
    WheelLegVmcState actual = {0}, expected = {0};
    WheelLegCoreLegCalc actualLeg[2] = {0}, expectedLeg[2] = {0};
    fp32 actualWheel[2], expectedWheel[2];
    fp32 customPoly[12][4] = {{0}};
    uint8_t i;
    unsigned step;

    cfg.minLegM = 0.06f;
    cfg.maxLegM = 0.12f;
    cfg.wheelScale = 0.18f;
    cfg.hipScale = 0.18f;
    cfg.supportBiasN = 11.0f;
    cfg.maxSupportN = 60.0f;
    cfg.maxJointNm = 3.0f;
    cfg.maxWheelNm = 0.45f;
    cfg.turnKp = 0.6f;
    cfg.turnKd = 0.08f;
    cfg.turnMax = 0.20f;
    cfg.rollKp = 25.0f;
    cfg.rollKd = 3.0f;
    cfg.rollMax = 15.0f;
    cfg.pitchOffsetRad[WHEELLEG_SIDE_LEFT] = 0.01f;
    cfg.pitchOffsetRad[WHEELLEG_SIDE_RIGHT] = -0.02f;
    customPoly[0][0] = -250.0f;
    customPoly[0][3] = -0.1f;
    cfg.lqrPoly = customPoly;

    WheelLegCorePidConfigure(&actual.legPid[0], 280.0f, 0.0f, 1200.0f, 35.0f, 0.0f);
    WheelLegCorePidConfigure(&actual.legPid[1], 280.0f, 0.0f, 1200.0f, 35.0f, 0.0f);
    WheelLegCorePidConfigure(&actual.splitPid, 30.0f, 0.0f, 1.0f, 2.0f, 0.0f);
    expected = actual;
    for (i = 0u; i < 2u; i++)
    {
        actualLeg[i].l1 = 0.05f;
        actualLeg[i].l4 = 0.05f;
        actualLeg[i].length = 0.09f;
        actualLeg[i].phi0 = 0.15f;
        actualLeg[i].phi1 = -0.6f;
        actualLeg[i].phi2 = -0.3f;
        actualLeg[i].phi3 = 0.5f;
        actualLeg[i].phi4 = 0.9f;
    }
    memcpy(expectedLeg, actualLeg, sizeof(actualLeg));

    for (step = 0u; step < 48u; step++)
    {
        in.dt = 0.003f;
        in.pitch = 0.03f * sinf((fp32)step * 0.13f);
        in.roll = 0.02f * cosf((fp32)step * 0.11f);
        in.yaw = 0.1f + 0.003f * (fp32)step;
        in.gyroX = -0.02f;
        in.gyroY = 0.06f;
        in.gyroZ = 0.01f;
        in.xM = 0.01f;
        in.vMps = 0.02f;
        in.pitchAutoOffsetRad = 0.005f;
        in.targetV = (step % 3u == 0u) ? 0.0f : 0.015f;
        in.targetLeg = 0.095f;
        in.targetFootX = 0.004f;
        in.targetYawRate = (step % 4u == 0u) ? 0.0f : 0.02f;
        in.useLqrHip = (uint8_t)(step % 2u);
        actualLeg[WHEELLEG_SIDE_LEFT].length = expectedLeg[WHEELLEG_SIDE_LEFT].length =
            (step == 47u) ? 0.005f : 0.09f;
        assert(WheelLegVmcStep(&cfg, &in, &actual, actualLeg, actualWheel) ==
               RefStep(&cfg, &in, &expected, expectedLeg, expectedWheel));
        for (i = 0u; i < 2u; i++)
        {
            Near(actualWheel[i], expectedWheel[i]);
            Near(actualLeg[i].f0, expectedLeg[i].f0);
            Near(actualLeg[i].tp, expectedLeg[i].tp);
            Near(actualLeg[i].joint_torque[0], expectedLeg[i].joint_torque[0]);
            Near(actualLeg[i].joint_torque[1], expectedLeg[i].joint_torque[1]);
            assert(actualLeg[i].contact == expectedLeg[i].contact);
            Near(actual.legPid[i].last_err, expected.legPid[i].last_err);
        }
        Near(actual.splitPid.last_err, expected.splitPid.last_err);
        Near(actual.yawSet, expected.yawSet);
    }
    assert(WheelLegVmcStep(&cfg, &in, &actual, actualLeg, NULL) == 0u);
    TestCommonForward();
    TestObserve();
    TestKinematicsRates();
    TestParkHold();
    TestVirtualWork();
    TestMotionSlew();
    return 0;
}
