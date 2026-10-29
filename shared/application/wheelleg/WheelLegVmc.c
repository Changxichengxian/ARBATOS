/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "WheelLegVmc.h"

#include <math.h>
#include <stddef.h>

#define WHEELLEG_VMC_WHEEL_SCALE_DEFAULT 0.18f
#define WHEELLEG_VMC_HIP_SCALE_DEFAULT 0.18f
#define WHEELLEG_VMC_X_HOLD_LIMIT_M 0.05f
#define WHEELLEG_VMC_MOTION_EPS 1.0e-4f
#define WHEELLEG_VMC_PARK_ERROR_LIMIT_M 0.10f
#define WHEELLEG_VMC_PARK_INTEGRAL_LIMIT 0.60f
#define WHEELLEG_VMC_PARK_SPEED_LIMIT_MPS 0.15f
#define WHEELLEG_VMC_PARK_SLEW_MPS2 0.30f
#define WHEELLEG_VMC_TRIM_GAIN_DEFAULT 0.08f
#define WHEELLEG_VMC_TRIM_MAX_RAD_DEFAULT 0.18f
#define WHEELLEG_VMC_TRIM_RATE_RADPS_DEFAULT 0.015f
#define WHEELLEG_VMC_TRIM_V_DEADBAND_MPS_DEFAULT 0.015f
#define WHEELLEG_VMC_TRIM_LPF_DEFAULT 0.03f

static const fp32 s_defaultLqrPoly[WHEELLEG_CORE_LQR_ROW_COUNT][WHEELLEG_CORE_LQR_COEFF_COUNT] = {
    {-243.932f, 105.148f, -19.1838f, -0.199759f},
    {-6.33721f, 2.6174f, -1.08798f, -0.0047227f},
    {-43.8763f, 16.3233f, -2.10154f, -0.127721f},
    {-52.0411f, 19.3719f, -2.60375f, -0.168927f},
    {-805.793f, 328.293f, -48.8092f, 2.92903f},
    {-40.1396f, 16.7832f, -2.61208f, 0.17602f},
    {-962.682f, 417.508f, -67.9889f, 4.84346f},
    {-89.4595f, 37.3246f, -5.80403f, 0.414703f},
    {-618.557f, 251.264f, -37.1404f, 2.18751f},
    {-800.904f, 324.39f, -47.7682f, 2.80481f},
    {3575.2f, -1332.91f, 172.196f, 9.48315f},
    {202.812f, -76.6035f, 10.1013f, 0.345984f},
};

typedef struct
{
    fp32 theta_err;
    fp32 d_theta;
    fp32 x_err;
    fp32 v_err;
    fp32 pitch_err;
    fp32 gyro_y;
} WheelLegVmcSideState;

const fp32 *WheelLegVmcLqrRow(const WheelLegVmcConfig *cfg, uint8_t index)
{
    const fp32 *configured;

    if (index >= WHEELLEG_CORE_LQR_ROW_COUNT)
    {
        return s_defaultLqrPoly[0];
    }
    configured = (cfg != NULL && cfg->lqrPoly != NULL) ? cfg->lqrPoly[index] : NULL;
    return (WheelLegCoreLqrRowIsZero(configured) == 0u) ?
               configured : s_defaultLqrPoly[index];
}

void WheelLegVmcEvalLqr(const WheelLegVmcConfig *cfg, fp32 legLength,
                        fp32 out[WHEELLEG_CORE_LQR_ROW_COUNT])
{
    fp32 length = legLength;
    uint8_t i;

    if (cfg == NULL || out == NULL)
    {
        return;
    }
    if (cfg->minLegM < cfg->maxLegM)
    {
        length = WheelLegCoreClamp(length, cfg->minLegM, cfg->maxLegM);
    }
    for (i = 0u; i < WHEELLEG_CORE_LQR_ROW_COUNT; i++)
    {
        out[i] = WheelLegCorePoly4(WheelLegVmcLqrRow(cfg, i), length);
    }
}

uint8_t WheelLegVmcKinematics(const WheelLegCoreGeometry *geo,
                              WheelLegCoreLegCalc *leg,
                              fp32 front, fp32 back,
                              fp32 frontRate, fp32 backRate,
                              fp32 pitch, fp32 gyroY, fp32 dt,
                              fp32 *rodRate)
{
    WheelLegCoreLegCalc forceLeg;
    WheelLegCoreLegCalc hipLeg;
    fp32 deltaVx;
    fp32 deltaVy;
    fp32 sin32;
    fp32 prevLengthRate;
    fp32 prevThetaRate;
    uint8_t first;

    if (geo == NULL || leg == NULL || rodRate == NULL ||
        !isfinite(front) || !isfinite(back) ||
        !isfinite(frontRate) || !isfinite(backRate) ||
        !isfinite(pitch) || !isfinite(gyroY) ||
        !isfinite(dt) || dt <= 0.0f)
    {
        return 0u;
    }
    first = (leg->first == 0u) ? 1u : 0u;
    prevLengthRate = leg->d_length;
    prevThetaRate = leg->d_theta;
    if (WheelLegCoreCalcKinematics(geo, leg, front, back, pitch, gyroY, 0u, dt) == 0u)
    {
        return 0u;
    }

    /* VMC 的力列是 dL/dq，虚扭矩列是 -dα/dq，直接复用力映射避免两套雅可比。 */
    forceLeg = *leg;
    forceLeg.f0 = 1.0f;
    forceLeg.tp = 0.0f;
    hipLeg = *leg;
    hipLeg.f0 = 0.0f;
    hipLeg.tp = 1.0f;
    if (WheelLegCoreCalcVmc(&forceLeg) == 0u ||
        WheelLegCoreCalcVmc(&hipLeg) == 0u)
    {
        return 0u;
    }
    leg->d_length = forceLeg.joint_torque[0] * frontRate +
                    forceLeg.joint_torque[1] * backRate;
    leg->d_alpha = -(hipLeg.joint_torque[0] * frontRate +
                     hipLeg.joint_torque[1] * backRate);
    leg->d_theta = leg->d_alpha - gyroY;
    leg->dd_length = (first != 0u) ? 0.0f : (leg->d_length - prevLengthRate) / dt;
    leg->dd_theta = (first != 0u) ? 0.0f : (leg->d_theta - prevThetaRate) / dt;
    leg->last_d_length = leg->d_length;
    leg->last_d_theta = leg->d_theta;

    /* 约束闭环速度，对后下杆方向投影后得到前下杆角速度。 */
    sin32 = sinf(leg->phi3 - leg->phi2);
    deltaVx = -leg->l4 * sinf(leg->phi4) * backRate +
               leg->l1 * sinf(leg->phi1) * frontRate;
    deltaVy = leg->l4 * cosf(leg->phi4) * backRate -
               leg->l1 * cosf(leg->phi1) * frontRate;
    *rodRate = (deltaVx * cosf(leg->phi3) + deltaVy * sinf(leg->phi3)) /
               (leg->l2 * sin32);
    return (isfinite(leg->d_length) && isfinite(leg->d_theta) &&
            isfinite(*rodRate)) ? 1u : 0u;
}

static void WheelLegVmcFillSide(uint8_t side, const WheelLegVmcConfig *cfg,
                                const WheelLegVmcInput *in,
                                const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                                fp32 targetTheta, fp32 targetV, fp32 pitchTrim, fp32 xErr,
                                WheelLegVmcSideState *out)
{
    const fp32 sign = (cfg->commonForward != 0u || side == WHEELLEG_SIDE_RIGHT) ?
                          1.0f : -1.0f;

    if (cfg->commonForward != 0u)
    {
        out->theta_err = 0.5f * (leg[WHEELLEG_SIDE_RIGHT].theta +
                                leg[WHEELLEG_SIDE_LEFT].theta) - targetTheta;
        out->d_theta = 0.5f * (leg[WHEELLEG_SIDE_RIGHT].d_theta +
                              leg[WHEELLEG_SIDE_LEFT].d_theta);
    }
    else
    {
        out->theta_err = leg[side].theta - targetTheta;
        out->d_theta = leg[side].d_theta;
    }
    out->x_err = sign * xErr;
    out->v_err = sign * (in->vMps - targetV);
    out->pitch_err = sign * in->pitch -
                     (cfg->pitchOffsetRad[side] + sign * pitchTrim);
    out->gyro_y = sign * in->gyroY;
}

static fp32 WheelLegVmcPositiveOrDefault(fp32 value, fp32 fallback, fp32 maxValue)
{
    if (value <= 0.0f)
    {
        value = fallback;
    }
    return WheelLegCoreClamp(value, 0.0f, maxValue);
}

static void WheelLegVmcTrimUpdate(const WheelLegVmcConfig *cfg,
                                   const WheelLegVmcInput *in,
                                   WheelLegVmcState *state,
                                   uint8_t driving)
{
    fp32 lpf;
    fp32 gain;
    fp32 maxOffset;
    fp32 maxRate;
    fp32 deadband;
    fp32 vErr;
    fp32 rate;

    if (cfg->trimEnable == 0u || in->dt <= 0.0f)
    {
        state->pitchAutoOffsetRad = 0.0f;
        state->trimVFiltered = 0.0f;
        return;
    }
    if (driving != 0u)
    {
        state->trimVFiltered = 0.0f;
        return;
    }

    maxOffset = WheelLegVmcPositiveOrDefault(cfg->trimMaxRad,
                    WHEELLEG_VMC_TRIM_MAX_RAD_DEFAULT, 0.35f);
    maxRate = WheelLegVmcPositiveOrDefault(cfg->trimRateRadps,
                  WHEELLEG_VMC_TRIM_RATE_RADPS_DEFAULT, 0.20f);
    deadband = WheelLegVmcPositiveOrDefault(cfg->trimVDeadbandMps,
                   WHEELLEG_VMC_TRIM_V_DEADBAND_MPS_DEFAULT, 0.20f);
    lpf = (cfg->trimLpf <= 0.0f) ? WHEELLEG_VMC_TRIM_LPF_DEFAULT : cfg->trimLpf;
    lpf = WheelLegCoreClamp(lpf, 0.001f, 1.0f);
    state->trimVFiltered += lpf * (in->vMps - state->trimVFiltered);
    gain = (cfg->trimGain == 0.0f) ? WHEELLEG_VMC_TRIM_GAIN_DEFAULT : cfg->trimGain;
    if (WheelLegCoreAbs(state->trimVFiltered) <= deadband)
    {
        vErr = 0.0f;
    }
    else
    {
        vErr = (state->trimVFiltered > 0.0f) ?
                   (state->trimVFiltered - deadband) :
                   (state->trimVFiltered + deadband);
    }
    rate = WheelLegCoreClamp(gain * vErr, -maxRate, maxRate);
    state->pitchAutoOffsetRad = WheelLegCoreClamp(
        state->pitchAutoOffsetRad + rate * in->dt, -maxOffset, maxOffset);
}

static void WheelLegVmcMotionUpdate(const WheelLegVmcInput *in,
                                    const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                                    WheelLegVmcState *state)
{
    const fp32 height = (leg[WHEELLEG_SIDE_RIGHT].length > leg[WHEELLEG_SIDE_LEFT].length) ?
                           leg[WHEELLEG_SIDE_RIGHT].length : leg[WHEELLEG_SIDE_LEFT].length;
    const fp32 high = WheelLegCoreClamp((height - 0.080f) / 0.040f, 0.0f, 1.0f);
    const fp32 speedError = in->targetV - state->motionSpeed;
    const fp32 accelLimit = 0.75f - 0.25f * high;
    const fp32 trackingError = state->motionSpeed - in->vMps;
    fp32 wantedAccel = WheelLegCoreClamp(speedError / 0.25f,
                                          -accelLimit, accelLimit);
    fp32 speedStep;

    if (WheelLegCoreAbs(in->vMps) > 0.10f && wantedAccel * in->vMps < 0.0f &&
        wantedAccel * trackingError > 0.0f)
    {
        const fp32 factor = WheelLegCoreClamp(
            (0.25f - 0.05f * high - WheelLegCoreAbs(trackingError)) / 0.08f,
            0.0f, 1.0f);
        wantedAccel *= factor;
    }
    state->motionAccel += WheelLegCoreClamp(wantedAccel - state->motionAccel,
                                            -6.0f * in->dt, 6.0f * in->dt);
    speedStep = state->motionAccel * in->dt;
    if (speedStep * speedError > 0.0f &&
        WheelLegCoreAbs(speedStep) >= WheelLegCoreAbs(speedError))
    {
        state->motionSpeed = in->targetV;
        state->motionAccel = 0.0f;
    }
    else
    {
        state->motionSpeed += speedStep;
    }
    state->yawRate += WheelLegCoreClamp(in->targetYawRate - state->yawRate,
                                        -4.0f * in->dt, 4.0f * in->dt);
}

uint8_t WheelLegVmcObserveRates(WheelLegVmcObserveState *state,
                               const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                               const fp32 wheelRate[WHEELLEG_SIDE_COUNT],
                               const fp32 rodRate[WHEELLEG_SIDE_COUNT],
                               fp32 gyroY, fp32 radius, fp32 lpf, fp32 dt,
                               fp32 *xM, fp32 *vMps)
{
    fp32 rateSum = 0.0f;
    uint8_t i;

    if (state == NULL || leg == NULL || wheelRate == NULL || xM == NULL || vMps == NULL ||
        !isfinite(gyroY) || !isfinite(radius) || !isfinite(lpf) || !isfinite(dt) ||
        radius <= 0.0f || dt <= 0.0f)
    {
        return 0u;
    }
    if (state->initialized == 0u)
    {
        state->xM = *xM;
        state->vMps = *vMps;
    }
    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        fp32 currentRodRate = 0.0f;
        if (!isfinite(leg[i].phi2) || !isfinite(wheelRate[i]) ||
            (rodRate != NULL && !isfinite(rodRate[i])))
        {
            return 0u;
        }
        if (rodRate != NULL)
        {
            currentRodRate = rodRate[i];
        }
        else if (state->initialized != 0u)
        {
            currentRodRate = WheelLegCoreWrapPi(leg[i].phi2 - state->lastPhi2[i]) / dt;
        }
        rateSum += wheelRate[i] + currentRodRate + gyroY;
    }
    lpf = WheelLegCoreClamp(lpf, 0.01f, 1.0f);
    state->vMps += lpf * (rateSum * 0.5f * radius - state->vMps);
    state->xM += state->vMps * dt;
    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        state->lastPhi2[i] = leg[i].phi2;
    }
    state->initialized = 1u;
    *xM = state->xM;
    *vMps = state->vMps;
    return 1u;
}

uint8_t WheelLegVmcObserve(WheelLegVmcObserveState *state,
                           const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                           const fp32 wheelRate[WHEELLEG_SIDE_COUNT],
                           fp32 gyroY, fp32 radius, fp32 lpf, fp32 dt,
                           fp32 *xM, fp32 *vMps)
{
    return WheelLegVmcObserveRates(state, leg, wheelRate, NULL,
                                  gyroY, radius, lpf, dt, xM, vMps);
}

uint8_t WheelLegVmcStep(const WheelLegVmcConfig *cfg,
                        const WheelLegVmcInput *in,
                        WheelLegVmcState *state,
                        WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                        fp32 wheelTorque[WHEELLEG_SIDE_COUNT])
{
    fp32 kRight[WHEELLEG_CORE_LQR_ROW_COUNT];
    fp32 kLeft[WHEELLEG_CORE_LQR_ROW_COUNT];
    WheelLegVmcSideState rightState;
    WheelLegVmcSideState leftState;
    const WheelLegVmcSideState *sideState[WHEELLEG_SIDE_COUNT] = {
        [WHEELLEG_SIDE_RIGHT] = &rightState,
        [WHEELLEG_SIDE_LEFT] = &leftState,
    };
    const fp32 *sideK[WHEELLEG_SIDE_COUNT] = {
        [WHEELLEG_SIDE_RIGHT] = kRight,
        [WHEELLEG_SIDE_LEFT] = kLeft,
    };
    fp32 turnT;
    fp32 rollF0;
    fp32 splitTp;
    fp32 wheelScale;
    fp32 hipScale;
    fp32 targetTheta;
    fp32 targetV;
    fp32 pitchTrim;
    fp32 xErr;
    uint8_t driving;
    uint8_t wasDriving;
    uint8_t i;

    if (cfg == NULL || in == NULL || state == NULL || leg == NULL || wheelTorque == NULL)
    {
        return 0u;
    }
    if (cfg->commonForward != 0u)
    {
        if (!isfinite(in->dt) || in->dt <= 0.0f ||
            !isfinite(in->pitch) || !isfinite(in->roll) || !isfinite(in->yaw) ||
            !isfinite(in->gyroX) || !isfinite(in->gyroY) || !isfinite(in->gyroZ) ||
            !isfinite(in->xM) || !isfinite(in->vMps) ||
            !isfinite(in->pitchAutoOffsetRad) || !isfinite(in->targetV) ||
            !isfinite(in->targetLeg) || in->targetLeg <= 0.0f ||
            !isfinite(in->targetFootX) || !isfinite(in->targetYawRate) ||
            !isfinite(cfg->supportBiasN) || !isfinite(cfg->maxSupportN) ||
            !isfinite(cfg->maxJointNm) || !isfinite(cfg->maxWheelNm) ||
            !isfinite(state->yawSet) || !isfinite(state->parkIntegral) ||
            !isfinite(state->parkSpeed) || !isfinite(state->pitchAutoOffsetRad) ||
            !isfinite(state->trimVFiltered) || !isfinite(state->motionSpeed) ||
            !isfinite(state->motionAccel) || !isfinite(state->yawRate))
        {
            return 0u;
        }
        for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
        {
            if (!isfinite(leg[i].length) || !isfinite(leg[i].theta) ||
                !isfinite(leg[i].d_theta) || !isfinite(leg[i].phi0) ||
                !isfinite(leg[i].phi1) || !isfinite(leg[i].phi2) ||
                !isfinite(leg[i].phi3) || !isfinite(leg[i].phi4) ||
                !isfinite(leg[i].l1) || !isfinite(leg[i].l4))
            {
                return 0u;
            }
        }
    }

    targetTheta = WheelLegCoreTargetThetaFromFootX(in->targetFootX, in->targetLeg);
    targetV = in->targetV;
    wasDriving = state->wasDriving;
    if (cfg->commonForward != 0u)
    {
        WheelLegVmcMotionUpdate(in, leg, state);
        driving = (WheelLegCoreAbs(in->targetV) > WHEELLEG_VMC_MOTION_EPS ||
                   WheelLegCoreAbs(in->targetYawRate) > WHEELLEG_VMC_MOTION_EPS ||
                   WheelLegCoreAbs(state->motionSpeed) > WHEELLEG_VMC_MOTION_EPS ||
                   WheelLegCoreAbs(state->yawRate) > WHEELLEG_VMC_MOTION_EPS) ? 1u : 0u;
        if (state->holdInited == 0u || driving != 0u)
        {
            state->holdXM = in->xM;
            state->holdInited = 1u;
        }
        state->wasDriving = driving;
        xErr = in->xM - state->holdXM;
        if (driving != 0u)
        {
            state->parkIntegral = 0.0f;
            state->parkSpeed = 0.0f;
        }
        else
        {
            /* 沿用 WheelLegBench.c 的驻坡位置积分和慢速补偿，轮程用本控制器停车参考。 */
            const fp32 parkError = WheelLegCoreClamp(xErr,
                -WHEELLEG_VMC_PARK_ERROR_LIMIT_M, WHEELLEG_VMC_PARK_ERROR_LIMIT_M);
            fp32 wantedPark;
            state->parkIntegral = WheelLegCoreClamp(state->parkIntegral + parkError * in->dt,
                -WHEELLEG_VMC_PARK_INTEGRAL_LIMIT, WHEELLEG_VMC_PARK_INTEGRAL_LIMIT);
            wantedPark = WheelLegCoreClamp(-parkError - 0.20f * state->parkIntegral -
                                           0.25f * in->vMps,
                                           -WHEELLEG_VMC_PARK_SPEED_LIMIT_MPS,
                                           WHEELLEG_VMC_PARK_SPEED_LIMIT_MPS);
            state->parkSpeed += WheelLegCoreClamp(wantedPark - state->parkSpeed,
                -WHEELLEG_VMC_PARK_SLEW_MPS2 * in->dt,
                WHEELLEG_VMC_PARK_SLEW_MPS2 * in->dt);
        }
        targetV = state->motionSpeed + state->parkSpeed;
        WheelLegVmcTrimUpdate(cfg, in, state, driving);
        pitchTrim = state->pitchAutoOffsetRad;
    }
    else
    {
        driving = (WheelLegCoreAbs(in->targetV) > WHEELLEG_VMC_MOTION_EPS ||
                   WheelLegCoreAbs(in->targetYawRate) > WHEELLEG_VMC_MOTION_EPS) ? 1u : 0u;
        xErr = in->xM;
        pitchTrim = in->pitchAutoOffsetRad;
    }
    xErr = WheelLegCoreLqrXError(xErr, in->targetV, in->targetYawRate,
                                 WHEELLEG_VMC_X_HOLD_LIMIT_M,
                                 WHEELLEG_VMC_MOTION_EPS);
    wheelScale = (cfg->commonForward != 0u) ? 1.0f :
                     WheelLegCoreClamp((cfg->wheelScale <= 0.0f) ?
                                           WHEELLEG_VMC_WHEEL_SCALE_DEFAULT : cfg->wheelScale,
                                       0.0f, 2.0f);
    hipScale = (cfg->commonForward != 0u) ? 1.0f :
               (in->useLqrHip != 0u) ?
                   WheelLegCoreClamp((cfg->hipScale <= 0.0f) ?
                                         WHEELLEG_VMC_HIP_SCALE_DEFAULT : cfg->hipScale,
                                     0.0f, 2.0f) : 0.0f;

    WheelLegVmcEvalLqr(cfg, leg[WHEELLEG_SIDE_RIGHT].length, kRight);
    WheelLegVmcEvalLqr(cfg, leg[WHEELLEG_SIDE_LEFT].length, kLeft);

    if (state->yawInited == 0u)
    {
        state->yawSet = in->yaw;
        state->lastYaw = in->yaw;
        state->yawInited = 1u;
    }
    if (cfg->commonForward != 0u)
    {
        if (wasDriving != 0u && driving == 0u)
        {
            state->yawSet = in->yaw;
        }
        const fp32 yawErr = WheelLegCoreWrapPi(state->yawSet +
                             state->yawRate * in->dt - in->yaw);
        state->yawSet = WheelLegCoreWrapPi(in->yaw +
                        WheelLegCoreClamp(yawErr, -0.15f, 0.15f));
        turnT = WheelLegCoreClamp(
                    cfg->turnKp * WheelLegCoreWrapPi(state->yawSet - in->yaw) +
                    cfg->turnKd * (state->yawRate - in->gyroZ),
                    -cfg->turnMax, cfg->turnMax);
    }
    else
    {
        state->yawSet += in->targetYawRate * in->dt;
        turnT = WheelLegCoreTurnTorque(state->yawSet, in->yaw, in->gyroZ,
                                      cfg->turnKp, cfg->turnKd, cfg->turnMax);
    }
    rollF0 = WheelLegCoreRollForce(0.0f, in->roll, in->gyroX,
                                  cfg->rollKp, cfg->rollKd, cfg->rollMax);

    if (in->useLqrHip != 0u)
    {
        if (cfg->commonForward != 0u)
        {
            splitTp = WheelLegCorePidCalc(&state->splitPid,
                          leg[WHEELLEG_SIDE_RIGHT].theta - leg[WHEELLEG_SIDE_LEFT].theta,
                          0.0f);
        }
        else
        {
            splitTp = WheelLegCorePidCalc(&state->splitPid,
                          leg[WHEELLEG_SIDE_RIGHT].theta + leg[WHEELLEG_SIDE_LEFT].theta,
                          2.0f * targetTheta);
        }
    }
    else
    {
        splitTp = 0.0f;
        WheelLegCorePidClear(&state->splitPid);
    }

    WheelLegVmcFillSide(WHEELLEG_SIDE_RIGHT, cfg, in, leg, targetTheta, targetV, pitchTrim, xErr, &rightState);
    WheelLegVmcFillSide(WHEELLEG_SIDE_LEFT, cfg, in, leg, targetTheta, targetV, pitchTrim, xErr, &leftState);

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        wheelTorque[i] = WheelLegCoreLqrWheelOutput(sideK[i],
                        sideState[i]->theta_err, sideState[i]->d_theta,
                        sideState[i]->x_err, sideState[i]->v_err,
                        sideState[i]->pitch_err, sideState[i]->gyro_y);
        leg[i].tp = 0.0f;
        if (in->useLqrHip != 0u)
        {
            const fp32 sideSplit = (cfg->commonForward != 0u && i == WHEELLEG_SIDE_LEFT) ?
                                       -splitTp : splitTp;
            leg[i].tp = hipScale * WheelLegCoreLqrHipOutput(sideK[i],
                            sideState[i]->theta_err, sideState[i]->d_theta,
                            sideState[i]->x_err, sideState[i]->v_err,
                            sideState[i]->pitch_err, sideState[i]->gyro_y, sideSplit);
            if (cfg->commonForward != 0u)
            {
                leg[i].tp = -leg[i].tp;
            }
        }
    }

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        fp32 cosTheta = cosf(leg[i].theta);
        if (WheelLegCoreAbs(cosTheta) < 0.1f)
        {
            cosTheta = (cosTheta >= 0.0f) ? 0.1f : -0.1f;
        }
        leg[i].f0 = cfg->supportBiasN / cosTheta +
                    WheelLegCorePidCalc(&state->legPid[i], leg[i].length, in->targetLeg);
        if (cfg->commonForward != 0u)
        {
            leg[i].f0 += (i == WHEELLEG_SIDE_RIGHT) ? -rollF0 : rollF0;
        }
        else
        {
            leg[i].f0 += (i == WHEELLEG_SIDE_RIGHT) ? rollF0 : -rollF0;
        }
        leg[i].f0 = WheelLegCoreClamp(leg[i].f0, -cfg->maxSupportN, cfg->maxSupportN);
        leg[i].tp = WheelLegCoreClamp(leg[i].tp, -cfg->maxJointNm, cfg->maxJointNm);
        const fp32 turn = (cfg->commonForward != 0u) ?
                              ((i == WHEELLEG_SIDE_RIGHT) ? turnT : -turnT) : -turnT;
        wheelTorque[i] = WheelLegCoreClamp(wheelTorque[i] * wheelScale + turn,
                                          -cfg->maxWheelNm, cfg->maxWheelNm);
        leg[i].contact = (leg[i].f0 * cosf(leg[i].theta) > 3.0f) ? 1u : 0u;
        if (WheelLegCoreCalcVmc(&leg[i]) == 0u)
        {
            return 0u;
        }
        leg[i].joint_torque[0] = WheelLegCoreClamp(leg[i].joint_torque[0],
                                                 -cfg->maxJointNm, cfg->maxJointNm);
        leg[i].joint_torque[1] = WheelLegCoreClamp(leg[i].joint_torque[1],
                                                 -cfg->maxJointNm, cfg->maxJointNm);
        if (cfg->commonForward != 0u &&
            (!isfinite(leg[i].f0) || !isfinite(leg[i].tp) ||
             !isfinite(wheelTorque[i]) || !isfinite(leg[i].joint_torque[0]) ||
             !isfinite(leg[i].joint_torque[1])))
        {
            return 0u;
        }
    }
    return 1u;
}
