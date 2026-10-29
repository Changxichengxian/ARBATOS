/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEELLEG_VMC_H
#define WHEELLEG_VMC_H

#include "WheelLegCore.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const fp32 (*lqrPoly)[WHEELLEG_CORE_LQR_COEFF_COUNT];
    fp32 minLegM;
    fp32 maxLegM;
    fp32 wheelScale;
    fp32 hipScale;
    fp32 supportBiasN;
    fp32 maxSupportN;
    fp32 maxJointNm;
    fp32 maxWheelNm;
    fp32 turnKp;
    fp32 turnKd;
    fp32 turnMax;
    fp32 rollKp;
    fp32 rollKd;
    fp32 rollMax;
    fp32 pitchOffsetRad[WHEELLEG_SIDE_COUNT];
    uint8_t commonForward;
    uint8_t trimEnable;
    fp32 trimGain;
    fp32 trimMaxRad;
    fp32 trimRateRadps;
    fp32 trimVDeadbandMps;
    fp32 trimLpf;
} WheelLegVmcConfig;

typedef struct
{
    fp32 dt;
    fp32 pitch;
    fp32 roll;
    fp32 yaw;
    fp32 gyroX;
    fp32 gyroY;
    fp32 gyroZ;
    fp32 xM;
    fp32 vMps;
    fp32 pitchAutoOffsetRad;
    fp32 targetV;
    fp32 targetLeg;
    fp32 targetFootX;
    fp32 targetYawRate;
    uint8_t useLqrHip;
} WheelLegVmcInput;

typedef struct
{
    fp32 lastPhi2[WHEELLEG_SIDE_COUNT];
    fp32 xM;
    fp32 vMps;
    uint8_t initialized;
} WheelLegVmcObserveState;

typedef struct
{
    WheelLegCorePid legPid[WHEELLEG_SIDE_COUNT];
    WheelLegCorePid splitPid;
    WheelLegVmcObserveState observer;
    fp32 yawSet;
    fp32 lastYaw;
    fp32 holdXM;
    fp32 parkIntegral;
    fp32 parkSpeed;
    fp32 motionSpeed;
    fp32 motionAccel;
    fp32 yawRate;
    fp32 pitchAutoOffsetRad;
    fp32 trimVFiltered;
    uint8_t yawInited;
    uint8_t holdInited;
    uint8_t wasDriving;
} WheelLegVmcState;

/* 改动仅限传入的状态、腿计算结果和轮端输出；失败时保留已计算的前半帧。 */
uint8_t WheelLegVmcStep(const WheelLegVmcConfig *cfg,
                        const WheelLegVmcInput *in,
                        WheelLegVmcState *state,
                        WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                        fp32 wheelTorque[WHEELLEG_SIDE_COUNT]);

void WheelLegVmcEvalLqr(const WheelLegVmcConfig *cfg, fp32 legLength,
                        fp32 out[WHEELLEG_CORE_LQR_ROW_COUNT]);
const fp32 *WheelLegVmcLqrRow(const WheelLegVmcConfig *cfg, uint8_t index);

/* 关节速度已转为统一前向；杆角速度由同一几何正解在当前速度方向求导。 */
uint8_t WheelLegVmcKinematics(const WheelLegCoreGeometry *geo,
                              WheelLegCoreLegCalc *leg,
                              fp32 front, fp32 back,
                              fp32 frontRate, fp32 backRate,
                              fp32 pitch, fp32 gyroY, fp32 dt,
                              fp32 *rodRate);

/* 轮速按统一前向为正；x/v 首次调用前由调用方给出初值。 */
uint8_t WheelLegVmcObserve(WheelLegVmcObserveState *state,
                           const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                           const fp32 wheelRate[WHEELLEG_SIDE_COUNT],
                           fp32 gyroY, fp32 radius, fp32 lpf, fp32 dt,
                           fp32 *xM, fp32 *vMps);
uint8_t WheelLegVmcObserveRates(WheelLegVmcObserveState *state,
                               const WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT],
                               const fp32 wheelRate[WHEELLEG_SIDE_COUNT],
                               const fp32 rodRate[WHEELLEG_SIDE_COUNT],
                               fp32 gyroY, fp32 radius, fp32 lpf, fp32 dt,
                               fp32 *xM, fp32 *vMps);

#ifdef __cplusplus
}
#endif

#endif
