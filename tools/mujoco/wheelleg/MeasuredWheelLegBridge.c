/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "WheelLegCore.h"

/* 左右模型统一使用前向 X、向下腿坐标，实车镜像和编码器零位不在此处猜测。 */
int WheelLegSimLeg(const float geometry[5], float front, float back, float out[8])
{
    WheelLegCoreGeometry geo = {0};
    WheelLegCoreFootPoint point = {0};
    WheelLegCoreLegCalc leg = {0};
    if (geometry == NULL || out == NULL) {
        return 0;
    }
    memset(out, 0, 8u * sizeof(float));
    geo.l1_m = geometry[0];
    geo.l2_m = geometry[1];
    geo.l3_m = geometry[2];
    geo.l4_m = geometry[3];
    geo.l5_m = geometry[4];
    if (!WheelLegCoreForwardPoint(&geo, front, back, &point) ||
        !WheelLegCoreCalcKinematics(&geo, &leg, front, back, 0.0f, 0.0f, 0u, 0.003f)) {
        return 0;
    }
    out[0] = point.x_m;
    out[1] = point.y_m;
    out[2] = point.length_m;
    out[3] = leg.alpha;
    leg.f0 = 1.0f;
    leg.tp = 0.0f;
    if (!WheelLegCoreCalcVmc(&leg)) {
        return 0;
    }
    out[4] = leg.joint_torque[0];
    out[5] = leg.joint_torque[1];
    leg.f0 = 0.0f;
    leg.tp = 1.0f;
    if (!WheelLegCoreCalcVmc(&leg)) {
        return 0;
    }
    out[6] = leg.joint_torque[0];
    out[7] = leg.joint_torque[1];
    return 1;
}

void WheelLegSimBalance(const float gain[12], const float state[6], float out[2])
{
    out[0] = WheelLegCoreLqrWheelOutput(gain, state[0], state[1], state[2],
                                     state[3], state[4], state[5]);
    out[1] = WheelLegCoreLqrHipOutput(gain, state[0], state[1], state[2],
                                   state[3], state[4], state[5], 0.0f);
}
