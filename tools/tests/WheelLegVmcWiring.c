/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * MINI_BENCH 的 VMC 输入和输出必须与稳定板凳共用关节零位及方向。
 */

#include "WheelLegBench.h"
#include "WheelLegCore.h"

#include <assert.h>
#include <math.h>

static void Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 1.0e-6f);
}

int main(void)
{
    const WheelLegCoreGeometry geo = {0.05f, 0.105f, 0.105f, 0.05f, 0.07f};
    WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT] = {0};
    WheelLegCoreOutput out;
    float raw[4];
    float joint[4];
    float wheelMotor[WHEELLEG_SIDE_COUNT] = {
        [WHEELLEG_SIDE_RIGHT] = -0.2f,
        [WHEELLEG_SIDE_LEFT] = 0.2f,
    };
    unsigned i;

    for (i = 0u; i < 4u; i++)
    {
        const float expectedDir = (i < 2u) ? 1.0f : -1.0f;
        const float angle = (i % 2u == 0u) ? 0.1f : -0.1f;
        Near(WheelLegBenchJointDir(i), expectedDir);
        raw[i] = WheelLegBenchJointToRaw(i, angle);
        joint[i] = WheelLegBenchJointFromRaw(i, raw[i]);
        Near(joint[i], angle);
    }
    assert(WheelLegCoreCalcKinematics(&geo, &leg[WHEELLEG_SIDE_RIGHT],
                                      joint[0], joint[1], 0.02f, 0.03f, 0u, 0.003f));
    assert(WheelLegCoreCalcKinematics(&geo, &leg[WHEELLEG_SIDE_LEFT],
                                      joint[2], joint[3], 0.02f, 0.03f, 0u, 0.003f));
    Near(leg[WHEELLEG_SIDE_RIGHT].length, leg[WHEELLEG_SIDE_LEFT].length);
    Near(leg[WHEELLEG_SIDE_RIGHT].theta, leg[WHEELLEG_SIDE_LEFT].theta);
    Near(leg[WHEELLEG_SIDE_RIGHT].phi2, leg[WHEELLEG_SIDE_LEFT].phi2);

    leg[WHEELLEG_SIDE_RIGHT].joint_torque[0] = 0.8f;
    leg[WHEELLEG_SIDE_RIGHT].joint_torque[1] = -0.7f;
    leg[WHEELLEG_SIDE_LEFT].joint_torque[0] = 0.6f;
    leg[WHEELLEG_SIDE_LEFT].joint_torque[1] = -0.5f;
    WheelLegCoreOutputClear(&out);
    WheelLegCoreSetVmcJointTorques(&out, leg,
                                   (int8_t)WheelLegBenchJointDir(0u),
                                   (int8_t)WheelLegBenchJointDir(1u),
                                   (int8_t)WheelLegBenchJointDir(2u),
                                   (int8_t)WheelLegBenchJointDir(3u));
    Near(out.actuator[WHEELLEG_CORE_ACT_RIGHT_FRONT].tau, 0.8f);
    Near(out.actuator[WHEELLEG_CORE_ACT_RIGHT_BACK].tau, -0.7f);
    Near(out.actuator[WHEELLEG_CORE_ACT_LEFT_FRONT].tau, -0.6f);
    Near(out.actuator[WHEELLEG_CORE_ACT_LEFT_BACK].tau, 0.5f);
    WheelLegCoreSetWheelTorques(&out, wheelMotor);
    Near(out.actuator[WHEELLEG_CORE_ACT_RIGHT_WHEEL].tau, -0.2f);
    Near(out.actuator[WHEELLEG_CORE_ACT_LEFT_WHEEL].tau, 0.2f);
    return 0;
}
