/* 小轮腿仿真只做输入输出适配，VMC 运算与固件共用。 */
#include <stdlib.h>
#include <string.h>
#include "WheelLegVmc.h"
#include "WheelLegBench.h"

typedef struct
{
    WheelLegVmcConfig config;
    WheelLegVmcState control;
    WheelLegCoreGeometry geometry;
    WheelLegCoreLegCalc leg[2];
    WheelLegVmcObserveState observer;
    WheelLegBenchWheel wheels[2];
    WheelLegCoreTargetSmooth target;
    float radius;
    float observerLpf;
    float attitudeLimit;
} MeasuredVmc;

/* 此文件由 VmcFirmware.py 从当前车型配置原文生成，不维护第二份参数。 */
#include "MeasuredVmcConfig.inc"

void *MeasuredVmcCreate(void)
{
    MeasuredVmc *vmc = calloc(1u, sizeof(*vmc));
    if (vmc != NULL) {
        MeasuredVmcConfigure(vmc);
    }
    return vmc;
}

void MeasuredVmcFree(void *handle)
{
    free(handle);
}

/* q 为统一前向的 RF/RB/LF/LB；轮速也已转为统一前向。 */
int MeasuredVmcStep(void *handle, const float q[4], const float dq[4],
                    const float wheel[2], const float wheelRate[2],
                    const float attitude[3], const float gyro[3], float dt,
                    float targetV, float targetYaw, float targetLength,
                    float pitchTrim, uint32_t tick, float out[20])
{
    MeasuredVmc *vmc = handle;
    WheelLegVmcInput input = {0};
    float torque[2] = {0};
    float rodRate[2] = {0};
    unsigned i;
    if (vmc == NULL || out == NULL) return 0;
    memset(out, 0, 20u * sizeof(float));
    for (i = 0; i < 3; ++i) {
        if (!isfinite(attitude[i]) || !isfinite(gyro[i])) return 0;
    }
    if (fabsf(attitude[0]) > vmc->attitudeLimit ||
        fabsf(attitude[1]) > vmc->attitudeLimit) return 0;
    for (i = 0; i < 2; ++i) {
        const unsigned side = i == 0 ? WHEELLEG_SIDE_RIGHT : WHEELLEG_SIDE_LEFT;
        if (!WheelLegVmcKinematics(&vmc->geometry, &vmc->leg[side],
                q[2*i], q[2*i+1], dq[2*i], dq[2*i+1], attitude[1], gyro[1], dt,
                &rodRate[side])) return 0;
    }
    input.dt = dt;
    input.roll = attitude[0]; input.pitch = attitude[1]; input.yaw = attitude[2];
    input.gyroX = gyro[0]; input.gyroY = gyro[1]; input.gyroZ = gyro[2];
    float sideRate[2];
    for (i = 0; i < 2; ++i) {
        const unsigned side = i == 0 ? WHEELLEG_SIDE_RIGHT : WHEELLEG_SIDE_LEFT;
        if (!WheelLegBenchWheelObserve(&vmc->wheels[side], wheel[i], wheelRate[i], tick))
            return -(int)(100u + i*10u + vmc->wheels[side].reject);
        sideRate[side] = vmc->wheels[side].rate;
    }
    if (!WheelLegVmcObserveRates(&vmc->observer, vmc->leg, sideRate, rodRate, gyro[1],
            vmc->radius, vmc->observerLpf, dt, &input.xM, &input.vMps)) return 0;
    input.pitchAutoOffsetRad = pitchTrim;
    input.targetV = targetV; input.targetYawRate = targetYaw;
    if (!WheelLegCoreTargetSmoothUpdateXy(&vmc->target, 0.0f, targetLength,
            .5f*(vmc->leg[0].length+vmc->leg[1].length),
            .5f*(vmc->leg[0].alpha+vmc->leg[1].alpha),
            vmc->config.minLegM, vmc->config.maxLegM, WHEELLEG_MANUAL_FOOT_X_RANGE_M,
            WHEELLEG_TARGET_FOOT_X_SLEW_MPS, WHEELLEG_TARGET_LEG_SLEW_MPS, .003f, dt)) return 0;
    input.targetLeg = vmc->target.length_m;
    input.targetFootX = vmc->target.foot_x_m;
    input.useLqrHip = 1u;
    if (!WheelLegVmcStep(&vmc->config, &input, &vmc->control, vmc->leg, torque)) return 0;
    for (i = 0; i < 2; ++i) {
        const unsigned side = i == 0 ? WHEELLEG_SIDE_RIGHT : WHEELLEG_SIDE_LEFT;
        out[3*i] = vmc->leg[side].joint_torque[0];
        out[3*i+1] = vmc->leg[side].joint_torque[1];
        out[3*i+2] = torque[side];
        out[6+i] = vmc->leg[side].length;
        out[8+i] = vmc->leg[side].alpha;
        out[10+i] = vmc->leg[side].f0;
        out[12+i] = vmc->leg[side].tp;
    }
    out[14] = input.xM; out[15] = input.vMps;
    out[16] = input.targetLeg;
    out[17] = vmc->control.pitchAutoOffsetRad;
    out[18] = vmc->control.motionSpeed;
    out[19] = vmc->control.yawRate;
    return 1;
}
