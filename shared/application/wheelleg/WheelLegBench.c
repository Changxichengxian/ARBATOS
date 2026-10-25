/* 固定高度板凳：与实测模型共用几何、有限关节刚度和轮端六状态反馈。 */
#include "WheelLegBench.h"
#include "WheelLegCore.h"
#include <math.h>
#include <string.h>

#include "WheelLegBenchGains.inc"

static float BenchClip(float value, float limit)
{
    return fminf(fmaxf(value, -limit), limit);
}

static float BenchWrap(float value)
{
    return remainderf(value, 2.0f * WHEELLEG_CORE_PI);
}

static int BenchLeg(const float joint[2], float out[5])
{
    const WheelLegCoreGeometry geometry = {.l1_m = .05f, .l2_m = .105f,
        .l3_m = .105f, .l4_m = .05f, .l5_m = .07f};
    WheelLegCoreFootPoint foot;
    WheelLegCoreLegCalc leg = {0};
    if (!WheelLegCoreForwardPoint(&geometry, joint[0], joint[1], &foot) ||
        !WheelLegCoreCalcKinematics(&geometry, &leg, joint[0], joint[1], 0, 0, 0, .003f)) {
        return 0;
    }
    out[0] = foot.length_m;
    out[1] = leg.alpha;
    const float elbowX = -.035f - .05f * sinf(joint[0]);
    const float elbowDown = .05f * cosf(joint[0]);
    out[2] = atan2f(-(foot.x_m - elbowX), foot.y_m - elbowDown);
    leg.f0 = 1.0f;
    leg.tp = 0.0f;
    if (!WheelLegCoreCalcVmc(&leg)) return 0;
    out[3] = leg.joint_torque[0];
    out[4] = leg.joint_torque[1];
    for (unsigned i = 0; i < 5; i++) if (!isfinite(out[i])) return 0;
    return 1;
}

void WheelLegBenchReset(WheelLegBenchState *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

void WheelLegBenchAutoReset(WheelLegBenchAuto *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

static int BenchAutoReject(WheelLegBenchAuto *state, uint32_t reason, float value)
{
    state->reject = reason;
    state->rejectValue = value;
    return 0;
}

int WheelLegBenchAutoStep(WheelLegBenchAuto *state, const WheelLegBenchInput *in,
                         WheelLegBenchOutput *out)
{
    if (state == NULL || in == NULL || out == NULL) return 0;
    memset(out, 0, sizeof(*out));
    if (!isfinite(in->dt) || in->dt < .001f || in->dt > .006f ||
        !isfinite(in->pitch) || !isfinite(in->roll) ||
        fabsf(in->pitch) > .35f || fabsf(in->roll) > .35f) return BenchAutoReject(state, 1, in->dt);
    for (unsigned i = 0; i < 4; i++) {
        if (!isfinite(in->joint[i]) || !isfinite(in->jointRate[i])) return BenchAutoReject(state, 2, (float)i);
        if (fabsf(in->jointRate[i]) > 3.0f) return BenchAutoReject(state, 20u+i, in->jointRate[i]);
    }
    if (state->phase == 0u) {
        const WheelLegCoreGeometry geometry = {.l1_m=.05f, .l2_m=.105f,
            .l3_m=.105f, .l4_m=.05f, .l5_m=.07f};
        const WheelLegCoreFootPoint target = {.x_m=0, .y_m=.100f};
        float front, back;
        /* 固定选用仿真已经验证的膝向外分支，不追随任意折叠分支。 */
        if (!WheelLegCoreInversePoint(&geometry, &target, 1.0f, -1.0f, &front, &back)) return BenchAutoReject(state, 3, 0);
        float travel = 0;
        for (unsigned i = 0; i < 4; i++) {
            state->start[i] = in->joint[i];
            state->target[i] = i % 2u == 0u ? front : back;
            const float delta = fabsf(state->target[i] - state->start[i]);
            /* 超过半圈不能仅凭编码器推断无碰撞路线。 */
            if (delta > 2.8f) return BenchAutoReject(state, 4, delta);
            travel = fmaxf(travel, delta);
        }
        state->duration = fmaxf(2.5f, travel * 1.5f / .30f);
        state->phase = 1u;
    }
    if (state->phase == 3u) {
        return WheelLegBenchStep(&state->balance, in, out) ? 1 : BenchAutoReject(state, 9, 0);
    }
    if (state->phase != 1u && state->phase != 2u) return BenchAutoReject(state, 5, (float)state->phase);

    state->elapsed += in->dt;
    if (state->elapsed > state->duration + 4.0f) return BenchAutoReject(state, 6, state->elapsed);
    const float fraction = fminf(state->elapsed / state->duration, 1.0f);
    const float blend = fraction * fraction * (3.0f - 2.0f * fraction);
    const float gravityRamp = fminf(state->elapsed / .15f, 1.0f);
    float torques[6] = {0};
    float maxError = 0, maxRate = 0;
    for (unsigned side = 0; side < 2; side++) {
        float leg[5];
        if (!BenchLeg(&in->joint[side*2], leg)) return BenchAutoReject(state, 7, (float)side);
        out->length[side] = leg[0];
        out->alpha[side] = leg[1];
        for (unsigned j = 0; j < 2; j++) {
            unsigned i = side*2+j;
            float delta = state->target[i] - state->start[i];
            float reference = state->start[i] + blend * delta;
            float error = reference - in->joint[i];
            if (fabsf(error) > .35f) return BenchAutoReject(state, 10u+i, error);
            /* 延迟的速度反馈会把阻尼变成激振；主控只算较缓的位置项。 */
            torques[side*3+j] = BenchClip(6.0f * error +
                gravityRamp * leg[3+j] * (2.2648f * 9.81f * .5f), 1.0f);
            maxError = fmaxf(maxError, fabsf(state->target[i] - in->joint[i]));
            maxRate = fmaxf(maxRate, fabsf(in->jointRate[i]));
        }
    }
    memcpy(out->torque, torques, sizeof(torques));
    out->jointDamping = .40f;
    if (fraction >= 1.0f) {
        state->phase = 2u;
        state->settled = maxError < .03f && maxRate < .20f ? state->settled + in->dt : 0;
        if (state->settled >= .4f) {
            WheelLegBenchReset(&state->balance);
            if (!WheelLegBenchStep(&state->balance, in, out)) return BenchAutoReject(state, 8, 0);
            state->phase = 3u;
        }
    }
    return 1;
}

int WheelLegBenchStep(WheelLegBenchState *state, const WheelLegBenchInput *in,
                     WheelLegBenchOutput *out)
{
    if (state == NULL || in == NULL || out == NULL) return 0;
    memset(out, 0, sizeof(*out));
    /* 拒绝坏反馈，不能让 NaN 经限幅变成非零力矩。 */
    for (unsigned i = 0; i < 4; i++) {
        if (!isfinite(in->joint[i]) || !isfinite(in->jointRate[i])) return 0;
        if (fabsf(in->jointRate[i]) > 3.0f) return 0;
    }
    for (unsigned i = 0; i < 2; i++) {
        if (!isfinite(in->wheel[i]) || !isfinite(in->wheelRate[i])) return 0;
    }
    for (unsigned i = 0; i < 3; i++) if (!isfinite(in->gyro[i])) return 0;
    if (!isfinite(in->pitch) || !isfinite(in->roll) || !isfinite(in->yaw) ||
        !isfinite(in->targetV) || !isfinite(in->targetYaw) ||
        !isfinite(in->dt) || in->dt < .001f || in->dt > .006f ||
        fabsf(in->pitch) > .35f || fabsf(in->roll) > .35f) return 0;

    float leg[2][5], alphaRate = 0, wheelPosition = 0, velocity = 0;
    for (unsigned side = 0; side < 2; side++) {
        unsigned j = side * 2;
        float future[5], next[2] = {in->joint[j] + in->jointRate[j] * .0001f,
                                   in->joint[j+1] + in->jointRate[j+1] * .0001f};
        if (!BenchLeg(&in->joint[j], leg[side]) || !BenchLeg(next, future)) return 0;
        out->length[side] = leg[side][0];
        out->alpha[side] = leg[side][1];
        /* 当前增益只覆盖该站姿区间，不对接近伸直的腿外推。 */
        if (out->length[side] < .083f || out->length[side] > .122f ||
            fabsf(out->alpha[side]) > .25f) return 0;
        alphaRate += .5f * BenchWrap(future[1] - leg[side][1]) / .0001f;
        float rodRate = BenchWrap(future[2] - leg[side][2]) / .0001f;
        wheelPosition += .5f * .03275f * (in->wheel[side] + leg[side][2] + in->pitch);
        velocity += .5f * .03275f * (in->wheelRate[side] + rodRate + in->gyro[1]);
    }
    if (fabsf(out->length[0] - out->length[1]) > .015f) return 0;

    if (!state->ready) {
        for (unsigned i = 0; i < 4; i++) {
            state->hold[i] = in->joint[i];
            state->feedforward[i] = leg[i/2][3+i%2] * (2.2648f * 9.81f * .5f);
        }
        state->reference = wheelPosition;
        state->yawReference = in->yaw;
        state->ready = 1;
    }
    for (unsigned i = 0; i < 4; i++) {
        if (fabsf(state->hold[i] - in->joint[i]) > .25f) return 0;
    }
    float targetV = BenchClip(in->targetV, .45f);
    float targetYaw = BenchClip(in->targetYaw, 1.2f);
    float acceleration = (targetV * state->speed < 0 || fabsf(targetV) < fabsf(state->speed)) ? 2.0f : 1.5f;
    state->speed += BenchClip(targetV - state->speed, acceleration * in->dt);
    if (fabsf(targetV) > .001f) state->driving = 1;
    if (state->driving) {
        state->reference = wheelPosition;
        if (fabsf(state->speed) < .001f && fabsf(velocity) < .025f) state->driving = 0;
    }
    out->state[0] = .5f * (out->alpha[0] + out->alpha[1]) - in->pitch;
    out->state[1] = alphaRate - in->gyro[1];
    out->state[2] = BenchClip(wheelPosition - state->reference, .10f);
    out->state[3] = velocity - state->speed;
    out->state[4] = in->pitch;
    out->state[5] = in->gyro[1];
    float length = .5f * (out->length[0] + out->length[1]);
    unsigned index = 0;
    while (index + 2 < sizeof(BenchGains)/sizeof(BenchGains[0]) &&
           length > BenchGains[index+1][0]) index++;
    float fraction = fminf(fmaxf((length - BenchGains[index][0]) /
        (BenchGains[index+1][0] - BenchGains[index][0]), 0), 1);
    float torque = 0;
    for (unsigned i = 0; i < 6; i++) {
        float gain = BenchGains[index][i+1] * (1-fraction) + BenchGains[index+1][i+1] * fraction;
        torque += gain * out->state[i];
    }
    state->yawRate += BenchClip(targetYaw - state->yawRate, 4.0f * in->dt);
    float yawError = BenchClip(BenchWrap(state->yawReference + state->yawRate * in->dt - in->yaw), .15f);
    state->yawReference = BenchWrap(in->yaw + yawError);
    float turn = BenchClip(.5f * yawError + .12f * (state->yawRate - in->gyro[2]), .10f);
    for (unsigned side = 0; side < 2; side++) {
        for (unsigned end = 0; end < 2; end++) {
            unsigned i = side*2 + end;
            out->torque[side*3+end] = BenchClip(state->feedforward[i] +
                6.0f * (state->hold[i] - in->joint[i]), 1.5f);
        }
        out->torque[side*3+2] = BenchClip(torque + (side == 0 ? turn : -turn), .45f);
    }
    for (unsigned i = 0; i < 6; i++) if (!isfinite(out->torque[i])) {
        memset(out->torque, 0, sizeof(out->torque));
        return 0;
    }
    out->jointDamping = .40f;
    return 1;
}
