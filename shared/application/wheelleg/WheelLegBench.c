/* 固定高度板凳：与实测模型共用几何、有限关节刚度和轮端六状态反馈。 */
#include "WheelLegBench.h"
#include "WheelLegCore.h"
#include <math.h>
#include <string.h>

#include "WheelLegBenchGains.inc"

/* 2026-10-24 手动竖直、水平原始读数；顺序为右前、右后、左前、左后。
 * 水平时前关节为正角、后关节为负角，不能把前后都映射为同号。 */
static const float BenchVertical[4] = {-1.197642481f, .496490425f, .537308309f, -1.273937591f};
static const float BenchHorizontal[4] = {.340466926f, -1.014915694f, -1.029411765f, .293926909f};

float WheelLegBenchJointDir(unsigned index)
{
    return index < 2u ? 1.0f : -1.0f;
}

float WheelLegBenchJointFromRaw(unsigned index, float raw)
{
    return index < 4u ? (raw - BenchVertical[index]) * WheelLegBenchJointDir(index) : NAN;
}

float WheelLegBenchJointToRaw(unsigned index, float angle)
{
    return index < 4u ? BenchVertical[index] + angle * WheelLegBenchJointDir(index) : NAN;
}

static float BenchClip(float value, float limit)
{
    return fminf(fmaxf(value, -limit), limit);
}

static float BenchWrap(float value)
{
    return remainderf(value, 2.0f * WHEELLEG_CORE_PI);
}

int WheelLegBenchWheelObserve(WheelLegBenchWheel *state, float raw, float reportedRate,
                             uint32_t tick)
{
    if (state == NULL || !isfinite(raw) || fabsf(raw) > 12.51f ||
        !isfinite(reportedRate) || fabsf(reportedRate) > 35.0f) return 0;
    if (!state->ready) {
        if (fabsf(reportedRate) > 2.0f) return 0;
        state->previous = raw;
        state->position = state->rate = 0;
        state->tick = tick;
        state->ready = 1u;
        return 1;
    }
    const uint32_t elapsed = tick - state->tick;
    if (elapsed == 0u) return 1;
    if (elapsed > 80u) return 0;
    const float dt = (float)elapsed * .001f;
    /* H3510 的反馈范围为 -12.5..12.5 rad，跨界周期是 25 rad，不是 2*pi。 */
    const float delta = remainderf(raw - state->previous, 25.0f);
    if (fabsf(delta) > 35.0f * dt + .001f) return 0;
    state->position += delta;
    /* 电机速度估计在出力时会快速摆动；角度差使用帧间隔，滤掉编码量化台阶。 */
    state->rate += dt / (.020f + dt) * (delta / dt - state->rate);
    state->previous = raw;
    state->tick = tick;
    return 1;
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
    return WheelLegBenchPoseStep(state, in, out, 1u);
}

int WheelLegBenchPoseStep(WheelLegBenchAuto *state, const WheelLegBenchInput *in,
                         WheelLegBenchOutput *out, uint32_t allowWheels)
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
        const WheelLegCoreFootPoint target = {.x_m=0, .y_m=.090f};
        float front, back;
        /* 固定选用仿真已经验证的膝向外分支，不追随任意折叠分支。 */
        if (!WheelLegCoreInversePoint(&geometry, &target, 1.0f, -1.0f, &front, &back)) return BenchAutoReject(state, 3, 0);
        float travel = 0;
        for (unsigned i = 0; i < 4; i++) {
            state->start[i] = in->joint[i];
            state->target[i] = i % 2u == 0u ? front : back;
            const float rawTarget = WheelLegBenchJointToRaw(i, state->target[i]);
            if (rawTarget < fminf(BenchVertical[i], BenchHorizontal[i]) ||
                rawTarget > fmaxf(BenchVertical[i], BenchHorizontal[i])) {
                return BenchAutoReject(state, 30u+i, rawTarget);
            }
            const float delta = fabsf(state->target[i] - state->start[i]);
            /* 超过半圈不能仅凭编码器推断无碰撞路线。 */
            if (delta > 2.8f) return BenchAutoReject(state, 4, delta);
            travel = fmaxf(travel, delta);
        }
        state->duration = fmaxf(2.5f, travel * 1.5f / .30f);
        state->phase = 1u;
    }
    if (state->phase == 3u) {
        if (allowWheels) {
            return WheelLegBenchStep(&state->balance, in, out) ? 1 : BenchAutoReject(state, 9, 0);
        }
        state->phase = 4u;
        WheelLegBenchReset(&state->balance);
    }
    if (state->phase != 1u && state->phase != 2u && state->phase != 4u) return BenchAutoReject(state, 5, (float)state->phase);

    /* 到位后持续保持四个固定角度，等落地确认，不因等候时间到而突然松腿。 */
    if (state->phase != 4u) {
        state->elapsed += in->dt;
        if (state->elapsed > state->duration + 4.0f) return BenchAutoReject(state, 6, state->elapsed);
    }
    const float fraction = fminf(state->elapsed / state->duration, 1.0f);
    const float blend = fraction * fraction * (3.0f - 2.0f * fraction);
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
            out->jointTarget[i] = reference;
            maxError = fmaxf(maxError, fabsf(state->target[i] - in->joint[i]));
            maxRate = fmaxf(maxRate, fabsf(in->jointRate[i]));
        }
    }
    memcpy(out->torque, torques, sizeof(torques));
    out->jointDamping = .40f;
    out->jointStiffness = 8.0f;
    if (fraction >= 1.0f) {
        if (state->phase != 4u) state->phase = 2u;
        state->settled = maxError < .08f && maxRate < .20f ? state->settled + in->dt : 0;
        if (state->settled >= .4f) {
            state->phase = 4u;
            if (allowWheels) {
                WheelLegBenchReset(&state->balance);
                if (!WheelLegBenchStep(&state->balance, in, out)) return BenchAutoReject(state, 8, 0);
                for (unsigned i = 0; i < 4; i++) {
                    state->balance.hold[i] = state->target[i];
                    out->jointTarget[i] = state->target[i];
                }
                state->phase = 3u;
            }
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
        /* 独立运行也保留原电脑端轮速保护；静止接入，避免异常反馈直接打满力矩。 */
        if (fabsf(in->wheelRate[i]) > 35.0f ||
            (!state->ready && fabsf(in->wheelRate[i]) > 2.0f)) return 0;
    }
    for (unsigned i = 0; i < 3; i++) if (!isfinite(in->gyro[i])) return 0;
    if (!isfinite(in->pitch) || !isfinite(in->roll) || !isfinite(in->yaw) ||
        !isfinite(in->targetV) || !isfinite(in->targetYaw) ||
        !isfinite(in->pitchTrim) || fabsf(in->pitchTrim) > WHEELLEG_BENCH_TRIM_LIMIT ||
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
    /* 固定腿姿的平衡目标整体慢移；轮程和倾倒保护仍使用真实俯仰。 */
    out->state[0] = .5f * (out->alpha[0] + out->alpha[1]) - in->pitch + in->pitchTrim;
    out->state[1] = alphaRate - in->gyro[1];
    out->state[2] = BenchClip(wheelPosition - state->reference, .10f);
    out->state[3] = velocity - state->speed;
    out->state[4] = in->pitch - in->pitchTrim;
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
            out->jointTarget[i] = state->hold[i];
        }
        out->torque[side*3+2] = BenchClip(torque + (side == 0 ? turn : -turn), .45f);
    }
    for (unsigned i = 0; i < 6; i++) if (!isfinite(out->torque[i])) {
        memset(out->torque, 0, sizeof(out->torque));
        return 0;
    }
    out->jointDamping = .40f;
    out->jointStiffness = 8.0f;
    /* H3510 无阻尼力矩测试出现轮速冲高；将快速抑制交给电机本地执行。 */
    out->wheelDamping = .03f;
    return 1;
}
