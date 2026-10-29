/* 板凳控制：共用几何、有限关节刚度、缓慢升降和轮端六状态反馈。 */
#include "WheelLegBench.h"
#include "WheelLegCore.h"
#include <math.h>
#include <string.h>

#include "WheelLegBenchGains.inc"

/* 2026-10-24 手动竖直、水平原始读数；顺序为右前、右后、左前、左后。
 * 水平时前关节为正角、后关节为负角，不能把前后都映射为同号。 */
static const float BenchVertical[4] = {-1.197642481f, .496490425f, .537308309f, -1.273937591f};

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
    if (state == NULL) return 0;
    state->reject = 0;
    state->rejectValue = 0;
    if (!isfinite(raw) || fabsf(raw) > 12.51f) {
        state->reject = 1; state->rejectValue = raw; return 0;
    }
    if (!isfinite(reportedRate) || fabsf(reportedRate) > 50.0f) {
        state->reject = 2; state->rejectValue = reportedRate; return 0;
    }
    if (!state->ready) {
        if (fabsf(reportedRate) > 2.0f) {
            state->reject = 3; state->rejectValue = reportedRate; return 0;
        }
        state->previous = raw;
        state->position = state->rate = 0;
        state->tick = tick;
        state->ready = 1u;
        return 1;
    }
    const uint32_t elapsed = tick - state->tick;
    if (elapsed == 0u) return 1;
    if (elapsed > 80u) {
        state->reject = 4; state->rejectValue = (float)elapsed; return 0;
    }
    /* 0044 中电机上报速度有单帧 35~36 rad/s 尖峰，轮角速度仍约 24。
     * 保留 35 的持续超速检查及独立轮角检查，严重异常立即拒绝。 */
    if (fabsf(reportedRate) > 35.0f) {
        if (!state->overspeed) state->overspeedTick = tick;
        state->overspeed = 1u;
        if ((uint32_t)(tick-state->overspeedTick) >= 12u) {
            state->reject = 2; state->rejectValue = reportedRate; return 0;
        }
    } else {
        state->overspeed = 0u;
    }
    const float dt = (float)elapsed * .001f;
    /* H3510 的反馈范围为 -12.5..12.5 rad，跨界周期是 25 rad，不是 2*pi。 */
    const float delta = remainderf(raw - state->previous, 25.0f);
    /* 接收任务的毫秒时间不是电机采样时间。0042 中 1 ms 的接收间隔
     * 对应约 3 ms 的角度变化；短间隔先合并到下一帧，不把除法尖峰当超速。
     * 不推进 previous/tick，累计位移仍受原 35 rad/s 及 80 ms 保护检查。 */
    if (elapsed < 3u) {
        if (fabsf(delta) > 35.0f * (dt + .002f) + .001f) {
            state->reject = 5; state->rejectValue = delta/dt; return 0;
        }
        return 1;
    }
    if (fabsf(delta) > 35.0f * dt + .001f) {
        state->reject = 5; state->rejectValue = delta/dt; return 0;
    }
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

int WheelLegBenchSetHeight(WheelLegBenchState *state, float length, float dt)
{
    if (state == NULL || !state->ready || !isfinite(length) ||
        length < WHEELLEG_BENCH_MIN_HEIGHT || length > WHEELLEG_BENCH_MAX_HEIGHT ||
        !isfinite(dt) || dt < .001f || dt > .006f) {
        return 0;
    }
    state->height += BenchClip(length - state->height, .010f * dt);
    return 1;
}

float WheelLegBenchGetHeight(const WheelLegBenchState *state)
{
    return state != NULL && state->ready ? state->height : NAN;
}

float WheelLegBenchSpeedLimit(float height)
{
    /* 0044 实车高腿反打曾先打满力矩再倾倒，改用更保守的实测修订范围。 */
    return .70f - 5.0f*fminf(fmaxf(height-.080f, 0.0f), .040f);
}

static float BenchPitchRate(WheelLegBenchState *state, float value, float dt)
{
    /* 0044 静止片段的轮角、力矩和俯仰角速度共有约 36 Hz 振动。
     * 仅抑制该频段；原始姿态/角速度继续记录，倾倒保护仍用原始姿态。 */
    if (!state->ready) {
        state->pitchRateX1 = state->pitchRateX2 = value;
        state->pitchRateY1 = state->pitchRateY2 = value;
    }
    const float omega = 2.0f*WHEELLEG_CORE_PI*36.0f*dt;
    const float alpha = sinf(omega)*.25f;
    const float b0 = 1.0f/(1.0f+alpha);
    const float b1 = -2.0f*cosf(omega)*b0;
    const float a2 = (1.0f-alpha)*b0;
    const float filtered = b0*value+b1*state->pitchRateX1+b0*state->pitchRateX2-
                           b1*state->pitchRateY1-a2*state->pitchRateY2;
    state->pitchRateX2 = state->pitchRateX1;
    state->pitchRateX1 = value;
    state->pitchRateY2 = state->pitchRateY1;
    state->pitchRateY1 = filtered;
    /* 完全消除该频段会削弱机械阻尼，保留 75% 即时反馈。 */
    return .75f*value+.25f*filtered;
}

static int BenchLevelLegs(WheelLegBenchState *state, const WheelLegBenchInput *in,
                          const float leg[2][5])
{
    const float dt = in->dt;
    state->rollTarget += BenchClip(BenchClip(in->targetRoll, WHEELLEG_BENCH_MAX_ROLL) -
                                   state->rollTarget, .13962634f * dt);
    /* 右腿在前。用实际腿长和横滚估算左轮支撑面相对右轮的高差，
     * 不读取仿真地形；支撑高差与主动侧倾共用有限伸缩行程。 */
    const float delta = (leg[0][0]*cosf(leg[0][1]) - leg[1][0]*cosf(leg[1][1])) *
                        cosf(in->roll) + .180f*sinf(in->roll);
    state->supportDelta += dt/(.10f+dt) * (BenchClip(delta, .035f) - state->supportDelta);
    const float bias = BenchClip(.5f*(state->supportDelta - .180f*sinf(state->rollTarget)) /
        cosf(state->rollTarget) + .025f*(in->roll-state->rollTarget) + .006f*in->gyro[0], .020f);
    state->rollBias += BenchClip(bias-state->rollBias, .015f*dt);
    const float mean = fminf(fmaxf(state->height, WHEELLEG_BENCH_MIN_HEIGHT+fabsf(state->rollBias)),
                            WHEELLEG_BENCH_MAX_HEIGHT-fabsf(state->rollBias));
    const WheelLegCoreGeometry geometry = {.l1_m=.05f, .l2_m=.105f,
        .l3_m=.105f, .l4_m=.05f, .l5_m=.07f};
    float next[4];
    for (unsigned side = 0; side < 2; side++) {
        float held[5];
        if (!BenchLeg(&state->hold[side*2], held)) return 0;
        const float wanted = mean + (side == 0 ? state->rollBias : -state->rollBias);
        const WheelLegCoreFootPoint target = {.x_m=0,
            .y_m=held[0]+BenchClip(wanted-held[0], .020f*dt)};
        if (!WheelLegCoreInversePoint(&geometry, &target, 1.0f, -1.0f,
                                      &next[side*2], &next[side*2+1])) return 0;
        /* 两侧分别变化，保持原折叠分支和关节速度上限。 */
        for (unsigned j = side*2; j < side*2+2; j++) {
            next[j] = state->hold[j] + BenchClip(next[j]-state->hold[j], .60f*dt);
        }
    }
    memcpy(state->hold, next, sizeof(next));
    return 1;
}

int WheelLegBenchHeightAxis(WheelLegBenchState *state, float axis, float dt)
{
    if (!isfinite(axis) || !isfinite(dt) || dt < .001f || dt > .006f) return 0;
    float height = WheelLegBenchGetHeight(state);
    if (!isfinite(height)) return 0;
    /* 回中保留已有角度，避免数值往返使高度缓慢漂移。 */
    if (fabsf(axis) < .001f) return 1;
    height = fminf(fmaxf(height + BenchClip(axis, 1.0f) * .010f * dt,
                        WHEELLEG_BENCH_MIN_HEIGHT), WHEELLEG_BENCH_MAX_HEIGHT);
    return WheelLegBenchSetHeight(state, height, dt);
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
        const WheelLegCoreFootPoint target = {.x_m=0, .y_m=WHEELLEG_BENCH_START_HEIGHT};
        float front, back;
        /* 固定选用仿真已经验证的膝向外分支，不追随任意折叠分支。 */
        if (!WheelLegCoreInversePoint(&geometry, &target, 1.0f, -1.0f, &front, &back)) return BenchAutoReject(state, 3, 0);
        float travel = 0;
        for (unsigned i = 0; i < 4; i++) {
            state->start[i] = in->joint[i];
            state->target[i] = i % 2u == 0u ? front : back;
            const float rawTarget = WheelLegBenchJointToRaw(i, state->target[i]);
            /* 手工“近水平”是标定姿态，不是机械限位；60 mm 沿同一外折分支继续收短。 */
            if (!isfinite(rawTarget) || fabsf(rawTarget) > 12.5f ||
                fabsf(state->target[i]) > 2.4f) {
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
    uint8_t poseReady = 1u;
    for (unsigned side = 0; side < 2; side++) {
        float leg[5];
        if (!BenchLeg(&in->joint[side*2], leg)) return BenchAutoReject(state, 7, (float)side);
        out->length[side] = leg[0];
        out->alpha[side] = leg[1];
        float wanted[5];
        if (!BenchLeg(&state->target[side*2], wanted)) return BenchAutoReject(state, 7, (float)side);
        if (fabsf(leg[0] - wanted[0]) > .006f ||
            fabsf(leg[1]) > .08f) poseReady = 0u;
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
        /* 60 mm 承重时位置环靠角差提供支撑力；0042 的稳定角差约 .084 rad。
         * 同时确认实际腿长、居中角和静止状态，避免只差一点角度就等到超时。 */
        state->settled = poseReady && maxError < .12f && maxRate < .20f ? state->settled + in->dt : 0;
        if (state->settled >= .4f) {
            state->phase = 4u;
            if (allowWheels) {
                WheelLegBenchReset(&state->balance);
                if (!WheelLegBenchStep(&state->balance, in, out)) return BenchAutoReject(state, 8, 0);
                for (unsigned i = 0; i < 4; i++) {
                    state->balance.hold[i] = state->target[i];
                    out->jointTarget[i] = state->target[i];
                }
                state->balance.height = WHEELLEG_BENCH_START_HEIGHT;
                state->phase = 3u;
            }
        }
    }
    return 1;
}

static int BenchReject(WheelLegBenchState *state, uint32_t reason, float value)
{
    state->reject = reason;
    state->rejectValue = value;
    return 0;
}

int WheelLegBenchStep(WheelLegBenchState *state, const WheelLegBenchInput *in,
                     WheelLegBenchOutput *out)
{
    if (state == NULL || in == NULL || out == NULL) return 0;
    memset(out, 0, sizeof(*out));
    state->reject = 0;
    state->rejectValue = 0;
    /* 拒绝坏反馈，不能让 NaN 经限幅变成非零力矩。 */
    for (unsigned i = 0; i < 4; i++) {
        if (!isfinite(in->joint[i]) || !isfinite(in->jointRate[i])) return BenchReject(state, 10u+i, in->joint[i]);
        if (fabsf(in->jointRate[i]) > 3.0f) return BenchReject(state, 20u+i, in->jointRate[i]);
    }
    for (unsigned i = 0; i < 2; i++) {
        if (!isfinite(in->wheel[i]) || !isfinite(in->wheelRate[i])) return BenchReject(state, 30u+i, in->wheelRate[i]);
        /* 独立运行也保留原电脑端轮速保护；静止接入，避免异常反馈直接打满力矩。 */
        if (fabsf(in->wheelRate[i]) > 35.0f ||
            (!state->ready && fabsf(in->wheelRate[i]) > 2.0f)) return BenchReject(state, 32u+i, in->wheelRate[i]);
    }
    for (unsigned i = 0; i < 3; i++) if (!isfinite(in->gyro[i])) return BenchReject(state, 40u+i, in->gyro[i]);
    const float finiteInput[8] = {in->pitch, in->roll, in->yaw, in->targetV,
        in->targetYaw, in->targetRoll, in->pitchTrim, in->dt};
    for (unsigned i = 0; i < 8; i++) {
        if (!isfinite(finiteInput[i])) return BenchReject(state, 80u+i, finiteInput[i]);
    }
    if (fabsf(in->pitchTrim) > WHEELLEG_BENCH_TRIM_LIMIT) return BenchReject(state, 88, in->pitchTrim);
    if (in->dt < .001f || in->dt > .006f) return BenchReject(state, 89, in->dt);
    if (fabsf(in->pitch) > .35f) return BenchReject(state, 90, in->pitch);
    if (fabsf(in->roll) > .35f) return BenchReject(state, 91, in->roll);

    const float pitchRate = BenchPitchRate(state, in->gyro[1], in->dt);
    float leg[2][5], alphaRate = 0, wheelPosition = 0, velocity = 0;
    for (unsigned side = 0; side < 2; side++) {
        unsigned j = side * 2;
        float future[5], next[2] = {in->joint[j] + in->jointRate[j] * .0001f,
                                   in->joint[j+1] + in->jointRate[j+1] * .0001f};
        if (!BenchLeg(&in->joint[j], leg[side]) || !BenchLeg(next, future)) return BenchReject(state, 50u+side, in->joint[j]);
        out->length[side] = leg[side][0];
        out->alpha[side] = leg[side][1];
        /* 当前增益只覆盖该站姿区间，不对接近伸直的腿外推。 */
        if (out->length[side] < .053f || out->length[side] > .132f) return BenchReject(state, 52u+side, out->length[side]);
        if (fabsf(out->alpha[side]) > .25f) return BenchReject(state, 56u+side, out->alpha[side]);
        alphaRate += .5f * BenchWrap(future[1] - leg[side][1]) / .0001f;
        float rodRate = BenchWrap(future[2] - leg[side][2]) / .0001f;
        wheelPosition += .5f * .03275f * (in->wheel[side] + leg[side][2] + in->pitch);
        velocity += .5f * .03275f * (in->wheelRate[side] + rodRate + pitchRate);
    }
    if (fabsf(out->length[0] - out->length[1]) > .045f) return BenchReject(state, 54, out->length[0]-out->length[1]);

    if (!state->ready) {
        for (unsigned i = 0; i < 4; i++) {
            state->hold[i] = in->joint[i];
        }
        state->reference = wheelPosition;
        state->yawReference = in->yaw;
        state->height = fminf(fmaxf(.5f*(leg[0][0]+leg[1][0]), WHEELLEG_BENCH_MIN_HEIGHT),
                              WHEELLEG_BENCH_MAX_HEIGHT);
        state->ready = 1;
    }
    for (unsigned i = 0; i < 4; i++) {
        if (fabsf(state->hold[i] - in->joint[i]) > .25f) return BenchReject(state, 60u+i, state->hold[i]-in->joint[i]);
    }
    float heldRight[5], heldLeft[5];
    if (!BenchLeg(state->hold, heldRight) || !BenchLeg(state->hold+2, heldLeft) ||
        fabsf((leg[0][0]-leg[1][0])-(heldRight[0]-heldLeft[0])) > .015f) return BenchReject(state, 64, leg[0][0]-leg[1][0]);
    if (!BenchLevelLegs(state, in, leg)) return BenchReject(state, 65, state->rollBias);
    const float limitHeight = fmaxf(state->height, fmaxf(heldRight[0], heldLeft[0]));
    /* 大侧倾缩小支撑余量，先根据摇杆意图降速，回正后再恢复；不能等已经倾斜才减速。 */
    const float rollUse = fmaxf(fabsf(in->roll), fmaxf(fabsf(state->rollTarget),
                                fabsf(BenchClip(in->targetRoll, WHEELLEG_BENCH_MAX_ROLL))));
    const float lean = fminf(fmaxf((rollUse-.06981317f)/.10471976f, 0.0f), 1.0f);
    const float normalSpeed = WheelLegBenchSpeedLimit(limitHeight);
    const float speedLimit = normalSpeed + lean*(.30f-normalSpeed);
    const float yawLimit = WHEELLEG_BENCH_MAX_YAW + lean*(1.20f-WHEELLEG_BENCH_MAX_YAW);
    float targetV = BenchClip(in->targetV, speedLimit);
    float targetYaw = BenchClip(in->targetYaw, yawLimit);
    /* 直行和转向共享外侧轮目标预算，给实车轮速振动和瞬态留出余量。 */
    const float demand = fabsf(targetV) + .090f*fabsf(targetYaw);
    if (demand > .70f) {
        targetV *= .70f/demand;
        targetYaw *= .70f/demand;
    }
    /* 反打时按腿高限制加速度及其变化率，给机身留出收回俯仰的时间。 */
    float speedError = targetV - state->speed;
    const float high = fminf(fmaxf((limitHeight-.080f)/.040f, 0.0f), 1.0f);
    float wantedAcceleration = BenchClip(speedError / .25f, .75f-.25f*high);
    const float trackingError = state->speed-velocity;
    if (wantedAcceleration*trackingError > 0.0f) {
        /* 0044 高腿反打时速度目标先跑开约 0.47 m/s，随后力矩饱和。
         * 只减缓目标继续跑开的趋势，不把测速振动直接写回速度目标。 */
        wantedAcceleration *= fminf(fmaxf((.25f-.05f*high-fabsf(trackingError))/.08f, 0.0f), 1.0f);
    }
    state->acceleration += BenchClip(wantedAcceleration - state->acceleration, 6.0f * in->dt);
    float speedStep = state->acceleration * in->dt;
    if (speedStep * speedError > 0 && fabsf(speedStep) >= fabsf(speedError)) {
        state->speed = targetV;
        state->acceleration = 0;
    } else {
        state->speed += speedStep;
    }
    if (fabsf(targetV) > .001f) state->driving = 1;
    if (state->driving) {
        state->reference = wheelPosition;
        if (fabsf(targetV) < .001f && fabsf(state->speed) < .001f) state->driving = 0;
    }
    /* 回中后锁住轮程，用有限的低速补偿抵消坡面持续下滑。
     * 不能一直等实际速度接近零才锁位置，否则坡上会始终处于滑行状态。 */
    const float parkError = BenchClip(wheelPosition-state->reference, .10f);
    if (state->driving || fabsf(targetYaw) > .001f) {
        state->parkIntegral = state->parkSpeed = 0;
    } else {
        state->parkIntegral = BenchClip(state->parkIntegral + parkError*in->dt, .60f);
        const float wantedPark = BenchClip(-1.0f*parkError - .20f*state->parkIntegral - .25f*velocity, .15f);
        state->parkSpeed += BenchClip(wantedPark-state->parkSpeed, .30f*in->dt);
    }
    /* 固定腿姿的平衡目标整体慢移；轮程和倾倒保护仍使用真实俯仰。 */
    out->state[0] = .5f * (out->alpha[0] + out->alpha[1]) - in->pitch + in->pitchTrim;
    out->state[1] = alphaRate - pitchRate;
    out->state[2] = BenchClip(wheelPosition - state->reference, .10f);
    out->state[3] = velocity - state->speed - state->parkSpeed;
    out->state[4] = in->pitch - in->pitchTrim;
    out->state[5] = pitchRate;
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
    state->yawRate = BenchClip(state->yawRate, fmaxf(0.0f, (.70f-fabsf(state->speed))/.090f));
    float yawError = BenchClip(BenchWrap(state->yawReference + state->yawRate * in->dt - in->yaw), .15f);
    state->yawReference = BenchWrap(in->yaw + yawError);
    float turn = BenchClip(.5f * yawError + .12f * (state->yawRate - in->gyro[2]), .10f);
    for (unsigned side = 0; side < 2; side++) {
        for (unsigned end = 0; end < 2; end++) {
            unsigned i = side*2 + end;
            out->jointTarget[i] = state->hold[i];
        }
        out->torque[side*3+2] = BenchClip(torque + (side == 0 ? turn : -turn), .45f);
        /* 原来始终制动到零速，会让平衡环持续抵消行驶阻力并占用力矩余量。
         * 这里只移动本地阻尼的参考速度；俯仰平衡仍由六状态反馈负责。 */
        out->wheelTargetRate[side] = (state->speed + (side == 0 ? 1.0f : -1.0f) *
            .090f * state->yawRate) / .03275f;
    }
    for (unsigned i = 0; i < 6; i++) if (!isfinite(out->torque[i])) {
        memset(out->torque, 0, sizeof(out->torque));
        return BenchReject(state, 70u+i, (float)i);
    }
    out->jointDamping = .40f;
    out->jointStiffness = 8.0f;
    /* H3510 无阻尼力矩测试出现轮速冲高；将快速抑制交给电机本地执行。 */
    out->wheelDamping = .03f;
    return 1;
}
