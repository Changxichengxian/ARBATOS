/* SPDX-License-Identifier: Apache-2.0 */
#include "ImuAttitude.h"
#include "Ahrs.h"

#include <math.h>
#include <string.h>

#define IMU_GRAVITY 9.80665f
#define IMU_PI 3.14159265358979323846f
#define IMU_RAD (IMU_PI / 180.0f)
#define IMU_KP 0.25f

static uint8_t ImuVectorFinite(const float v[3])
{
    return v != NULL && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static float ImuNorm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float ImuWrap(float angle)
{
    if (angle > IMU_PI) angle -= 2.0f * IMU_PI;
    if (angle < -IMU_PI) angle += 2.0f * IMU_PI;
    return angle;
}

static void ImuGravity(const float q[4], float gravity[3])
{
    gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    gravity[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    const float norm = ImuNorm(gravity);
    for (unsigned i = 0; i < 3; ++i) gravity[i] /= norm;
}

void ImuAttitudeInit(ImuAttitude *state)
{
    memset(state, 0, sizeof(*state));
    mahony_imu_init(&state->mahony, 0.002f);
    state->quat[0] = 1.0f;
    state->accRef = IMU_GRAVITY;
}

void ImuAttitudeSetAccRef(ImuAttitude *state, float norm)
{
    if (isfinite(norm) && norm > 0.9f * IMU_GRAVITY && norm < 1.1f * IMU_GRAVITY) {
        state->accRef = norm;
    }
}

static void ImuTrustUpdate(ImuAttitude *state, uint32_t now)
{
    ImuAccTrust *debug = &state->acc;
    memset(debug, 0, sizeof(*debug));
    const float norm = ImuNorm(state->filteredAcc);
    debug->accNormG = norm / IMU_GRAVITY;
    debug->accRefG = state->accRef / IMU_GRAVITY;
    debug->normErrG = fabsf(norm - state->accRef) / IMU_GRAVITY;
    debug->healthy = norm > 0.9f * IMU_GRAVITY && norm < 1.1f * IMU_GRAVITY;
    float gravity[3];
    ImuGravity(state->quat, gravity);
    float trust = 0.0f;
    if (debug->healthy) {
        float dot = 0.0f;
        for (unsigned i = 0; i < 3; ++i) dot += state->filteredAcc[i] * gravity[i] / norm;
        dot = fminf(fmaxf(dot, -1.0f), 1.0f);
        debug->angleDeg = acosf(dot) / IMU_RAD;
        const float angleTrust = fminf(fmaxf((18.0f - debug->angleDeg) / 12.0f, 0.0f), 1.0f);
        const float normTrust = fminf(fmaxf((0.08f - debug->normErrG) / 0.06f, 0.0f), 1.0f);
        trust = fminf(angleTrust, normTrust);
    }
    /* 保留旧 C 板干扰后180ms禁用修正；差值比较同时处理毫秒计数回绕。 */
    if (trust < 0.35f) {
        state->rejectUntil = now + 180u;
        state->rejecting = 1u;
    }
    if (state->rejecting && (int32_t)(now - state->rejectUntil) >= 0) state->rejecting = 0u;
    debug->rejected = state->rejecting;
    debug->trust = state->rejecting ? 0.0f : trust;
    for (unsigned i = 0; i < 3; ++i) {
        const float predicted = gravity[i] * state->accRef;
        state->fusedAcc[i] = predicted + debug->trust * (state->filteredAcc[i] - predicted);
    }
}

static float ImuGainUpdate(ImuAttitude *state, const float gyro[3], uint32_t now)
{
    if (state->acc.trust <= 0.05f || fabsf(gyro[0]) > 15.0f * IMU_RAD ||
        fabsf(gyro[1]) > 15.0f * IMU_RAD || fabsf(gyro[2]) > 15.0f * IMU_RAD) {
        state->gainState = 1u;
        state->gainUntil = now + 250u;
        return IMU_KP;
    }
    if (state->gainState == 1u) {
        if ((int32_t)(now - state->gainUntil) >= 0) {
            state->gainState = 2u;
            state->gainUntil = now + 500u;
        }
        return IMU_KP;
    }
    if (state->gainState == 2u) {
        if ((int32_t)(now - state->gainUntil) >= 0) state->gainState = 0u;
        return IMU_KP * 100.0f;
    }
    return IMU_KP * 10.0f;
}

static void ImuAnglesUpdate(ImuAttitude *state, float dt, uint8_t reset)
{
    float angles[3];
    get_angle(state->quat, &angles[0], &angles[2], &angles[1]);
    if (reset) {
        state->continuousYaw = angles[0];
    } else {
        /* 必须和上次原始yaw比较，不能拿已低通的输出再次累加。 */
        state->continuousYaw += ImuWrap(angles[0] - state->lastYaw);
    }
    state->lastYaw = angles[0];
    angles[0] = state->continuousYaw;
    const float alpha = reset ? 1.0f : fminf(2.0f * IMU_PI * 60.0f * dt, 1.0f);
    for (unsigned i = 0; i < 3; ++i) state->angle[i] += alpha * (angles[i] - state->angle[i]);
}

uint8_t ImuAttitudeUpdate(ImuAttitude *state, float dt, uint32_t now,
                          const float gyro[3], const float accel[3],
                          const float mag[3], uint8_t mode)
{
    if (!ImuVectorFinite(gyro) || !ImuVectorFinite(accel) || !isfinite(dt) || dt <= 0.0f || dt > 0.02f) {
        return 0u;
    }
    const uint8_t magValid = ImuVectorFinite(mag) && ImuNorm(mag) > 1.0e-6f;
    mode = mode == 1u && magValid ? 1u : 0u;
    if (!state->initialized) {
        const float norm = ImuNorm(accel);
        if (norm <= 0.9f * IMU_GRAVITY || norm >= 1.1f * IMU_GRAVITY) return 0u;
        /* 先从实测重力初始化，避免斜放开机被方向可信度永久拒绝。 */
        AHRS_init(state->quat, accel, magValid ? mag : NULL);
        memcpy(state->mahony.quat, state->quat, sizeof(state->quat));
        const float coefficients[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
        for (unsigned i = 0; i < 3; ++i) {
            second_order_filter_init(&state->accelFilter[i], coefficients, accel[i]);
            state->filteredAcc[i] = accel[i];
        }
        state->initialized = 1u;
        state->mode = mode;
        ImuAnglesUpdate(state, dt, 1u);
    } else {
        for (unsigned i = 0; i < 3; ++i) {
            state->filteredAcc[i] = second_order_filter_cali(&state->accelFilter[i], accel[i]);
        }
    }
    if (state->mode != mode) {
        /* 运行中切换/磁力计暂失不重置当前姿态，避免控制反馈突然跳回零点。 */
        mahony_imu_reset(&state->mahony);
        memcpy(state->mahony.quat, state->quat, sizeof(state->quat));
        state->mode = mode;
    }
    ImuTrustUpdate(state, now);
    if (mode == 1u) {
        (void)AHRS_update(state->quat, dt, gyro, state->fusedAcc, mag);
    } else {
        state->acc.kpGain = ImuGainUpdate(state, gyro, now) * state->acc.trust;
        mahony_imu_update(&state->mahony, dt, gyro, state->fusedAcc,
                          state->acc.trust > 0.05f, state->acc.kpGain);
        memcpy(state->quat, state->mahony.quat, sizeof(state->quat));
    }
    ImuAnglesUpdate(state, dt, 0u);
    return 1u;
}
