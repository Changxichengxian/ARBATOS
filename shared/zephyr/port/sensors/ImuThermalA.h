#ifndef IMU_THERMAL_A_H
#define IMU_THERMAL_A_H

#include <math.h>
#include <stdint.h>

/* 固定工作温度与持久记录温度一致；升温阶段不做零偏采样。 */
#define IMU_A_TARGET_C 48.0f

typedef struct {
    uint32_t lastMs, windowMs;
    float integral, filtered, low, high;
    uint16_t pwm;
    uint8_t initialized, slow, stable;
} ImuThermalA;

static inline void ImuThermalAUpdate(ImuThermalA *s, float temp, uint32_t now, float configMax)
{
    if (!isfinite(temp) || temp < -40.0f || temp > 60.0f || !isfinite(configMax)) {
        *s = (ImuThermalA){0};
        return;
    }
    if (!s->initialized) {
        s->initialized = 1u;
        s->filtered = s->low = s->high = temp;
        s->lastMs = s->windowMs = now;
    }
    /* 过温立即关断，不等待 100 ms 控制周期。 */
    if (temp >= IMU_A_TARGET_C + 1.0f) {
        s->pwm = 0u;
        s->integral = 0.0f;
        s->stable = s->slow = 0u;
        s->windowMs = now;
        s->lastMs = now;
        s->filtered = s->low = s->high = temp;
        return;
    }
    uint32_t elapsed = now - s->lastMs;
    if (elapsed < 100u) return;
    float dt = fminf(elapsed * 0.001f, 0.2f);
    s->lastMs = now;
    s->filtered += (temp - s->filtered) * dt / (1.0f + dt);
    float err = IMU_A_TARGET_C - s->filtered;
    /* PWM 周期 10000；加热最多给 30%，积分补稳态散热。 */
    float limit = fminf(fmaxf(configMax, 0.0f), 3000.0f);
    float candidate = fminf(fmaxf(s->integral + err * 10.0f * dt, 0.0f), limit);
    if (err <= 0.0f || 200.0f * err + candidate < limit) s->integral = candidate;
    s->pwm = (uint16_t)fminf(fmaxf(200.0f * err + s->integral, 0.0f), limit);
    s->low = fminf(s->low, s->filtered);
    s->high = fmaxf(s->high, s->filtered);
    if (now - s->windowMs >= 20000u) {
        s->slow = s->high - s->low <= 0.20f;
        s->low = s->high = s->filtered;
        s->windowMs = now;
    }
    s->stable = s->slow && fabsf(temp - IMU_A_TARGET_C) <= 0.35f &&
                s->high - s->low <= 0.20f;
}

#endif
