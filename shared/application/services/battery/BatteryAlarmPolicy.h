#ifndef BATTERY_ALARM_POLICY_H
#define BATTERY_ALARM_POLICY_H

#include <stdint.h>

typedef struct
{
    uint32_t lowMs;
    uint8_t pending;
    uint8_t active;
} BatteryAlarmState;

static inline uint8_t BatteryAlarmUpdate(BatteryAlarmState *state, uint8_t enabled,
    uint8_t valid, float voltage, float threshold, uint32_t delayMs, uint32_t periodMs)
{
    if (!enabled) {
        state->lowMs = 0u;
        state->pending = 0u;
        state->active = 0u;
    } else if (!valid || voltage <= threshold) {
        /* 首个低压样本只开始计时，后续连续样本才累计确认时间。 */
        if (!state->pending) {
            state->pending = 1u;
            state->lowMs = 0u;
        } else if (state->lowMs < delayMs) {
            uint32_t remaining = delayMs - state->lowMs;
            state->lowMs += periodMs < remaining ? periodMs : remaining;
        }
        if (state->lowMs >= delayMs) {
            state->active = 1u;
        }
    } else {
        state->lowMs = 0u;
        state->pending = 0u;
        if (voltage > threshold + 0.5f) {
            state->active = 0u;
        }
    }
    return state->active;
}

#endif
