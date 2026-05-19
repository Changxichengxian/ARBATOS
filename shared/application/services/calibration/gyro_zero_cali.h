#ifndef GYRO_ZERO_CALI_H
#define GYRO_ZERO_CALI_H

#include "types.h"
#include "config.h"
#include "control_input.h"
#include "manual_input.h"

#include <math.h>
#include <string.h>

#define GYRO_ZERO_CALI_TEST_SAMPLES          30000U
#define GYRO_ZERO_CALI_BOOT_ADJUST_SAMPLES   3000U
#define GYRO_ZERO_CALI_TEMP_STABLE_MS        2000U
#define GYRO_ZERO_CALI_TEMP_ERR_C            0.5f
#define GYRO_ZERO_CALI_MOVING_LIMIT_DPS      5.0f
#define GYRO_ZERO_CALI_ACC_TOL_G             0.05f
#define GYRO_ZERO_CALI_DEG_TO_RAD            0.01745329251994329577f

typedef struct
{
    fp32 gyro_sum[3];
    fp32 accel_norm_sum;
    fp32 accel_norm_ref;
    uint32_t sample_count;
    uint8_t accel_norm_valid;
} gyro_zero_cali_sample_state_t;

typedef struct
{
    uint32_t stable_since_ms;
} gyro_zero_cali_temp_state_t;

static __inline void gyro_zero_cali_sample_reset(gyro_zero_cali_sample_state_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

static __inline void gyro_zero_cali_temp_reset(gyro_zero_cali_temp_state_t *state)
{
    if (state != NULL)
    {
        state->stable_since_ms = 0u;
    }
}

static __inline bool_t gyro_zero_cali_temp_stable_update(gyro_zero_cali_temp_state_t *state,
                                                       fp32 temp_c,
                                                       fp32 target_temp_c,
                                                       uint32_t now_ms,
                                                       uint8_t heater_stable)
{
    if (state == NULL)
    {
        return 0;
    }

    if (heater_stable == 0u || fabsf(temp_c - target_temp_c) > GYRO_ZERO_CALI_TEMP_ERR_C)
    {
        state->stable_since_ms = 0u;
        return 0;
    }

    if (state->stable_since_ms == 0u)
    {
        state->stable_since_ms = (now_ms == 0u) ? 1u : now_ms;
        return 0;
    }

    return ((uint32_t)(now_ms - state->stable_since_ms) >= GYRO_ZERO_CALI_TEMP_STABLE_MS) ? 1 : 0;
}

static __inline bool_t gyro_zero_cali_collect_sample(gyro_zero_cali_sample_state_t *state,
                                                   const fp32 gyro_rot[3],
                                                   const fp32 accel_rot[3],
                                                   uint32_t target_samples)
{
    if (state == NULL || gyro_rot == NULL || accel_rot == NULL || target_samples == 0u)
    {
        return 0;
    }

    const fp32 move_limit_rad = GYRO_ZERO_CALI_MOVING_LIMIT_DPS * GYRO_ZERO_CALI_DEG_TO_RAD;
    if (fabsf(gyro_rot[0]) > move_limit_rad ||
        fabsf(gyro_rot[1]) > move_limit_rad ||
        fabsf(gyro_rot[2]) > move_limit_rad)
    {
        gyro_zero_cali_sample_reset(state);
        return 0;
    }

    const fp32 accel_norm = sqrtf(accel_rot[0] * accel_rot[0] +
                                  accel_rot[1] * accel_rot[1] +
                                  accel_rot[2] * accel_rot[2]);
    if (accel_norm < 1.0e-3f)
    {
        gyro_zero_cali_sample_reset(state);
        return 0;
    }

    if (state->accel_norm_valid == 0u)
    {
        state->accel_norm_ref = accel_norm;
        state->accel_norm_valid = 1u;
    }

    const fp32 acc_tol = state->accel_norm_ref * GYRO_ZERO_CALI_ACC_TOL_G;
    if (fabsf(accel_norm - state->accel_norm_ref) > acc_tol)
    {
        gyro_zero_cali_sample_reset(state);
        return 0;
    }

    state->gyro_sum[0] += gyro_rot[0];
    state->gyro_sum[1] += gyro_rot[1];
    state->gyro_sum[2] += gyro_rot[2];
    state->accel_norm_sum += accel_norm;
    state->sample_count++;

    return (state->sample_count >= target_samples) ? 1 : 0;
}

static __inline void gyro_zero_cali_calc_offset(const gyro_zero_cali_sample_state_t *state,
                                              fp32 offset[3])
{
    if (state == NULL || offset == NULL || state->sample_count == 0u)
    {
        return;
    }

    const fp32 inv_samples = 1.0f / (fp32)state->sample_count;
    offset[0] = -state->gyro_sum[0] * inv_samples;
    offset[1] = -state->gyro_sum[1] * inv_samples;
    offset[2] = -state->gyro_sum[2] * inv_samples;
}

static __inline fp32 gyro_zero_cali_accel_norm_avg(const gyro_zero_cali_sample_state_t *state,
                                                 fp32 fallback)
{
    if (state == NULL || state->sample_count == 0u)
    {
        return fallback;
    }

    return state->accel_norm_sum / (fp32)state->sample_count;
}

static __inline uint8_t gyro_zero_cali_boot_adjust_allowed_by_input(void)
{
    const uint8_t rc_disconnected =
        (manual_input_get_active_source() == MANUAL_INPUT_SRC_AUTO) ? 1u : 0u;

    const uint8_t gimbal_safe = input_switch_is_pos(input_switch(INPUT_SW_GIMBAL_MODE),
                                                    g_config.manual_input.semantics.gimbal_safe_pos);
    const uint8_t chassis_safe = input_switch_is_pos(input_switch(INPUT_SW_CHASSIS_MODE),
                                                     g_config.manual_input.semantics.chassis_safe_pos);
    const uint8_t safe_mode = (gimbal_safe != 0u && chassis_safe != 0u) ? 1u : 0u;

    return (rc_disconnected != 0u || safe_mode != 0u) ? 1u : 0u;
}

#endif
