#ifndef GYRO_ZERO_CALI_H
#define GYRO_ZERO_CALI_H

#include "Types.h"

#include <math.h>
#include <string.h>

#define GYRO_ZERO_CALI_TEST_SAMPLES          30000U
#define GYRO_ZERO_CALI_BOOT_ADJUST_SAMPLES   3000U
#define GYRO_ZERO_CALI_TEMP_STABLE_MS        2000U
#define GYRO_ZERO_CALI_TEMP_ERR_C            3.0f
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
} GyroZeroCaliSampleState;

typedef struct
{
    uint32_t stable_since_ms;
} GyroZeroCaliTempState;

typedef void (*GyroZeroCaliRotateFn)(fp32 rotated[3], const fp32 raw[3]);
typedef void (*GyroZeroCaliApplyOffsetFn)(const fp32 offset[3], void *ctx);
typedef bool_t (*GyroZeroCaliSaveOffsetFn)(const fp32 offset[3], void *ctx);
typedef void (*GyroZeroCaliSampleDoneFn)(const GyroZeroCaliSampleState *state, void *ctx);

typedef struct
{
    GyroZeroCaliSampleState boot_adjust_sample;
    GyroZeroCaliTempState boot_adjust_temp;
    GyroZeroCaliSampleState test_sample;
    GyroZeroCaliTempState test_temp;
    uint8_t boot_adjust_finished;
    uint8_t test_finished;
    uint8_t calibrating;
    uint8_t calibrated;
    uint8_t result;
} GyroZeroCaliRuntimeState;

typedef struct
{
    uint8_t test_mode_active;
    fp32 temp_c;
    fp32 target_temp_c;
    uint8_t heater_stable;
    uint8_t boot_adjust_allowed;
    uint32_t now_ms;
    GyroZeroCaliRotateFn rotate_gyro;
    GyroZeroCaliRotateFn rotate_accel;
    GyroZeroCaliApplyOffsetFn apply_offset;
    GyroZeroCaliSaveOffsetFn save_offset;
    GyroZeroCaliSampleDoneFn sample_done;
    void *ctx;
} GyroZeroCaliRuntimeCfg;

#define GYRO_ZERO_CALI_RESULT_PENDING 0u
#define GYRO_ZERO_CALI_RESULT_SUCCESS 1u
#define GYRO_ZERO_CALI_RESULT_FAILED  2u

static __inline void GyroZeroCaliSampleReset(GyroZeroCaliSampleState *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

static __inline void GyroZeroCaliTempReset(GyroZeroCaliTempState *state)
{
    if (state != NULL)
    {
        state->stable_since_ms = 0u;
    }
}

static __inline bool_t GyroZeroCaliTempStableUpdate(GyroZeroCaliTempState *state,
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

static __inline bool_t GyroZeroCaliCollectSample(GyroZeroCaliSampleState *state,
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
        GyroZeroCaliSampleReset(state);
        return 0;
    }

    const fp32 accel_norm = sqrtf(accel_rot[0] * accel_rot[0] +
                                  accel_rot[1] * accel_rot[1] +
                                  accel_rot[2] * accel_rot[2]);
    if (accel_norm < 1.0e-3f)
    {
        GyroZeroCaliSampleReset(state);
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
        GyroZeroCaliSampleReset(state);
        return 0;
    }

    state->gyro_sum[0] += gyro_rot[0];
    state->gyro_sum[1] += gyro_rot[1];
    state->gyro_sum[2] += gyro_rot[2];
    state->accel_norm_sum += accel_norm;
    state->sample_count++;

    return (state->sample_count >= target_samples) ? 1 : 0;
}

static __inline void GyroZeroCaliCalcOffset(const GyroZeroCaliSampleState *state,
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

static __inline fp32 GyroZeroCaliAccelNormAvg(const GyroZeroCaliSampleState *state,
                                                 fp32 fallback)
{
    if (state == NULL || state->sample_count == 0u)
    {
        return fallback;
    }

    return state->accel_norm_sum / (fp32)state->sample_count;
}

static __inline void GyroZeroCaliRuntimeReset(GyroZeroCaliRuntimeState *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
        state->result = GYRO_ZERO_CALI_RESULT_PENDING;
    }
}

static __inline bool_t GyroZeroCaliRuntimeIsCalibrated(const GyroZeroCaliRuntimeState *state)
{
    return (state != NULL && state->calibrated != 0u) ? 1 : 0;
}

static __inline bool_t GyroZeroCaliRuntimeIsCalibrating(const GyroZeroCaliRuntimeState *state)
{
    return (state != NULL && state->calibrating != 0u) ? 1 : 0;
}

static __inline uint8_t GyroZeroCaliRuntimeResult(const GyroZeroCaliRuntimeState *state)
{
    return (state != NULL) ? state->result : GYRO_ZERO_CALI_RESULT_PENDING;
}

static __inline bool_t GyroZeroCaliRuntimeReady(const GyroZeroCaliRuntimeState *state,
                                                   const GyroZeroCaliRuntimeCfg *cfg,
                                                   const fp32 gyro_raw[3],
                                                   const fp32 accel_raw[3])
{
    return (state != NULL &&
            cfg != NULL &&
            gyro_raw != NULL &&
            accel_raw != NULL &&
            cfg->rotate_gyro != NULL &&
            cfg->rotate_accel != NULL &&
            cfg->apply_offset != NULL) ? 1 : 0;
}

static __inline void GyroZeroCaliRuntimeFinishTest(GyroZeroCaliRuntimeState *state,
                                                       const GyroZeroCaliRuntimeCfg *cfg)
{
    fp32 offset[3] = {0.0f, 0.0f, 0.0f};
    GyroZeroCaliCalcOffset(&state->test_sample, offset);
    cfg->apply_offset(offset, cfg->ctx);
    if (cfg->sample_done != NULL)
    {
        cfg->sample_done(&state->test_sample, cfg->ctx);
    }

    const bool_t saved = (cfg->save_offset != NULL) ? cfg->save_offset(offset, cfg->ctx) : 0;
    state->calibrated = (saved != 0) ? 1u : 0u;
    state->result = (saved != 0) ? GYRO_ZERO_CALI_RESULT_SUCCESS : GYRO_ZERO_CALI_RESULT_FAILED;
    state->test_finished = 1u;
    state->calibrating = 0u;
    GyroZeroCaliSampleReset(&state->test_sample);
    GyroZeroCaliSampleReset(&state->boot_adjust_sample);
    GyroZeroCaliTempReset(&state->boot_adjust_temp);
}

static __inline void GyroZeroCaliRuntimeFinishBootAdjust(GyroZeroCaliRuntimeState *state,
                                                              const GyroZeroCaliRuntimeCfg *cfg)
{
    fp32 offset[3] = {0.0f, 0.0f, 0.0f};
    GyroZeroCaliCalcOffset(&state->boot_adjust_sample, offset);
    cfg->apply_offset(offset, cfg->ctx);
    if (cfg->sample_done != NULL)
    {
        cfg->sample_done(&state->boot_adjust_sample, cfg->ctx);
    }

    state->calibrated = 1u;
    state->result = GYRO_ZERO_CALI_RESULT_SUCCESS;
    state->boot_adjust_finished = 1u;
    state->calibrating = 0u;
    GyroZeroCaliSampleReset(&state->boot_adjust_sample);
}

static __inline void GyroZeroCaliRuntimeUpdate(GyroZeroCaliRuntimeState *state,
                                                  const GyroZeroCaliRuntimeCfg *cfg,
                                                  const fp32 gyro_raw[3],
                                                  const fp32 accel_raw[3])
{
    if (!GyroZeroCaliRuntimeReady(state, cfg, gyro_raw, accel_raw))
    {
        return;
    }

    if (cfg->test_mode_active != 0u)
    {
        GyroZeroCaliSampleReset(&state->boot_adjust_sample);
        GyroZeroCaliTempReset(&state->boot_adjust_temp);
        if (state->test_finished != 0u)
        {
            state->calibrating = 0u;
            return;
        }

        state->calibrating = 1u;
        if (!GyroZeroCaliTempStableUpdate(&state->test_temp,
                                               cfg->temp_c,
                                               cfg->target_temp_c,
                                               cfg->now_ms,
                                               cfg->heater_stable))
        {
            GyroZeroCaliSampleReset(&state->test_sample);
            return;
        }

        fp32 gyro_rot[3];
        fp32 accel_rot[3];
        cfg->rotate_gyro(gyro_rot, gyro_raw);
        cfg->rotate_accel(accel_rot, accel_raw);
        if (GyroZeroCaliCollectSample(&state->test_sample, gyro_rot, accel_rot, GYRO_ZERO_CALI_TEST_SAMPLES))
        {
            GyroZeroCaliRuntimeFinishTest(state, cfg);
        }
        return;
    }

    state->test_finished = 0u;
    GyroZeroCaliSampleReset(&state->test_sample);
    GyroZeroCaliTempReset(&state->test_temp);

    if (state->boot_adjust_finished != 0u)
    {
        state->calibrating = 0u;
        return;
    }

    if (!GyroZeroCaliTempStableUpdate(&state->boot_adjust_temp,
                                           cfg->temp_c,
                                           cfg->target_temp_c,
                                           cfg->now_ms,
                                           cfg->heater_stable) ||
        cfg->boot_adjust_allowed == 0u)
    {
        state->calibrating = 0u;
        GyroZeroCaliSampleReset(&state->boot_adjust_sample);
        return;
    }

    fp32 gyro_rot[3];
    fp32 accel_rot[3];
    cfg->rotate_gyro(gyro_rot, gyro_raw);
    cfg->rotate_accel(accel_rot, accel_raw);

    state->calibrating = 1u;
    if (GyroZeroCaliCollectSample(&state->boot_adjust_sample,
                                      gyro_rot,
                                      accel_rot,
                                      GYRO_ZERO_CALI_BOOT_ADJUST_SAMPLES))
    {
        GyroZeroCaliRuntimeFinishBootAdjust(state, cfg);
    }
}

#endif
