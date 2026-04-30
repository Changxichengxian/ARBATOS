/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include <float.h>
#include <math.h>
#include "detect_task.h"
#include "config.h"
#include "sdlog.h"
#include "chassis_power_limiter.h"
#include "robot_task_profile.h"

#define CHASSIS_POWER_BUFFER_FILTER_TAU_S      0.08f
#define CHASSIS_POWER_LIMIT_FILTER_TAU_S       0.12f
#define CHASSIS_POWER_MOTION_FILTER_TAU_S      0.10f
#define CHASSIS_POWER_BUFFER_RELEASE_HYST_J    2.0f
#define CHASSIS_POWER_MOTION_BASE_SCALE        0.15f
#define CHASSIS_POWER_CONTROL_MIN_PERIOD_S     0.001f
#define CHASSIS_POWER_ALLOC_EPSILON            1.0e-4f
#define CHASSIS_POWER_ALLOC_MAX_ROUNDS         4u
#define CHASSIS_POWER_ALLOC_RESERVED_TRIGGER_RATIO 0.675f
#define CHASSIS_POWER_ALLOC_RESERVED_PER_MOTOR_RATIO 0.10f

static const power_config_t *const power_cfg = &g_config.power;

typedef struct
{
    uint8_t inited;
    uint8_t buffer_limit_active;
    uint8_t model_fallback_active;
    fp32 filtered_buffer;
    fp32 filtered_power_limit;
    fp32 motion_limit_scale;
} chassis_power_runtime_state_t;

static chassis_power_runtime_state_t s_chassis_power_runtime = {0};

static fp32 chassis_power_clamp(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static fp32 chassis_power_lowpass(fp32 prev, fp32 input, fp32 dt, fp32 tau)
{
    fp32 alpha = 1.0f;

    if (tau <= 0.0f || dt <= 0.0f)
    {
        return input;
    }

    alpha = dt / (tau + dt);
    alpha = chassis_power_clamp(alpha, 0.0f, 1.0f);
    return prev + alpha * (input - prev);
}

static fp32 chassis_power_get_control_period_s(void)
{
    fp32 dt = (fp32)robot_profile_chassis_control_period_s();
    if (dt < CHASSIS_POWER_CONTROL_MIN_PERIOD_S)
    {
        dt = CHASSIS_POWER_CONTROL_MIN_PERIOD_S;
    }
    return dt;
}

static fp32 chassis_power_budget_to_motion_scale(fp32 power_budget, fp32 power_limit)
{
    fp32 ratio = 1.0f;

    if (power_limit > 0.0f)
    {
        ratio = chassis_power_clamp(power_budget / power_limit, 0.0f, 1.0f);
    }

    return CHASSIS_POWER_MOTION_BASE_SCALE + (1.0f - CHASSIS_POWER_MOTION_BASE_SCALE) * ratio;
}

static void chassis_power_update_motion_scale(fp32 target_scale)
{
    const fp32 dt = chassis_power_get_control_period_s();

    target_scale = chassis_power_clamp(target_scale, 0.0f, 1.0f);
    if (s_chassis_power_runtime.inited == 0u)
    {
        s_chassis_power_runtime.motion_limit_scale = target_scale;
        return;
    }

    s_chassis_power_runtime.motion_limit_scale =
        chassis_power_lowpass(s_chassis_power_runtime.motion_limit_scale,
                              target_scale,
                              dt,
                              CHASSIS_POWER_MOTION_FILTER_TAU_S);
}

static void chassis_power_update_runtime_state(fp32 raw_power_limit,
                                               fp32 raw_buffer,
                                               fp32 *out_power_limit,
                                               fp32 *out_buffer)
{
    const fp32 dt = chassis_power_get_control_period_s();
    const fp32 effective_power_limit = (raw_power_limit > 0.0f) ? raw_power_limit : power_cfg->power_limit;
    const fp32 effective_buffer = (raw_buffer > 0.0f) ? raw_buffer : 0.0f;

    if (s_chassis_power_runtime.inited == 0u)
    {
        s_chassis_power_runtime.inited = 1u;
        s_chassis_power_runtime.filtered_power_limit = effective_power_limit;
        s_chassis_power_runtime.filtered_buffer = effective_buffer;
        s_chassis_power_runtime.motion_limit_scale = 1.0f;
    }
    else
    {
        s_chassis_power_runtime.filtered_power_limit =
            chassis_power_lowpass(s_chassis_power_runtime.filtered_power_limit,
                                  effective_power_limit,
                                  dt,
                                  CHASSIS_POWER_LIMIT_FILTER_TAU_S);
        s_chassis_power_runtime.filtered_buffer =
            chassis_power_lowpass(s_chassis_power_runtime.filtered_buffer,
                                  effective_buffer,
                                  dt,
                                  CHASSIS_POWER_BUFFER_FILTER_TAU_S);
    }

    if (s_chassis_power_runtime.filtered_buffer < power_cfg->warning_power_buffer)
    {
        s_chassis_power_runtime.buffer_limit_active = 1u;
    }
    else if (s_chassis_power_runtime.filtered_buffer >
             (power_cfg->warning_power_buffer + CHASSIS_POWER_BUFFER_RELEASE_HYST_J))
    {
        s_chassis_power_runtime.buffer_limit_active = 0u;
    }

    if (out_power_limit != NULL)
    {
        *out_power_limit = s_chassis_power_runtime.filtered_power_limit;
    }
    if (out_buffer != NULL)
    {
        *out_buffer = s_chassis_power_runtime.filtered_buffer;
    }
}

static chassis_power_limiter_config_t chassis_power_build_runtime_cfg(fp32 runtime_power_limit)
{
    chassis_power_limiter_config_t cfg = {0};
    fp32 effective_power_limit = runtime_power_limit;
    fp32 scale = 1.0f;

    if (effective_power_limit <= 0.0f)
    {
        effective_power_limit = power_cfg->power_limit;
    }

    if (power_cfg->power_limit > 0.0f && effective_power_limit > 0.0f)
    {
        scale = effective_power_limit / power_cfg->power_limit;
    }
    if (scale < 0.0f)
    {
        scale = 0.0f;
    }

    cfg.power_limit = effective_power_limit;
    cfg.warning_power = power_cfg->warning_power * scale;
    if (cfg.warning_power > cfg.power_limit)
    {
        cfg.warning_power = cfg.power_limit;
    }
    cfg.warning_power_buffer = power_cfg->warning_power_buffer;
    cfg.no_judge_total_current_limit = power_cfg->no_judge_total_current_limit * scale;
    cfg.buffer_total_current_limit = power_cfg->buffer_total_current_limit * scale;
    cfg.power_total_current_limit = power_cfg->power_total_current_limit * scale;

    return cfg;
}

static void chassis_power_allocate_remaining_budget(const fp32 demand[4],
                                                    const uint8_t active[4],
                                                    const fp32 wheel_want_power[4],
                                                    fp32 wheel_limit_power[4],
                                                    fp32 *budget_remaining)
{
    if (budget_remaining == NULL)
    {
        return;
    }

    for (uint8_t round = 0u; round < CHASSIS_POWER_ALLOC_MAX_ROUNDS; round++)
    {
        fp32 demand_sum = 0.0f;
        fp32 granted_total = 0.0f;
        uint8_t unmet_count = 0u;

        if (*budget_remaining <= CHASSIS_POWER_ALLOC_EPSILON)
        {
            break;
        }

        for (uint8_t i = 0u; i < 4u; i++)
        {
            if (active[i] == 0u)
            {
                continue;
            }
            if (wheel_want_power[i] <= (wheel_limit_power[i] + CHASSIS_POWER_ALLOC_EPSILON))
            {
                continue;
            }

            demand_sum += demand[i];
            unmet_count++;
        }

        if (unmet_count == 0u)
        {
            break;
        }

        for (uint8_t i = 0u; i < 4u; i++)
        {
            fp32 want_remaining = 0.0f;
            fp32 grant = 0.0f;

            if (active[i] == 0u)
            {
                continue;
            }
            if (wheel_want_power[i] <= (wheel_limit_power[i] + CHASSIS_POWER_ALLOC_EPSILON))
            {
                continue;
            }

            want_remaining = wheel_want_power[i] - wheel_limit_power[i];
            if (demand_sum > CHASSIS_POWER_ALLOC_EPSILON)
            {
                grant = (*budget_remaining) * demand[i] / demand_sum;
            }
            else
            {
                grant = (*budget_remaining) / (fp32)unmet_count;
            }

            if (grant > want_remaining)
            {
                grant = want_remaining;
            }
            if (grant <= 0.0f)
            {
                continue;
            }

            wheel_limit_power[i] += grant;
            granted_total += grant;
        }

        if (granted_total <= CHASSIS_POWER_ALLOC_EPSILON)
        {
            break;
        }

        *budget_remaining -= granted_total;
    }
}

static uint8_t chassis_power_limit_currents_by_demand(fp32 currents[4],
                                                      fp32 power_model_currents[4],
                                                      const int16_t wheel_rpm[4],
                                                      const chassis_move_t *chassis_power_control,
                                                      const chassis_power_limiter_config_t *limiter_cfg,
                                                      fp32 power_budget)
{
    fp32 demand[4] = {0};
    fp32 wheel_want_power[4] = {0};
    fp32 wheel_limit_power[4] = {0};
    uint8_t active[4] = {0};
    uint8_t active_count = 0u;
    fp32 budget_remaining = power_budget;
    fp32 reserve_each = 0.0f;
    uint8_t changed = 0u;

    if (currents == NULL || power_model_currents == NULL || wheel_rpm == NULL ||
        chassis_power_control == NULL || limiter_cfg == NULL || power_budget <= 0.0f)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        const motor_node_param_t *const node = &g_config.motor.chassis[i];

        if (node->can_id == 0u)
        {
            continue;
        }

        active[i] = 1u;
        active_count++;
        demand[i] = fabsf(chassis_power_control->motor_chassis[i].speed_set -
                          chassis_power_control->motor_chassis[i].speed);
        if (demand[i] < CHASSIS_POWER_ALLOC_EPSILON)
        {
            demand[i] = 0.0f;
        }

        if (chassis_power_limiter_calc_motor_power(node,
                                                   wheel_rpm[i],
                                                   power_model_currents[i],
                                                   &wheel_want_power[i]) == 0u)
        {
            wheel_want_power[i] = 0.0f;
        }
    }

    if (active_count == 0u)
    {
        return 0u;
    }

    if (limiter_cfg->power_limit > 0.0f &&
        power_budget >= limiter_cfg->power_limit * CHASSIS_POWER_ALLOC_RESERVED_TRIGGER_RATIO)
    {
        reserve_each = limiter_cfg->power_limit * CHASSIS_POWER_ALLOC_RESERVED_PER_MOTOR_RATIO;
        if (reserve_each * (fp32)active_count > budget_remaining)
        {
            reserve_each = budget_remaining / (fp32)active_count;
        }
    }

    if (reserve_each > 0.0f)
    {
        for (uint8_t i = 0u; i < 4u; i++)
        {
            fp32 grant = 0.0f;

            if (active[i] == 0u)
            {
                continue;
            }

            grant = reserve_each;
            if (grant > wheel_want_power[i])
            {
                grant = wheel_want_power[i];
            }

            wheel_limit_power[i] = grant;
            budget_remaining -= grant;
        }
    }

    chassis_power_allocate_remaining_budget(demand,
                                            active,
                                            wheel_want_power,
                                            wheel_limit_power,
                                            &budget_remaining);

    for (uint8_t i = 0u; i < 4u; i++)
    {
        fp32 limited_model_current = 0.0f;
        const fp32 scale =
            chassis_power_limiter_limit_single_current_by_power_model(&g_config.motor.chassis[i],
                                                                      wheel_rpm[i],
                                                                      power_model_currents[i],
                                                                      wheel_limit_power[i],
                                                                      &limited_model_current,
                                                                      NULL);

        if (active[i] == 0u)
        {
            continue;
        }

        if (scale < 0.999f)
        {
            changed = 1u;
        }

        power_model_currents[i] = limited_model_current;
        currents[i] = limited_model_current * (fp32)g_config.chassis.motor_dir[i];
    }

    return changed;
}

void chassis_power_control_apply_speed_limit(chassis_move_t *chassis_power_control)
{
    fp32 motion_scale = 1.0f;

    if (chassis_power_control == NULL || chassis_power_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        return;
    }

    if (s_chassis_power_runtime.inited != 0u)
    {
        motion_scale = chassis_power_clamp(s_chassis_power_runtime.motion_limit_scale, 0.0f, 1.0f);
    }

    if (motion_scale >= 0.999f)
    {
        return;
    }

    chassis_power_control->vx_set *= motion_scale;
    chassis_power_control->vy_set *= motion_scale;
    chassis_power_control->wz_set *= motion_scale;
}

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 power_scale = 1.0f;
    fp32 motion_target_scale = 1.0f;
    uint8_t robot_id = get_robot_id();
    fp32 runtime_power_limit = power_cfg->power_limit;
    fp32 power_budget = 0.0f;
    uint8_t referee_offline = (uint8_t)toe_is_error(REFEREE_TOE);
    uint8_t use_total_current_fallback =
        (uint8_t)(referee_offline || robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0u);
    fp32 currents[4] = {0};
    fp32 power_model_currents[4] = {0};
    int16_t wheel_rpm[4] = {0};
    chassis_power_limiter_config_t limiter_cfg = chassis_power_build_runtime_cfg(runtime_power_limit);

    for (uint8_t i = 0u; i < 4u; i++)
    {
        currents[i] = chassis_power_control->motor_speed_pid[i].out;
        power_model_currents[i] = currents[i] * (fp32)g_config.chassis.motor_dir[i];
        wheel_rpm[i] = (chassis_power_control->motor_chassis[i].chassis_motor_measure != NULL) ?
                           chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm :
                           0;
    }
    if (use_total_current_fallback != 0u)
    {
        total_current_limit = limiter_cfg.no_judge_total_current_limit;
        (void)chassis_power_limiter_scale_currents(currents, total_current_limit, NULL);
        s_chassis_power_runtime.model_fallback_active = 0u;
        motion_target_scale = 1.0f;
    }
    else
    {
        const uint8_t power_model_ready = chassis_power_limiter_is_power_model_ready(g_config.motor.chassis);

        get_chassis_power_and_buffer((fp32 *)0, &chassis_power_buffer);
        runtime_power_limit = (fp32)get_chassis_power_limit();
        chassis_power_update_runtime_state(runtime_power_limit,
                                           chassis_power_buffer,
                                           &runtime_power_limit,
                                           &chassis_power_buffer);
        limiter_cfg = chassis_power_build_runtime_cfg(runtime_power_limit);

        if (power_model_ready != 0u)
        {
            /* Estimate desired chassis power first, then apply demand-aware allocation
             * before the existing whole-chassis model clamp. */
            (void)chassis_power_limiter_scale_currents_by_power_model(power_model_currents,
                                                                      g_config.motor.chassis,
                                                                      wheel_rpm,
                                                                      FLT_MAX,
                                                                      &chassis_power);

            power_budget = chassis_power_limiter_calc_power_budget(&limiter_cfg,
                                                                   chassis_power,
                                                                   chassis_power_buffer,
                                                                   s_chassis_power_runtime.buffer_limit_active);
            if (limiter_cfg.power_limit > 0.0f)
            {
                const fp32 nominal_total_current_limit =
                    limiter_cfg.buffer_total_current_limit + limiter_cfg.power_total_current_limit;
                const fp32 budget_ratio = chassis_power_clamp(power_budget / limiter_cfg.power_limit, 0.0f, 1.0f);
                total_current_limit = nominal_total_current_limit * budget_ratio;
            }

            (void)chassis_power_limit_currents_by_demand(currents,
                                                         power_model_currents,
                                                         wheel_rpm,
                                                         chassis_power_control,
                                                         &limiter_cfg,
                                                         power_budget);
            power_scale = chassis_power_limiter_scale_currents_by_power_model(power_model_currents,
                                                                              g_config.motor.chassis,
                                                                              wheel_rpm,
                                                                              power_budget,
                                                                              NULL);
            if (power_scale < 1.0f)
            {
                for (uint8_t i = 0u; i < 4u; i++)
                {
                    currents[i] = power_model_currents[i] * (fp32)g_config.chassis.motor_dir[i];
                }
            }
            s_chassis_power_runtime.model_fallback_active = 0u;
        }
        else
        {
            /* If the motor model is unavailable, fall back to conservative total-current limiting. */
            power_budget = chassis_power_limiter_calc_power_budget(&limiter_cfg,
                                                                   limiter_cfg.power_limit,
                                                                   chassis_power_buffer,
                                                                   s_chassis_power_runtime.buffer_limit_active);
            if (limiter_cfg.power_limit > 0.0f)
            {
                const fp32 nominal_total_current_limit =
                    limiter_cfg.buffer_total_current_limit + limiter_cfg.power_total_current_limit;
                const fp32 budget_ratio = chassis_power_clamp(power_budget / limiter_cfg.power_limit, 0.0f, 1.0f);
                total_current_limit = nominal_total_current_limit * budget_ratio;
            }
            (void)chassis_power_limiter_scale_currents(currents, total_current_limit, NULL);
            s_chassis_power_runtime.model_fallback_active = 1u;
        }

        motion_target_scale = chassis_power_budget_to_motion_scale(power_budget, limiter_cfg.power_limit);
    }

    chassis_power_update_motion_scale(motion_target_scale);

    for (uint8_t i = 0u; i < 4u; i++)
    {
        chassis_power_control->motor_speed_pid[i].out = currents[i];
    }
}
