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

#define CHASSIS_POWER_BUFFER_FILTER_TAU_S      0.08f
#define CHASSIS_POWER_LIMIT_FILTER_TAU_S       0.12f
#define CHASSIS_POWER_MOTION_FILTER_TAU_S      0.10f
#define CHASSIS_POWER_BUFFER_RELEASE_HYST_J    2.0f
#define CHASSIS_POWER_MOTION_BASE_SCALE        0.15f
#define CHASSIS_POWER_CONTROL_MIN_PERIOD_S     0.001f

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
    fp32 dt = (fp32)g_config.chassis.control_period_ms * 0.001f;
    if (dt < CHASSIS_POWER_CONTROL_MIN_PERIOD_S)
    {
        dt = CHASSIS_POWER_CONTROL_MIN_PERIOD_S;
    }
    return dt;
}

static fp32 chassis_power_calc_total_current(const fp32 currents[4])
{
    fp32 total_current = 0.0f;

    if (currents == NULL)
    {
        return 0.0f;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        total_current += fabsf(currents[i]);
    }

    return total_current;
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
    fp32 total_current = 0.0f;
    fp32 final_scale = 1.0f;
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
    total_current = chassis_power_calc_total_current(currents);

    if (use_total_current_fallback != 0u)
    {
        total_current_limit = limiter_cfg.no_judge_total_current_limit;
        final_scale = chassis_power_limiter_scale_currents(currents, total_current_limit, NULL);
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
            // 2026 裁判协议不再下发实时底盘功率，这里先用电机模型估算当前功率，
            // 再直接算出可用功率预算，最后只执行一次按功率缩放。
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

            power_scale = chassis_power_limiter_scale_currents_by_power_model(power_model_currents,
                                                                              g_config.motor.chassis,
                                                                              wheel_rpm,
                                                                              power_budget,
                                                                              NULL);
            if (power_scale < 1.0f)
            {
                for (uint8_t i = 0u; i < 4u; i++)
                {
                    currents[i] *= power_scale;
                }
            }
            final_scale = power_scale;
            s_chassis_power_runtime.model_fallback_active = 0u;
        }
        else
        {
            // 功率模型不可用时，不继续假装“精确限功率”，直接回到保守总电流限制。
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
            final_scale = chassis_power_limiter_scale_currents(currents, total_current_limit, NULL);
            s_chassis_power_runtime.model_fallback_active = 1u;
        }

        motion_target_scale = chassis_power_budget_to_motion_scale(power_budget, limiter_cfg.power_limit);
    }

    chassis_power_update_motion_scale(motion_target_scale);

    for (uint8_t i = 0u; i < 4u; i++)
    {
        chassis_power_control->motor_speed_pid[i].out = currents[i];
    }

    if (final_scale < 0.999f)
    {
        sdlog_chassis_power_limit_t log = {0};
        log.chassis_power = chassis_power;
        log.chassis_power_buffer = chassis_power_buffer;
        log.total_current = total_current;
        log.total_current_limit = total_current_limit;
        log.current_scale = final_scale;
        log.referee_offline = referee_offline;
        log.robot_id = robot_id;
        log.reserved[0] = s_chassis_power_runtime.buffer_limit_active;
        log.reserved[1] = s_chassis_power_runtime.model_fallback_active;
        sdlog_write(SDLOG_TAG_CHASSIS_POWER_LIMIT, &log, (uint16_t)sizeof(log));
    }
}
