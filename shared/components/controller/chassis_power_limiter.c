#include "chassis_power_limiter.h"

#include <math.h>

#include "motor_model_db.h"

#define CHASSIS_POWER_LIMITER_MIN_BUFFER_J 5.0f
#define CHASSIS_POWER_LIMITER_MOTOR_COUNT 4U
#define CHASSIS_POWER_LIMITER_RADPS_PER_RPM 0.10471975512f
#define CHASSIS_POWER_LIMITER_EPSILON 1.0e-6f

typedef struct
{
    fp32 k0;
    fp32 k1;
    fp32 k2;
    fp32 k3;
    fp32 k4;
    fp32 k5;
} chassis_power_limiter_motor_power_model_t;

static uint8_t chassis_power_limiter_get_motor_power_model(motor_model_e model,
                                                           chassis_power_limiter_motor_power_model_t *out_model);
static uint8_t chassis_power_limiter_model_current_to_ampere(const motor_node_param_t *node,
                                                             fp32 current_cmd,
                                                             fp32 *out_current_a);
static uint8_t chassis_power_limiter_build_motor_terms(const motor_node_param_t *node,
                                                       int16_t wheel_rpm,
                                                       fp32 current_cmd,
                                                       fp32 *out_a,
                                                       fp32 *out_b,
                                                       fp32 *out_c,
                                                       fp32 *out_full_power);
static fp32 chassis_power_limiter_get_nominal_total_current_limit(const chassis_power_limiter_config_t *cfg);
static fp32 chassis_power_limiter_calc_buffer_current_limit(const chassis_power_limiter_config_t *cfg,
                                                            fp32 chassis_power_buffer);
static fp32 chassis_power_limiter_eval_quadratic(fp32 a, fp32 b, fp32 c, fp32 scale);
static fp32 chassis_power_limiter_solve_quadratic_scale(fp32 a, fp32 b, fp32 c, fp32 power_limit);

fp32 chassis_power_limiter_calc_total_current_limit(const chassis_power_limiter_config_t *cfg,
                                                    fp32 chassis_power,
                                                    fp32 chassis_power_buffer)
{
    if (cfg == NULL)
    {
        return 0.0f;
    }

    if (chassis_power_buffer < cfg->warning_power_buffer)
    {
        return chassis_power_limiter_calc_buffer_current_limit(cfg, chassis_power_buffer);
    }

    if (chassis_power > cfg->warning_power)
    {
        fp32 power_scale = 0.0f;
        const fp32 denom = (cfg->power_limit - cfg->warning_power);
        if (chassis_power < cfg->power_limit && denom > 0.0f)
        {
            power_scale = (cfg->power_limit - chassis_power) / denom;
        }
        return cfg->buffer_total_current_limit + cfg->power_total_current_limit * power_scale;
    }

    return cfg->buffer_total_current_limit + cfg->power_total_current_limit;
}

fp32 chassis_power_limiter_calc_power_budget(const chassis_power_limiter_config_t *cfg,
                                             fp32 chassis_power,
                                             fp32 chassis_power_buffer,
                                             uint8_t buffer_limit_active)
{
    const fp32 nominal_total_current_limit = chassis_power_limiter_get_nominal_total_current_limit(cfg);
    fp32 total_current_limit = 0.0f;
    fp32 budget_scale = 0.0f;

    if (cfg == NULL || cfg->power_limit <= 0.0f || nominal_total_current_limit <= 0.0f)
    {
        return 0.0f;
    }

    if (buffer_limit_active != 0u)
    {
        total_current_limit = chassis_power_limiter_calc_buffer_current_limit(cfg, chassis_power_buffer);
    }
    else
    {
        total_current_limit = chassis_power_limiter_calc_total_current_limit(cfg, chassis_power, chassis_power_buffer);
    }

    budget_scale = total_current_limit / nominal_total_current_limit;
    if (budget_scale < 0.0f)
    {
        budget_scale = 0.0f;
    }
    if (budget_scale > 1.0f)
    {
        budget_scale = 1.0f;
    }

    return cfg->power_limit * budget_scale;
}

uint8_t chassis_power_limiter_is_power_model_ready(const motor_node_param_t motor_nodes[4])
{
    uint8_t active_count = 0u;

    if (motor_nodes == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < CHASSIS_POWER_LIMITER_MOTOR_COUNT; i++)
    {
        const motor_node_param_t *const node = &motor_nodes[i];
        const motor_model_db_entry_t *entry = NULL;
        chassis_power_limiter_motor_power_model_t model = {0};

        if (node->can_id == 0u)
        {
            continue;
        }

        active_count++;
        entry = motor_model_db_get(node->model);
        if (entry == NULL || entry->cmd_current_range_abs <= 0 || entry->torque_current_range_a <= 0.0f)
        {
            return 0u;
        }

        if (chassis_power_limiter_get_motor_power_model(node->model, &model) == 0u)
        {
            return 0u;
        }
    }

    return (active_count > 0u) ? 1u : 0u;
}

uint8_t chassis_power_limiter_calc_motor_power(const motor_node_param_t *node,
                                               int16_t wheel_rpm,
                                               fp32 current_cmd,
                                               fp32 *out_power)
{
    fp32 a = 0.0f;
    fp32 b = 0.0f;
    fp32 c = 0.0f;
    fp32 full_power = 0.0f;

    if (out_power != NULL)
    {
        *out_power = 0.0f;
    }

    if (chassis_power_limiter_build_motor_terms(node, wheel_rpm, current_cmd, &a, &b, &c, &full_power) == 0u)
    {
        return 0u;
    }

    if (out_power != NULL)
    {
        *out_power = (full_power > 0.0f) ? full_power : 0.0f;
    }

    return 1u;
}

fp32 chassis_power_limiter_limit_single_current_by_power_model(const motor_node_param_t *node,
                                                               int16_t wheel_rpm,
                                                               fp32 current_cmd,
                                                               fp32 power_limit,
                                                               fp32 *out_current,
                                                               fp32 *out_power)
{
    fp32 a = 0.0f;
    fp32 b = 0.0f;
    fp32 c = 0.0f;
    fp32 full_power = 0.0f;
    fp32 scale = 1.0f;

    if (out_current != NULL)
    {
        *out_current = current_cmd;
    }
    if (out_power != NULL)
    {
        *out_power = 0.0f;
    }

    if (chassis_power_limiter_build_motor_terms(node, wheel_rpm, current_cmd, &a, &b, &c, &full_power) == 0u)
    {
        return 1.0f;
    }

    if (out_power != NULL)
    {
        *out_power = (full_power > 0.0f) ? full_power : 0.0f;
    }

    if (full_power <= power_limit)
    {
        return 1.0f;
    }

    if (power_limit <= 0.0f)
    {
        if (out_current != NULL)
        {
            *out_current = 0.0f;
        }
        return 0.0f;
    }

    scale = chassis_power_limiter_solve_quadratic_scale(a, b, c, power_limit);
    if (out_current != NULL)
    {
        *out_current = current_cmd * scale;
    }

    return scale;
}

fp32 chassis_power_limiter_scale_currents(fp32 currents[4],
                                         fp32 total_current_limit,
                                         fp32 *out_total_current)
{
    if (currents == NULL)
    {
        return 1.0f;
    }

    fp32 total_current = 0.0f;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        total_current += fabsf(currents[i]);
    }

    if (out_total_current != NULL)
    {
        *out_total_current = total_current;
    }

    if (total_current_limit <= 0.0f || total_current <= total_current_limit || total_current <= 0.0f)
    {
        return 1.0f;
    }

    const fp32 scale = total_current_limit / total_current;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        currents[i] *= scale;
    }
    return scale;
}

fp32 chassis_power_limiter_scale_currents_by_power_model(fp32 currents[4],
                                                         const motor_node_param_t motor_nodes[4],
                                                         const int16_t wheel_rpm[4],
                                                         fp32 power_limit,
                                                         fp32 *out_total_power)
{
    fp32 quad_a = 0.0f;
    fp32 quad_b = 0.0f;
    fp32 quad_c = 0.0f;
    fp32 total_power = 0.0f;
    uint8_t supported_count = 0u;

    if (currents == NULL || motor_nodes == NULL || wheel_rpm == NULL)
    {
        return 1.0f;
    }

    for (uint8_t i = 0u; i < CHASSIS_POWER_LIMITER_MOTOR_COUNT; i++)
    {
        const motor_node_param_t *const node = &motor_nodes[i];
        chassis_power_limiter_motor_power_model_t model = {0};
        fp32 current_a = 0.0f;
        const fp32 omega = (fp32)wheel_rpm[i] * CHASSIS_POWER_LIMITER_RADPS_PER_RPM;

        if (node->can_id == 0u)
        {
            continue;
        }

        if (chassis_power_limiter_get_motor_power_model(node->model, &model) == 0u)
        {
            if (out_total_power != NULL)
            {
                *out_total_power = 0.0f;
            }
            return 1.0f;
        }

        if (chassis_power_limiter_model_current_to_ampere(node, currents[i], &current_a) == 0u)
        {
            if (out_total_power != NULL)
            {
                *out_total_power = 0.0f;
            }
            return 1.0f;
        }

        quad_a += model.k4 * current_a * current_a;
        quad_b += (model.k1 * current_a) + (model.k3 * current_a * omega);
        quad_c += model.k0 + (model.k2 * fabsf(omega)) + (model.k5 * omega * omega);
        supported_count++;
    }

    if (supported_count == 0u)
    {
        if (out_total_power != NULL)
        {
            *out_total_power = 0.0f;
        }
        return 1.0f;
    }

    total_power = chassis_power_limiter_eval_quadratic(quad_a, quad_b, quad_c, 1.0f);
    if (total_power < 0.0f)
    {
        total_power = 0.0f;
    }

    if (out_total_power != NULL)
    {
        *out_total_power = total_power;
    }

    if (power_limit <= 0.0f)
    {
        for (uint8_t i = 0u; i < CHASSIS_POWER_LIMITER_MOTOR_COUNT; i++)
        {
            currents[i] = 0.0f;
        }
        return 0.0f;
    }

    if (total_power <= power_limit)
    {
        return 1.0f;
    }

    {
        const fp32 scale = chassis_power_limiter_solve_quadratic_scale(quad_a, quad_b, quad_c, power_limit);
        for (uint8_t i = 0u; i < CHASSIS_POWER_LIMITER_MOTOR_COUNT; i++)
        {
            currents[i] *= scale;
        }
        return scale;
    }
}

static uint8_t chassis_power_limiter_build_motor_terms(const motor_node_param_t *node,
                                                       int16_t wheel_rpm,
                                                       fp32 current_cmd,
                                                       fp32 *out_a,
                                                       fp32 *out_b,
                                                       fp32 *out_c,
                                                       fp32 *out_full_power)
{
    chassis_power_limiter_motor_power_model_t model = {0};
    fp32 current_a = 0.0f;
    const fp32 omega = (fp32)wheel_rpm * CHASSIS_POWER_LIMITER_RADPS_PER_RPM;
    fp32 a = 0.0f;
    fp32 b = 0.0f;
    fp32 c = 0.0f;
    fp32 full_power = 0.0f;

    if (node == NULL)
    {
        return 0u;
    }

    if (node->can_id == 0u)
    {
        if (out_a != NULL)
        {
            *out_a = 0.0f;
        }
        if (out_b != NULL)
        {
            *out_b = 0.0f;
        }
        if (out_c != NULL)
        {
            *out_c = 0.0f;
        }
        if (out_full_power != NULL)
        {
            *out_full_power = 0.0f;
        }
        return 1u;
    }

    if (chassis_power_limiter_get_motor_power_model(node->model, &model) == 0u)
    {
        return 0u;
    }

    if (chassis_power_limiter_model_current_to_ampere(node, current_cmd, &current_a) == 0u)
    {
        return 0u;
    }

    a = model.k4 * current_a * current_a;
    b = (model.k1 * current_a) + (model.k3 * current_a * omega);
    c = model.k0 + (model.k2 * fabsf(omega)) + (model.k5 * omega * omega);
    full_power = chassis_power_limiter_eval_quadratic(a, b, c, 1.0f);

    if (out_a != NULL)
    {
        *out_a = a;
    }
    if (out_b != NULL)
    {
        *out_b = b;
    }
    if (out_c != NULL)
    {
        *out_c = c;
    }
    if (out_full_power != NULL)
    {
        *out_full_power = full_power;
    }

    return 1u;
}

static fp32 chassis_power_limiter_get_nominal_total_current_limit(const chassis_power_limiter_config_t *cfg)
{
    if (cfg == NULL)
    {
        return 0.0f;
    }

    return cfg->buffer_total_current_limit + cfg->power_total_current_limit;
}

static fp32 chassis_power_limiter_calc_buffer_current_limit(const chassis_power_limiter_config_t *cfg,
                                                            fp32 chassis_power_buffer)
{
    fp32 power_scale = 0.0f;
    const fp32 denom = (cfg != NULL) ? cfg->warning_power_buffer : 0.0f;

    if (cfg == NULL || cfg->buffer_total_current_limit <= 0.0f)
    {
        return 0.0f;
    }

    if (denom > 0.0f)
    {
        if (chassis_power_buffer >= denom)
        {
            power_scale = 1.0f;
        }
        else if (chassis_power_buffer > CHASSIS_POWER_LIMITER_MIN_BUFFER_J)
        {
            power_scale = chassis_power_buffer / denom;
        }
        else
        {
            power_scale = CHASSIS_POWER_LIMITER_MIN_BUFFER_J / denom;
        }
    }

    return cfg->buffer_total_current_limit * power_scale;
}

static uint8_t chassis_power_limiter_get_motor_power_model(motor_model_e model,
                                                           chassis_power_limiter_motor_power_model_t *out_model)
{
    if (out_model == NULL)
    {
        return 0u;
    }

    switch (model)
    {
    case MOTOR_MODEL_3508:
        out_model->k0 = 0.5991965102f;
        out_model->k1 = 0.0088903207f;
        out_model->k2 = 0.0027051432f;
        out_model->k3 = 0.0176490289f;
        out_model->k4 = 0.1639722776f;
        out_model->k5 = 0.0000179271f;
        return 1u;
    default:
        break;
    }

    return 0u;
}

static uint8_t chassis_power_limiter_model_current_to_ampere(const motor_node_param_t *node,
                                                             fp32 current_cmd,
                                                             fp32 *out_current_a)
{
    const motor_model_db_entry_t *entry = NULL;

    if (node == NULL || out_current_a == NULL)
    {
        return 0u;
    }

    entry = motor_model_db_get(node->model);
    if (entry == NULL || entry->cmd_current_range_abs <= 0 || entry->torque_current_range_a <= 0.0f)
    {
        return 0u;
    }

    *out_current_a = current_cmd * (entry->torque_current_range_a / (fp32)entry->cmd_current_range_abs);
    return 1u;
}

static fp32 chassis_power_limiter_eval_quadratic(fp32 a, fp32 b, fp32 c, fp32 scale)
{
    return (a * scale * scale) + (b * scale) + c;
}

static fp32 chassis_power_limiter_solve_quadratic_scale(fp32 a, fp32 b, fp32 c, fp32 power_limit)
{
    const fp32 base_power = chassis_power_limiter_eval_quadratic(a, b, c, 0.0f);
    const fp32 full_power = chassis_power_limiter_eval_quadratic(a, b, c, 1.0f);
    const fp32 rhs = c - power_limit;
    fp32 scale = 1.0f;

    if (base_power >= power_limit)
    {
        return 0.0f;
    }

    if (full_power <= power_limit)
    {
        return 1.0f;
    }

    if (fabsf(a) <= CHASSIS_POWER_LIMITER_EPSILON)
    {
        if (fabsf(b) <= CHASSIS_POWER_LIMITER_EPSILON)
        {
            return 0.0f;
        }

        scale = (power_limit - c) / b;
        if (scale < 0.0f)
        {
            scale = 0.0f;
        }
        if (scale > 1.0f)
        {
            scale = 1.0f;
        }
        return scale;
    }

    {
        fp32 discriminant = (b * b) - (4.0f * a * rhs);
        if (discriminant < 0.0f)
        {
            discriminant = 0.0f;
        }

        {
            const fp32 sqrt_disc = sqrtf(discriminant);
            const fp32 denom = 2.0f * a;
            const fp32 root_a = (-b + sqrt_disc) / denom;
            const fp32 root_b = (-b - sqrt_disc) / denom;

            scale = root_a;
            if (scale < 0.0f || scale > 1.0f)
            {
                scale = root_b;
            }
        }
    }

    if (scale < 0.0f)
    {
        scale = 0.0f;
    }
    if (scale > 1.0f)
    {
        scale = 1.0f;
    }
    return scale;
}
