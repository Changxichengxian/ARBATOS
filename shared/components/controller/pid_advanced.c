#include "pid_advanced.h"
#include <stddef.h>

static fp32 pid_adv_abs(fp32 x)
{
    return (x >= 0.0f) ? x : -x;
}

static void pid_adv_output_limit(pid_adv_t *pid)
{
    if (pid->output > pid->max_out)
    {
        pid->output = pid->max_out;
    }
    if (pid->output < -pid->max_out)
    {
        pid->output = -pid->max_out;
    }
}

static void pid_adv_proportion_limit(pid_adv_t *pid)
{
    if (pid->pout > pid->max_out)
    {
        pid->pout = pid->max_out;
    }
    if (pid->pout < -pid->max_out)
    {
        pid->pout = -pid->max_out;
    }
}

static void pid_adv_trapezoid_integral(pid_adv_t *pid)
{
    pid->iterm = pid->ki * ((pid->err + pid->last_err) * 0.5f);
}

static void pid_adv_changing_integral_rate(pid_adv_t *pid)
{
    if ((pid->err * pid->iout) <= 0.0f)
    {
        return;
    }

    const fp32 abs_err = pid_adv_abs(pid->err);

    if (abs_err <= pid->scalar_b)
    {
        return;
    }

    if (abs_err <= (pid->scalar_a + pid->scalar_b))
    {
        if (pid->scalar_a > 0.000001f)
        {
            pid->iterm *= (pid->scalar_a - abs_err + pid->scalar_b) / pid->scalar_a;
        }
        else
        {
            pid->iterm = 0.0f;
        }
        return;
    }

    pid->iterm = 0.0f;
}

static void pid_adv_integral_limit(pid_adv_t *pid)
{
    const fp32 next_iout = pid->iout + pid->iterm;
    const fp32 temp_output = pid->pout + pid->iout + pid->dout;

    if ((pid_adv_abs(temp_output) > pid->max_out) && ((pid->err * pid->iout) > 0.0f))
    {
        pid->iterm = 0.0f;
    }

    if (next_iout > pid->integral_limit)
    {
        pid->iterm = 0.0f;
        pid->iout = pid->integral_limit;
    }
    if (next_iout < -pid->integral_limit)
    {
        pid->iterm = 0.0f;
        pid->iout = -pid->integral_limit;
    }
}

static void pid_adv_derivative_on_measurement(pid_adv_t *pid)
{
    pid->dout = pid->kd * (pid->last_measure - pid->measure);
}

static void pid_adv_derivative_filter(pid_adv_t *pid)
{
    const fp32 coef = pid->derivative_filter_coef;
    pid->dout = pid->dout * coef + pid->last_dout * (1.0f - coef);
}

static void pid_adv_output_filter(pid_adv_t *pid)
{
    const fp32 coef = pid->output_filter_coef;
    pid->output = pid->output * coef + pid->last_output * (1.0f - coef);
}

static void pid_adv_error_handle(pid_adv_t *pid)
{
    if (pid_adv_abs(pid->output) < (pid->max_out * 0.01f))
    {
        return;
    }

    fp32 denom = pid_adv_abs(pid->last_nonzero_target);
    if (denom < 0.000001f)
    {
        denom = pid_adv_abs(pid->target);
    }
    if (denom < 0.000001f)
    {
        pid->error_handler.error_count = 0;
        return;
    }

    const fp32 rel_err = pid_adv_abs(pid->target - pid->measure) / denom;

    if (rel_err > 0.9f)
    {
        pid->error_handler.error_count++;
    }
    else
    {
        pid->error_handler.error_count = 0;
    }

    if (pid->error_handler.error_count > 1000u)
    {
        pid->error_handler.error_type = PID_ADV_ERROR_MOTOR_BLOCKED;
    }
}

void pid_adv_clear(pid_adv_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target = 0.0f;
    pid->last_nonzero_target = 0.0f;

    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;

    pid->measure = 0.0f;
    pid->last_measure = 0.0f;

    pid->err = 0.0f;
    pid->last_err = 0.0f;

    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->iterm = 0.0f;

    pid->output = 0.0f;
    pid->last_output = 0.0f;
    pid->last_dout = 0.0f;

    pid->max_out = 0.0f;
    pid->integral_limit = 0.0f;
    pid->dead_band = 0.0f;
    pid->max_err = 0.0f;

    pid->scalar_a = 0.0f;
    pid->scalar_b = 0.0f;

    pid->output_filter_coef = 0.0f;
    pid->derivative_filter_coef = 0.0f;

    pid->improve = PID_ADV_IMPROVE_NONE;

    pid->error_handler.error_count = 0;
    pid->error_handler.error_type = PID_ADV_ERROR_NONE;
}

void pid_adv_init(pid_adv_t *pid,
                  fp32 max_out,
                  fp32 integral_limit,
                  fp32 deadband,
                  fp32 kp,
                  fp32 ki,
                  fp32 kd,
                  fp32 changing_integral_a,
                  fp32 changing_integral_b,
                  fp32 output_filter_coef,
                  fp32 derivative_filter_coef,
                  uint8_t improve)
{
    if (pid == NULL)
    {
        return;
    }

    pid_adv_clear(pid);

    pid->dead_band = pid_adv_abs(deadband);
    pid->integral_limit = pid_adv_abs(integral_limit);
    pid->max_out = pid_adv_abs(max_out);
    pid->max_err = pid->max_out * 2.0f;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->scalar_a = pid_adv_abs(changing_integral_a);
    pid->scalar_b = pid_adv_abs(changing_integral_b);

    pid->output_filter_coef = output_filter_coef;
    pid->derivative_filter_coef = derivative_filter_coef;

    pid->improve = improve;
}

void pid_adv_reset(pid_adv_t *pid, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    if (pid->ki == 0.0f)
    {
        pid->iout = 0.0f;
    }
}

fp32 pid_adv_calculate(pid_adv_t *pid, fp32 measure, fp32 target)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if ((pid->improve & PID_ADV_IMPROVE_ERROR_HANDLE) != 0u)
    {
        pid_adv_error_handle(pid);
        if (pid->error_handler.error_type != PID_ADV_ERROR_NONE)
        {
            pid->output = 0.0f;
            return 0.0f;
        }
    }

    pid->measure = measure;
    pid->target = target;
    pid->err = pid->target - pid->measure;

    if (pid_adv_abs(pid->target) > 0.000001f)
    {
        pid->last_nonzero_target = pid->target;
    }

    if (pid_adv_abs(pid->err) > pid->dead_band)
    {
        pid->pout = pid->kp * pid->err;
        pid->iterm = pid->ki * pid->err;
        pid->dout = pid->kd * (pid->err - pid->last_err);

        if ((pid->improve & PID_ADV_IMPROVE_TRAPEZOID_INTEGRAL) != 0u)
        {
            pid_adv_trapezoid_integral(pid);
        }
        if ((pid->improve & PID_ADV_IMPROVE_CHANGING_INTEGRAL_RATE) != 0u)
        {
            pid_adv_changing_integral_rate(pid);
        }
        if ((pid->improve & PID_ADV_IMPROVE_INTEGRAL_LIMIT) != 0u)
        {
            pid_adv_integral_limit(pid);
        }
        if ((pid->improve & PID_ADV_IMPROVE_DERIVATIVE_ON_MEASUREMENT) != 0u)
        {
            pid_adv_derivative_on_measurement(pid);
        }
        if ((pid->improve & PID_ADV_IMPROVE_DERIVATIVE_FILTER) != 0u)
        {
            pid_adv_derivative_filter(pid);
        }

        pid->iout += pid->iterm;
        pid->output = pid->pout + pid->iout + pid->dout;

        if ((pid->improve & PID_ADV_IMPROVE_OUTPUT_FILTER) != 0u)
        {
            pid_adv_output_filter(pid);
        }

        pid_adv_output_limit(pid);
        pid_adv_proportion_limit(pid);
    }

    pid->last_measure = pid->measure;
    pid->last_output = pid->output;
    pid->last_dout = pid->dout;
    pid->last_err = pid->err;

    return pid->output;
}

