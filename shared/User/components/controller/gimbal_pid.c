#include "gimbal_pid.h"

#include "user_lib.h"

void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;

    const fp32 err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

void gimbal_PID_clear(gimbal_PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err = 0.0f;
    pid->get = 0.0f;
    pid->set = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

