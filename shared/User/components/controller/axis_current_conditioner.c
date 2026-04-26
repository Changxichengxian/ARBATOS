#include "axis_current_conditioner.h"

#include "user_lib.h"

#include <math.h>

fp32 axis_current_conditioner_apply(fp32 current,
                                   fp32 speed_set,
                                   fp32 kick_up,
                                   fp32 kick_down,
                                   fp32 angle,
                                   fp32 angle_min,
                                   fp32 angle_max,
                                   fp32 current_limit_abs,
                                   fp32 *angle_pid_iout,
                                   fp32 *speed_pid_iout,
                                   axis_current_conditioner_info_t *info)
{
    axis_current_conditioner_info_t local = {0};

    fp32 current_set = current;

    const fp32 kick_up_abs = fabsf(kick_up);
    const fp32 kick_down_abs = fabsf(kick_down);

    if (speed_set > 0.0f && kick_up_abs > 0.0f)
    {
        current_set += kick_up_abs;
    }
    else if (speed_set < 0.0f && kick_down_abs > 0.0f)
    {
        current_set -= kick_down_abs;
    }

    local.current_before = current_set;

    const bool_t pushing_up = (speed_set > 0.0f);
    const bool_t pushing_down = (speed_set < 0.0f);
    if ((angle >= angle_max && pushing_up) || (angle <= angle_min && pushing_down))
    {
        current_set = 0.0f;
        local.soft_limited = 1u;
        if (angle_pid_iout != NULL)
        {
            *angle_pid_iout = 0.0f;
        }
        if (speed_pid_iout != NULL)
        {
            *speed_pid_iout = 0.0f;
        }
    }

    const fp32 limit = fabsf(current_limit_abs);
    local.current_limit = limit;
    abs_limit(&current_set, limit);
    local.current_after = current_set;

    local.current_limited =
        ((limit > 0.0f) &&
         (fabsf(local.current_before) > (limit + 1e-3f)) &&
         (fabsf(local.current_after) >= (limit - 1e-3f)))
            ? 1u
            : 0u;

    if (info != NULL)
    {
        *info = local;
    }
    return current_set;
}

