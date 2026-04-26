#ifndef AXIS_CURRENT_CONDITIONER_H
#define AXIS_CURRENT_CONDITIONER_H

#include "struct_typedef.h"

typedef struct
{
    uint8_t soft_limited;
    uint8_t current_limited;
    fp32 current_before;
    fp32 current_after;
    fp32 current_limit;
} axis_current_conditioner_info_t;

extern fp32 axis_current_conditioner_apply(fp32 current,
                                          fp32 speed_set,
                                          fp32 kick_up,
                                          fp32 kick_down,
                                          fp32 angle,
                                          fp32 angle_min,
                                          fp32 angle_max,
                                          fp32 current_limit_abs,
                                          fp32 *angle_pid_iout,
                                          fp32 *speed_pid_iout,
                                          axis_current_conditioner_info_t *info);

#endif

