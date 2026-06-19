#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#include "Types.h"

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} GimbalPid;

extern void GimbalPidInit(GimbalPid *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
extern fp32 GimbalPidCalc(GimbalPid *pid, fp32 get, fp32 set, fp32 error_delta);
extern void GimbalPidClear(GimbalPid *pid);

#endif

