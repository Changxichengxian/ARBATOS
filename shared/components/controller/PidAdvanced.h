#ifndef PID_ADVANCED_H
#define PID_ADVANCED_H
/**
  ******************************************************************************
  * @file    PidAdvanced.h/.c
  * @brief   Optional "advanced" PID controller (not used by default).
  *
  *          This module keeps a feature-rich PID implementation available for
  *          later use (anti-windup, derivative on measurement, filtering, etc.)
  ******************************************************************************
  */

#include "types.h"

typedef enum
{
    PID_ADV_IMPROVE_NONE = 0x00u,
    PID_ADV_IMPROVE_INTEGRAL_LIMIT = 0x01u,
    PID_ADV_IMPROVE_DERIVATIVE_ON_MEASUREMENT = 0x02u,
    PID_ADV_IMPROVE_TRAPEZOID_INTEGRAL = 0x04u,
    PID_ADV_IMPROVE_PROPORTIONAL_ON_MEASUREMENT = 0x08u, // reserved (not implemented)
    PID_ADV_IMPROVE_OUTPUT_FILTER = 0x10u,
    PID_ADV_IMPROVE_CHANGING_INTEGRAL_RATE = 0x20u,
    PID_ADV_IMPROVE_DERIVATIVE_FILTER = 0x40u,
    PID_ADV_IMPROVE_ERROR_HANDLE = 0x80u,
} PidAdvImprovement;

typedef enum
{
    PID_ADV_ERROR_NONE = 0x00u,
    PID_ADV_ERROR_MOTOR_BLOCKED = 0x01u,
} PidAdvErrorType;

typedef struct
{
    uint64_t error_count;
    PidAdvErrorType error_type;
} PidAdvErrorHandler;

typedef struct
{
    fp32 target;
    fp32 last_nonzero_target;

    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 measure;
    fp32 last_measure;

    fp32 err;
    fp32 last_err;

    fp32 pout;
    fp32 iout;
    fp32 dout;
    fp32 iterm;

    fp32 output;
    fp32 last_output;
    fp32 last_dout;

    fp32 max_out;
    fp32 integral_limit;
    fp32 dead_band;
    fp32 max_err;

    fp32 scalar_a;
    fp32 scalar_b;

    fp32 output_filter_coef;
    fp32 derivative_filter_coef;

    uint8_t improve;

    PidAdvErrorHandler error_handler;
} PidAdv;

extern void PidAdvInit(PidAdv *pid,
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
                         uint8_t improve);
extern void PidAdvReset(PidAdv *pid, fp32 kp, fp32 ki, fp32 kd);
extern void PidAdvClear(PidAdv *pid);
extern fp32 PidAdvCalculate(PidAdv *pid, fp32 measure, fp32 target);

#endif

