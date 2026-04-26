#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "struct_typedef.h"

typedef struct
{
    fp32 x; // state estimate
    fp32 p; // estimate covariance
    fp32 q; // process noise covariance
    fp32 r; // measurement noise covariance
} kalman_1d_t;

extern void kalman_1d_init(kalman_1d_t *kf, fp32 x0, fp32 p0, fp32 q, fp32 r);
extern fp32 kalman_1d_predict(kalman_1d_t *kf);
extern fp32 kalman_1d_update(kalman_1d_t *kf, fp32 z);
extern fp32 kalman_1d_step(kalman_1d_t *kf, fp32 z);

// 2-state Kalman filter with scalar measurement: z = H * x + v
// - state x = [x0, x1]^T
// - covariance P is symmetric (P01 == P10)
typedef struct
{
    fp32 x0;
    fp32 x1;

    fp32 P00;
    fp32 P01;
    fp32 P11;

    fp32 Q00;
    fp32 Q01;
    fp32 Q11;

    fp32 R; // measurement noise variance
    fp32 H0;
    fp32 H1;
} kalman_2x1_t;

extern void kalman_2x1_init(kalman_2x1_t *kf,
                           fp32 x0,
                           fp32 x1,
                           fp32 p00,
                           fp32 p01,
                           fp32 p11,
                           fp32 q00,
                           fp32 q01,
                           fp32 q11,
                           fp32 r,
                           fp32 h0,
                           fp32 h1);

extern void kalman_2x1_set_Q(kalman_2x1_t *kf, fp32 q00, fp32 q01, fp32 q11);
extern void kalman_2x1_set_R(kalman_2x1_t *kf, fp32 r);
extern void kalman_2x1_set_H(kalman_2x1_t *kf, fp32 h0, fp32 h1);

// Convenience: constant-velocity process noise from acceleration variance (q_a)
// Q = q_a * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
extern void kalman_2x1_set_Q_cv(kalman_2x1_t *kf, fp32 dt, fp32 accel_var);

extern void kalman_2x1_predict(kalman_2x1_t *kf, fp32 f00, fp32 f01, fp32 f10, fp32 f11);
extern void kalman_2x1_predict_u(kalman_2x1_t *kf, fp32 f00, fp32 f01, fp32 f10, fp32 f11, fp32 b0, fp32 b1, fp32 u);

// return 1 if update was applied, 0 if skipped (e.g. invalid S)
extern bool_t kalman_2x1_update(kalman_2x1_t *kf, fp32 z);

extern fp32 kalman_2x1_measurement(const kalman_2x1_t *kf);

#endif

