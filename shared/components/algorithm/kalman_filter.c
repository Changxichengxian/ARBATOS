#include "kalman_filter.h"

static fp32 kalman_safe_div(fp32 num, fp32 denom)
{
    if (denom > 1e-12f || denom < -1e-12f)
    {
        return num / denom;
    }
    return 0.0f;
}

void kalman_1d_init(kalman_1d_t *kf, fp32 x0, fp32 p0, fp32 q, fp32 r)
{
    if (kf == NULL)
    {
        return;
    }
    kf->x = x0;
    kf->p = p0;
    kf->q = q;
    kf->r = r;
}

fp32 kalman_1d_predict(kalman_1d_t *kf)
{
    if (kf == NULL)
    {
        return 0.0f;
    }
    kf->p += kf->q;
    return kf->x;
}

fp32 kalman_1d_update(kalman_1d_t *kf, fp32 z)
{
    if (kf == NULL)
    {
        return 0.0f;
    }

    const fp32 denom = kf->p + kf->r;
    const fp32 k = kalman_safe_div(kf->p, denom);

    kf->x += k * (z - kf->x);
    kf->p = (1.0f - k) * kf->p;
    return kf->x;
}

fp32 kalman_1d_step(kalman_1d_t *kf, fp32 z)
{
    (void)kalman_1d_predict(kf);
    return kalman_1d_update(kf, z);
}

void kalman_2x1_init(kalman_2x1_t *kf,
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
                     fp32 h1)
{
    if (kf == NULL)
    {
        return;
    }

    kf->x0 = x0;
    kf->x1 = x1;

    kf->P00 = p00;
    kf->P01 = p01;
    kf->P11 = p11;

    kf->Q00 = q00;
    kf->Q01 = q01;
    kf->Q11 = q11;

    kf->R = r;
    kf->H0 = h0;
    kf->H1 = h1;
}

void kalman_2x1_set_Q(kalman_2x1_t *kf, fp32 q00, fp32 q01, fp32 q11)
{
    if (kf == NULL)
    {
        return;
    }
    kf->Q00 = q00;
    kf->Q01 = q01;
    kf->Q11 = q11;
}

void kalman_2x1_set_R(kalman_2x1_t *kf, fp32 r)
{
    if (kf == NULL)
    {
        return;
    }
    kf->R = r;
}

void kalman_2x1_set_H(kalman_2x1_t *kf, fp32 h0, fp32 h1)
{
    if (kf == NULL)
    {
        return;
    }
    kf->H0 = h0;
    kf->H1 = h1;
}

void kalman_2x1_set_Q_cv(kalman_2x1_t *kf, fp32 dt, fp32 accel_var)
{
    if (kf == NULL)
    {
        return;
    }

    if (dt <= 0.0f || accel_var <= 0.0f)
    {
        kf->Q00 = 0.0f;
        kf->Q01 = 0.0f;
        kf->Q11 = 0.0f;
        return;
    }

    const fp32 dt2 = dt * dt;
    const fp32 dt3 = dt2 * dt;
    const fp32 dt4 = dt2 * dt2;

    kf->Q00 = 0.25f * accel_var * dt4;
    kf->Q01 = 0.5f * accel_var * dt3;
    kf->Q11 = accel_var * dt2;
}

void kalman_2x1_predict(kalman_2x1_t *kf, fp32 f00, fp32 f01, fp32 f10, fp32 f11)
{
    kalman_2x1_predict_u(kf, f00, f01, f10, f11, 0.0f, 0.0f, 0.0f);
}

void kalman_2x1_predict_u(kalman_2x1_t *kf, fp32 f00, fp32 f01, fp32 f10, fp32 f11, fp32 b0, fp32 b1, fp32 u)
{
    if (kf == NULL)
    {
        return;
    }

    const fp32 x0 = kf->x0;
    const fp32 x1 = kf->x1;

    kf->x0 = f00 * x0 + f01 * x1 + b0 * u;
    kf->x1 = f10 * x0 + f11 * x1 + b1 * u;

    const fp32 P00 = kf->P00;
    const fp32 P01 = kf->P01;
    const fp32 P11 = kf->P11;

    const fp32 FP00 = f00 * P00 + f01 * P01;
    const fp32 FP01 = f00 * P01 + f01 * P11;
    const fp32 FP10 = f10 * P00 + f11 * P01;
    const fp32 FP11 = f10 * P01 + f11 * P11;

    kf->P00 = FP00 * f00 + FP01 * f01 + kf->Q00;
    kf->P01 = FP00 * f10 + FP01 * f11 + kf->Q01;
    kf->P11 = FP10 * f10 + FP11 * f11 + kf->Q11;
}

bool_t kalman_2x1_update(kalman_2x1_t *kf, fp32 z)
{
    if (kf == NULL)
    {
        return 0u;
    }

    const fp32 H0 = kf->H0;
    const fp32 H1 = kf->H1;

    const fp32 z_hat = H0 * kf->x0 + H1 * kf->x1;
    const fp32 y = z - z_hat;

    const fp32 P00 = kf->P00;
    const fp32 P01 = kf->P01;
    const fp32 P11 = kf->P11;

    const fp32 PH0 = P00 * H0 + P01 * H1;
    const fp32 PH1 = P01 * H0 + P11 * H1;

    const fp32 S = H0 * PH0 + H1 * PH1 + kf->R;
    if (S > -1e-12f && S < 1e-12f)
    {
        return 0u;
    }

    const fp32 invS = 1.0f / S;
    const fp32 K0 = PH0 * invS;
    const fp32 K1 = PH1 * invS;

    kf->x0 += K0 * y;
    kf->x1 += K1 * y;

    kf->P00 = P00 - K0 * PH0;
    kf->P01 = P01 - K0 * PH1;
    kf->P11 = P11 - K1 * PH1;

    return 1u;
}

fp32 kalman_2x1_measurement(const kalman_2x1_t *kf)
{
    if (kf == NULL)
    {
        return 0.0f;
    }
    return kf->H0 * kf->x0 + kf->H1 * kf->x1;
}

