/*
 * GCC-side AHRS implementation for projects that cannot link the ARMCC-only
 * AHRS.lib. Keil builds still use the binary library recorded in .uvprojx.
 */

#include "AHRS.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AHRS_GRAVITY_M_S2 9.80665f
#define AHRS_EPSILON 1.0e-6f

static fp32 ahrs_inv_norm3(fp32 x, fp32 y, fp32 z)
{
    const fp32 n2 = x * x + y * y + z * z;
    return (n2 > AHRS_EPSILON) ? (1.0f / sqrtf(n2)) : 0.0f;
}

static void ahrs_normalize_quat(fp32 quat[4])
{
    const fp32 n2 = quat[0] * quat[0] + quat[1] * quat[1] +
                    quat[2] * quat[2] + quat[3] * quat[3];
    if (n2 <= AHRS_EPSILON)
    {
        quat[0] = 1.0f;
        quat[1] = 0.0f;
        quat[2] = 0.0f;
        quat[3] = 0.0f;
        return;
    }

    const fp32 inv = 1.0f / sqrtf(n2);
    quat[0] *= inv;
    quat[1] *= inv;
    quat[2] *= inv;
    quat[3] *= inv;
}

static void ahrs_euler_to_quat(fp32 yaw, fp32 pitch, fp32 roll, fp32 quat[4])
{
    const fp32 cy = cosf(yaw * 0.5f);
    const fp32 sy = sinf(yaw * 0.5f);
    const fp32 cp = cosf(pitch * 0.5f);
    const fp32 sp = sinf(pitch * 0.5f);
    const fp32 cr = cosf(roll * 0.5f);
    const fp32 sr = sinf(roll * 0.5f);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;
    ahrs_normalize_quat(quat);
}

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    fp32 roll = 0.0f;
    fp32 pitch = 0.0f;
    fp32 yaw = 0.0f;

    if (quat == NULL)
    {
        return;
    }

    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

    if (accel == NULL)
    {
        return;
    }

    const fp32 ax = accel[0];
    const fp32 ay = accel[1];
    const fp32 az = accel[2];
    if (ahrs_inv_norm3(ax, ay, az) == 0.0f)
    {
        return;
    }

    roll = atan2f(ay, az);
    pitch = -atan2f(ax, sqrtf(ay * ay + az * az));

    if (mag != NULL && ahrs_inv_norm3(mag[0], mag[1], mag[2]) != 0.0f)
    {
        const fp32 sr = sinf(roll);
        const fp32 cr = cosf(roll);
        const fp32 sp = sinf(pitch);
        const fp32 cp = cosf(pitch);
        const fp32 mx = mag[0];
        const fp32 my = mag[1];
        const fp32 mz = mag[2];
        const fp32 mx2 = mx * cp + mz * sp;
        const fp32 my2 = mx * sr * sp + my * cr - mz * sr * cp;
        yaw = atan2f(-my2, mx2);
    }

    ahrs_euler_to_quat(yaw, pitch, roll, quat);
}

bool_t AHRS_update(fp32 quat[4],
                   const fp32 timing_time,
                   const fp32 gyro[3],
                   const fp32 accel[3],
                   const fp32 mag[3])
{
    fp32 gx;
    fp32 gy;
    fp32 gz;
    fp32 ax;
    fp32 ay;
    fp32 az;
    fp32 ex = 0.0f;
    fp32 ey = 0.0f;
    fp32 ez = 0.0f;

    if (quat == NULL || gyro == NULL || timing_time <= 0.0f)
    {
        return 0;
    }

    ahrs_normalize_quat(quat);

    gx = gyro[0];
    gy = gyro[1];
    gz = gyro[2];

    if (accel != NULL)
    {
        const fp32 inv_acc = ahrs_inv_norm3(accel[0], accel[1], accel[2]);
        if (inv_acc != 0.0f)
        {
            ax = accel[0] * inv_acc;
            ay = accel[1] * inv_acc;
            az = accel[2] * inv_acc;

            const fp32 qw = quat[0];
            const fp32 qx = quat[1];
            const fp32 qy = quat[2];
            const fp32 qz = quat[3];
            const fp32 vx = 2.0f * (qx * qz - qw * qy);
            const fp32 vy = 2.0f * (qw * qx + qy * qz);
            const fp32 vz = qw * qw - qx * qx - qy * qy + qz * qz;

            ex += ay * vz - az * vy;
            ey += az * vx - ax * vz;
            ez += ax * vy - ay * vx;
        }
    }

    if (mag != NULL)
    {
        const fp32 inv_mag = ahrs_inv_norm3(mag[0], mag[1], mag[2]);
        if (inv_mag != 0.0f)
        {
            fp32 yaw;
            fp32 target[4];

            get_angle(quat, &yaw, NULL, NULL);
            AHRS_init(target, accel, mag);
            ez += 0.25f * sinf(get_yaw(target) - yaw);
        }
    }

    gx += 2.0f * ex;
    gy += 2.0f * ey;
    gz += 2.0f * ez;

    gx *= 0.5f * timing_time;
    gy *= 0.5f * timing_time;
    gz *= 0.5f * timing_time;

    const fp32 qw = quat[0];
    const fp32 qx = quat[1];
    const fp32 qy = quat[2];
    const fp32 qz = quat[3];

    quat[0] += -qx * gx - qy * gy - qz * gz;
    quat[1] +=  qw * gx + qy * gz - qz * gy;
    quat[2] +=  qw * gy - qx * gz + qz * gx;
    quat[3] +=  qw * gz + qx * gy - qy * gx;
    ahrs_normalize_quat(quat);

    return 1;
}

fp32 get_yaw(const fp32 quat[4])
{
    if (quat == NULL)
    {
        return 0.0f;
    }
    return atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                  1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

fp32 get_pitch(const fp32 quat[4])
{
    if (quat == NULL)
    {
        return 0.0f;
    }

    fp32 value = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }
    return asinf(value);
}

fp32 get_roll(const fp32 quat[4])
{
    if (quat == NULL)
    {
        return 0.0f;
    }
    return atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                  1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
}

void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    if (yaw != NULL)
    {
        *yaw = get_yaw(quat);
    }
    if (pitch != NULL)
    {
        *pitch = get_pitch(quat);
    }
    if (roll != NULL)
    {
        *roll = get_roll(quat);
    }
}

fp32 get_carrier_gravity(void)
{
    return AHRS_GRAVITY_M_S2;
}
