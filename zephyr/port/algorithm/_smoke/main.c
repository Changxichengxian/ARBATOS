#include "AHRS.h"
#include "AhrsMiddleware.h"
#include "arm_math.h"

int main(void)
{
    float32_t quat[4];
    const float32_t accel[3] = {0.0f, 0.0f, 9.80665f};
    const float32_t gyro[3] = {0.0f, 0.0f, 0.0f};
    const float32_t mag[3] = {1.0f, 0.0f, 0.0f};
    float32_t yaw;
    float32_t pitch;
    float32_t roll;

    AHRS_init(quat, accel, mag);
    (void)AHRS_update(quat, 0.001f, gyro, accel, mag);
    get_angle(quat, &yaw, &pitch, &roll);

    volatile float32_t result =
        arm_sin_f32(yaw) +
        arm_cos_f32(pitch) +
        AHRS_sinf(roll) +
        AHRS_cosf(roll) +
        get_carrier_gravity();
    return (result == 0.0f) ? 1 : 0;
}
