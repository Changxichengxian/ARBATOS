#ifndef TEST_IMU_MOUNT_H
#define TEST_IMU_MOUNT_H

#if TEST_IMU_MOUNT == 1
/* 旧默认方向：[rawY, -rawX, rawZ]。 */
static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[1];
    out[1] = -raw[0];
    out[2] = raw[2];
}
#elif TEST_IMU_MOUNT == 2
/* HERO-M 当前方向。 */
static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[0];
    out[1] = raw[1];
    out[2] = raw[2];
}
#elif TEST_IMU_MOUNT == 3
/* 额外的正交轴置换，用来覆盖保存格式与具体车型方向解耦。 */
static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = -raw[2];
    out[1] = raw[0];
    out[2] = -raw[1];
}
#else
#error "TEST_IMU_MOUNT must be 1, 2 or 3"
#endif

#define ROBOT_IMU_FRAME_VERSION 3u

#endif
