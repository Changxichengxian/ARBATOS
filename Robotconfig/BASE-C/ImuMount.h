#ifndef BASE_C_IMU_MOUNT_H
#define BASE_C_IMU_MOUNT_H

// 沿用公共 BMI088 坐标变换；板子实际安装方向仍需按新车确认。
#define ROBOT_IMU_FRAME_VERSION 1u

static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[1];
    out[1] = -raw[0];
    out[2] = raw[2];
}

#endif
