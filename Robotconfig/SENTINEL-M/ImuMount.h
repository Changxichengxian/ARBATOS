#ifndef SENTINEL_M_IMU_MOUNT_H
#define SENTINEL_M_IMU_MOUNT_H

// 沿用当前默认安装方向；实际安装仍需实车确认。
#define ROBOT_IMU_FRAME_VERSION 3u

static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[1];
    out[1] = -raw[0];
    out[2] = raw[2];
}

#endif
