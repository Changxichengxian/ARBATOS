#ifndef INFANTRY_TIAN_A_IMU_MOUNT_H
#define INFANTRY_TIAN_A_IMU_MOUNT_H

// 暂沿用本工程 A 板旋转，固定部件与实际朝向仍需实车核对。
#define ROBOT_IMU_FRAME_VERSION 1u

static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[1];
    out[1] = -raw[0];
    out[2] = raw[2];
}

#endif
