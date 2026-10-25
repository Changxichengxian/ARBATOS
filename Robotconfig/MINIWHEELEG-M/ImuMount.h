#ifndef MINIWHEELEG_M_IMU_MOUNT_H
#define MINIWHEELEG_M_IMU_MOUNT_H

// 2026-10-24 主控侧置 90°；实测车头下压对应 raw Y 正转，使用传感器原坐标。
// 版本 3 表示 Flash 零偏存储在传感器坐标，改变安装矩阵无需改变存储格式。
#define ROBOT_IMU_FRAME_VERSION 3u

static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[0];
    out[1] = raw[1];
    out[2] = raw[2];
}

#endif
