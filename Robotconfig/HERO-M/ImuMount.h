#ifndef HERO_M_IMU_MOUNT_H
#define HERO_M_IMU_MOUNT_H

/* 2026-09-06 实测：元件面朝上、24V侧朝后，板固定于单云台。
 * 沿用既有 HERO 控制接口：俯仰反馈取 -INS roll，yaw取INS yaw。
 * 对此安装方向，BMI088原始轴即可满足该接口；不要照搬C板旋转。 */
#define HERO_M_IMU_FRAME_VERSION 2u

static inline void ImuMountRotate(float out[3], const float raw[3])
{
    out[0] = raw[0];
    out[1] = raw[1];
    out[2] = raw[2];
}

#endif
