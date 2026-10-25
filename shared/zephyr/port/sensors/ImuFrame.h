#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <stdint.h>
#include "RobotTargetConfig.h"
#if ROBOT_IMU_MOUNT_CONFIGURED
#include "ImuMount.h"
#endif

static inline uint32_t ImuFrameVersion(void)
{
#if ROBOT_IMU_MOUNT_CONFIGURED
    return ROBOT_IMU_FRAME_VERSION;
#else
    return 1u;
#endif
}

static inline void ImuFrameRotate(float out[3], const float raw[3])
{
#if ROBOT_IMU_MOUNT_CONFIGURED
    ImuMountRotate(out, raw);
#else
    float x = raw[0], y = raw[1], z = raw[2];
    out[0] = y;
    out[1] = -x;
    out[2] = z;
#endif
}

/* 安装矩阵为正交轴置换；转置即逆矩阵。Flash 保存传感器坐标，换安装方向仍可复用零偏。 */
static inline void ImuFrameUnrotate(float raw[3], const float body[3])
{
    for (unsigned axis = 0; axis < 3; ++axis) {
        float basis[3] = {0}, column[3];
        basis[axis] = 1.0f;
        ImuFrameRotate(column, basis);
        raw[axis] = column[0] * body[0] + column[1] * body[1] + column[2] * body[2];
    }
}

/* 旧参数是在 [rawY,-rawX,rawZ] 中保存的。先还原传感器坐标，
 * 再用当前安装矩阵转换；角度和零偏必须使用同一个坐标系。 */
static inline int ImuFrameBiasConvert(float out[3], const float bias[3], uint32_t version)
{
    float raw[3];
    if (version == 1u) {
        raw[0] = -bias[1];
        raw[1] = bias[0];
        raw[2] = bias[2];
    } else if (version == 2u || version == 3u) {
        raw[0] = bias[0];
        raw[1] = bias[1];
        raw[2] = bias[2];
    } else {
        return 0;
    }
    ImuFrameRotate(out, raw);
    return 1;
}

#endif
