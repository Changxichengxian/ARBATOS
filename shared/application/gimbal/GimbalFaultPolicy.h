/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GIMBAL_FAULT_POLICY_H
#define GIMBAL_FAULT_POLICY_H

#include <stdint.h>

#include "RobotConfigTypes.h"

#define GIMBAL_FAULT_AXIS_YAW       0u
#define GIMBAL_FAULT_AXIS_YAW_UPPER 1u
#define GIMBAL_FAULT_AXIS_PITCH     2u
#define GIMBAL_FAULT_AXIS_COUNT     3u

#define GIMBAL_FAULT_MASK_YAW       (1u << GIMBAL_FAULT_AXIS_YAW)
#define GIMBAL_FAULT_MASK_YAW_UPPER (1u << GIMBAL_FAULT_AXIS_YAW_UPPER)
#define GIMBAL_FAULT_MASK_PITCH     (1u << GIMBAL_FAULT_AXIS_PITCH)

/* 只把 IMU 故障映射到当前运行变体实际依赖 IMU 的云台轴。 */
static inline uint32_t GimbalFaultImuAxisMask(robot_run_variant_e variant,
                                              uint32_t configuredMask,
                                              uint8_t yawControlIsUpper,
                                              uint8_t dualYawOutputActive)
{
    const uint32_t controlYawMask = (yawControlIsUpper != 0u) ?
                                        GIMBAL_FAULT_MASK_YAW_UPPER :
                                        GIMBAL_FAULT_MASK_YAW;
    uint32_t requiredMask = 0u;

    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        requiredMask = controlYawMask | GIMBAL_FAULT_MASK_PITCH;
        if (dualYawOutputActive != 0u && yawControlIsUpper == 0u)
        {
            requiredMask |= GIMBAL_FAULT_MASK_YAW_UPPER;
        }
        break;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY:
        requiredMask = controlYawMask;
        break;
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        requiredMask = GIMBAL_FAULT_MASK_PITCH;
        break;
    default:
        break;
    }

    return requiredMask & configuredMask;
}

/* 射击就绪只依赖当前控制 yaw 与 pitch；辅助 yaw 单轴故障不扩大成发射域停机。 */
static inline uint32_t GimbalFaultAimAxisMask(robot_run_variant_e variant,
                                              uint32_t configuredMask,
                                              uint8_t yawControlIsUpper)
{
    const uint32_t controlYawMask = (yawControlIsUpper != 0u) ?
                                        GIMBAL_FAULT_MASK_YAW_UPPER :
                                        GIMBAL_FAULT_MASK_YAW;
    uint32_t requiredMask = 0u;

    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        requiredMask = controlYawMask | GIMBAL_FAULT_MASK_PITCH;
        break;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
        requiredMask = controlYawMask;
        break;
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        requiredMask = GIMBAL_FAULT_MASK_PITCH;
        break;
    default:
        break;
    }
    return requiredMask & configuredMask;
}

#endif
