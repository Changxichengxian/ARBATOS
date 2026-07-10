#ifndef ROBOT_LIFECYCLE_TEST_ROBOT_CONFIG_H
#define ROBOT_LIFECYCLE_TEST_ROBOT_CONFIG_H

#include <stdint.h>

typedef struct
{
    uint8_t GimbalSafePos;
    uint8_t ChassisSafePos;
    uint8_t ChassisFollowPos;
    uint8_t ChassisSpinPos;
    uint8_t ShootStopPos;
    uint8_t ShootReadyPos;
    uint8_t ShootFirePos;
    uint8_t image_vt13_shoot_switch_input;
} ManualInputSemanticsConfig;

typedef struct
{
    ManualInputSemanticsConfig semantics;
} RobotLifecycleTestManualInputConfig;

typedef struct
{
    RobotLifecycleTestManualInputConfig manual_input;
    struct
    {
        uint8_t gimbalModeInvert;
    } input;
} RobotLifecycleTestConfig;

extern RobotLifecycleTestConfig g_config;

#endif
