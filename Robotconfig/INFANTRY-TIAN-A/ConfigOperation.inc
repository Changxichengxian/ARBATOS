#include "RobotTargetProfile.inc"

    // 全车运行；安全档、启动条件和失联停机沿用 HERO。
    .operation =
        {
            .mode = ROBOT_RUN_MODE_FULL,
            .target_task = ROBOT_TASK_MODULE_NONE,
            .target_motor = (uint8_t)MotorCount,
            .variant = ROBOT_RUN_VARIANT_NORMAL,
            .cali_target = ROBOT_CALI_TARGET_NONE,
        },
