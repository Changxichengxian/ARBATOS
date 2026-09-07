// Included by RobotConfig.c inside g_config's initializer; do not compile directly.
#include "RobotTargetProfile.inc"

    .operation =
        {
            // mode options:
            //   ROBOT_RUN_MODE_FULL         - normal full run
            //   ROBOT_RUN_MODE_SINGLE_TASK  - only .target_task is released
            //   ROBOT_RUN_MODE_SINGLE_MOTOR - only .target_motor output is released
            //   ROBOT_RUN_MODE_CALIBRATION  - calibration flow
            //   ROBOT_RUN_MODE_ENTERTAIN    - entertainment/demo
            .mode = ROBOT_RUN_MODE_FULL,
            .target_task = ROBOT_TASK_MODULE_NONE,
            .target_motor = (uint8_t)MotorCount,
            .variant = ROBOT_RUN_VARIANT_NORMAL,
            .cali_target = ROBOT_CALI_TARGET_NONE,
        },
