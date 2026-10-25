#include "RobotTargetProfile.inc"

    // 新模板不选择输出电机，装配确认后再修改运行模式。
    .operation =
        {
            .mode = ROBOT_RUN_MODE_SINGLE_MOTOR,
            .target_task = ROBOT_TASK_MODULE_NONE,
            .target_motor = (uint8_t)MotorCount,
            .variant = ROBOT_RUN_VARIANT_NORMAL,
            .cali_target = ROBOT_CALI_TARGET_NONE,
        },
