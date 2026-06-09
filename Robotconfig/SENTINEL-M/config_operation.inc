// Included by config.c inside g_config's initializer; do not compile directly.
    .profile =
        {
            .task_module_count = 8u,
            .task_modules =
                {
                    ROBOT_TASK_MODULE_RC_SBUS,
                    ROBOT_TASK_MODULE_HEALTH_MONITOR,
                    ROBOT_TASK_MODULE_SDLOG,
                    ROBOT_TASK_MODULE_CAN_COMMAND_TX,
                    ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
                    ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
                    ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL,
                    ROBOT_TASK_MODULE_IMU,
                },
        },

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
            .variant = ROBOT_RUN_VARIANT_CHASSIS_ONLY,
            .cali_target = ROBOT_CALI_TARGET_NONE,
        },
