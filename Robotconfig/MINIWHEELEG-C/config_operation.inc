// Included by config.c inside g_config's initializer; do not compile directly.
    .profile =
        {
            .task_module_count = 15u,
            .task_modules =
                {
                    ROBOT_TASK_MODULE_STARTUP_SERVICE,
                    ROBOT_TASK_MODULE_CALIBRATION,
                    ROBOT_TASK_MODULE_WHEELLEG_MIT,
                    ROBOT_TASK_MODULE_CAN_COMMAND_TX,
                    ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
                    ROBOT_TASK_MODULE_RC_SBUS,
                    ROBOT_TASK_MODULE_HEALTH_MONITOR,
                    ROBOT_TASK_MODULE_IMU,
                    ROBOT_TASK_MODULE_STATUS_LED,
                    ROBOT_TASK_MODULE_REFEREE_RX,
                    ROBOT_TASK_MODULE_HOST_LINK,
                    ROBOT_TASK_MODULE_ELRS_LINK,
                    ROBOT_TASK_MODULE_BATTERY_MONITOR,
                    ROBOT_TASK_MODULE_SERVO,
                    ROBOT_TASK_MODULE_SDLOG,
                },
        },

    .operation =
        {
            .mode = ROBOT_RUN_MODE_ENTERTAIN,
            .target_task = ROBOT_TASK_MODULE_NONE,
            .target_motor = (uint8_t)MotorCount,
            .variant = ROBOT_RUN_VARIANT_NORMAL,
            .cali_target = ROBOT_CALI_TARGET_NONE,
        },
