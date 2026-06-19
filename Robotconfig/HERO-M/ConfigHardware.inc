// Included by RobotConfig.c inside g_config's initializer; do not compile directly.
    .devices = ROBOT_DEFAULT_DEVICE_TABLE,
    .motor =
        {
            .chassis =
                {
                    {MOTOR_MODEL_3508, 1u},
                    {MOTOR_MODEL_3508, 2u},
                    {MOTOR_MODEL_3508, 7u},
                    {MOTOR_MODEL_3508, 5u},
                },
            .friction =
                {
                    {MOTOR_MODEL_3510, 1u},
                    {MOTOR_MODEL_3510, 2u},
                    {MOTOR_MODEL_3510, 3u},
                    {MOTOR_MODEL_3510, 4u},
                },
            .yaw = {MOTOR_MODEL_6020, 2u},
            .yaw_upper = {MOTOR_MODEL_6020, 0u},
            .pitch = {MOTOR_MODEL_3510, 3u},
            .trigger = {MOTOR_MODEL_3510, 4u},
            .arm = {0},
        },
