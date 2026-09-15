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
            .yaw = {MOTOR_MODEL_6020, 2u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .yaw_upper = {MOTOR_MODEL_6020, 0u},
            .pitch = {MOTOR_MODEL_3510, 5u, .can_bus = 2u},
            .trigger = {MOTOR_MODEL_3510, 4u},
            .arm = {0},
        },
    .powerMeter =
        {
            .enable = 1u,
            .canBus = 1u,
            .canId = 0x212u,
            .freshTimeoutMs = 20u,
        },
