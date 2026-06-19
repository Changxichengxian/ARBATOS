// Included by RobotConfig.c inside g_config's initializer; do not compile directly.
    .devices = ROBOT_DEFAULT_DEVICE_TABLE,
    .motor =
        {
            .chassis =
                {
                    {MOTOR_MODEL_3508, 0u},
                    {MOTOR_MODEL_3508, 0u},
                    {MOTOR_MODEL_3508, 0u},
                    {MOTOR_MODEL_3508, 0u},
                },
            .friction =
                {
                    {MOTOR_MODEL_3510, 0u},
                    {MOTOR_MODEL_3510, 0u},
                    {MOTOR_MODEL_3510, 0u},
                    {MOTOR_MODEL_3510, 0u},
                },
            .yaw = {MOTOR_MODEL_6020, 0u},
            .yaw_upper = {MOTOR_MODEL_6020, 0u},
            .pitch = {MOTOR_MODEL_3510, 0u},
            .trigger = {MOTOR_MODEL_3510, 0u},
            .arm =
                {
                    {.model = MOTOR_MODEL_DM_H3510_V10, .can_id = 6u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 6u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 1u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 1u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 2u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 2u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 3u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 3u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 4u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 4u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_H3510_V10, .can_id = 5u, .can_bus = MINIWHEELEG_MIT_MOTOR_CAN_BUS, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 5u, .transport = MOTOR_TRANSPORT_CAN},
                },
        },
