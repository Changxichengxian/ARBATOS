// Included by config.c inside g_config's initializer; do not compile directly.
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
            .yaw =
                {
                    .model = MOTOR_MODEL_UNITREE_GO_M8010_6,
                    .can_id = 1u,
                    .protocol = MOTOR_PROTOCOL_UNITREE_RS485,
                    .control_mode = MOTOR_CONTROL_MODE_MIT,
                    .transport = MOTOR_TRANSPORT_RS485,
                    .rs485_port = 0u,
                    .baudrate = 4000000u,
                    .rx_timeout_ms = 50u,
                },
            .yaw_upper = {MOTOR_MODEL_6020, 4u},
            .pitch = {MOTOR_MODEL_6020, 3u},
            .trigger = {MOTOR_MODEL_3510, 4u},
            .arm =
                {
                    [0] = {.model = MOTOR_MODEL_DM_6215, .can_id = 1u, .can_bus = 1u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 0u, .feedback_id_enable = 1u},
                    [1] = {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 6u, .can_bus = 1u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 3u, .feedback_id_enable = 1u},
                    [2] = {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 8u, .can_bus = 1u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 4u, .feedback_id_enable = 1u},
                    [3] = {.model = MOTOR_MODEL_DM_6215, .can_id = 1u, .can_bus = 2u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 0u, .feedback_id_enable = 1u},
                    [4] = {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 6u, .can_bus = 2u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 3u, .feedback_id_enable = 1u},
                    [5] = {.model = MOTOR_MODEL_DM_J4310_2EC_V11, .can_id = 8u, .can_bus = 2u, .protocol = MOTOR_PROTOCOL_INHERIT, .control_mode = MOTOR_CONTROL_MODE_INHERIT, .transport = MOTOR_TRANSPORT_CAN, .feedback_id = 4u, .feedback_id_enable = 1u},
                },
        },
