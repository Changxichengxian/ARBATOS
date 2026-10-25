// C 板模板装配；总线、地址和转向须与新车实物核对。
    .devices = ROBOT_DEFAULT_DEVICE_TABLE,
    .motor = {
            .chassis =
                {
                    {.model = MOTOR_MODEL_3508, .can_id = 1u, .can_bus = 1u},
                    {.model = MOTOR_MODEL_3508, .can_id = 2u, .can_bus = 1u},
                    {.model = MOTOR_MODEL_3508, .can_id = 3u, .can_bus = 1u},
                    {.model = MOTOR_MODEL_3508, .can_id = 4u, .can_bus = 1u},
                },
            .friction =
                {
                    {.model = MOTOR_MODEL_3510, .can_id = 1u, .can_bus = 2u},
                    {.model = MOTOR_MODEL_3510, .can_id = 2u, .can_bus = 2u},
                    {.model = MOTOR_MODEL_3510, .can_id = 0u, .can_bus = 2u},
                    {.model = MOTOR_MODEL_3510, .can_id = 0u, .can_bus = 2u},
                },
            .yaw = {.model = MOTOR_MODEL_6020, .can_id = 1u, .can_bus = 2u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .yaw_upper = {.model = MOTOR_MODEL_6020, .can_id = 0u, .can_bus = 2u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .pitch = {.model = MOTOR_MODEL_6020, .can_id = 2u, .can_bus = 2u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .trigger = {.model = MOTOR_MODEL_2006, .can_id = 3u, .can_bus = 2u},
            .arm =
                {
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                    {.model = MOTOR_MODEL_DM_J4310_2EC_V12, .can_id = 0u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_MIT, .master_id = 0u, .transport = MOTOR_TRANSPORT_CAN},
                },
        },
