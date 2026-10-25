// 包含于 g_config；只参考 Ackerman_soldier 的接线，不引入它的控制实现。
    .devices = ROBOT_DEFAULT_DEVICE_TABLE,
    .motor =
        {
            // 型号暂按常见步兵装配填写，参考源码未注明实物型号；启用前确认。
            // X-drive 顺序为左前、右前、右后、左后；实车逐轮手转确认 ID 为 3、2、4、1。
            .chassis =
                {
                    {MOTOR_MODEL_3508, 3u, .can_bus = 1u},
                    {MOTOR_MODEL_3508, 2u, .can_bus = 1u},
                    {MOTOR_MODEL_3508, 4u, .can_bus = 1u},
                    {MOTOR_MODEL_3508, 1u, .can_bus = 1u},
                },
            // CAN2 0x200 的前两槽；反馈分别为 0x201、0x202。
            .friction =
                {
                    {MOTOR_MODEL_3508, 1u, .can_bus = 2u},
                    {MOTOR_MODEL_3508, 2u, .can_bus = 2u},
                    {MOTOR_MODEL_3508, 0u},
                    {MOTOR_MODEL_3508, 0u},
                },
            // 6020 的节点号 1/2 对应反馈 0x205/0x206、0x1FF 前两槽。
            .yaw = {MOTOR_MODEL_6020, 1u, .can_bus = 1u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .yaw_upper = {MOTOR_MODEL_6020, 0u},
            .pitch = {MOTOR_MODEL_6020, 2u, .can_bus = 2u, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE, .command_limit_abs = 16384},
            .trigger = {MOTOR_MODEL_2006, 7u, .can_bus = 2u},
            .arm = {0},
        },
