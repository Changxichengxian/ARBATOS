# boards

`boards/` 放“这块硬件板有什么”的内容。这里关心芯片、引脚、外设和板级启动，不关心某台车的 PID 或电机 ID。

## 当前板级目录

| Board | 芯片 | 说明 |
|---|---|---|
| `DJI_C_F407` | STM32F407 | DJI C 开发板适配 |
| `DJI_A_F427` | STM32F427 | DJI A 开发板适配 |
| `DM_MC02_H7` | STM32H723 | 达妙 MC02 H7 开发板适配 |

## 应该放这里

- 板级端口配置：UART、CAN、SPI、I2C、PWM、GPIO。
- 板载设备适配：IMU、蜂鸣器、按键、SD 卡、裁判串口等。
- 板级启动代码：只有独立板级实验入口才放 `board_main.c`、`board_freertos.c`。
- 和某块板子强绑定的端口文件，例如 `bsp_board_ports.h`、`bsp_imu_pwm_cfg.h`。

## 不应该放这里

- 某台车的 PID、电机 ID、输入映射：放 `Robotconfig/`。
- 可直接打开编译的完整 Keil 工程：放 `projects/`。
- 跨板复用的控制逻辑和协议驱动：放 `shared/`。
- 跨板复用的外设封装：放 `shared/hal/`，这里的 `bsp/` 只放具体板子的端口和配置。

如果一个文件只因为“换板子”才需要改，它通常属于这里；如果只是换车型参数，不应该动这里。
