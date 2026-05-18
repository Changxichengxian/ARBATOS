# shared

`shared/` 放跨板、跨车型复用的代码。这里的代码应该尽量面向抽象后的输入、状态、执行器和协议能力，不写死某台车或某块板子的细节。

## 目录分工

```text
shared/
|-- application/   # 控制任务、通信链路、输入、电机、诊断、日志
|   |-- arm/
|   |-- chassis/
|   |-- comm/
|   |-- gimbal/
|   |-- input/
|   |-- motors/
|   |-- robot/
|   |-- services/
|   |-- shoot/
|   `-- wheelleg/
|-- hal/           # 跨板复用的硬件适配实现
`-- components/    # 算法、控制器、设备驱动、基础支持库和通用类型
```

`shared/hal/` 和 `boards/<BOARD>/bsp/` 的边界：前者放多块板共用的 CAN、UART、USB、PWM 等实现；后者放某块板子的引脚、端口、设备安装方式和少量强板子相关代码。

## 应该放这里

- 能被多台车复用的控制任务：底盘、云台、射击、机械臂运动抽象。
- 输入链路：DBUS/SBUS、ELRS/CRSF、图传遥控、语义输入映射。
- 执行器和电机协议：`actuator_cmd`、`motor_instance`、`motor_model_db`、CAN/MIT/Unitree 驱动。
- 主机通信、视觉链路、裁判系统、日志、诊断观察。
- 通用算法和控制器：PID、滤波、AHRS、功率限制等。

## 不应该放这里

- 具体车型的默认参数和电机装配：放 `Robotconfig/`。
- 某块板子的引脚和端口分配：放 `boards/`。
- Keil 工程文件和 CubeMX 工程入口：放 `projects/`。

如果共享代码里必须区分车型，优先通过 `g_config`、任务族选择或电机能力表传进来，不要在共享逻辑里写死 `HERO-C`、`INFANTRY-A` 这种目标名。
