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
|-- zephyr/        # 当前共用系统适配、兼容层与外设实现
|-- hal/           # 历史 HAL 接口与实现参考
`-- components/    # 算法、控制器、设备驱动、基础支持库和通用类型
```

`shared/zephyr/port/` 放当前系统上的共用外设适配；`shared/hal/` 保留接口与历史实现参考。`boards/<BOARD>/` 放具体板子的引脚、设备树、端口和传感器选择，车型安装方向放 `Robotconfig/`。

## 常用服务入口

- `application/services/calibration/`：校准服务。`GyroZeroCali.h` 放陀螺仪零偏采样状态机，板级 INS 只负责传入旋转函数、保存函数和安全条件；`CalibrateTask.c` 保留传统设备校准流程，持久保存能力取决于当前板级后端；`PitchCali.c` 负责 pitch 补偿校准。
- `application/services/diagnostics/`：运行观察和故障状态。优先看 `Watch.c`、`RtProf.c`。
- `application/services/storage/`：TF/SD 日志。高频任务写日志前要先考虑频率和数据量；使用和留样规则见 `../manual/sdlog.md`。
- `application/services/startup/`：启动期服务、状态灯和提示输出。

## 任务实现拆分约定

高频任务或跨车型任务如果单文件过大，可以把同一个任务内部的辅助实现拆成同目录的私有 `.inc` 文件，再由原来的 `.c` 文件 `#include` 回来。Zephyr 显式源码清单仍只编译原来的 `.c`，函数的 `static` 范围也不变。

- `.inc` 只放这个任务私有的实现块，不要加入工程源文件列表，也不要被其他 `.c` 直接包含。
- 任务入口函数尽量留在原 `.c` 里，主循环一眼能看到；日志、快照、调参、协议打包、状态发布、控制辅助等可以按功能拆出去。
- 新增拆分后要跑 `tools/build.ps1 -Action check`。该检查目前验证构建配置、板级文件与源码引用；高频任务和故障策略要运行对应的主机回归测试。
- 如果某段代码已经能被多个任务复用，优先抽成正常的 `.c/.h` 模块；`.inc` 只用于降低单个任务文件的阅读负担。

## 应该放这里

- 能被多台车复用的控制任务：底盘、云台、双 yaw 云台、射击、轮腿、机械臂运动抽象。
- 输入链路：DBUS/SBUS、ELRS/CRSF、图传遥控、语义输入映射。
- 外部运动意图：算法、主机或后续链路给底盘的 `vx/vy/wz` 目标先统一进 `application/robot/ExternalMotionIntent.c`，底盘不要直接依赖具体串口包。
- 执行器和电机协议：`LowCmd`、`MotorInst`、`MotorModelDb`、CAN/MIT/Unitree 驱动。
- 主机通信、视觉链路、裁判系统、日志、诊断观察。
- 通用算法、校准状态机和控制器：PID、滤波、AHRS、陀螺零偏采样、功率限制等。

## 不应该放这里

- 具体车型的默认参数和电机装配：放 `Robotconfig/`。
- 某块板子的引脚和端口分配：放 `boards/`。
- 正式工程配置、显式源码清单和启动入口：放 `projects/`。

如果共享代码里必须区分车型，优先通过 `g_config`、任务模块选择或电机能力表传进来，不要在共享逻辑里写死目标名。
