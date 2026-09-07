# 可复用底盘和云台

这里放“同一种机构怎样运动”的控制算法。车型在 `Robotconfig/<车型>/RobotConfig.toml` 选择算法、绑定电机和填写参数。别人复用时无需修改启动入口、工程源码清单或 CAN 驱动。

## 为什么分成两部分

- `ControlAlgorithm` 是控制算法：接收语义输入和电机反馈，计算这一帧的电机命令。它只依赖标准 C，方便在电脑上测试或接入其他系统。
- `ControlRuntime` 是共用执行流程：在初始化时按名字绑定电机，每周期检查输入、反馈和输出权限，运行算法，再通过原有 `MotorInst → LowCmd → CanTxTask` 发送。失联、急停和过期命令的处理由这里统一承担。

这样拆开是为了让新作者集中写机构逻辑，同时保持所有控制器使用相同的安全出口。`ControlMgr` 继续管理谁有权控制这些电机，并未再增加另一套控制权。

读取路径：`RobotConfig.toml` → 生成的 `RobotControlSelection.h` → `RobotControlRegistry.h` → `ControlRuntime.c` → 本目录的算法 `step()` → `MotorInst / LowCmd`。生成文件在 `local/build/<车型>/generated/robotconfig/`，可阅读，不手改。

## 选用已有算法

例如把一种双轮差速底盘接到现有电机装配：

```toml
[controllers.chassis]
type = "differential"
motors = ["motor.chassis0", "motor.chassis1"] # 顺序：左轮、右轮
period_ms = 2

[controllers.chassis.parameters]
track_width_m = 0.40
wheel_radius_m = 0.076
gear_ratio = 19.0
max_current = 2000.0
```

电机名字对应同目录 `ConfigHardware.inc` 中的设备。总线、ID、型号、方向和限额继续在那里设置；算法内不写总线号或 CAN ID。物理安装的正反方向必须实测核对。

| 类型 | 用途 | 目前边界 |
|---|---|---|
| `differential` | 两轮差速运动学与转速比例控制 | 使用 RM 转速和电流命令；示例未做实车调参，不含底盘功率限制 |
| `speed_gimbal` | yaw、pitch 两轴转速控制 | 顺序为 yaw、pitch；使用 RM 转速与电流命令，不包含姿态保持或机械限位策略 |

这两个示例用于证明接入与复用路径。现有 `classic`、`single`、`dual_yaw`、`mit` 仍保留原有控制逻辑；不会因为增加示例就替换正式车型算法。

现有 `shoot.rm` 仍由 `single/dual_yaw` 云台任务调用，不能与新云台直接组合；生成器会拒绝这种没有发射执行入口的配置。使用新云台时先关闭 `shoot`，独立发射任务还需另行迁移。

## 写一种新控制器

1. 复制最接近的算法目录到 `shared/controllers/<名字>/`。
2. 在 `Controller.toml` 写明唯一名字、`chassis` 或 `gimbal`、头文件、导出符号、源码、依赖任务和输出数量。
3. 实现 `configure/reset/step` 和一个 `const ControlAlgorithm`。默认值、范围只写在 `Controller.toml`；生成的 `<导出符号>Params.h` 提供参数定义和具名下标，调整参数顺序不会改变语义。
4. 在车型 TOML 的 `controllers` 选这个名字。构建器自动发现并加入源文件，不修改核心枚举、任务表或工程清单。

`step()` 输入的五个语义轴是归一化的 `[-1, 1]`：前后、横移、旋转、云台 yaw、云台 pitch。电机数组顺序与 `motors` 一致；`speedRpm/current` 是 RM 反馈字段，`q/dq/tau` 是相应协议提供的位置、速度和力矩字段，不能假设所有协议都提供相同反馈。算法输出不含电机 ID，索引只能指向声明的绑定，不能写其他轴。

需要姿态反馈时，在控制器的 `Controller.toml` 声明 `requires = ["IMU"]`。执行流程会提供同一次采样的 `input->imu`，包括四元数、角速度（弧度/秒）、yaw/roll/pitch（弧度）、数据年龄和有效标记；超时或非有限数值会阻止输出。算法无需包含 `InsTask.h` 或调用传感器驱动。坐标以车型安装说明和 IMU 安装变换为准。未声明该依赖时，`imu.valid` 为 0。

算法不得延时、访问总线、申请动态内存或绕过统一输出接口。可变状态放在 runtime 提供的固定容量状态区，`stateBytes` 超限会拒绝配置。运行周期不是可随意更改的性能承诺，需结合控制稳定性和总线负载测试。

构建目录的 `robot-config.json` 列出解析后的任务、依赖来源、栈、优先级、控制器参数和源码。配置拼错或冲突时在编译前报错；设备绑定是否存在、是否重复以及实际发送能力还会在启动时检查。

## 验证

在根目录运行 `pwsh -File tools/tests/RunTests.ps1 -Name ControllerAlgorithms`，在电脑上编译运行同一份算法；将名称换成 `ControlRuntime` 可验证共用安全出口。正式接入还需要单轴、方向、限位、断反馈和长时负载验证；主机测试与固件编译不代替这些实物检查。

当前完整固件装配支持 MC02 H7。A、C 板定义继续保留，但其运行栈尚未完成相同迁移，生成器会明确拒绝该组合。标准 C 算法可以独立复用；迁移整套机器人固件仍需要对应系统的输入、反馈和输出适配。
