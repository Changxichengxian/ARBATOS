# 当前运行层

ARBATOS 由开发板支持、车型配置、共享控制逻辑和工程入口组成。调度保持静态，设备、电机、控制器使用固定容量的配置与实例表；当前仍保留业务任务和兼容接口。

## 启动和任务

`projects/src/main.c` 进入 Zephyr 应用，`ArbatosTarget.c` 选择车型，`ArbatosRuntime.c` 完成运行初始化和任务创建。旧业务代码通过 `shared/zephyr/compat/` 使用任务、延时等兼容接口，实际由 Zephyr 调度。

任务同时受两层配置约束：

1. 工程 Kconfig、显式源码清单及 `RobotTaskBuildConfig.h` 决定哪些实现编入固件。
2. `g_config.profile.task_modules` 决定启动映射中哪些业务任务启用。

`ArbatosRuntime.c` 以 `ARB_STATIC_THREAD(...)` 分配固定栈，在 `ArbCreateModuleTasks()` 的 `moduleTasks[]` 中关联模块、句柄和创建函数，再调用 `AppCreateEnabledModuleTasks()`。新增任务必须补齐这些位置，见 [模块说明](module-system.md)。

模块声明中的周期、预算、资源和默认栈用于描述与观察，尚未全面替代实际任务创建参数。Zephyr 专用服务及准备/接收/音乐测试模式也有独立启动分支，不能只用车型任务表推断所有线程。

## 输入、控制和输出

```text
DBUS/SBUS、ELRS/CRSF、图传遥控等
  → ManualInput → ControlInput → 控制任务 / 控制器
  → LowCmd → CanTxTask → CAN / RS485 执行器

CAN 中断 → 接收队列 → CanRxTask → CanReceive / MotorInst
  → 状态快照、控制任务、Watch、SD 日志
```

控制任务写语义命令或已绑定执行器命令，协议层处理电机型号、总线、ID、帧格式和发送限额。高频循环优先读取缓存或快照，初始化时把设备名解析成 ID，避免反复查询整份配置。

## 设备和控制器

- `g_config.devices` 描述设备实例，`RobotDeviceConfig.h` 统一读取；`g_config.motor.*` 仍承载具体电机参数。
- `MotorInstRefresh()` 依据配置建立电机实例。可用 `MotorInstFindByName()` 查询，再通过绑定 ID 发命令、读反馈。
- `ControlMgr` 管理控制域、注册、资源占用、切换、停止和故障状态。默认控制器由 `RobotControlRegistry.h` 按车型配置注册。
- 底盘、云台、射击等仍有各自任务循环和状态机；不能把已有控制器接口理解为所有调度已统一。
- `g_watch.runtime` 按任务、设备、电机、控制器和控制域提供观测；SD 启动记录包含配置和设备信息，固件身份的限制见 [日志说明](sdlog.md)。

新增硬件优先补设备与驱动，新增算法优先复用控制器和输入输出接口，新增机器人优先改车型配置。确需独立周期或栈时再添加任务模块。

## 故障和验证边界

当前有安全档、输入超时、离线检测、限幅、局部故障与输出锁等保护，具体动作仍由对应控制任务和协议实现决定。实车必须逐项验证断输入、断反馈、异常恢复及停机行为。

Zephyr CAN 后端的 `BspCanFaultTx()` 当前返回失败，`BspCanZephyrFaultTxSupported()` 为 0。普通运行中的输出锁不构成“CPU 致命异常后仍能直接发停机帧”的保证。需要这项能力时，要补充经过审查和实测的芯片专用实现，并结合硬件断能措施。

`BspResetEvidence.c` 当前只使用普通静态 SRAM，启动时清空；它不保存跨重启的致命故障证据，也未提供可靠的上次 RCC/异常栈快照。不要把旧 HAL 备份 SRAM 方案写成当前 Zephyr 已实现的功能。

CAN 异常需区分配置、解析、队列、任务延时、调试暂停、总线负载和物理接线。仅凭丢帧计数不能断定硬件或软件根因。电脑上的检查与逻辑测试可验证部分代码行为，实际总线、温升、机械动作和断能效果需实物确认。

HERO-M 的整车运动、音乐、校准、RTC 和 SD 测试范围见 [验证记录](../tests/ZephyrMusicM/Validation.md)。迁移后 A/C 板有独立编译入口，其外设缺项按各板 README 和原理图核对记录处理。
