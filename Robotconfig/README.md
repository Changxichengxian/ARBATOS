# 车型配置

`Robotconfig/` 放“这台机器人怎么装配”的内容。这里选择开发板、控制器、服务和车型专属实现，填写接线与参数；公共构建规则和板级驱动无需随车型复制。

查看[现有车型](#当前机器人配置)、[配置分层](#配置分层)、[新增车型](#新增车型)或[安装说明模板](#安装说明模板)。

## 当前机器人配置

| 配置 | 说明 | 主要文件 |
|---|---|---|
| `HERO-M` | 英雄机器人，已完整改用 MC02 H7 和 V2 副板接线 | `RobotConfig.toml`、`RobotConfig.c`、`Config*.inc`、`DetectTask.c`、`PitchCaliBuiltin.c` |
| `SENTINEL-M` | 哨兵机器人接 MC02 H7 板 | `RobotConfig.c`、`Config*.inc`、`RobotConfig.h`、`DetectTask.c`、`Mc02Compat.c` |
| `MINIWHEELEG-M` | H7 接板和机械臂实验 | `RobotConfig.c`、`Config*.inc`、`RobotConfig.h`、`DetectTask.c`、`ArmMotorTable.c` |

目标目录采用扁平结构：

```text
Robotconfig/<TARGET>/
|-- RobotConfig.h
|-- RobotConfig.toml
|-- RobotConfig.c
|-- ConfigOperation.inc
|-- ConfigHardware.inc
|-- ConfigTuning.inc
|-- ConfigInput.inc
|-- ConfigDiagnostics.inc
|-- DetectTask.c
`-- 目标私有补充文件
```

`RobotConfig.c` 只保留 `g_config` 总入口。调参块表和块启用判断共用 `shared/application/robot/RobotConfigBlocks.c`。真正要改默认值时，按下面几个片段找：

- `ConfigOperation.inc`：运行模式和目标任务/电机；任务模块列表由 `RobotConfig.toml` 生成。
- `ConfigHardware.inc`：设备表和电机装配。
- `安装说明.md`：控制板固定在哪个机械部件上、开发板朝向、INS 姿态代表谁。
- `ConfigTuning.inc`：云台、底盘、轮腿、射击、功率、IMU、电压、蜂鸣器、LED 参数。
- `ConfigInput.inc`：输入源策略、遥控/键鼠/图传映射。
- `ConfigDiagnostics.inc`：在线检测、AUX 实时遥测、SD 日志默认值。

## 应该放这里

- 默认参数和车型配置：`g_config`、PID、限位、输入映射和任务模块选择。
- 轴电机装配：哪个轴用什么电机、哪个 CAN ID、正反方向、反馈 ID。
- 机械安装坐标：控制板固定在底盘、云台、大 yaw、轮腿本体还是其他部件上，开发板 `+X/+Y/+Z` 朝哪里。
- 目标在线检测：这台车关心哪些设备、哪些离线算故障。
- 目标私有的小补丁：例如某个目标不接 USB 主机链路，就放对应空实现。
- 目标专属装配表：例如 `MINIWHEELEG-M` 的机械臂关节表。

## 不应该放这里

- 公共构建规则和启动入口：放 `projects/`；车型自己的额外源文件在 TOML 的 `[build]` 登记。
- 某块板子的串口、CAN、IMU、蜂鸣器、按键、SD 卡适配：放 `boards/`。
- 可复用控制逻辑、电机协议、输入链路、日志、诊断：放 `shared/`。
- 厂商手册和资料包放 `local/docs/`，参考工程放 `local/reference/`，临时生成文件放 `local/cache/`。

判断标准很简单：如果换一台同板子的机器人也要改它，它大概率属于 `Robotconfig/`；如果换一块板子才要改它，它大概率属于 `boards/`。

## 配置分层

建议按这五类理解目标配置：

1. 运行编排：`ConfigOperation.inc` 里的 `g_config.operation` 决定默认怎么跑；`RobotConfig.toml` 选择哪些任务静态启用。
2. 硬件装配：`ConfigHardware.inc` 里的 `g_config.devices` 和 `g_config.motor`，决定有哪些设备、电机怎么接；`安装说明.md` 决定控制板和主要部件装在哪里、朝哪里。
3. 控制参数：`ConfigTuning.inc`，放云台、底盘、射击、功率、轮腿、机械臂等 PID、限幅和几何参数。
4. 输入映射：`ConfigInput.inc`，放输入源策略、遥控通道、语义开关和安全档。
5. 诊断记录：`ConfigDiagnostics.inc`，放在线检测、AUX 实时遥测、SD 日志默认值。

`operation` 是当前运行方式入口。任务仍按 `profile.task_modules` 静态创建，`operation` 只决定谁允许输出。常用组合：

- 全任务正常：`mode=ROBOT_RUN_MODE_FULL`，`variant=ROBOT_RUN_VARIANT_NORMAL`。
- 单任务：`mode=ROBOT_RUN_MODE_SINGLE_TASK`，`target_task` 填一个 `ROBOT_TASK_MODULE_*`。
- 单电机：`mode=ROBOT_RUN_MODE_SINGLE_MOTOR`，`target_motor` 填 `MotorId`。
- 校准：`mode=ROBOT_RUN_MODE_CALIBRATION`，`cali_target` 填校准对象。
- 娱乐/演示：`mode=ROBOT_RUN_MODE_ENTERTAIN`。

AUX 临时调参只改 RAM，重启后恢复配置文件默认值。运行模式只决定业务输出，不会自动开放 Flash 写入；M 板校准仅专用准备模式可保存，正式固件只读，A/C 板持久校准尚未完成。

## 新增车型

新车接入先追求“能证明每一步对”，不要一开始就追求手感和完整功能。最小目标是：能编译、能下载、能观察输入和反馈、能安全地让单个子系统动起来。

### 1. 选基准

先找最像的现有目标，不要从零开始。

| 新目标条件 | 优先复制 |
|---|---|
| DJI A F427 / DJI C F407 | 当前仅保留板级验证；完整机器人运行栈尚未完成迁移 |
| DM MC02 H7 | `HERO-M`、`SENTINEL-M` 或 `MINIWHEELEG-M` |
| 经典底盘 + 单云台 | `HERO-M` |
| 双 yaw 云台 | `SENTINEL-M` |
| MIT 轮腿实验 | `MINIWHEELEG-M` |

A/C 板的 IMU、存储、串口、输出能力与 M 板不同，生成器会拒绝将当前完整运行栈装到 A/C 板上。板级缺项见各板 README。

默认用生成器从 `Robotconfig/<OLD>` 创建 `Robotconfig/<NEW>`；车型接入不再要求修改 `projects/`、CMake 预设或构建工具。正式构建不读取 `.uvprojx`。

已移除的旧工程如需核对，使用 `git show 951857f:<path>` 或查看 `zephyr` 分支的 `6bdf19e`，不要把它们作为新目标入口。

### 2. 创建目标身份

运行 `pwsh -File tools/build.ps1 -Action new -Project NEW-TARGET -From HERO-M`。目录名就是车型名，开发板由 TOML 的 `board` 选择；身份宏、日志身份和构建配置自动生成，不再手改头文件。新副本默认停在未选电机的单电机模式，接线和参数检查完后再修改 `ConfigOperation.inc`。

### 3. 配 profile 和任务模块

在 `RobotConfig.toml` 选择需要的服务和控制器。例如：

```toml
schema = 1
board = "dm_mc02_h7"
profile = "custom"
services = ["RC_SBUS", "HEALTH_MONITOR", "SDLOG"]

[controllers.chassis]
type = "classic"

[controllers.gimbal]
type = "single"
```

几个规则：

- CAN 收发、IMU 等依赖按控制器选择自动加入，任务数量、编译开关和启动表自动保持一致。
- 同一控制域只能选一种实现；经典底盘与轮腿、两种云台等冲突会提前报错。
- 栈和优先级有公共默认值，确需覆盖时用 `[tasks.<任务名>]`，无需修改启动源码。
- 未完成当前平台迁移的服务会明确报错。M 板 `ELRS_LINK` 已支持外置 CRSF 接收机，需在 `[ports]` 选口；`SERVO` 已支持四路 PWM，需填写 `.servo` 通道表，未配置时全部关闭。`WHEELLEG_SERVO` 仍未接入。
- 现有任务容量为 16，超过会报错，不会截断列表。

新车先少开模块：输入、在线检测、CAN 收发、IMU，加一个要调的子系统。确认能跑后再把日志、遥测、裁判、发射等模块补上。

### 4. 配安装坐标

将本页末尾的[安装说明模板](#安装说明模板)复制到 `Robotconfig/<TARGET>/安装说明.md`，把这几件事写清：

- 控制板固定在哪个机械部件上：底盘、大 yaw、小 yaw、云台、轮腿本体，还是其他位置。
- 控制板 `+X/+Y/+Z` 分别朝固定部件的哪个方向。
- INS yaw/pitch/roll 主要代表哪个部件。
- 算法接口里的 `q[4]` 和 `frame=2` 在这台车上该怎么理解。

这一步和电机 ID 一样重要。没确认前写“待实车确认”，不要把猜测写成结论。

### 5. 配电机装配

看 `Robotconfig/<TARGET>/ConfigHardware.inc` 里的 `.motor`。新车第一次上电前至少确认：

- 每个轴的 `model` 是否正确。RM 电流电机、达妙 MIT、宇树电机不要混填。
- `can_id`、`can_bus`、`feedback_id` 是否和实物一致；RS485 电机还要确认串口、波特率和超时。
- 不用的轴必须设成 `can_id = 0`，否则后面会被当成有效轴参与状态判断或输出。
- 协议和控制模式要匹配：RM 电流闭环走电流命令，达妙 / 宇树 MIT 轴走力矩、位置速度等协议命令。
- 限幅先保守：电流、力矩、速度、位置范围都先给小值，确认方向和反馈后再放开。
- 双 yaw 云台要分清 `yaw`、`yaw_upper`、`pitch`；`yaw_upper` 是上 yaw，不是 pitch。

不要靠改控制任务里的符号来修电机方向。底盘方向优先改底盘方向配置；云台方向优先改 `yaw_turn`、`pitch_turn` 或安装矩阵。

第一次验证顺序：

1. 只看反馈，不给输出：手转电机，看反馈 ID、速度方向、温度是否对应。
2. 开单轴小输出：每次只允许一个轴动作，确认正方向。
3. 再接入控制闭环：先低限幅、短时间动作，确认没有反向追飞。
4. 最后才恢复正常限幅和多轴联动。

### 6. 配输入和安全档

输入分两层：

- `manual_input`：输入源选择、超时、合并策略，例如 DBUS、ELRS、图传遥控。
- `input`：把通道映射成语义轴和语义开关。

新车必须先确认安全档。安全档不仅影响控制任务，也影响 IMU 零偏微调能不能开始。上车前要确认：

- 遥控器断开时不会产生有效运动命令。
- 安全档时底盘、云台、射击输出都能停。
- 输入源切换不会突然跳到另一个非零命令。

### 7. 配 DetectTask

`Robotconfig/<TARGET>/DetectTask.c` 是这台车的在线检测表。不要为了让灯变绿就关检测。

优先加这些检测：

- 遥控器 / ELRS / 图传输入。
- 底盘电机、云台电机、拨盘、摩擦轮。
- IMU、裁判系统、视觉链路。
- SD 卡、主机链路、关键板级外设。

验收标准不是“没有红灯”，而是设备拔掉时能正确变红，插回去能恢复。

配置时按模块来想：

- `RC_SBUS` 或 `ELRS_LINK` 开了，就要能判断对应输入是否在线。
- `CLASSIC_CHASSIS` 开了，就要关心底盘电机反馈。
- `SINGLE_GIMBAL` 或 `DUAL_YAW_GIMBAL` 开了，就要关心云台关键电机和 IMU。
- `CAN_FEEDBACK_RX` 开了，就要确认 CAN 接收计数和电机刷新都在动。
- `SDLOG`、`HOST_LINK`、`REFEREE_RX` 这类服务模块，至少要能在 `g_watch` 或遥测里看到状态。

不要把还没接线的设备硬塞进检测表。实物没接就先不列对应任务模块或不启用对应检测；等接线确定后再补。这样上车时红灯才有意义。

### 8. 生成构建配置

车型能力只写在 `Robotconfig/<TARGET>/RobotConfig.toml`。它选择开发板、服务、内置控制链、可选控制器、专属源文件和 overlay；不再去 `projects/`、Kconfig、CMake 预设或构建脚本重复登记车型。

从最接近的车型创建目录：

```powershell
pwsh -File .\tools\build.ps1 -Action new -Project NEW -From HERO-M
pwsh -File .\tools\build.ps1 -Action check -Project NEW
pwsh -File .\tools\build.ps1 -Action presets -Project all
```

`new` 只创建 `Robotconfig/NEW` 下的声明和车型文件。`presets` 只补齐缺少的本机 `CMakeUserPresets.json` 项，不覆盖已有个人预设。生成器会在构建目录生成 `RobotTargetConfig.h`、任务表、profile、CMake 输入、`robot.conf` 和 `robot-config.json`；这些文件不能手改。`robot-config.json` 会列出选中的任务、依赖、优先级和栈，先看它再处理配置报错。

`RobotConfig.toml` 只做装配选择；`Config*.inc` 仍放接线、电机、PID、限幅、输入和安装参数。任务编号、名称、默认优先级、默认栈、入口和任务实现源只在 `shared/application/robot/RobotTaskCatalog.def` 维护。当前 classic、single、dual_yaw、mit、arm 仍复用已经存在的控制链，新车型不表示新板已自动支持，也不代表已经实车验证。

可复用的新底盘或云台算法放 `shared/controllers/<name>/`，其中 `Controller.toml` 声明参数默认值、范围、依赖和源码，`.c/.h` 放实现。生成器会自动发现它；车型在 TOML 里按电机实例名绑定。算法层 `ControlAlgorithm` 只算控制量，`ControlRuntime` 负责输入、反馈、安全检查和提交，所有输出仍必须经过 `LowCmd` 的许可、急停和限幅，不能由控制器直接绕过。

独立板级验证可先使用 [tests/Boards](../tests/README.md)，不必恢复已经删除的 A/C 旧车型。

### 9. 第一次检查

在仓库根目录先跑：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check -Project <TARGET>
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action build -Project <TARGET> -Pristine
```

前者检查 Zephyr 源清单、正式配置和语法路径；后者从干净目录构建目标。它们不等价于实车验收。

### 最小验收

新目标至少满足这些，才算初步接起来：

- `tools/build.ps1 -Action check -Project <TARGET>` 通过。
- 对应 Zephyr 目标从干净目录构建通过。
- SD 日志的 target、board 和配置正确；正常构建自动刷新 Git 身份，留存 ELF/BIN 与哈希，见 [日志说明](../manual/调试与日志.md#日志与复盘)。
- `g_watch` 能看到任务状态和主要设备状态。
- 遥控输入、CAN 反馈、IMU 姿态都能观察。
- 每个子系统都能单独关闭或单独测试。
- 上车检查清单有记录，不只靠口头记忆。

## 安装说明模板

复制下面的内容到新车型的 `安装说明.md`，按实际安装填写；没有确认的方向保留“待实车确认”。

````markdown
# <TARGET> 安装说明

这张表说明控制板和主要部件固定在哪个机械坐标上。坐标总口径见仓库的 `manual/算法协议与坐标.md`。

## 当前结论

| 项目 | 当前定义 |
| --- | --- |
| 控制板 | 待填写 |
| 控制板固定部件 | 底盘 / 大 yaw / 小 yaw / 云台 / 轮腿本体 / 其他 |
| INS 姿态主要代表 | 待填写 |
| 底盘坐标 | `+X` 车体正前，`+Y` 车体左侧，`+Z` 向上 |
| 云台坐标 | 有云台时填写；无云台写“当前目标没有云台任务” |
| 算法 `q[4]` | 写清楚它更接近哪个部件的姿态 |
| 算法 `frame=2` | 写清楚 yaw 零点来自哪里，以及是否建议算法使用 |

## 机械层级

```text
场地
  -> 底盘车体
      -> ...
```

## 控制板方向

| 板轴 | 当前记录 |
| --- | --- |
| 板 `+X` | 待实车确认 |
| 板 `+Y` | 待实车确认 |
| 板 `+Z` | 待实车确认 |

## 待确认项

- 控制板固定在哪个机械部件上。
- 控制板 `+X/+Y/+Z` 相对固定部件的具体方向。
- 云台 yaw 零位和底盘正前是否重合。
- 相机光轴、枪管方向和云台 `+X` 是否一致。
- 上电或融合重置时，`frame=2` 的场地 `+X` 怎么确定。
````
