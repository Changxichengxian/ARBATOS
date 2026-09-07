# 车型配置

`Robotconfig/` 放“这台机器人是谁”的内容。这里描述目标本身，不描述 Zephyr 工程怎么编译，也不描述某块开发板有哪些引脚。

查看[现有车型](#当前机器人配置)、[配置分层](#配置分层)、[新增车型](#新增车型)或[安装说明模板](#安装说明模板)。

## 当前机器人配置

| 配置 | 说明 | 主要文件 |
|---|---|---|
| `HERO-M` | 英雄机器人，已完整改用 MC02 H7 和 V2 副板接线 | `RobotConfig.c`、`Config*.inc`、`RobotConfig.h`、`DetectTask.c`、`PitchCaliBuiltin.c` |
| `SENTINEL-M` | 哨兵机器人接 MC02 H7 板 | `RobotConfig.c`、`Config*.inc`、`RobotConfig.h`、`DetectTask.c`、`Mc02Compat.c` |
| `MINIWHEELEG-M` | H7 接板和机械臂实验 | `RobotConfig.c`、`Config*.inc`、`RobotConfig.h`、`DetectTask.c`、`ArmMotorTable.c` |

目标目录采用扁平结构：

```text
Robotconfig/<TARGET>/
|-- RobotConfig.h
|-- RobotConfig.c
|-- ConfigOperation.inc
|-- ConfigHardware.inc
|-- ConfigTuning.inc
|-- ConfigInput.inc
|-- ConfigDiagnostics.inc
|-- DetectTask.c
`-- 目标私有补充文件
```

`RobotConfig.c` 只保留 `g_config` 总入口、调参块表和块启用判断。真正要改默认值时，按下面几个片段找：

- `ConfigOperation.inc`：运行模式、目标任务/电机、任务模块列表。
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

- 正式工程配置、显式源码清单和启动入口：放 `projects/`。
- 某块板子的串口、CAN、IMU、蜂鸣器、按键、SD 卡适配：放 `boards/`。
- 可复用控制逻辑、电机协议、输入链路、日志、诊断：放 `shared/`。
- 厂商包、参考工程、临时材料：放 `local/docs/` 或 `local/`。

判断标准很简单：如果换一台同板子的机器人也要改它，它大概率属于 `Robotconfig/`；如果换一块板子才要改它，它大概率属于 `boards/`。

## 配置分层

建议按这五类理解目标配置：

1. 运行编排：`ConfigOperation.inc` 里的 `g_config.operation` 和 `g_config.profile`，决定默认怎么跑、哪些任务静态启用。
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
| DJI A F427 / DJI C F407 | 复用 `boards/DjiAF427` / `boards/DjiCF407`，按机构参考现有车型业务配置 |
| DM MC02 H7 | `HERO-M`、`SENTINEL-M` 或 `MINIWHEELEG-M` |
| 经典底盘 + 单云台 | `HERO-M` |
| 双 yaw 云台 | `SENTINEL-M` |
| MIT 轮腿实验 | `MINIWHEELEG-M` |

A/C 板的 IMU、存储、串口、输出能力与 M 板不同，不能直接照搬 M 板引脚或安装方向。板级缺项见各板 README；新目标初期先使用已经实现的接口。

默认先复制 `Robotconfig/<OLD>/` 到 `Robotconfig/<NEW>/`，再在 `projects/<TARGET>/`、板级定义、CMake 预设和 `projects/cmake/ArbatosLegacy.cmake` 的显式清单中补齐该目标。正式构建不读取 `.uvprojx`。

已移除的旧工程如需核对，使用 `git show 951857f:<path>` 或查看 `zephyr` 分支的 `6bdf19e`，不要把它们作为新目标入口。

### 2. 填目标身份

在 `Robotconfig/<TARGET>/RobotConfig.h` 顶部写清楚：

```c
#define ARBATOS_TARGET_NAME "NEW-TARGET"
#define ARBATOS_BOARD_NAME "DmMc02H7"
```

这两个值会进入 SD 日志的 `BUILD_INFO`。以后只拿到一张 SD 卡，也能知道日志来自哪台车、哪块板。

### 3. 配 profile 和任务模块

先改 `Robotconfig/<TARGET>/ConfigOperation.inc` 里的 `.profile`。

现在 profile 只做一件事：列出这台车要启用哪些任务模块。任务创建、调参块是否显示、观测块是否显示，都按这张表走。例子：

```c
.task_module_count = 8u,
.task_modules =
    {
        ROBOT_TASK_MODULE_RC_SBUS,
        ROBOT_TASK_MODULE_HEALTH_MONITOR,
        ROBOT_TASK_MODULE_SDLOG,
        ROBOT_TASK_MODULE_CAN_COMMAND_TX,
        ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
        ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
        ROBOT_TASK_MODULE_SINGLE_GIMBAL,
        ROBOT_TASK_MODULE_IMU,
    },
```

几个规则：

- `task_module_count` 必须等于下面实际列出来的模块数量。
- 模块需同时具备编译实现和启动映射，加入列表后才会创建对应业务任务；没有列出的业务模块不启用。
- 底盘二选一：经典底盘用 `ROBOT_TASK_MODULE_CLASSIC_CHASSIS`，MIT 轮腿用 `ROBOT_TASK_MODULE_WHEELLEG_MIT`。
- 云台二选一：单云台用 `ROBOT_TASK_MODULE_SINGLE_GIMBAL`，双 yaw 云台用 `ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL`。
- 通信和服务类任务也要显式列，比如 `RC_SBUS`、`HOST_LINK`、`ELRS_LINK`、`REFEREE_RX`、`SDLOG`。
- `ROBOT_TASK_MODULE_MAX` 是模块表上限，超过它要先扩表，不要偷偷多写。

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

### 8. 配 Zephyr 工程入口

在 `projects/` 及工具入口补齐以下内容：

1. 新建 `<TARGET>/prj.conf`，需要改默认引脚或串口用途时增加 `app.overlay`；公共配置在 `projects/prj.conf`。
2. 在 `projects/Kconfig` 增加目标选项，在 `projects/src/ArbatosTarget.c` 增加对应的目标选择。当前选择器对未识别目标会报错。
3. 在 `projects/cmake/ArbatosLegacy.cmake` 增加源码和头文件目录，只接入本车的 `RobotConfig.c`、检测和板级实现，不读取 `.uvprojx`。
4. 在 `projects/CMakePresets.json` 增加配置/构建预设；A、C、M 板分别选 `dji_a_f427`、`dji_c_f407`、`dm_mc02_h7`。本机绝对路径写进不提交的 `CMakeUserPresets.json`。
5. 更新 `tools/build.ps1`、`tools/build-matrix.ps1`、`tools/CheckZephyr.py` 的车型表和对应板名；下载时也要使用该板的 OpenOCD 配置。
6. 增加新的业务任务时，补 `RobotTaskBuildConfig.h` 的编译选择，以及 `projects/src/ArbatosRuntime.c` 的固定栈、创建函数和模块映射，详见 [模块声明](../manual/开发与代码规范.md#新增任务)。

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
- SD 日志的 target、board 和配置正确；需记录 Git 身份时，构建前手动刷新 `GenBuildInfo.ps1`，并留存 ELF/BIN 与哈希，见 [日志说明](../manual/调试与日志.md#日志与复盘)。
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
