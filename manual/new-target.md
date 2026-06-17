# 新车接入流程

新车接入先追求“能证明每一步对”，不要一开始就追求手感和完整功能。最小目标是：能编译、能下载、能观察输入和反馈、能安全地让单个子系统动起来。

## 1. 选基准

先找最像的现有目标，不要从零开始。

| 新目标条件 | 优先复制 |
|---|---|
| DJI C 板 | `HERO-C` 或 `MINIWHEELEG-C` |
| DJI A 板 | `INFANTRY-A` 或 `CARRIER-A` |
| DM MC02 H7 | `HERO-M`、`SENTINEL-M` 或 `MINIWHEELEG-M` |
| 经典底盘 + 单云台 | `HERO-C` / `INFANTRY-A` |
| 双 yaw 云台 | `SENTINEL-M` |
| MIT 轮腿实验 | `MINIWHEELEG-M` / `MINIWHEELEG-C` |

复制时通常要复制两块：

- `Robotconfig/<OLD>/` 到 `Robotconfig/<NEW>/`
- `projects/<OLD>/` 到 `projects/<NEW>/`

然后改 Keil 工程名、输出名、include path、source group 里的 Robotconfig 路径。

GCC/CMake 路线不需要单独复制一套工程清单。它会从新目标的 `.uvprojx` 生成 `build/gcc/<TARGET>/`，所以先把 Keil 工程里的文件列表、宏、头文件路径、启动文件和 scatter 文件改对。

## 2. 填目标身份

在 `Robotconfig/<TARGET>/config.h` 顶部写清楚：

```c
#define ARBATOS_TARGET_NAME "NEW-TARGET"
#define ARBATOS_BOARD_NAME "DJI_C_F407"
```

这两个值会进入 SD 日志的 `BUILD_INFO`。以后只拿到一张 SD 卡，也能知道日志来自哪台车、哪块板。

## 3. 配 profile 和任务模块

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
- 列了模块，任务入口才会创建对应任务；没列就视为这台车不用它。
- 底盘二选一：经典底盘用 `ROBOT_TASK_MODULE_CLASSIC_CHASSIS`，MIT 轮腿用 `ROBOT_TASK_MODULE_WHEELLEG_MIT`。
- 云台二选一：单云台用 `ROBOT_TASK_MODULE_SINGLE_GIMBAL`，双 yaw 云台用 `ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL`。
- 通信和服务类任务也要显式列，比如 `RC_SBUS`、`HOST_LINK`、`ELRS_LINK`、`REFEREE_RX`、`SDLOG`。
- `ROBOT_TASK_MODULE_MAX` 是模块表上限，超过它要先扩表，不要偷偷多写。

新车先少开模块：输入、在线检测、CAN 收发、IMU，加一个要调的子系统。确认能跑后再把日志、遥测、裁判、发射等模块补上。

## 4. 配电机装配

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

## 5. 配输入和安全档

输入分两层：

- `manual_input`：输入源选择、超时、合并策略，例如 DBUS、ELRS、图传遥控。
- `input`：把通道映射成语义轴和语义开关。

新车必须先确认安全档。安全档不仅影响控制任务，也影响 IMU 零偏微调能不能开始。上车前要确认：

- 遥控器断开时不会产生有效运动命令。
- 安全档时底盘、云台、射击输出都能停。
- 输入源切换不会突然跳到另一个非零命令。

## 6. 配 DetectTask

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

## 7. 配工程入口

在 `projects/<TARGET>/MDK-ARM/<TARGET>.uvprojx` 里确认：

- Include Path 只指向一个 `Robotconfig/<TARGET>`。
- Source Group 里没有混进别的目标的 `config.c`。
- 目标使用的 board 路径正确。
- `BeforeMake` 里有构建信息生成脚本：

```text
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ..\..\..\tools\gen_build_info.ps1
```

这个脚本会生成 `shared/generated/build_info_autogen.h`，让 SD 日志里带 Git 提交、编译时间和 dirty 状态。

如果这个目标也要支持 GCC/CMake，不要手写第二套 CMake 文件。确认 `.uvprojx` 后用生成脚本检查：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project <TARGET> -FailOnGccBlockers
```

## 8. 第一次检查

在仓库根目录先跑：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\check_all.ps1
```

这个脚本不会替代 Keil 编译，也不会默认跑完整 GCC 编译，但能先抓出工程引用、缺文件、profile 和任务创建不匹配、Python 工具语法这类低级问题。

然后再用 Keil 做一次 Rebuild，确认固件能从干净状态编出来。

如果要确认 GCC/CMake 路线也可用，再跑：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project <TARGET>
```

## 最小验收

新目标至少满足这些，才算初步接起来：

- `tools/check_all.ps1` 通过。
- Keil Rebuild 通过。
- 如果目标承诺支持 GCC/CMake，`tools/build.ps1 -Action gcc-build -Project <TARGET>` 通过。
- SD 日志 `BUILD_INFO` 能显示正确 target、board、Git、编译时间。
- `g_watch` 能看到任务状态和主要设备状态。
- 遥控输入、CAN 反馈、IMU 姿态都能观察。
- 每个子系统都能单独关闭或单独测试。
- 上车检查清单有记录，不只靠口头记忆。
