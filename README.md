# ARBATOS

ARBATOS 是一套面向 RoboMaster 类机器人的 STM32 / FreeRTOS 下位机控制代码。它的目标不是做成“大而全的上位机系统”，而是把底盘、云台、射击、机械臂、通信、诊断和日志这些下位机功能收进一套能复用、能扩展、还能保证实时性的工程结构里。

这份 README 按当前代码写。旧文档如果和代码不一致，先以 `projects/`、`Robotconfig/`、`shared/`、`boards/` 里的代码为准。

**Author:** 谢宇瀚 <2811158416@qq.com>  
**Repo:** https://github.com/Changxichengxian/ARBATOS.git

## 当前状态

当前代码已经不只是“基础框架移植”。主线已经有：

- 多目标工程入口：`HERO-C`、`HERO-M`、`INFANTRY-A`、`SENTINEL-A`、`CARRIER-A`、`MINIWHEELEG-M`、`MINIWHEELEG-C`。
- 多硬件板支持：DJI C 板、DJI A 板、DM MC02 H7 板。
- 静态 FreeRTOS 任务创建，并按任务族配置（profile，用配置决定启哪些任务）选择底盘、云台等任务；MC02 H7 实验入口还按配置接机械臂任务。
- 多源手动输入：DBUS/SBUS、ELRS/CRSF、图传遥控、板载按键。
- 统一执行器命令层：控制任务只面向“轴”发命令，CAN 发送任务再决定具体协议。
- 电机型号和协议能力表：RM 电机、达妙 3 模式 / 扩展协议、宇树 GO-M8010-6 等。
- 运行诊断和日志：`g_watch`、运行耗时统计、TF/SD 二进制日志、AUX 口遥测和临时调参。

还有几块故意没写成“已完成”：

- 高频控制任务还没有统一读快照，部分地方仍直接读 `g_config`。
- 双云台、轮腿舵机、轮腿 MIT 这些任务族有配置入口，但实际控制任务还没补完整。
- 高频日志虽然已经是环形缓冲，但仍要继续实测成本，避免影响 1ms / 2ms 控制链。
- 根目录还没有统一的命令行编译入口，也没有正式的自动编译检查（CI，提交后自动跑编译）。

剩余架构任务放在本地文档 `local/docs/90_待办和清理/架构剩余任务.md`。

## 授权和商用

本仓库主体代码采用 `PolyForm-Noncommercial-1.0.0`。

- 可以在非商业用途范围内阅读、复制、修改、调试和评估本项目。
- 如果用于商业产品、商业服务、收费交付、公司业务或其他商业用途，需要另外取得商业授权。
- 这不是开源协议，也没有自动放开的日期。
- 仓库里的第三方组件、厂商 SDK、参考工程和附带资料仍按各自原始许可执行。

具体边界见 `LICENSE`、`legal/COMMERCIAL.md`、`legal/THIRD_PARTY.md`、`legal/CONTRIBUTING.md` 和 `legal/CLA.md`。

## 目录

- [结构概览](#结构概览)
- [工程入口](#工程入口)
- [启动和任务](#启动和任务)
- [当前控制链路](#当前控制链路)
- [输入链路](#输入链路)
- [主机链路](#主机链路)
- [状态共享](#状态共享)
- [电机和执行器](#电机和执行器)
- [实时性路线](#实时性路线)
- [诊断和日志](#诊断和日志)
- [共享模块入口](#共享模块入口)
- [配置和调试](#配置和调试)
- [快速开始](#快速开始)
- [目录结构](#目录结构)
- [更多文档](#更多文档)
- [扩展建议](#扩展建议)

## 结构概览

ARBATOS 采用“三层复用结构，加一层项目入口”：

```text
boards/<BOARD>/
  硬件板支持、外设适配、板级启动代码

Robotconfig/<TARGET>/
  车型参数、目标差异、默认配置、设备在线检测

shared/
  跨板、跨车型复用的控制逻辑、驱动封装、算法和通用组件

projects/<TARGET>/
  可直接打开编译的 Keil 工程入口，车和实验目标的完整工程放这里
```

`board` 和 `Robotconfig` 不是一回事：

- `board` 是硬件板，负责芯片、外设、引脚和启动。
- `Robotconfig` 是具体机器人目标，负责 PID 参数、电机 ID、通道映射和行为策略。
- `shared` 放能跨机器人复用的代码。
- `projects` 放最终打开编译的工程入口。`HERO-C`、`HERO-M`、`INFANTRY-A`、`SENTINEL-A`、`CARRIER-A` 这类车名目录只放在这里，`MINIWHEELEG-M`、`MINIWHEELEG-C` 这种实验入口也放这里。

### 板级工程

| 板级工程 | 芯片 | 说明 |
|---|---|---|
| `DJI_C_F407` | STM32F407 | DJI C 开发板 |
| `DJI_A_F427` | STM32F427 | DJI A 开发板 |
| `DM_MC02_H7` | STM32H723 | 达妙 MC02 开发板 |

### 车型配置

| Target | 说明 |
|---|---|
| `HERO-C` | 西北农林科技大学英雄机器人 |
| `HERO-M` | 英雄机器人临时接 MC02 H7 板 |
| `INFANTRY-A` | 西北农林科技大学步兵机器人 |
| `SENTINEL-A` | 西北农林科技大学哨兵机器人 |
| `CARRIER-A` | 西北农林科技大学工程机器人 |
| `MINIWHEELEG-M` | H7 接板和轮腿 MIT 调试入口 |
| `MINIWHEELEG-C` | 小轮腿临时接 DJI C 板 |

每个 `Robotconfig/<TARGET>/` 当前至少提供：

- `config.h`：参数结构、任务族、输入映射、遥测信号枚举、轴电机装配结构。
- `config.c`：默认参数、全局变量 `g_config`、只读电机型号表 `g_motor_config`、AUX 临时调参表。
- `detect_task.c`：目标设备在线检测、状态汇总、部分日志上报。
- 目标私有补充文件，例如 `host_link_task_stub.c`、`usb_task_stub.c`、机械臂装配表等。

### 共享层

```text
shared/
|-- application/   # 控制任务、通信链路、输入、电机、诊断、日志
|-- hal/           # ADC、CAN、SPI、I2C、UART、USB、PWM 等跨板外设封装
`-- components/    # AHRS、Mahony、Kalman、PID、设备驱动、CRC、FIFO、通用类型
```

## 工程入口

当前正式入口仍是 Keil 工程：

| Target | Keil 工程 | 默认配置 |
|---|---|---|
| `HERO-C` | `projects/HERO-C/MDK-ARM/HERO-C.uvprojx` | `Robotconfig/HERO-C/config.c` |
| `HERO-M` | `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` | `Robotconfig/HERO-M/config.c` |
| `INFANTRY-A` | `projects/INFANTRY-A/MDK-ARM/INFANTRY-A.uvprojx` | `Robotconfig/INFANTRY-A/config.c` |
| `SENTINEL-A` | `projects/SENTINEL-A/MDK-ARM/SENTINEL-A.uvprojx` | `Robotconfig/SENTINEL-A/config.c` |
| `CARRIER-A` | `projects/CARRIER-A/MDK-ARM/CARRIER-A.uvprojx` | `Robotconfig/CARRIER-A/config.c` |
| `MINIWHEELEG-M` | `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` | `Robotconfig/MINIWHEELEG-M/config.c` |
| `MINIWHEELEG-C` | `projects/MINIWHEELEG-C/MDK-ARM/MINIWHEELEG-C.uvprojx` | `Robotconfig/MINIWHEELEG-C/config.c` |

注意：

- 根目录目前没有正式维护的 `Makefile` / `CMakeLists.txt` 入口。
- `projects/*/MDK-ARM/*.uvprojx` 是当前最可靠的编译入口。
- 部分 `projects/*/MDK-ARM/codex_build_*.log` 是历史编译记录，不代表统一构建系统。

## 启动和任务

启动顺序大致是：

```text
main.c
  |
  +-- HAL / CubeMX 外设初始化
  +-- 板级和共享模块初始化
  +-- manual_input_init
  +-- osKernelStart
        |
        +-- MX_FREERTOS_Init
              |
              +-- 创建静态任务
              +-- 按 g_config.profile 选择部分任务
```

F4 车工程主要在 `projects/<TARGET>/Core/Src/freertos.c` 创建任务。MC02 H7 板级实验入口主要在 `boards/DM_MC02_H7/app/board_freertos.c` 创建任务。

当前常见任务：

| 任务 | 作用 |
|---|---|
| `startup_service_task` | 启动期服务，包含 USB 初始化等 |
| `imu_fusion_task` | IMU 姿态融合 |
| `calibrate_task` | 校准相关处理 |
| `chassis_control_task` | 经典底盘控制，按任务族决定是否创建 |
| `gimbal_control_task` | 单云台控制，按任务族决定是否创建 |
| `can_feedback_rx_task` | 消费 CAN 接收环形缓冲，更新电机反馈 |
| `can_command_tx_task` | 汇总执行器命令，统一发 CAN |
| `rc_sbus_task` | DBUS/SBUS 输入解析 |
| `elrs_link_task` | ELRS/CRSF 输入解析 |
| `host_link_task` | AUX / USB 主机链路、视觉、调参、遥测、图传输入 |
| `referee_rx_task` | 裁判系统接收和解析 |
| `battery_monitor_task` | 电池状态采样和估算 |
| `servo_control_task` | 舵机输出 |
| `arm_task` | 机械臂任务，当前主要在 H7 / 机械臂实验入口接入 |
| `sdlog_task` | 低优先级刷新 TF/SD 日志文件 |
| `health_monitor_task` | 在线检测、状态汇总、系统统计 |
| `status_led_task` | 状态灯和提示输出 |

任务选择入口在 `shared/application/robot/robot_task_profile.h`。现在已经有经典底盘、单云台、机械臂的创建判断；双云台、轮腿舵机、轮腿 MIT 的判断入口也有，但对应实际控制任务还没补完整。

## 当前控制链路

整体采用“多源输入先汇总，控制任务各自闭环，执行器统一发送”的结构：

```text
DBUS/SBUS       ELRS/CRSF       图传遥控       板载按键
   |               |              |             |
   +---------------+--------------+-------------+
                          |
                    manual_input
                          |
                    control_input
                          |
        +-----------------+-----------------+
        |                 |                 |
chassis_control_task  gimbal_control_task  shoot / arm
        |                 |                 |
        +-----------------+-----------------+
                          |
                    actuator_cmd
                          |
                  can_command_tx_task
                          |
                         CAN
```

反馈链路单独走：

```text
CAN 中断
  |
bsp_can RX 环形缓冲
  |
can_feedback_rx_task
  |
CAN_receive / motor_instance
  |
actuator_feedback + 旧电机反馈结构
  |
控制任务 / g_watch / sdlog
```

这套结构的边界是：控制任务不应该关心某个轴到底是 3508、6020、达妙还是宇树；它只发“这个轴要什么动作”。具体电机型号、协议、CAN ID 和限幅由配置、电机表、执行器发送任务处理。

## 输入链路

输入链路现在分三层：

- `rc_sbus_task.c`、`elrs_task.c`、`image_remote_link.c`：负责解析不同来源的原始输入。
- `manual_input.c`：负责多输入源状态保存、超时判断、输入源选择、合并、板载按键叠加和安全处理。
- `control_input.c`：把通道、开关、鼠标、键盘映射成控制任务能直接读的语义轴和语义开关。

当前支持的输入来源：

| 来源 | 主要文件 | 说明 |
|---|---|---|
| DBUS/SBUS | `rc_sbus_task.c`、`manual_input.c` | 常规遥控器输入 |
| ELRS/CRSF | `elrs_task.c` | 通过 AUX 链路接收 CRSF 通道 |
| 图传遥控 | `image_remote_link.c` | 支持自定义帧和 VT13 风格输入 |
| 板载按键 | `manual_input.c`、板级按键配置 | 作为额外按键掩码叠加进输入状态 |

`manual_input_config_t` 决定输入源优先级、合并策略、超时和图传通道映射。新控制代码应优先读 `control_input` 的语义接口，不要在控制任务里直接写死遥控通道号。

## 主机链路

主机链路分 USB 和 AUX 两类物理接口：

- USB CDC 当前主要给视觉链路使用。
- AUX 口承担 ELRS/CRSF、图传遥控、临时调参、遥测输出等功能。

`host_link_task.c` 是任务级入口，当前责任比较多：

- 调用 `vision_link.c` 处理视觉链路收发。
- 根据 AUX 口配置和波特率处理图传遥控、临时调参、JustFloat 遥测。
- 按 AUX 模式启动图传解析；ELRS/CRSF 由 `elrs_task.c` 独立消费同一 AUX 链路。
- 把系统状态、控制状态、日志统计等打包给上位机观察。

后续如果继续扩展，优先把 `host_link_task.c` 拆得更清楚：AUX 模式管理、临时调参、遥测输出、图传输入、视觉链路分别收口，避免一个任务文件继续变大。

## 状态共享

控制任务之间不使用动态字符串消息系统。当前有两类共享方式：

- `actuator_cmd` / `actuator_feedback`：执行器命令和反馈，按固定轴编号读写。
- 共享状态表：`state_store.c` + `robot_state.h`，目前主要保存云台、底盘、射击状态，给诊断、遥测、日志读取。

`state_store` 使用固定枚举、固定内存和短临界区拷贝，适合下位机实时场景。代价是扩展不如动态消息中心灵活；新增一类状态时，需要补枚举、结构体和读写包装。

## 电机和执行器

电机相关边界现在按下面分：

- `g_motor_config`：只读电机型号表，记录每种电机的 CAN 基址、最大电流和减速比。
- `motor_model_db.c`：共享能力表，记录协议类型、控制方式、MIT 控制范围和反馈解析方式。
- `g_config.motor`：当前目标的轴装配表，记录底盘、摩擦轮、yaw、pitch、trigger、机械臂关节等轴分别装什么电机、用哪个 CAN ID。
- `motor_instance.c`：根据配置生成运行期电机实例，并负责把 CAN 反馈归到对应轴。
- `actuator_cmd.c`：控制任务写入执行器命令，支持电流、状态力矩、位置速度、速度、力位等模式。
- `CAN_receive.c`：解析 CAN 反馈，更新旧电机测量结构和新执行器反馈。
- `can_command_tx_task.c`：把执行器命令转成实际 CAN 帧，统一处理 RM 电流组包、MIT 风格控制、达妙扩展协议和安全限幅。

控制任务应优先按“角色”和“轴”调用接口，例如底盘 0 号轮、yaw、pitch、trigger、friction、机械臂关节。不要在底盘、云台、射击任务里直接写某个 CAN ID 或某个电机协议的细节。

## 实时性路线

现在代码已经按“高频控制链”和“低频服务链”分开了一部分。

高频控制链只放必须准时完成的内容：

- IMU 姿态融合。
- CAN 电机反馈消费。
- 云台 1ms 控制。
- CAN 电机命令统一发送。
- 底盘 2ms 控制。
- 少量运行耗时统计。

低频服务链处理可延迟一点的内容：

- 主机通信和视觉协议。
- 裁判系统解析。
- 遥控输入源选择和输入融合。
- 参数临时修改。
- TF/SD 日志文件刷新。
- 状态灯、蜂鸣器、调试观察。
- 设备在线检测和系统状态汇总。

已经完成的方向：

- 任务创建尽量使用静态 TCB 和静态栈。
- CAN 接收中断只做轻量入队，再由 `can_feedback_rx_task` 消费。
- CAN 发送集中到 `can_command_tx_task`。
- SD 日志先写环形缓冲，再由低优先级任务写到文件里。
- `rt_profiler` 记录部分关键路径耗时。

还需要继续收紧的方向：

- 高频控制任务读配置时逐步改成快照读取，减少直接读大块 `g_config`。
- 高频任务里的 `sdlog_write` 调用继续实测成本，必要时降频或批量缓存。
- 双云台、轮腿舵机、轮腿 MIT 等任务族补齐后，再接入板级任务创建。
- 根目录补一个可重复的命令行编译入口，后面再接自动编译检查。

## 诊断和日志

当前主要诊断入口：

- `g_watch`：给 Keil 观察窗口（Watch Window，用来直接看变量）查看运行状态。
- `watch.c`：记录任务状态、设备状态、执行器状态、控制状态和故障信息。
- `rt_profiler.c`：统计关键路径耗时、最大值和超预算次数。
- `detect_task.c`：设备在线检测、状态摘要、重要事件记录。
- `sdlog.c` / `sdlog_task.c`：TF/SD 二进制日志，先入环形缓冲，再低优先级写文件。
- `host_link_task.c`：AUX 遥测和临时调参。

常见日志内容包括：

- IMU、PID、云台、底盘、CAN、电池、裁判系统、视觉链路。
- 手动输入原始帧和图传链路统计。
- 当前配置快照、系统统计、运行事件、运行耗时。

注意：`sdlog_write` 不是直接写文件，但它仍然会复制数据并进入短临界区。高频任务里新增日志前，需要先考虑频率、数据量和最坏耗时。

## 共享模块入口

常用入口如下：

| 模块 | 入口 |
|---|---|
| 手动输入 | `shared/application/input/manual_input.c` |
| 输入映射 | `shared/application/input/control_input.c` |
| 主机链路 | `shared/application/comm/host/host_link_task.c` |
| 视觉链路 | `shared/application/comm/vision/vision_link.c` |
| 图传遥控输入 | `shared/application/input/image_remote_link.c` |
| ELRS/CRSF 输入 | `shared/application/input/elrs_task.c` |
| 状态共享 | `shared/application/robot/state_store.c`、`shared/application/robot/robot_state.h` |
| 消息基础类型 | `shared/application/robot/robot_msg.h` |
| 底盘状态消息 | `shared/application/chassis/chassis_state.h` |
| 云台状态消息 | `shared/application/gimbal/gimbal_state.h` |
| 射击状态消息 | `shared/application/shoot/shoot_state.h` |
| 机械臂命令和状态消息 | `shared/application/arm/arm_msg.h` |
| 轮腿命令和状态消息 | `shared/application/wheelleg/wheelleg_msg.h` |
| 控制器生命周期 | `shared/application/robot/control_manager.c` |
| 执行器命令 | `shared/application/robot/actuator_cmd.c` |
| 电机实例 | `shared/application/motors/motor_instance.c` |
| 电机参数查询 | `shared/application/motors/motor_config.h` |
| 电机协议能力表 | `shared/application/motors/motor_model_db.c` |
| 共享电机驱动 | `shared/application/motors/can_mit_motor_driver.c`、`shared/application/motors/unitree_motor_driver.c` |
| CAN 接收 | `shared/application/comm/can/can_feedback_rx_task.c`、`shared/application/comm/can/CAN_receive.c` |
| CAN 发送 | `shared/application/comm/can/can_command_tx_task.c` |
| 底盘控制 | `shared/application/chassis/chassis_control_task.c` |
| 云台控制 | `shared/application/gimbal/gimbal_control_task.c` |
| 射击控制 | `shared/application/shoot/shoot.c` |
| 机械臂运动 | `shared/application/arm/arm_motion.c` |
| 裁判系统 | `shared/application/comm/referee/referee_rx_task.c`、`shared/application/comm/referee/referee.c` |
| 电池监测 | `shared/application/services/battery/battery_monitor_task.c` |
| 舵机输出 | `shared/application/services/servo/servo_control_task.c` |
| 状态灯 | `shared/application/services/startup/status_led_task.c` |
| 诊断观察 | `shared/application/services/diagnostics/watch.c` |
| 运行耗时统计 | `shared/application/services/diagnostics/rt_profiler.c` |
| TF/SD 日志 | `shared/application/services/storage/sdlog_task.c`、`shared/application/services/storage/sdlog.c` |

`robot_msg.h` 和各 `*_state.h` / `*_msg.h` 是 ARBATOS 原生结构体消息。它们只使用 C 结构体和固定状态表，不引入动态分配、运行时反射或外部中间件。`robot_state.h` 是聚合入口，新代码也可以只包含自己需要的模块头文件。

`control_manager.c` 是 MCU 轻量版控制器生命周期层。它只保存静态 controller 描述、处理 pending switch/stop、按 domain 保证单 active controller，并用资源 claim mask 防止多个 controller 同时拥有同一批执行器。它不新建任务、不动态加载、不分配堆内存；真正的 `enter/update/exit/stop` 仍然在对应控制任务自己的周期内执行。

## 配置和调试

`config.h` 定义当前主线使用的配置结构，运行时通过 `g_config` 访问。每个目标在自己的 `config.c` 里给出默认值。

`g_config` 里既有运行中可调的参数，也有启动前确定的装配信息。AUX 临时调参能改哪些字段，只看 `config.c` 里的 `g_config_blocks`；没有放进调参表的字段就不会被 AUX 改。

电机相关边界：

- `g_motor_config` 是只读电机型号表。
- `motor_model_db.c` 是共享协议能力表。
- `g_config.motor` 是当前目标的轴装配表。
- `g_config.motor` 没放进 AUX 调参表；换电机或换接线时，改对应 Robotconfig 的 `config.c`。
- 底盘、云台、射击和机械臂只对“轴”发命令，通过 `motor_config.h` 和 `motor_instance.c` 处理电机差异。

主要配置块包括：

- `task_profile_t`：底盘、云台、机械臂等任务族选择。
- `test_config_t`：测试模式。
- `gimbal_config_t`：云台 PID、软限位、灵敏度和校准参数。
- `chassis_config_t`：底盘 PID、轮型、运动学和摇摆参数。
- `shoot_config_t`：摩擦轮、拨弹和热量参数。
- `arm_j0_unitree_config_t` 等机械臂相关配置：机械臂关节、端口、电机 ID 和控制参数。
- `power_config_t`：功率限制。
- `detect_config_t`：设备在线检测。
- `imu_config_t`：融合模式和温控参数。
- `buzzer_config_t`、`led_config_t`：提示输出。
- `manual_input_config_t`：输入源选择、合并策略、图传输入映射。
- `input_config_t`：语义轴和开关映射。
- `aux_telem_config_t`：AUX 遥测信号选择。
- `sdlog_config_t`：TF/SD 日志格式、压缩和高频日志分频。

调试入口：

- 运行状态：看 `g_watch`。
- 输入问题：看 `manual_input_state`、`control_input` 和对应输入源统计。
- 电机问题：先看 `g_config.motor`、`motor_instance`、`actuator_feedback`，再看 CAN 收发统计。
- 控制问题：看 `robot_state.h` 里的云台、底盘、射击状态。
- 日志问题：看 `sdlog_get_stats()` 返回的环形缓冲、丢包、写文件状态。
- AUX 调参问题：先确认字段所在配置块是否在 `g_config_blocks`。

## 快速开始

1. 安装 Keil MDK-ARM v5 和对应 STM32F4 / STM32H7 芯片包。
2. 打开目标工程，例如 `projects/HERO-C/MDK-ARM/HERO-C.uvprojx`。
3. 确认当前 Robotconfig 的 `config.c` 符合硬件接线，尤其是 `g_config.profile` 和 `g_config.motor`。
4. 确认板级串口、CAN、IMU、蜂鸣器、按键等配置在 `boards/<BOARD>/` 下匹配当前硬件。
5. 编译并下载到对应板卡。
6. 首次上车前，先用 `g_watch`、AUX 遥测或 TF/SD 日志确认输入、电机反馈、任务状态都正常。

## 目录结构

```text
ARBATOS/
|-- boards/
|   |-- DJI_C_F407/
|   |-- DJI_A_F427/
|   `-- DM_MC02_H7/
|-- projects/
|   |-- HERO-C/
|   |-- HERO-M/
|   |-- INFANTRY-A/
|   |-- SENTINEL-A/
|   |-- CARRIER-A/
|   |-- MINIWHEELEG-M/
|   `-- MINIWHEELEG-C/
|-- Robotconfig/
|   |-- HERO-C/
|   |-- HERO-M/
|   |-- INFANTRY-A/
|   |-- SENTINEL-A/
|   |-- CARRIER-A/
|   |-- MINIWHEELEG-M/
|   `-- MINIWHEELEG-C/
|-- shared/
|   |-- application/
|   |-- bsp/
|   `-- components/
|-- legal/
|-- local/
`-- tools/
```

注意：这里没有 `Robotconfig/<TARGET>/User/application/`。如果看到旧文档或旧工程里还写这个路径，那就是过期引用。

## 更多文档

- `legal/`：授权、商用、第三方组件和贡献边界，继续跟随 Git。
- `local/docs/`：本地文档、接入记录、厂商资料和清理清单，不再提交到 Git。
- `projects/README.md`：项目入口说明。
- `Robotconfig/README.md`：机器人配置层说明。
- `boards/README.md`：板级适配层说明。
- `shared/README.md`：共享代码层说明。
- `boards/*/README.md`：各硬件板说明。
- `tools/mp3_to_u8/README.md`：把 MP3 转成蜂鸣器使用的 `.U8` 原始音频。
- `tools/pid_autotune/README.md`：PID 自动整定工具说明。

## 扩展建议

新增 Robotconfig 时：

1. 在 `Robotconfig/` 下创建新的目标目录。
2. 准备 `config.h` / `config.c`。
3. 配好 `g_config.profile`、`g_config.motor` 和输入映射。
4. 按需要补 `detect_task.c`、`INS_task.c` 或主机链路空实现。
5. 在对应 Keil 工程里加入目标文件。

新增硬件板时：

1. 在 `boards/` 下创建板级目录。
2. 补外设端口、串口分配、CAN、IMU、按键、蜂鸣器、SD 卡等板级配置。
3. 如果是独立实验入口，补 `board_main.c` 和 `board_freertos.c`。
4. 在 `projects/` 下创建能直接打开的工程入口。

新增电机型号时：

1. 在 `config.h` 里补 `motor_model_e`。
2. 在 `g_motor_config` 里补固定参数。
3. 在 `motor_model_db.c` 里补协议能力、反馈格式和控制范围。
4. 如果需要新协议，补对应驱动，并接进 `CAN_receive.c` / `can_command_tx_task.c`。
5. 最后只在 `g_config.motor` 里把某个轴装成这个电机。

新增输入来源时：

1. 新建解析文件，把原始帧转成 `manual_input_state_t`。
2. 通过 `manual_input_update_source()` 更新输入源。
3. 在 `manual_input_config_t` 里补优先级、超时和映射策略。
4. 控制任务仍然只读 `control_input` 的语义接口。

新增任务族时：

1. 在 `config.h` 的任务族枚举里补新值。
2. 在目标的 `g_config.profile` 中选择该任务族。
3. 新建控制任务文件。
4. 在 `robot_task_profile.h` 里补判断函数。
5. 在对应项目工程的 `freertos.c` 或 `board_freertos.c` 里按判断结果创建任务。
6. 给这个任务族单独准备参数块、诊断字段和最小验证方式。
