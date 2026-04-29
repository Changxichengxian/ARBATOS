# ARBATOS

ARBATOS 是一套面向 RoboMaster 类机器人的 STM32 / FreeRTOS 下位机控制代码。它的目标不是做成“大而全的上位机系统”，而是把底盘、云台、射击、机械臂、通信和日志这些下位机功能收进一套能复用、能扩展、还能保证实时性的工程结构里。

**Author:** 谢宇瀚 <2811158416@qq.com>  
**Repo:** https://github.com/Changxichengxian/ARBATOS.git

## 当前重点

现在的主线目标是两件事：

- 先把实时性框架定住：高频控制链路要短、稳、可测。
- 再做更高级的模块化：配置、诊断、日志、上位机、裁判系统这些放到低频链路里，不污染 1ms / 2ms 控制任务。

剩余架构任务见 `docs/architecture_remaining_tasks.md`。

## 授权和商用

本仓库主体代码采用 `PolyForm-Noncommercial-1.0.0`。

- 可以在非商业用途范围内阅读、复制、修改、调试和评估本项目。
- 如果用于商业产品、商业服务、收费交付、公司业务或其他商业用途，需要另外取得商业授权。
- 这不是开源协议，也没有自动放开的日期。
- 仓库里的第三方组件、厂商 SDK、参考工程和附带资料仍按各自原始许可执行。

具体边界见 `LICENSE`、`docs/legal/COMMERCIAL.md`、`docs/legal/THIRD_PARTY.md`、`CONTRIBUTING.md` 和 `docs/legal/CLA.md`。

## 目录

- [结构概览](#结构概览)
- [工程入口](#工程入口)
- [当前控制链路](#当前控制链路)
- [实时性路线](#实时性路线)
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
  硬件板工程、外设初始化、板级启动、板级 FreeRTOS 任务创建

target/<TARGET>/
  车型参数、目标差异、默认配置、设备在线检测

shared/
  跨板、跨车型复用的控制逻辑、驱动封装、算法和通用组件

projects/<TARGET>/
  可直接打开编译的 Keil 工程入口
```

`board` 和 `target` 不是一回事：

- `board` 是硬件板，负责芯片、外设、引脚和启动。
- `target` 是具体机器人目标，负责 PID 参数、电机 ID、通道映射和行为策略。
- `shared` 放能跨机器人复用的代码。
- `projects` 放最终打开编译的工程入口。

### 板级工程

| 板级工程 | 芯片 | 说明 |
|---|---|---|
| `DJI_C_F407` | STM32F407 | DJI C 板硬件层 |
| `DJI_A_F427` | STM32F427 | DJI A 板硬件层 |
| `DM_MC02_H7` | STM32H723 | H7 实验端口 |

### 目标配置

| Target | 说明 |
|---|---|
| `HERO` | 云台 / 射击主目标 |
| `INFANTRY` | 步兵目标 |
| `SENTINEL` | 哨兵目标 |
| `CARRIER` | 当前默认只启底盘 |
| `MC02_BASE` | H7 接板和机械臂实验 |

每个 `target/<TARGET>/User/application/` 当前至少提供：

- `config.h`：参数结构、输入映射、遥测信号枚举。
- `config.c`：默认参数和全局变量 `g_config`。
- `detect_task.c`：目标设备在线检测。
- 目标私有补充文件，例如 `INS_task.c`、`host_link_task_stub.c`、机械臂装配表等。

### 共享层

```text
shared/
|-- application/   # 控制任务、输入链路、主机链路、watch、日志
|-- bsp/           # ADC、CAN、SPI、I2C、UART、USB、PWM 等外设封装
|-- components/    # AHRS、Mahony、Kalman、PID、设备驱动、CRC、FIFO
`-- common/        # 通用类型定义
```

## 工程入口

| Target | Keil 工程 | 板级工程 | 说明 |
|---|---|---|---|
| `HERO` | `projects/HERO/MDK-ARM/HERO.uvprojx` | `DJI_C_F407` | 云台 / 射击主入口 |
| `INFANTRY` | `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx` | `DJI_A_F427` | 步兵目标 |
| `SENTINEL` | `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx` | `DJI_A_F427` | 哨兵目标 |
| `CARRIER` | `projects/CARRIER/MDK-ARM/CARRIER.uvprojx` | `DJI_A_F427` | 当前默认只启底盘 |
| `MC02_BASE` | `boards/DM_MC02_H7/MDK-ARM/MC02_BASE.uvprojx` | `DM_MC02_H7` | H7 接板和机械臂实验 |

常用打开入口：

```text
open_HERO.cmd
open_INFANTRY.cmd
open_SENTINEL.cmd
open_CARRIER.cmd
```

`MC02_BASE` 当前直接打开 `boards/DM_MC02_H7/MDK-ARM/MC02_BASE.uvprojx`。

## 当前控制链路

当前主线采用“多源输入先汇总，再由各控制任务各自闭环”的结构。

```text
rc_sbus_task      elrs_task      host_link_task      板载按键
     |                |                |                |
     +----------------+----------------+----------------+
                              |
                         manual_input
                              |
                         control_input
                              |
            +-----------------+-----------------+
            |                 |                 |
 chassis_control_task  gimbal_control_task   shoot / arm
            |                 |                 |
            +-----------------+-----------------+
                              |
                         actuator_cmd
                              |
                    can_command_tx_task
```

### 输入链路

- `manual_input.c`：负责 SBUS/DBUS、ELRS/CRSF、图传输入和板载按键的选择、合并与安全处理。
- `control_input.c`：负责把原始通道映射成语义轴和开关。
- `rc_sbus_task.c`：负责 SBUS/DBUS 接收。
- `elrs_task.c`：负责 ELRS/CRSF 接收。

### 主机链路

- `host_link_task.c`：任务级入口，负责调度主机链路相关处理。
- `vision_link.c`：视觉链路协议处理，当前由 USB CDC 接入。
- `image_remote_link.c`：图传遥控输入和遥测处理，实际串口由 AUX 口配置决定。

物理接口仍然包括 USB CDC 和 AUX 口。老 C 板上的 AUX 口当前是 UART1，但业务代码不应直接依赖这个细节。

### 控制和执行

- `chassis_control_task.c`：底盘闭环任务。
- `gimbal_control_task.c`：云台闭环任务，当前射击逻辑仍在该任务周期内调用。
- `shoot.c`：摩擦轮、拨弹和热量相关控制。
- `arm_task.c`：机械臂任务入口，只管顶层运动节奏。
- `arm_motion.c`：机械臂运动抽象，把按键和安全条件转换成关节动作。
- `motor_model_db.c`、`motor_config.h`、`can_mit_motor_driver.c`、`unitree_motor_driver.c`：通用电机型号表、参数查询和共享电机驱动。
- `target/MC02_BASE/User/application/arm_motor_table.c`：机械臂关节装配表，只描述关节、型号、CAN 号、方向和按键；MIT 范围等型号参数放在 `motor_model_db.c`。
- `can_feedback_rx_task.c`：接收电机反馈。
- `can_command_tx_task.c`：统一读取 `actuator_cmd` 并发送 CAN 电流指令。

## 实时性路线

后续架构升级先按“快路径”和“慢路径”分开。

快路径只放必须准时完成的控制链：

- IMU 或姿态快照。
- CAN 电机反馈快照。
- 云台 1kHz 控制。
- 射击 1kHz 控制。
- 底盘 500Hz 控制。
- CAN 电机命令统一发送。
- 极少量耗时统计。

慢路径放可低频处理的功能：

- 上位机通信。
- 视觉协议解析。
- 裁判系统解析。
- 遥控输入源选择和输入融合。
- 参数更新。
- 日志写入。
- 状态灯、蜂鸣器、调试观察。
- 设备在线检测。

已完成的基础改造不再放路线文档里重复写。后续只看 `docs/architecture_remaining_tasks.md` 里的剩余任务。

## 共享模块入口

常用入口如下：

| 模块 | 入口 |
|---|---|
| 手动输入 | `shared/application/manual_input.c` |
| 输入映射 | `shared/application/control_input.c` |
| 主机链路 | `shared/application/host_link_task.c` |
| 视觉链路 | `shared/application/vision_link.c` |
| 图传遥控输入 | `shared/application/image_remote_link.c` |
| 底盘控制 | `shared/application/chassis_control_task.c` |
| 云台控制 | `shared/application/gimbal_control_task.c` |
| 射击控制 | `shared/application/shoot.c` |
| 机械臂运动 | `shared/application/arm_motion.c` |
| 电机型号表 | `shared/application/motor_model_db.c`、`shared/application/motor_config.h` |
| 共享电机驱动 | `shared/application/can_mit_motor_driver.c`、`shared/application/unitree_motor_driver.c` |
| CAN 接收 | `shared/application/can_feedback_rx_task.c` |
| CAN 发送 | `shared/application/can_command_tx_task.c` |
| 裁判系统 | `shared/application/referee_rx_task.c`、`shared/application/referee.c` |
| 电池监测 | `shared/application/battery_monitor_task.c` |
| 状态灯 | `shared/application/status_led_task.c` |
| SD 卡日志 | `shared/application/sdlog_task.c`、`shared/application/sdlog.c` |

## 配置和调试

`config.h` 定义当前主线使用的配置结构，运行时通过 `g_config` 访问。每个目标在自己的 `config.c` 里给出默认值。

主要配置块包括：

- `test_config_t`：测试模式。
- `gimbal_config_t`：云台 PID、软限位、灵敏度和校准参数。
- `chassis_config_t`：底盘 PID、轮型、运动学和摇摆参数。
- `shoot_config_t`：摩擦轮、拨弹和热量参数。
- `power_config_t`：功率限制。
- `detect_config_t`：设备在线检测。
- `imu_config_t`：融合模式和温控参数。
- `motor_config_t`：电机型号和 CAN ID；MIT 范围等型号参数由 `motor_model_db.c` 统一给出。
- `manual_input_config_t`：输入源选择和合并策略。
- `input_config_t`：语义轴和开关映射。

调试入口：

- `g_watch`：Keil Watch Window 里观察运行状态。
- `sdlog_task`：SD 卡记录遥测数据。
- `detect_task`：目标设备在线状态。

## 快速开始

1. 安装 Keil MDK-ARM v5 和对应 STM32F4 / STM32H7 芯片包。
2. 打开上表中的目标工程。
3. 确认当前 target 的 `config.c` 符合硬件接线。
4. 编译并下载到对应板卡。

## 目录结构

```text
ARBATOS/
|-- boards/
|   |-- DJI_C_F407/
|   |-- DJI_A_F427/
|   `-- DM_MC02_H7/
|-- projects/
|   |-- HERO/
|   |-- INFANTRY/
|   |-- SENTINEL/
|   `-- CARRIER/
|-- target/
|   |-- HERO/User/application/
|   |-- INFANTRY/User/application/
|   |-- SENTINEL/User/application/
|   |-- CARRIER/User/application/
|   `-- MC02_BASE/User/application/
|-- shared/
|   |-- application/
|   |-- bsp/
|   |-- components/
|   `-- common/
|-- docs/
`-- tools/
```

## 更多文档

- `docs/README.md`：文档导航。
- `docs/architecture_remaining_tasks.md`：还没完成的架构任务。
- `docs/board_port_config.md`：板级口子配置说明。
- `docs/pitch_cali_model.md`：pitch 校准模型。
- `projects/README.md`：项目入口说明。
- `boards/*/README.md`：各硬件板说明。

## 扩展建议

新增 target 时：

1. 在 `target/` 下创建新的目标目录。
2. 准备 `config.h` / `config.c`。
3. 按需要补 `detect_task.c`、`INS_task.c` 或 `host_link_task_stub.c`。
4. 在对应工程里加入目标文件。

新增任务家族时：

1. 在 `config.h` 的家族枚举里补新值。
2. 在目标的 `g_config.profile` 中选择该家族。
3. 新建任务文件。
4. 在板级 `freertos.c` 或 `board_freertos.c` 里创建任务。
5. 给这个任务家族单独准备参数块。
