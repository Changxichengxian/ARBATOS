# Robotconfig

`Robotconfig/` 放“这台机器人是谁”的内容。这里描述目标本身，不描述 Keil 工程怎么编译，也不描述某块开发板有哪些引脚。

## 当前机器人配置

| 配置 | 说明 | 主要文件 |
|---|---|---|
| `HERO-C` | 英雄机器人 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`pitch_cali_builtin.c` |
| `HERO-M` | 英雄机器人临时接 MC02 H7 板 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`pitch_cali_builtin.c` |
| `INFANTRY-A` | 步兵机器人 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `SENTINEL-A` | 哨兵机器人 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `CARRIER-A` | 工程机器人 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `MINIWHEELEG-M` | H7 接板和机械臂实验 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`arm_motor_table.c` |
| `MINIWHEELEG-C` | 小轮腿临时接 DJI C 板 | `config.c`、`config_*.inc`、`config.h`、`detect_task.c`、`arm_motor_table.c` |

目标目录采用扁平结构：

```text
Robotconfig/<TARGET>/
|-- config.h
|-- config.c
|-- config_operation.inc
|-- config_hardware.inc
|-- config_tuning.inc
|-- config_input.inc
|-- config_diagnostics.inc
|-- detect_task.c
`-- 目标私有补充文件
```

`config.c` 只保留 `g_config` 总入口、调参块表和块启用判断。真正要改默认值时，按下面几个片段找：

- `config_operation.inc`：运行模式、目标任务/电机、任务模块列表。
- `config_hardware.inc`：设备表和电机装配。
- `config_tuning.inc`：云台、底盘、轮腿、射击、功率、IMU、电压、蜂鸣器、LED 参数。
- `config_input.inc`：输入源策略、遥控/键鼠/图传映射。
- `config_diagnostics.inc`：在线检测、AUX 实时遥测、SD 日志默认值。

## 应该放这里

- 默认参数和车型配置：`g_config`、PID、限位、输入映射和任务模块选择。
- 轴电机装配：哪个轴用什么电机、哪个 CAN ID、正反方向、反馈 ID。
- 目标在线检测：这台车关心哪些设备、哪些离线算故障。
- 目标私有的小补丁：例如某个目标不接 USB 主机链路，就放对应空实现。
- 目标专属装配表：例如 `MINIWHEELEG-M`、`MINIWHEELEG-C` 的机械臂关节表。

## 不应该放这里

- Keil 工程、CubeMX 生成的 `Core/`、`Drivers/`、`Middlewares/`：放 `projects/`。
- 某块板子的串口、CAN、IMU、蜂鸣器、按键、SD 卡适配：放 `boards/`。
- 可复用控制逻辑、电机协议、输入链路、日志、诊断：放 `shared/`。
- 厂商包、参考工程、临时材料：放 `local/docs/` 或 `local/`。

判断标准很简单：如果换一台同板子的机器人也要改它，它大概率属于 `Robotconfig/`；如果换一块板子才要改它，它大概率属于 `boards/`。

## 配置分层

建议按这五类理解目标配置：

1. 运行编排：`config_operation.inc` 里的 `g_config.operation` 和 `g_config.profile`，决定默认怎么跑、哪些任务静态启用。
2. 硬件装配：`config_hardware.inc` 里的 `g_config.devices` 和 `g_config.motor`，决定有哪些设备、电机怎么接。
3. 控制参数：`config_tuning.inc`，放云台、底盘、射击、功率、轮腿、机械臂等 PID、限幅和几何参数。
4. 输入映射：`config_input.inc`，放输入源策略、遥控通道、语义开关和安全档。
5. 诊断记录：`config_diagnostics.inc`，放在线检测、AUX 实时遥测、SD 日志默认值。

`operation` 是当前运行方式入口。任务仍按 `profile.task_modules` 静态创建，`operation` 只决定谁允许输出。常用组合：

- 全任务正常：`mode=ROBOT_RUN_MODE_FULL`，`variant=ROBOT_RUN_VARIANT_NORMAL`。
- 单任务：`mode=ROBOT_RUN_MODE_SINGLE_TASK`，`target_task` 填一个 `ROBOT_TASK_MODULE_*`。
- 单电机：`mode=ROBOT_RUN_MODE_SINGLE_MOTOR`，`target_motor` 填 `MotorId`。
- 校准：`mode=ROBOT_RUN_MODE_CALIBRATION`，`cali_target` 填校准对象。
- 娱乐/演示：`mode=ROBOT_RUN_MODE_ENTERTAIN`。

## 新车配置优先级

新建目标时，建议先复制最接近的一台车，再按这个顺序改：

1. `config_operation.inc`：先决定默认运行方式，再列 `task_modules`。这台车启用什么任务，就在这里写什么模块；任务创建、调参块和观测块都按它判断。
2. `config_hardware.inc`：填电机型号、CAN ID、总线、反馈 ID、协议和限幅；不用的轴先设 `can_id = 0`。
3. `config_input.inc`：确认遥控通道、语义开关、安全档和输入源策略。
4. `config_tuning.inc`：底盘、云台、射击、功率、IMU 等参数先保守限幅，再逐步调手感。
5. `config_diagnostics.inc` 和 `detect_task.c`：按已启用模块补在线检测；设备拔掉能报错、插回能恢复，才算检测有效。

常用做法是先用 `operation` 开单任务或单电机，再回到全任务正常运行做整车联调。陀螺仪零偏校准使用 `ROBOT_RUN_MODE_CALIBRATION + ROBOT_CALI_TARGET_IMU_GYRO`：温度升到 40 度并稳定后，静止采 30 秒并保存。

完整新车接入步骤见 `../manual/new-target.md`，上车检查见 `../manual/bringup-checklist.md`。
