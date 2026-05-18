# Robotconfig

`Robotconfig/` 放“这台机器人是谁”的内容。这里描述目标本身，不描述 Keil 工程怎么编译，也不描述某块开发板有哪些引脚。

## 当前机器人配置

| 配置 | 说明 | 主要文件 |
|---|---|---|
| `HERO` | 英雄机器人 | `config.c`、`config.h`、`detect_task.c`、`pitch_cali_builtin.c` |
| `INFANTRY` | 步兵机器人 | `config.c`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `SENTINEL` | 哨兵机器人 | `config.c`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `CARRIER` | 工程机器人 | `config.c`、`config.h`、`detect_task.c`、`usb_task_stub.c` |
| `miniwheeleg` | H7 接板和机械臂实验 | `config.c`、`config.h`、`detect_task.c`、`arm_motor_table.c` |

目标目录采用扁平结构：

```text
Robotconfig/<TARGET>/
|-- config.h
|-- config.c
|-- detect_task.c
`-- 目标私有补充文件
```

## 应该放这里

- 默认参数和车型配置：`g_config`、PID、限位、输入映射、任务族选择。
- 轴电机装配：哪个轴用什么电机、哪个 CAN ID、正反方向、反馈 ID。
- 目标在线检测：这台车关心哪些设备、哪些离线算故障。
- 目标私有的小补丁：例如某个目标不接 USB 主机链路，就放对应空实现。
- 目标专属装配表：例如 `miniwheeleg` 的机械臂关节表。

## 不应该放这里

- Keil 工程、CubeMX 生成的 `Core/`、`Drivers/`、`Middlewares/`：放 `projects/`。
- 某块板子的串口、CAN、IMU、蜂鸣器、按键、SD 卡适配：放 `boards/`。
- 可复用控制逻辑、电机协议、输入链路、日志、诊断：放 `shared/`。
- 厂商包、参考工程、临时材料：放 `local/docs/` 或 `local/`。

判断标准很简单：如果换一台同板子的机器人也要改它，它大概率属于 `Robotconfig/`；如果换一块板子才要改它，它大概率属于 `boards/`。
