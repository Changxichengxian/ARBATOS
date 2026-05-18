# DM MC02 H7

`boards/DM_MC02_H7/` 现在只放 H7 板级适配，不再放完整 Keil/CubeMX 工程。

## 当前入口

- H7 实验入口：`projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx`
- 英雄换板入口：`projects/HERO-M/MDK-ARM/HERO-M.uvprojx`
- 根目录可以用 `open_MINIWHEELEG-M.cmd` 或 `open_HERO-M.cmd` 直接打开。

## 这一层负责什么

- 板级启动：`app/board_main.c`
- FreeRTOS 任务挂接：`app/board_freertos.c`
- 板级 IMU：`app/INS_task.c`
- H7 串口、遥控器、裁判系统、SD 卡等板级适配：`bsp/`
- FatFs 磁盘接口：`bsp/diskio.c`

## 当前任务创建逻辑

`app/board_freertos.c` 默认创建：

- `defaultTask`
- `rc_sbus_task`
- `referee_rx_task`
- `health_monitor_task`
- `sdlog_task`
- `can_command_tx_task`
- `can_feedback_rx_task`
- `imu_fusion_task`

按机器人 profile 条件创建：

- `chassis_control_task`
- `gimbal_control_task`
- `arm_task`

## 当前状态

- `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` 使用 `Robotconfig/MINIWHEELEG-M/config.c`
- `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` 使用 `Robotconfig/HERO-M/config.c`
- 配置入口统一是 `config.c` / `config.h`，没有单独的板子配置文件名
- H7 板级启动、任务挂接和 BMI088 IMU 已有 ARBATOS 自己的实现
- AUX 口调参、ELRS 和 USB 业务任务在这套 H7 板级入口里还没有默认接上
- 如果要看能直接打开编译的完整工程，去 `projects/`，别看这里
