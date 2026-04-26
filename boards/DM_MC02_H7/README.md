# DM MC02 H7

`boards/DM_MC02_H7/` 是 H7 板级实验目录，不是当前根快捷方式默认打开的主入口。

## 当前入口

- H7 板级实验入口：`MDK-ARM/MC02_BASE.uvprojx`
- `MDK-ARM/CARRIER.uvprojx` 还在，但它当前也是实验工程，不是根目录快捷方式入口
- 根目录 `open_SENTINEL.cmd` 和 `open_CARRIER.cmd` 现在分别打开 `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx` 和 `projects/CARRIER/MDK-ARM/CARRIER.uvprojx`

## 这一层负责什么

- 板级启动：`boards/DM_MC02_H7/User/application/board_main.c`
- FreeRTOS 任务挂接：`boards/DM_MC02_H7/User/application/board_freertos.c`
- 板级 IMU：`boards/DM_MC02_H7/User/application/INS_task.c`
- H7 板级 CubeMX / Keil 工程：`Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/`、`MDK-ARM/`

## 当前任务创建逻辑

`boards/DM_MC02_H7/User/application/board_freertos.c` 默认创建：

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

- `MDK-ARM/MC02_BASE.uvprojx` 当前使用 `target/MC02_BASE/User/application/config.c`
- `MDK-ARM/CARRIER.uvprojx` 现在也还是连到 `target/MC02_BASE/User/application/config.c`，属于实验残留
- H7 板级启动、任务挂接和 BMI088 IMU 已有 ARBATOS 自己的实现
- AUX 口调参、ELRS 和 USB 业务任务在这套 H7 板级入口里还没有默认接上
- 如果你要看当前能直接打开编译的 `SENTINEL` / `CARRIER` 主入口，去 `projects/`，别看这里
