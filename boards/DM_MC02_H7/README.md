# DM MC02 H7

`boards/DM_MC02_H7/` 现在只放 H7 板级适配，不再放完整 Keil/CubeMX 工程。

## 当前入口

- H7 实验入口：`projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx`
- 英雄换板入口：`projects/HERO-M/MDK-ARM/HERO-M.uvprojx`
- 根目录可以用 `open_MINIWHEELEG-M.cmd` 或 `open_HERO-M.cmd` 直接打开。

## 这一层负责什么

- 板级启动：`app/BoardMain.c`
- FreeRTOS 任务挂接：`app/BoardFreertos.c`
- 板级 IMU：`app/InsTask.c`
- H7 串口、遥控器、裁判系统、SD 卡等板级适配：`bsp/`
- FatFs 磁盘接口：`bsp/Diskio.c`

`app/InsTask.c` 只保留 MC02 H7 的 BMI088 读取、安装矩阵、温控和姿态融合差异。陀螺仪零偏采样流程共用 `shared/application/services/calibration/GyroZeroCali.h`：正常上电温稳后静止 3 秒微调；`ROBOT_RUN_MODE_CALIBRATION + ROBOT_CALI_TARGET_IMU_GYRO` 下温度到 40 度后静止 30 秒并保存。

## 当前任务创建逻辑

`app/BoardFreertos.c` 默认创建：

- `defaultTask`
- `RcSbusTask`
- `RefereeRxTask`
- `HealthMonitorTask`
- `SdLogTask`
- `BatteryMonitorTask`
- `CanTxTask`
- `CanRxTask`
- `ImuFusionTask`

按机器人 profile 条件创建：

- `ChassisControlTask`
- `WheelLegMitTask`
- `GimbalControlTask`
- `ArmTask`

这些任务现在按 `profile.task_modules` 里的 `ROBOT_TASK_MODULE_*` 创建。`CARRIER_DIRECT_ARM_BRINGUP` 打开时，板级入口会跳过底盘、轮腿 MIT 和云台任务，只保留直接机械臂调试需要的任务。

## 当前状态

- `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` 使用 `Robotconfig/MINIWHEELEG-M/RobotConfig.c`
- `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` 使用 `Robotconfig/HERO-M/RobotConfig.c`
- 配置入口统一是 `RobotConfig.c` / `RobotConfig.h`，没有单独的板子配置文件名
- H7 板级启动、任务挂接和 BMI088 IMU 已有 ARBATOS 自己的实现
- USB Device 初始化在 `defaultTask` 里执行；AUX 口调参、ELRS 和 host_link 业务任务在这套 H7 板级入口里还没有默认接上
- MIT 轮腿任务已经接入 H7 板级创建入口，当前主要服务 `MINIWHEELEG-M` 这类实验目标
- 如果要看能直接打开编译的完整工程，去 `projects/`，别看这里
