# 工程目标对应表

这张表回答三个问题：

- `projects/`：从哪里打开、编译、下载固件。
- `Robotconfig/`：这份固件使用哪台机器人的参数和装配。
- `boards/`：这份固件按哪块硬件板适配。

| Project | Target | Board | Keil 工程 | 当前用途 |
|---|---|---|---|---|
| `projects/HERO-C` | `Robotconfig/HERO-C` | `boards/DJI_C_F407` | `projects/HERO-C/MDK-ARM/HERO-C.uvprojx` | 英雄 C 板主入口 |
| `projects/HERO-M` | `Robotconfig/HERO-M` | `boards/DM_MC02_H7` | `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` | 英雄 H7 接板入口 |
| `projects/INFANTRY-A` | `Robotconfig/INFANTRY-A` | `boards/DJI_A_F427` | `projects/INFANTRY-A/MDK-ARM/INFANTRY-A.uvprojx` | 步兵 A 板入口 |
| `projects/SENTINEL-A` | `Robotconfig/SENTINEL-A` | `boards/DJI_A_F427` | `projects/SENTINEL-A/MDK-ARM/SENTINEL-A.uvprojx` | 哨兵 A 板入口 |
| `projects/CARRIER-A` | `Robotconfig/CARRIER-A` | `boards/DJI_A_F427` | `projects/CARRIER-A/MDK-ARM/CARRIER-A.uvprojx` | 工程 A 板入口 |
| `projects/MINIWHEELEG-M` | `Robotconfig/MINIWHEELEG-M` | `boards/DM_MC02_H7` | `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` | H7 轮腿 MIT 调试入口 |
| `projects/MINIWHEELEG-C` | `Robotconfig/MINIWHEELEG-C` | `boards/DJI_C_F407` | `projects/MINIWHEELEG-C/MDK-ARM/MINIWHEELEG-C.uvprojx` | C 板小轮腿临时入口 |

## 分工

- 改 `projects/`：通常是在改 Keil 工程、CubeMX 生成代码、启动入口、任务创建、编译前命令。
- 改 `Robotconfig/`：通常是在改车型参数、电机装配、输入映射、在线检测、目标身份。
- 改 `boards/`：通常是在改硬件板引脚、串口、CAN、IMU、按键、蜂鸣器、SD 卡。
- 改 `shared/`：通常是在改可复用控制逻辑、协议、诊断、日志、离线解析参考结构。

当前 project 和 Robotconfig 基本是 1 对 1 同名，但概念上不是一回事。以后可以出现多个 project 共用一个 Robotconfig，也可以出现同一个 Robotconfig 有 F4 / H7 两套工程入口。

## 当前接入状态

| 方向 | 状态 |
|---|---|
| 经典底盘 | 已有任务模块和共享控制任务，仍需按目标实车调参 |
| 单云台 | 已有任务模块和共享控制任务 |
| 双 yaw 云台 | 已有任务模块和 `dual_yaw_gimbal_control_task`，主要在哨兵方向继续实测 |
| MIT 轮腿 | 已有实验任务、状态日志、fault 标志和故障清输出逻辑；下一步重点是实车基线和保护边界继续收束 |
| 舵机轮腿 | 保留配置入口，当前不作为近期主线 |
| 机械臂 | H7 / 小轮腿实验入口有装配和任务基础，仍按具体目标验证 |
| 通用运行层 | 设备表、motor instance、controller registry、`watch.runtime` 和 SD 启动设备记录已接入；控制器统一调度和安全策略还在演进 |

这张状态表只描述主线代码结构，不等于每台车都已经完成上车验证。上车前仍按 [上车检查清单](bringup-checklist.md) 做。
