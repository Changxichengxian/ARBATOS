# 工程目标对应表

这张表回答三个问题：

- `zephyr/`：当前从哪里配置、构建和下载固件。
- `Robotconfig/`：这份固件使用哪台机器人的参数和装配。
- `boards/`：这份固件按哪块硬件板适配。

每个 `Robotconfig/<TARGET>/MountLayout.md` 记录这台车的控制板固定位置、开发板正方向和 INS 姿态含义；坐标总口径见 [坐标系和安装基准](coordinate-frames.md)。

| Zephyr target | Robotconfig | Board | 当前用途 |
|---|---|---|---|
| `hero-m` | `Robotconfig/HERO-M` | `boards/DmMc02H7` | 英雄 H7 Zephyr 目标 |
| `sentinel-m` | `Robotconfig/SENTINEL-M` | `boards/DmMc02H7` | 哨兵 H7 Zephyr 目标 |
| `miniwheeleg-m` | `Robotconfig/MINIWHEELEG-M` | `boards/DmMc02H7` | H7 轮腿 Zephyr 目标 |

## 分工

- 改 `zephyr/`：通常是在改正式目标配置、板级定义、启动映射和显式源码清单。

已移除的旧 Keil/CubeMX 工程只用于历史恢复：`git show 951857f:<path>` 或 `zephyr` 分支的 `6bdf19e`。
- 改 `Robotconfig/`：通常是在改车型参数、电机装配、输入映射、在线检测、目标身份。
- 改 `boards/`：通常是在改硬件板引脚、串口、CAN、IMU、按键、蜂鸣器、SD 卡。
- 改 `shared/`：通常是在改可复用控制逻辑、协议、诊断、日志、离线解析参考结构。

当前 project 和 Robotconfig 基本是 1 对 1 同名，但概念上不是一回事。以后可以出现多个 project 共用一个 Robotconfig，也可以出现同一个 Robotconfig 有 F4 / H7 两套工程入口。

## 当前接入状态

| 方向 | 状态 |
|---|---|
| 经典底盘 | 已有任务模块和共享控制任务，仍需按目标实车调参 |
| 单云台 | 已有任务模块和共享控制任务 |
| 双 yaw 云台 | 已有任务模块和 `DualYawGimbalControlTask`，主要在哨兵方向继续实测 |
| MIT 轮腿 | 已有实验任务、状态日志、fault 标志和故障清输出逻辑；下一步重点是实车基线和保护边界继续收束 |
| 舵机轮腿 | 保留配置入口，当前不作为近期主线 |
| 机械臂 | H7 / 小轮腿实验入口有装配和任务基础，仍按具体目标验证 |
| 通用运行层 | 设备表、motor instance、controller registry、`watch.runtime` 和 SD 启动设备记录已接入；控制器统一调度和安全策略还在演进 |

这张状态表只描述主线代码结构，不等于每台车都已经完成上车验证。上车前仍按 [上车检查清单](bringup-checklist.md) 做。
