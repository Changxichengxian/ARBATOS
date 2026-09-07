# ARBATOS

ARBATOS 是面向多种开发板、多种机器人的 STM32 / Zephyr 固件项目。共享底盘、云台、射击、机械臂、轮腿、输入、通信和日志代码，车型参数与开发板支持分开维护。

**作者：** Xie Yuhan <2811158416@qq.com>

**仓库：** <https://github.com/Changxichengxian/ARBATOS.git>

## 当前主线

Zephyr 迁移已完成，后续开发统一提交到 `main`；`zephyr` 分支保留为已结束的探索记录。

- 正式工程在 `projects/`，使用 Zephyr 4.4、Zephyr SDK、CMake 和 Ninja，CLion 打开该目录。
- 下载和调试使用 OpenOCD 与 ARM GDB。当前流程不需要 Keil 或 STM32CubeCLT。
- A、C、M 三种开发板支持全部保留；目前有 HERO-M、SENTINEL-M、MINIWHEELEG-M 三个整车目标。
- 旧 HERO-C、MINIWHEELEG-C、INFANTRY-A、CARRIER-A 车型已移除。新建 A/C 板车辆时复用板级支持，按新车的接线和装配建立配置。

2026-09-06 HERO-M 实车运行成功，用户在次日明确确认：**整车运动正常，包括底盘和云台俯仰**。现在 HERO 全部采用 M 板和 V2 副板接线，旧 HERO-C 成功记录对应的 C 板接线已不适用。

迁移成功表示开发路线已切换且 HERO-M 有实车运行依据。其他车辆、全部外设、长期负载、致命故障停机和 CLion 图形调试仍按各自范围验收，见 [验证记录](tests/ZephyrMusicM/Validation.md) 和 [运行层边界](manual/runtime-architecture.md)。

## 目录怎么分

```text
ARBATOS/
├─ boards/          A、C、M 板支持、引脚、设备树和调试配置
├─ shared/          共用控制、通信、算法、外设和 Zephyr 适配
├─ projects/        工程入口、车型构建配置、源码清单和启动代码
├─ Robotconfig/     每台车的任务、设备、电机、PID、输入和安装方向
├─ manual/          接车、开发、调试和日志手册
├─ tools/           构建入口、检查、日志解析和离线工具
├─ tests/           独立测试源码与验证记录
├─ legal/           许可证、贡献和第三方说明
└─ local/           本机环境、缓存、日志；新构建统一输出到 local/build/
```

换板子的引脚和外设改 `boards/`；换车的参数改 `Robotconfig/`；可复用逻辑改 `shared/`；构建和启动配置改 `projects/`。`shared/hal/` 保留历史接口与实现参考，当前系统适配在 `shared/zephyr/`。

| 开发板 | MCU | Zephyr 板名 | 当前整车目标 |
| --- | --- | --- | --- |
| DJI A（DjiAF427） | STM32F427II | `dji_a_f427` | 待新增，板级支持保留 |
| DJI C（DjiCF407） | STM32F407 | `dji_c_f407` | 待新增，板级支持保留 |
| DM MC02（DmMc02H7） | STM32H723 | `dm_mc02_h7` | HERO-M、SENTINEL-M、MINIWHEELEG-M |

A/C 板独立构建检查见 [tests/Boards](tests/Boards/README.md)，A 板引脚核对与尚未实现的接口见 [原理图核对记录](boards/DjiAF427/PinAudit.md)。板级编译通过不能代替新车实测。

## 编译、下载、调试

本机已准备好工具环境。首次使用或换电脑先看 [环境说明](manual/clion-zephyr.md)。在仓库根目录运行：

```powershell
# 检查工程文件，不编译
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
# 编译英雄，默认 2 个并行任务
pwsh -NoProfile -File .\tools\build.ps1 -Project HERO-M
# 接好目标板并准备好车辆后，下载并校验
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M
# 板上固件与 ELF 一致时进入调试，不重复下载
pwsh -NoProfile -File .\tools\build.ps1 -Action debug -Project HERO-M
```

产物在 `local/build/hero-m/zephyr/`，包括 `zephyr.elf`、`.bin`、`.hex` 和 `.map`。下载不会自动编译；改代码后先 build，再 flash。普通断点会暂停控制任务，带动力调试前先卸载或断开执行机构动力。

CLion 打开 `D:\ARBATOS\projects`，启用 `hero-m-local`，构建目标选 `zephyr_final`。每次只启用需要的车型预设。详细界面设置和其他车型入口见 [CLion 编译、下载和调试](manual/clion-zephyr.md)。

## 运行代码入口

- `projects/src/main.c`、`ArbatosTarget.c`、`ArbatosRuntime.c`：系统启动、目标选择和任务创建。
- `Robotconfig/<车型>/Config*.inc`：任务模块、设备装配、输入映射和控制参数。
- `shared/application/`：输入与控制任务；命令经 `LowCmd`、`CanTxTask` 发送，反馈经 `CanRxTask` 更新。
- `shared/application/robot/`：设备表、电机实例、控制器管理和状态快照。
- `shared/zephyr/port/`：CAN、UART、传感器、存储、平台和副板适配。

新增车型见 [新车接入](manual/new-target.md)，新增任务见 [模块声明](manual/module-system.md)，运行层结构和当前限制见 [运行层说明](manual/runtime-architecture.md)。

## 文档入口

| 内容 | 文档 |
| --- | --- |
| 第一次使用 | [快速上手](QuickStart.md) |
| 按任务查操作步骤 | [手册索引](manual/README.md) |
| 配置、板级、共享代码 | [Robotconfig](Robotconfig/README.md)、[boards](boards/README.md)、[shared](shared/README.md) |
| 上车前确认 | [检查清单](manual/bringup-checklist.md) |
| 日志与固件身份 | [SD 日志](manual/sdlog.md) |
| 脚本和离线测试 | [工具说明](tools/README.md) |
| 上位机算法通信 | [算法接入协议](AlgorithmAccessProtocol.md) |

被移除的 Keil/CubeMX 工程、旧车型和旧构建工具可在历史提交 `951857f` 或探索分支的 `6bdf19e` 查看。复现历史固件时使用单独检出目录，避免覆盖当前工程。

## 许可证

ARBATOS 自有代码和文档采用 [Apache-2.0](LICENSE)，允许商业使用，需保留许可证及署名等要求。第三方代码和工具继续遵守各自许可证，见 [第三方说明](legal/ThirdParty.md)、[贡献说明](legal/CONTRIBUTING.md) 和 [CLA](legal/CLA.md)。
