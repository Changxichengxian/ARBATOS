# ARBATOS

面向 STM32 开发板的机器人固件，使用 Zephyr。底盘、云台、射击、轮腿、机械臂和通信代码共用，每台车单独配置电机、参数和安装方向。

**作者：** 陈轩 <2811158416@qq.com>

**仓库：** <https://github.com/Changxichengxian/ARBATOS.git>

## 当前主线

- 后续开发使用 `main`；`zephyr` 分支保留已经结束的迁移历史。
- 当前整车目标为 HERO-M、SENTINEL-M、MINIWHEELEG-M，均使用 M 板。A、C 板定义独立保留，完整运行栈仍需迁移，不能仅新建车型配置就运行。
- 使用 CLion、Zephyr 4.4、Zephyr SDK、CMake、Ninja、OpenOCD 和 ARM GDB；当前流程不需要 Keil 或 CubeCLT。
- HERO-M 已有整车正常运动结果，包括底盘和云台俯仰；当前接线为 M 板 + V2 副板。旧 HERO-C 的接线和实测结论不适用于现车。
- 2026-09-08 目录整理后的构建尚未再次上车。其他车辆、全部外设、长期负载、致命故障停机和 CLion 图形调试仍需分别验收，具体限制写在对应开发板、接口和测试说明中。
- 已引入 ExpressLRS 4.1.0 源码，为 MC02 通过 SPI 连接 SX1281 副板做准备；当前未编入固件，接线条件和待移植部分见[ELRS 说明](shared/README.md#elrs-与-sx1281)。
- 构建自动记录版本和源码指纹；H723 已接入致命异常停机、备份 SRAM 重启记录及故障后输出锁。HERO-M 已加入 CAN1 `0x212` 功率计记录和模型对照，新增硬件行为尚待实测，见[调试与日志](manual/调试与日志.md#异常停机与重启记录)。

## 开始使用

在仓库根目录双击 [打开工程.cmd](打开工程.cmd)，会找到本机 CLion 并打开 `projects`。三个车型共用同一个工程；默认沿用当前的 `hero-m-local`，构建目标选择 `zephyr_final`。

双击 [编译固件.cmd](编译固件.cmd)，按菜单选择车型，直接回车编译 HERO-M。这个入口只编译，结束后保留结果窗口；默认并行数为 2，日常增量构建。换车型的 CLion 配置和其他操作见[CLion 开发指南](manual/CLion开发指南.md)。

也可以在仓库根目录运行：

```powershell
# 检查工程文件，不连接硬件
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
# 编译 HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Project HERO-M
```

产物在 `local/build/hero-m/zephyr/`。下载、调试和环境配置见 [CLion 开发指南](manual/CLion开发指南.md)。下载不会自动编译；断点会暂停控制任务，连接实车前按[调试与日志](manual/调试与日志.md#上车检查)准备。

### 桌面客户端

双击 [打开客户端.cmd](打开客户端.cmd) 可启动本机车型配置、串口曲线和编译工具。它会自动检查首次所需的 Python 与 npm 依赖，再启动本机服务和桌面窗口；功能边界、CSV 格式、烧录确认和实际测试状态见 [客户端说明](client/README.md)。

## 改东西先找哪里

| 要做什么 | 位置 |
| --- | --- |
| 选底盘、云台、服务，创建新车型 | `Robotconfig/<车型>/RobotConfig.toml` |
| 改电机型号、编号、总线 | `Robotconfig/<车型>/ConfigHardware.inc` |
| 改 PID、限幅、遥控输入和运行模式 | `Robotconfig/<车型>/ConfigTuning.inc`、`ConfigInput.inc`、`ConfigOperation.inc` |
| 确认板子安装位置和方向 | `Robotconfig/<车型>/安装说明.md` |
| 改开发板引脚和设备树 | `boards/` |
| 改共用控制、通信、算法、日志与系统适配 | `shared/` |
| 改构建、源码清单和启动 | `projects/`，启动从 `projects/src/main.c` 进入 |
| 查构建、日志、仿真和其他离线工具 | `tools/` |
| 使用本机车型配置、串口曲线和构建桌面工具 | `client/` |
| 查独立测试的操作和结果 | `tests/` |
| 本机环境、构建产物、日志和参考资料 | `local/`，不作为正式文档入口 |

编译开关、车型任务表和启动清单统一由 `RobotConfig.toml` 生成，运行模式另行限制输出。普通接车只改 `Robotconfig/`；新增一种可复用底盘或云台，在 `shared/controllers/` 写一次算法和声明，其他车型即可直接选用。具体操作见[车型配置](Robotconfig/README.md)和[可复用底盘与云台](shared/controllers/README.md)。

## 为什么这样拆分

车型配置回答“这台车装了什么、选哪种控制”；控制算法回答“给定输入和反馈，应该怎样运动”；共用执行流程负责绑定电机、检查失联、管理输出权限和发送命令。把这些职责分开，作者写新机构时就不用重新实现遥控解析、CAN 协议和急停逻辑。

`ControlAlgorithm` 是只依赖标准 C 的算法接口，`ControlRuntime` 是执行它的共用流程；名字直接对应职责。现有控制权仍由 `ControlMgr` 管理，发送仍走 `MotorInst → LowCmd → CanTxTask`。现有经典底盘、单云台、双 yaw、轮腿控制保留原控制链；差速底盘和双轴转速云台示例用于验证新的扩展方式，尚未实车调参。

任务入口、稳定编号、默认栈和优先级只在 `RobotTaskCatalog.def` 登记一次，生成器根据车型选择补齐依赖。`local/build/<车型>/generated/robotconfig/robot-config.json` 列出实际选择及原因，查问题时可直接顺着清单找到任务和算法。`shared/hal/` 保留历史接口，系统适配在 `shared/zephyr/`。

## 文档入口

| 要了解什么 | 文档 |
| --- | --- |
| CLion、编译、下载、调试 | [CLion 开发指南](manual/CLion开发指南.md) |
| 程序结构、新增任务、代码风格 | [开发与代码规范](manual/开发与代码规范.md) |
| 上车检查、PID、日志与复盘 | [调试与日志](manual/调试与日志.md) |
| 上位机协议、坐标和方向 | [算法协议与坐标](manual/算法协议与坐标.md) |
| 车型参数、新增车型和安装模板 | [车型配置](Robotconfig/README.md) |
| 三台车的具体安装 | [HERO-M](Robotconfig/HERO-M/安装说明.md)、[SENTINEL-M](Robotconfig/SENTINEL-M/安装说明.md)、[MINIWHEELEG-M](Robotconfig/MINIWHEELEG-M/安装说明.md) |
| A/C/M 板引脚、能力和限制 | [开发板说明](boards/README.md) |
| 共用代码与 Zephyr 外设接口 | [共享代码与接口](shared/README.md) |
| 编译检查、日志、音频转换和自动调参工具 | [工具说明](tools/README.md) |
| 本机配置、串口曲线、编译与烧录客户端 | [客户端说明](client/README.md) |
| 配置仿真、轮腿物理仿真与计算参数 | [仿真说明](tools/仿真说明.md)、[轮腿参数计算报告](tools/WheelLegLqr/轮腿参数计算报告.md) |
| A/C 板、SD 和音乐独立测试 | [测试说明](tests/README.md) |
| 作者、商用、贡献和第三方材料 | [授权与贡献说明](授权与贡献说明.md) |

文档按用途合并维护，自写文件名和标题使用中文；保留 `README.md`、CLion、Zephyr 等惯用名称。接口或路径变更时同步修改对应说明和链接。实测结论保留日期、硬件与固件条件；未验证内容明确写出，不把编译结果推广为实车结果。

`manual/` 和各目录的 README 是随代码提交的正式说明。`local/docs/` 只保留本机的 `厂商手册/` 与 `原厂资料包/`；旧参考工程放 `local/reference/`，SD 日志放 `local/sdlogs/`，下载缓存和仿真生成文件放 `local/cache/`，构建产物放 `local/build/`。`tools/` 保存可复用工具源码，正式说明和待办不在 `local/docs/` 再维护一份。

旧 Keil/CubeMX 工程和已删除车型可在历史提交 `951857f` 或探索分支的 `6bdf19e` 查看。复现历史固件使用单独检出目录。

## 许可证

ARBATOS 自有代码和文档采用 [Apache-2.0](LICENSE)。第三方内容按各自许可证执行，作者信息、商用说明及贡献条款见[授权与贡献说明](授权与贡献说明.md)。
