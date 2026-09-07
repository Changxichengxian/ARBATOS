# 快速上手

当前使用 CLion + Zephyr。A、C、M 开发板支持都保留，现有整车目标为 HERO-M、SENTINEL-M、MINIWHEELEG-M。

## 先打开工程

本机 CLion 和工具环境已准备。打开 `D:\ARBATOS\projects`，在 CMake 设置中只启用 `hero-m-local`，构建目标选 `zephyr_final`，点击构建。默认并行数为 2，平时使用增量构建。

换电脑时需要安装 CLion、Zephyr 4.4、SDK、Python/West、CMake、Ninja，并配置本机预设；SDK 不随仓库提交。完整说明见 [CLion 编译、下载和调试](manual/clion-zephyr.md)，当前不需要 CubeCLT。

## 编译和下载

在仓库根目录的 PowerShell 运行：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action probe
pwsh -NoProfile -File .\tools\build.ps1 -Project HERO-M
```

产物在 `local/build/hero-m/zephyr/`。接好目标 M 板与 CMSIS-DAP 调试器，并让执行机构断电或卸载后下载：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M
```

下载会复位并启动程序，不会自动编译。脚本只接受完整车型固件，音乐、准备和只接收测试镜像需要独立流程。调试入口：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action debug -Project HERO-M
```

该命令不重复下载；先确认板上固件与当前 ELF 一致。CLion 的图形调试设置见上述手册，需要单独完成一次实物验证。

## 改代码先找哪个目录

| 要改的内容 | 位置 |
| --- | --- |
| 电机型号、CAN ID、总线、装配 | `Robotconfig/<车型>/ConfigHardware.inc` |
| 输入通道与安全档 | `Robotconfig/<车型>/ConfigInput.inc` |
| 任务模块与运行模式 | `Robotconfig/<车型>/ConfigOperation.inc` |
| PID、方向与限幅 | `Robotconfig/<车型>/ConfigTuning.inc` |
| 板子安装方向 | `Robotconfig/<车型>/MountLayout.md`，以及该车实际使用的安装变换 |
| 在线检测 | `Robotconfig/<车型>/DetectTask.c` |
| 开发板引脚、外设 | `boards/<板名>/` 和 `shared/zephyr/port/` |
| 可复用控制、通信、算法 | `shared/` |
| 工程配置、源码清单、启动 | `projects/` |

配置任务时同时核对构建开关与 `.profile.task_modules`，缺少编译实现或启动映射的任务不会仅凭加入列表就运行。AUX 临时调参只改 RAM，重启后恢复配置文件默认值。

## 新写一辆车

按 [新车接入流程](manual/new-target.md) 建立 `Robotconfig/<新车型>/`、`projects/<新车型>/`，补齐目标选择、源码清单、预设和脚本车型表。

A 板可复用 `boards/DjiAF427`，C 板可复用 `boards/DjiCF407`；现有 M 板车型可作为业务配置参考，但接线、IMU、存储和外设能力必须按实际板子重新确认。删除旧车型不影响这两块板的支持。

## 第一次上车

1. 检查工程并编译：`pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all`。
2. 无动力确认程序、遥控、设备状态和日志。
3. 核对 IMU 安装方向、温度、零偏和 CAN 反馈 ID。
4. 每次只让一个子系统低限幅动作，检查方向、限位、安全档和失联停机。
5. 最后做联动，保存固件身份、接线、动作及日志记录。

M 板的 Flash 陀螺零偏保存仅在专用准备模式开放；正式固件只读校准，运行中保存返回 `-EPERM`。A/C 板持久校准尚未完成。不要只改运行模式就假定能够保存，见 [传感器说明](shared/zephyr/port/sensors/README.md)。

HERO-M 已有整车正常运动的 [实车记录](tests/ZephyrMusicM/Validation.md)。继续联调按 [上车检查清单](manual/bringup-checklist.md)、[PID 调试](manual/pid-tuning.md) 和 [SD 日志](manual/sdlog.md) 执行。
