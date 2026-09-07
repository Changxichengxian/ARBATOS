# 工程入口

`projects/` 负责如何构建和启动固件，CLion 打开这个目录。这里当前保留 HERO-M、SENTINEL-M、MINIWHEELEG-M 三个车型；A、C、M 板支持一直放在 `boards/`，不随车型删除。

```text
boards/
  DjiAF427/zephyr/       A 板设备树、引脚、调试配置
  DjiCF407/zephyr/       C 板设备树、引脚、调试配置
  DmMc02H7/zephyr/       M 板设备树、引脚、调试配置
  dts/                  共用设备树属性
shared/
  application/          共用机器人控制逻辑
  components/           算法、协议、设备
  zephyr/               共用系统适配、外设接口、兼容层
projects/
  CMakeLists.txt        统一工程入口
  CMakePresets.json     CLion 与命令行共用预设
  cmake/                显式源码清单
  src/                  main 与任务启动
  HERO-M/prj.conf       英雄构建配置；测试模式也在此目录
  SENTINEL-M/           哨兵构建配置与副板配置
  MINIWHEELEG-M/        小轮腿构建配置
Robotconfig/            车型参数、控制配置、安装方向
local/build/            所有新编译产物，不提交 Git
```

## 使用

在仓库根目录运行：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Project HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Action debug -Project HERO-M
```

编译默认 2 个并行任务，产物在 `local/build/hero-m/zephyr/`。下载与调试是独立操作，不会在编译后自动执行。CLion 启用 `hero-m-local`，构建目标选 `zephyr_final`。完整步骤见 [CLion 使用说明](../manual/clion-zephyr.md)。

本机仍使用 Zephyr 4.4.0、Zephyr SDK 1.0.1、West 1.5.0。构建不读取 Keil 工程，不需要 CubeCLT。旧工程和车型在 Git `951857f` 中可查，后续提交统一在 `main`。

## 新增 A、C 或其他板的车型

1. 在 `Robotconfig/<新车型>/` 放车型参数、设备表、安装方向和接口选择。
2. 在 `projects/<新车型>/prj.conf` 放构建配置，需要改默认引脚/串口用途时增加 `app.overlay`。
3. 在 `Kconfig` 添加车型选项，在 `cmake/ArbatosLegacy.cmake` 明确列出源码和使用的 `boards/<板名>`。不要从整车目录复制 CubeMX 驱动、启动汇编或 FreeRTOS 内核。
4. 为新车型加入 CMake 预设；A 板选 `dji_a_f427`，C 板选 `dji_c_f407`，M 板选 `dm_mc02_h7`。同时更新 `tools/build.ps1`、`tools/build-matrix.ps1` 和 `tools/CheckZephyr.py` 的车型表。
5. 先编译，再按供电、传感器、通讯、悬空电机顺序验收。独立板级测试在 `tests/Boards/`，不依赖任何旧车型。

`shared/zephyr/port/` 保留 A 板 MPU6500、C/M 板 BMI088、C 板 IST8310、F4 的 bxCAN 和 H7 的 FDCAN 等路径。保留板级支持并不意味着所有扩展口都已实现：A 板具体核对和缺项见 [A 板原理图记录](../boards/DjiAF427/PinAudit.md)。

2026-09-06 用户确认 HERO-M 整车运动正常，包括底盘和俯仰；这是当时 M 板接线与固件的实车记录，不能替代当前代码或其他开发板的硬件验证，见 [实车记录](../tests/ZephyrMusicM/NormalOutput-20260906.md)。
