# DM MC02 H7

这里是 M 板的硬件支持层，Zephyr 板名为 `dm_mc02_h7`。当前 HERO-M、SENTINEL-M、MINIWHEELEG-M 共用此板；A、C 板在相邻目录继续保留。

- `zephyr/`：STM32H723、时钟、引脚、外设、Flash 分区与 OpenOCD 配置。
- `bsp/`、`app/`：原有板级接口、安装方向和 HAL 实现参考。旧 `BoardMain.c`、`BoardFreertos.c`、`InsTask.c` 不直接编译进 Zephyr。
- 当前任务启动由 `projects/src/ArbatosRuntime.c` 执行，按车型的 `profile.task_modules` 选择任务；当前 IMU 和外设适配在 `shared/zephyr/port/`。

CLion 打开仓库的 `projects/` 目录，选择所需车型预设。编译、下载与调试步骤见 [使用说明](../../manual/clion-zephyr.md)。

HERO-M 于 2026-09-06 由用户确认整车运动正常，包括底盘和俯仰；当前接线为 M 板 + V2 副板。历史 HERO-C 的测试对应旧接线，不能套用到现在。
