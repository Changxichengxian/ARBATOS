# ARBATOS 手册

这里记录当前项目的操作流程。项目总览见 [README](../README.md)，第一次使用见 [快速上手](../QuickStart.md)。

| 要做什么 | 文档 |
| --- | --- |
| 编译、下载、调试或配置 CLion | [CLion 和 Zephyr](clion-zephyr.md) |
| 查看工程与开发板分层 | [工程入口](../projects/README.md)、[开发板](../boards/README.md) |
| 新增 A、C、M 或其他板的车型 | [新车接入](new-target.md) |
| 新增任务模块 | [模块声明](module-system.md) |
| 理解控制、设备、任务与故障边界 | [运行层说明](runtime-architecture.md) |
| 确认安装与坐标 | [坐标系和安装基准](coordinate-frames.md) |
| 第一次上电、换硬件、联调 | [上车检查清单](bringup-checklist.md) |
| 调控制参数 | [PID 调试](pid-tuning.md) |
| 查看日志、确认固件身份 | [SD 日志](sdlog.md) |
| 写代码、处理命名和注释 | [代码风格](coding-style.md) |
| 查看 HERO-M 已测结果与未完成项 | [验证记录](../tests/ZephyrMusicM/Validation.md) |

## 文档维护

- `boards/` 记录开发板支持；A、C、M 板独立于车型保留。
- `projects/` 记录构建和启动，`Robotconfig/` 记录车型装配和参数，`shared/` 记录共享实现。
- 接口或路径变更时同步更新相邻 README 和相关操作手册，避免保留两份互相矛盾的步骤。
- 已结束的迁移规划不再作为操作入口；实测结论保留日期、固件、硬件条件及限制。
- `local/docs/`、`local/reference/` 和本机日志不提交 Git。能指导复现的结论写进正式文档。
- 编译、电脑上的逻辑测试和实车测试分别说明；尚未验证的项目写明范围，不推广为全板、全车结论。
