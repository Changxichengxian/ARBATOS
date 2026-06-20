# ARBATOS 手册入口

这里放需要跟代码一起走的正式操作文档。根目录 `README.md` 负责讲项目是什么，这个目录负责讲怎么接车、怎么上车、怎么调、怎么留日志。

`local/docs/` 仍然保留给本机资料、厂商包、临时记录和不会提交的参考材料。以后能指导别人重复操作的内容，优先沉淀到这里。

## 先看哪份

| 你要做什么 | 先看 |
|---|---|
| 判断 project、Robotconfig、board 怎么对应 | [工程目标对应表](project-map.md) |
| 新接一辆车或复制一个新目标 | [新车接入流程](new-target.md) |
| 新接一台机器人或新增模块 | [新机器人和新模块接入模板](new-robot-runtime-template.md) |
| 新增或评审机器人模块边界 | [机器人模块声明](module-system.md) |
| 第一次上车、上电、联调前检查 | [上车检查清单](bringup-checklist.md) |
| 调云台、底盘、射击 PID | [PID 调试流程](pid-tuning.md) |
| 看 SD 日志、留基线日志、改日志 tag | [SD 日志和复盘](sdlog.md) |
| 说明 SD 发布纪律、CAN 硬件边界和评分口径 | [评分边界](evaluation-boundaries.md) |
| 写代码、补注释、处理旧风格 | [代码风格](coding-style.md) |
| 了解后续通用机器人运行层方向 | [运行层演进方向](runtime-architecture.md) |

## 文档分层

- `README.md`：项目总览和当前主线状态。
- `QuickStart.md`：非常短的入门路径，适合第一次打开仓库。
- `manual/`：正式操作手册，按具体任务组织。
- `projects/README.md`：工程入口层，只讲 Keil 工程和启动入口。
- `Robotconfig/README.md`：目标配置层，只讲车型参数、装配和检测。
- `boards/README.md`：板级适配层，只讲芯片、外设和端口。
- `shared/README.md`：共享代码层，只讲复用逻辑和边界。
- `tools/README.md`：本地脚本和离线工具。
- `local/docs/`：本机资料，不进 Git，不作为当前代码的唯一说明。

## 维护规则

1. 代码行为变了，优先更新离它最近的 README，再更新这里的操作手册。
2. 新车接入、上车记录、PID 调试经验，如果别人以后会复用，就从 `local/docs/` 搬到 `manual/`。
3. 厂商 SDK、PDF、参考工程继续留在 `local/docs/` 或 `local/reference/`，不要混进正式手册。
4. 文档里出现旧路径时，优先改成当前四层结构：`projects/`、`Robotconfig/`、`boards/`、`shared/`。
5. 不确定某段内容是否还对时，写明“待实测”或“只适用于某目标”，不要写成通用结论。
