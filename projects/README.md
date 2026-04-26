# Projects

`projects/` 放的是可直接打开和编译的目标项目入口。

这里表达的是：
- 具体机器人目标
- 对应的 CubeMX/Keil 工程
- 方便直接打开和构建的项目目录

这里不表达的是：
- 硬件板抽象
- 共用控制逻辑

当前目录约定：
- `projects/HERO/`
- `projects/INFANTRY/`
- `projects/SENTINEL/`
- `projects/CARRIER/`

配套分层：
- `boards/`：硬件板层，只保留板名
- `target/`：目标配置层
- `shared/`：共用逻辑层

这样分开以后：
- 看硬件差异，去 `boards/`
- 看机器人参数和目标差异，去 `target/`
- 想直接打开工程，去 `projects/`
