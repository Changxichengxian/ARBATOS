# Projects

`projects/` 放的是可以直接打开和编译的目标工程入口。

这里表达三件事：

- 具体机器人目标。
- 对应的 Keil 工程。
- 这个工程编译哪个 target 的 `config.c`。

这里不放硬件板抽象，也不放共用控制逻辑。看板级外设去 `boards/`，看机器人参数去 `target/`，看共用控制去 `shared/`。

## 当前入口

| Target | Keil 工程 | 默认配置 |
|---|---|---|
| `HERO` | `projects/HERO/MDK-ARM/HERO.uvprojx` | `target/HERO/User/application/config.c` |
| `INFANTRY` | `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx` | `target/INFANTRY/User/application/config.c` |
| `SENTINEL` | `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx` | `target/SENTINEL/User/application/config.c` |
| `CARRIER` | `projects/CARRIER/MDK-ARM/CARRIER.uvprojx` | `target/CARRIER/User/application/config.c` |

每个工程都直接编译对应 target 的 `config.c` 和 `config.h`。

## 配置边界

- `config.c` 里写默认参数、轴电机装配和 `g_motor_config` 电机型号表。
- `g_config.motor` 记录每个轴装什么电机、CAN ID 是多少。
- `g_motor_config` 只记录每种电机自己的固定参数。
- AUX 调参只改调参表里的字段，不改 `g_config.motor` 这种启动前就确定的装配信息。
