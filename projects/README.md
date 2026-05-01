# Projects

`projects/` 放可以直接打开和编译的目标工程入口。这里表达三件事：

- 具体机器人目标。
- 对应的 Keil 工程。
- 这个工程编译哪个 target 的 `config.c`。

这里不放硬件板抽象，也不放共享控制逻辑。看板级外设去 `boards/`，看机器人参数去 `target/`，看共用控制去 `shared/`。

## 当前入口

| Target | Keil 工程 | 默认配置 |
|---|---|---|
| `HERO` | `projects/HERO/MDK-ARM/HERO.uvprojx` | `target/HERO/User/application/config.c` |
| `INFANTRY` | `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx` | `target/INFANTRY/User/application/config.c` |
| `SENTINEL` | `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx` | `target/SENTINEL/User/application/config.c` |
| `CARRIER` | `projects/CARRIER/MDK-ARM/CARRIER.uvprojx` | `target/CARRIER/User/application/config.c` |
| `MC02_BASE` | `projects/MC02_BASE/MDK-ARM/MC02_BASE.uvprojx` | `target/MC02_BASE/User/application/config.c` |

每个工程都直接编译对应 target 的 `config.c` 和 `config.h`。

## 和 boards 的分工

`boards/` 只保留硬件板相关内容，比如 DJI A 板、DJI C 板、DM MC02 H7 板的板级支持、引脚和外设适配。

`projects/` 才放完整工程入口。车名目录，例如 `HERO`、`INFANTRY`、`SENTINEL`、`CARRIER`，以及 H7 实验入口 `MC02_BASE`，都应该在这里出现，不应该再放到 `boards/` 下。

当前 F4 车工程统一使用 CMSIS-RTOS v2，也就是 `CMSIS_RTOS_V2/cmsis_os2.*` 这层接口；FreeRTOS 内核版本仍由 CubeMX 带进来的内核文件决定。

## 配置边界

- `config.c` 里写默认参数、轴电机装配和 `g_motor_config` 电机型号表。
- `g_config.motor` 记录每个轴装什么电机、CAN ID 是多少。
- `g_motor_config` 只记录每种电机自己的固定参数。
- AUX 调参只改调参表里的字段，不改 `g_config.motor` 这种启动前就确定的装配信息。
