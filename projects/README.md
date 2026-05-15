# projects

`projects/` 放可以直接打开、编译、下载的固件工程入口。这里回答的是“怎么编译这份固件”，不是“这台车参数是什么”，也不是“这块板子有哪些引脚”。

## 当前入口

| Project | Keil 工程 | 使用的 target | 使用的 board |
|---|---|---|---|
| `HERO` | `projects/HERO/MDK-ARM/HERO.uvprojx` | `target/HERO` | `boards/DJI_C_F407` |
| `INFANTRY` | `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx` | `target/INFANTRY` | `boards/DJI_A_F427` |
| `SENTINEL` | `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx` | `target/SENTINEL` | `boards/DJI_A_F427` |
| `CARRIER` | `projects/CARRIER/MDK-ARM/CARRIER.uvprojx` | `target/CARRIER` | `boards/DJI_A_F427` |
| `MC02_BASE` | `projects/MC02_BASE/MDK-ARM/MC02_BASE.uvprojx` | `target/MC02_BASE` | `boards/DM_MC02_H7` |

更完整的对应表见本地文档 `local/docs/01_当前工程说明/工程-车型-板卡对应表.md`。

## 这一层负责什么

- Keil 工程文件：`MDK-ARM/*.uvprojx`。
- CubeMX 生成的启动、外设初始化和中断入口：`Core/`。
- HAL、CMSIS、FreeRTOS、USB 等工程内依赖：`Drivers/`、`Middlewares/`、`USB_DEVICE/`。
- 最终固件入口如何把 `target/`、`boards/`、`shared/` 编进来。

## 这一层不负责什么

- 车型参数、电机装配、输入映射：放 `target/<TARGET>/`。
- 板级端口和硬件适配：放 `boards/<BOARD>/`。
- 可复用控制任务、电机协议、输入链路、日志、诊断：放 `shared/`。

## 和 target / boards 的关系

现在大部分目录是同名 1 对 1，例如 `projects/HERO` 使用 `target/HERO`。但这只是当前安排，不是概念绑定。

以后可以有这些情况：

- 一个 target 有多个 project：同一台车分别做 F4 版、H7 版、实验版工程。
- 多个 project 共用一个 board：步兵、哨兵、工程都可以用 DJI A 板。
- 多个 target 共用同一批 shared 控制逻辑：底盘、云台、射击任务尽量复用。

判断文件该不该放 `projects/`：如果它影响“工程怎么编译和启动”，放这里；如果它影响“机器人怎么控制”，多数时候不该放这里。
