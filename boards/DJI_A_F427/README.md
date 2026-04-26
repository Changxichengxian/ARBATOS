# DJI A F427

`boards/DJI_A_F427/` 现在只表示硬件板这一层，不再混入具体机器人目标。

这里保留的内容：
- A 板共用的板级代码和资源
- A 板相关说明

当前已经归到板层的实现：
- `User/application/INS_task.c`
- `User/bsp/boards/bsp_rc_port.c`
- `User/bsp/boards/bsp_referee_port.c`
- `User/bsp/boards/bsp_usart.c`
- `User/components/support/fatfs/diskio.c`

这里不再承担的内容：
- `INFANTRY`、`SENTINEL`、`CARRIER` 这些目标项目入口
- 目标项目自己的 CubeMX/Keil 工程目录

当前使用这块板的项目入口：
- `INFANTRY`：`open_INFANTRY.cmd` 或 `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx`
- `SENTINEL`：`open_SENTINEL.cmd` 或 `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx`
- `CARRIER`：`open_CARRIER.cmd` 或 `projects/CARRIER/MDK-ARM/CARRIER.uvprojx`

边界约定：
- `boards/` 只放硬件板
- `target/` 只放目标配置
- `projects/` 只放可直接打开和编译的目标项目

后续如果继续收紧边界，应该优先把更多 A 板共用实现继续沉到这里，而不是再往 `projects/` 或 `target/` 里塞。
