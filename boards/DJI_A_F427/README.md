# DJI A F427

`boards/DJI_A_F427/` 只表示 DJI A 开发板这一层，不混入具体机器人目标。

## 当前内容

- `bsp/INS_task.c`：A 板 IMU 姿态任务。
- `bsp/bsp_rc_port.c`：遥控器端口适配。
- `bsp/bsp_referee_port.c`：裁判系统端口适配。
- `bsp/bsp_usart.c`：板级串口适配。
- `devices/mpu6500.*`：A 板使用的 MPU6500 驱动。
- `bsp/diskio.c`：FatFs 磁盘接口。

## 当前使用者

- `INFANTRY`：`open_INFANTRY.cmd` 或 `projects/INFANTRY/MDK-ARM/INFANTRY.uvprojx`
- `SENTINEL`：`open_SENTINEL.cmd` 或 `projects/SENTINEL/MDK-ARM/SENTINEL.uvprojx`
- `CARRIER`：`open_CARRIER.cmd` 或 `projects/CARRIER/MDK-ARM/CARRIER.uvprojx`

## 边界

- `boards/` 放硬件板相关代码。
- `Robotconfig/` 放机器人目标配置。
- `projects/` 放能直接打开和编译的完整工程。
- `shared/hal/` 放跨板复用的外设封装。
