# DJI A F427

`boards/DJI_A_F427/` 只表示 DJI A 开发板这一层，不混入具体机器人目标。

## 当前内容

- `bsp/InsTask.c`：A 板 IMU 姿态任务。
- `bsp/BspRcPort.c`：遥控器端口适配。
- `bsp/BspRefereePort.c`：裁判系统端口适配。
- `bsp/BspUsart.c`：板级串口适配。
- `devices/mpu6500.*`：A 板使用的 MPU6500 驱动。
- `bsp/Diskio.c`：FatFs 磁盘接口。

`bsp/InsTask.c` 只保留 A 板的 MPU6500 读取、安装矩阵、温控和姿态融合差异。陀螺仪零偏采样流程共用 `shared/application/services/calibration/GyroZeroCali.h`：正常上电温稳后静止 3 秒微调；`ROBOT_RUN_MODE_CALIBRATION + ROBOT_CALI_TARGET_IMU_GYRO` 下温度到 40 度后静止 30 秒并保存。

## 当前使用者

- `INFANTRY-A`：`open_INFANTRY-A.cmd` 或 `projects/INFANTRY-A/MDK-ARM/INFANTRY-A.uvprojx`
- `CARRIER-A`：`open_CARRIER-A.cmd` 或 `projects/CARRIER-A/MDK-ARM/CARRIER-A.uvprojx`

## 边界

- `boards/` 放硬件板相关代码。
- `Robotconfig/` 放机器人目标配置。
- `projects/` 放能直接打开和编译的完整工程。
- `shared/hal/` 放跨板复用的外设封装。
