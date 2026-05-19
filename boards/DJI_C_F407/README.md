# DJI C F407

`boards/DJI_C_F407/` 只表示 DJI C 开发板这一层，不再把 `HERO-C` 这种机器人目标混进来。

## 当前内容

- `bsp/INS_task.c`：C 板 IMU 姿态任务。
- `bsp/bsp_rc_port.c`：遥控器端口适配。
- `bsp/bsp_bmi088_cfg.h`：BMI088 板级配置。
- `bsp/bsp_buzzer_cfg.h`：蜂鸣器板级配置。
- `bsp/bsp_imu_pwm_cfg.h`：IMU 加热 PWM 配置。
- `bsp/bsp_key_cfg.h`：按键板级配置。
- `bsp/bsp_rc_cfg.h`：遥控器板级配置。
- `bsp/diskio.c`：FatFs 磁盘接口。

`bsp/INS_task.c` 只保留 C 板的 BMI088 读取、安装矩阵、温控和姿态融合差异。陀螺仪零偏采样流程共用 `shared/application/services/calibration/gyro_zero_cali.h`：正常上电温稳后静止 3 秒微调；`TEST_MODE_IMU_GYRO_CALI` 下温度到 40 度后静止 30 秒并保存。

## 当前使用者

- `HERO-C`：`open_HERO-C.cmd` 或 `projects/HERO-C/MDK-ARM/HERO-C.uvprojx`
- `MINIWHEELEG-C`：`open_MINIWHEELEG-C.cmd` 或 `projects/MINIWHEELEG-C/MDK-ARM/MINIWHEELEG-C.uvprojx`

## 边界

- `boards/` 放硬件板相关代码。
- `Robotconfig/` 放机器人目标配置。
- `projects/` 放能直接打开和编译的完整工程。
- `shared/hal/` 放跨板复用的外设封装。
