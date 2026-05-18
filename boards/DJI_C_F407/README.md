# DJI C F407

`boards/DJI_C_F407/` 只表示 DJI C 开发板这一层，不再把 `HERO` 这种机器人目标混进来。

## 当前内容

- `bsp/INS_task.c`：C 板 IMU 姿态任务。
- `bsp/bsp_rc_port.c`：遥控器端口适配。
- `bsp/bsp_bmi088_cfg.h`：BMI088 板级配置。
- `bsp/bsp_buzzer_cfg.h`：蜂鸣器板级配置。
- `bsp/bsp_imu_pwm_cfg.h`：IMU 加热 PWM 配置。
- `bsp/bsp_key_cfg.h`：按键板级配置。
- `bsp/bsp_rc_cfg.h`：遥控器板级配置。
- `bsp/diskio.c`：FatFs 磁盘接口。

## 当前使用者

- `HERO`：`open_HERO.cmd` 或 `projects/HERO/MDK-ARM/HERO.uvprojx`

## 边界

- `boards/` 放硬件板相关代码。
- `Robotconfig/` 放机器人目标配置。
- `projects/` 放能直接打开和编译的完整工程。
- `shared/hal/` 放跨板复用的外设封装。
