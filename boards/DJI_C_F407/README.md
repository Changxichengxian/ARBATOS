# DJI C F407

`boards/DJI_C_F407/` 现在只表示硬件板这一层，不再把 `HERO` 这种目标项目混进来。

这里负责的事情：
- 描述 DJI C F407 这块板本身
- 承接已经归位和后续继续抽离出来的板级共用实现

这里不再承担的事情：
- `HERO` 项目入口
- 目标项目自己的 CubeMX/Keil 工程目录

当前使用这块板的项目入口：
- `HERO`：`open_HERO.cmd` 或 `projects/HERO/MDK-ARM/HERO.uvprojx`

当前已经归到板层的实现：
- `app/INS_task.c`
- `bsp/bsp_rc_port.c`
- `bsp/bsp_bmi088_cfg.h`
- `bsp/bsp_buzzer_cfg.h`
- `bsp/bsp_imu_pwm_cfg.h`
- `bsp/bsp_key_cfg.h`
- `bsp/bsp_rc_cfg.h`
- `support/fatfs/diskio.c`

边界约定：
- `boards/` 只放硬件板
- `target/` 只放目标配置
- `projects/` 只放可直接打开和编译的目标项目

后续如果继续整理 C 板边界，重点应该是把还残留在目标层里的板级实现继续往板层收，而不是继续新增 `boards/HERO` 这种目录。
