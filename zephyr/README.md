# ARBATOS Zephyr 4.4 迁移工程

这里是 ARBATOS 七套机器人固件的正式构建入口。后续开发和提交统一在 `main`，`zephyr` 分支保留探索历史。
正式源码清单由这里的 CMake 维护，不读取旧 Keil 工程，也不要求安装 Keil；旧工程保留作历史参考。

CLion 导入、共享预设、工具路径及 OpenOCD 使用方式见 [CLion 工作流](../manual/clion-zephyr.md)。

当前固定使用 Zephyr `v4.4.0`。`west.yml` 直接锁定该标签，构建系统还会要求
找到 4.4，避免工作区版本悄悄漂移。当前本机验证组合是：

- Zephyr 4.4.0，提交 `684c9e8`
- Zephyr SDK 1.0.1，GNU Arm Embedded 14.3
- west 1.5.0

## 当前结论

迁移初期七个目标已把原 `Robotconfig/` 和共享业务源码接进 Zephyr，并完成全新构建。2026-09-06 22:22，用户确认 **HERO-M 整车运动正常，包括底盘和云台俯仰**，并于次日澄清该反馈范围。当前 HERO 已彻底改成 M 板与第二版副板接线，旧 HERO-C 上车通过的记录只适用于旧 C 板接线。

本次还完成 M 板 SD 高速读取与歌曲播放、安装坐标修正、IMU 校准保存和 HERO 报警/按键调整。详见 [本次实车节点及历史硬件区分](../tests/ZephyrMusicM/NormalOutput-20260906.md)。其他车型的整车实测不能由 HERO-M 的结果代替；新版音乐实体按钮、故障停机与高负载等未完成项仍按各自记录验收。

下表保留迁移初期的七目标内存用量，属于历史构建快照，不代表本次代码的全量重建结果。

| 目标 | Zephyr 板 | 芯片 | FLASH | 主 RAM | CCM/DTCM |
|---|---|---|---:|---:|---:|
| HERO-C | `dji_c_f407` | STM32F407 | 518,164 B / 1 MB，49.42% | 123,076 B / 128 KB，93.90% | 56,976 B / 64 KB，86.94% |
| HERO-M | `dm_mc02_h7` | STM32H723 | 508,440 B / 1 MB，48.49% | 187,588 B / 320 KB，57.25% | 56,024 B / 128 KB，42.74% |
| INFANTRY-A | `dji_a_f427` | STM32F427 | 504,404 B / 2 MB，24.05% | 111,300 B / 192 KB，56.61% | 48,272 B / 64 KB，73.66% |
| SENTINEL-M | `dm_mc02_h7` | STM32H723 | 502,336 B / 1 MB，47.91% | 180,036 B / 320 KB，54.94% | 55,896 B / 128 KB，42.65% |
| CARRIER-A | `dji_a_f427` | STM32F427 | 402,156 B / 2 MB，19.18% | 92,356 B / 192 KB，46.97% | 40,720 B / 64 KB，62.13% |
| MINIWHEELEG-M | `dm_mc02_h7` | STM32H723 | 478,672 B / 1 MB，45.65% | 178,628 B / 320 KB，54.51% | 50,776 B / 128 KB，38.74% |
| MINIWHEELEG-C | `dji_c_f407` | STM32F407 | 486,960 B / 1 MB，46.44% | 113,732 B / 128 KB，86.77% | 51,856 B / 64 KB，79.13% |

F407 容量最紧。任务栈、任务登记表和 CAN 的纯 CPU 状态已经放进 CCM；SD、USB、
UART 等可能参与外设传输的缓冲仍留在主 RAM，避免 DMA 访问不到 CCM。七个目标均
启用硬件单精度浮点和线程间浮点上下文共享。

## 怎么构建

在仓库根目录运行，默认使用当前正式 HERO-M：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project all
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
```

脚本优先使用项目本地 Zephyr 环境，也允许通过 `-West`、`-Ninja` 和环境变量覆盖。默认输出在 `out/zephyr/<target>/`；使用新的 `-BuildRoot` 可保留旧构建。`-Pristine` 用于重新生成选定构建目录，不是烧录操作。

CLion 可打开此目录，导入 `CMakePresets.json` 中的正式目标。共享预设要求已经配置 Zephyr 环境，本机 `CMakeUserPresets.json` 已准备 `hero-m-local`，个人文件不提交。

主要产物为 `zephyr.elf`、`zephyr.bin`、`zephyr.hex` 和 `zephyr.map`。SENTINEL-M 的副板 overlay 由构建脚本和预设自动带入。当前 M 板已配置 OpenOCD runner，构建不会自动烧录，详见 [CLion 与 OpenOCD](../manual/clion-zephyr.md)。

## 工程结构

```text
zephyr/
├─ boards/                  三块自定义板：F407、F427、H723
├─ targets/                 七个机器人目标配置；Sentinel 另有 RTC 叠加配置
├─ cmake/                   每个目标复用旧源码的显式清单
├─ compat/                  FreeRTOS、CMSIS-RTOS2、少量 STM32 兼容接口
├─ port/
│  ├─ algorithm/            AHRS 与最小 CMSIS-DSP 数学兼容
│  ├─ can/                  bxCAN/FDCAN、发送完成和故障锁
│  ├─ uart/                 DBUS、裁判、AUX、RS485
│  ├─ usb/                  Zephyr 4.4 新 USB 设备栈上的 CDC ACM
│  ├─ sensors/              BMI088、IST8310、MPU6500、IMU 加热
│  ├─ storage/              F427 SDMMC；F407/H723 SPI SD
│  ├─ subboard/             Sentinel PCF8563 RTC
│  └─ platform/             蜂鸣器、按键、灯、ADC、Flash 等板级边界
├─ src/                     Zephyr main 和七目标任务启动
└─ scripts/build-matrix.ps1 七目标构建入口
```

`cmake/ArbatosLegacy.cmake` 使用显式源码清单，不会扫描整个仓库。它保留
`Robotconfig/` 的车型参数、共享控制逻辑、通信协议、算法和 FatFs，同时明确排除：

- CubeMX `main.c`、启动汇编和时钟初始化
- 原 FreeRTOS 内核、CMSIS-RTOS 包装
- 原 `BoardMain.c`、`BoardFreertos.c`
- 直接依赖 STM32 HAL 句柄的旧板级实现
- ARMCC 专用 `.lib`

## 已迁移的运行边界

- 任务：七目标原任务组合由 Zephyr 线程启动；旧通知、信号量、互斥量、延时和
  有界堆接口由兼容层承接。控制线程的栈全部是固定静态内存并保留 Zephyr 栈保护区。
- CAN：F407/F427 两路 bxCAN，H723 三路 FDCAN；支持接收环、发送票据、
  物理发送完成、Bus-Off/错误状态和 H7 CAN FD/BRS。
- UART：遥控、裁判、AUX、H7 两路 RS485；固定双缓冲和环形队列，当前 STM32
  完整构建使用中断接收加空闲延时提交。
- USB：使用 Zephyr 4.4 新设备栈和 CDC ACM；保留 `CDC_Transmit_FS()`，
  固定 2 KB 非阻塞发送队列，接收继续交给 VisionLink。
- IMU：F407/H723 的 BMI088、F407 的 IST8310、F427 的 MPU6500，以及加热 PWM
  和共享 AHRS 入口；恢复原板 90° 安装矩阵、实际采样间隔和开机静止零偏修正。
- 存储：F427 SDMMC1 接 Zephyr `disk_access`；F407/H723 保留仓库的 SD SPI
  协议层，底层换成 Zephyr SPI。
- Sentinel 子板：I²C1 上的 PCF8563 RTC，失败后每秒重试。
- 平台：按键、蜂鸣器、单色状态灯兼容、复位原因内存记录，以及未确认资源的安全
  退化行为。

## 目前明确保留的限制

这些地方没有伪造“成功”，实机前必须正视：

1. 致命异常环境下，Zephyr 公共 CAN/RS485 接口不能保证绕过内核锁完成最后一帧。
   正常任务中的故障锁已和最终提交串行；异常上下文会原子锁定并立即复位。原始紧急
   发送仍明确返回不支持，执行器必须另有硬件断能或经审计的芯片专用寄存器实现。
2. M 板电池 ADC 已接入并读到电压，但尚未与万用表校准；C/A 板 ADC 仍保留不可用路径。
   低压报警现可配置，HERO 默认关闭；启用时按 21.6 V、连续低压 3 秒判定。
3. M 板已补充四路舵机与部分外设映射，新增外设尚未逐个完成实物通信或波形检查；
   RGB 灯及发射触发等未完成映射仍不能算作验收通过。
4. M 板新增 IMU 校准专用 Flash 双副本保存，擦写限定在静态校准模式；通用 Flash
   写入口仍拒绝写请求。致命复位证据尚不等价于原备份 SRAM 持久记录。
5. HERO-M 已按当前实车安装修正 IMU 轴向，并完成稳温校准、Flash 保存和重启加载
   检查。其他目标保留各自的安装矩阵和验证边界，不能沿用 HERO-M 的整车实测结论。
6. 当前遥控共享解析器实际接收 18 字节 DJI DBUS 帧。真正 25 字节 SBUS 帧会被拒绝，
   需要先修改共享解析协议。
7. Sentinel 旧实现的手动 SCL 脉冲清 I²C 总线没有照搬；总线被外设长期拉低时，要
   依靠子板电源复位或另做 Zephyr 下的恢复方案。
8. USB 描述符暂时延续旧工程的 ST `VID=0x0483`、`PID=0x5740` 和字符串，便于已有
   上位机继续识别。独立产品发布前必须换成合法取得的 VID/PID。
9. HERO-C 主 RAM 已用 93.90%，MINIWHEELEG-C 已用 86.77%。增加线程、USB 缓冲或日志缓存
   前必须重新看最终 map，并在实机用栈水位验证留量。

## 建议的实机验证顺序

不要第一次就接执行器满功率。建议按下面顺序逐步放开：

1. 只供电：确认时钟、Zephyr 启动、看门狗、复位和无意外 GPIO 翻转。
2. USB：Windows 枚举、DTR、连续收发、断开重连、队列满时 `BUSY` 行为。
3. 传感器：读取芯片 ID，记录静止原始数据，核对轴向、量程、温度和加热 PWM。
4. 遥控与裁判：帧长、丢帧、失联超时、重新连接和高流量接收。
5. CAN 空载：逐路回环或分析仪验证 1 Mbps、过滤器、三路 FDCAN 和错误恢复。
6. 电机离地低功率：先验证失联和停机，再验证方向、反馈 ID、限流和控制周期。
7. SD/RTC：插拔、满盘、掉电、长时间写入；Sentinel 同时验证 RTC 晚上电和 I²C 故障。
8. 最后才验证整机高负载、总线拥塞、USB/SD 并发和故障停机。

上板发现的引脚、极性、频率、量程和安全行为应优先写回 `boards/`、`targets/`
或对应 `port/`，不要重新把 CubeMX 初始化和 HAL 全局句柄带进来。
