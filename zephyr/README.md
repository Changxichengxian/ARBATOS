# ARBATOS Zephyr 4.4 迁移工程

这里是 ARBATOS 三套 M 板机器人固件的正式构建入口：`HERO-M`、`SENTINEL-M` 和 `MINIWHEELEG-M`。后续开发和提交统一在 `main`，`zephyr` 分支保留探索历史。
正式源码清单由这里的 CMake 维护。

CLion 导入、共享预设、工具路径及 OpenOCD 使用方式见 [CLion 工作流](../manual/clion-zephyr.md)。

当前固定使用 Zephyr `v4.4.0`。`west.yml` 直接锁定该标签，构建系统还会要求
找到 4.4，避免工作区版本悄悄漂移。当前本机验证组合是：

- Zephyr 4.4.0，提交 `684c9e8`
- Zephyr SDK 1.0.1，GNU Arm Embedded 14.3
- west 1.5.0

## 当前结论

迁移初期的工作已将当前三目标所需的 `Robotconfig/` 和共享业务源码接进 Zephyr。2026-09-06 实车测试后，用户确认 **HERO-M 整车运动正常，包括底盘和云台俯仰**。当前 HERO 使用 M 板与第二版副板接线；旧 C 板记录仅作历史例子，不能套用到当前接线。

本次还完成 M 板 SD 高速读取与歌曲播放、安装坐标修正、IMU 校准保存和 HERO 报警/按键调整。详见 [本次实车节点及历史硬件区分](../tests/ZephyrMusicM/NormalOutput-20260906.md)。其他车型的整车实测不能由 HERO-M 的结果代替；新版音乐实体按钮、故障停机与高负载等未完成项仍按各自记录验收。

三目标此前均已编译通过；本次删除旧工程后重新编译 HERO-M，通过后占用 FLASH 533,136 B / 768 KiB、RAM 238,660 B / 320 KiB。其他两车型本轮做配置和源码检查，未重新进行整车实测。

## 怎么构建

在仓库根目录运行，默认使用当前正式 HERO-M：

```powershell
pwsh -NoProfile -File .\tools\build.ps1
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project all -Pristine
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Action debug -Project HERO-M
```

脚本优先使用项目本地 Zephyr 环境，也允许通过 `-West`、`-Ninja` 和环境变量覆盖。默认输出在 `out/zephyr/<target>/`；使用新的 `-BuildRoot` 可保留旧构建。`-Pristine` 用于重新生成选定构建目录，不是烧录操作。

CLion 可打开此目录，导入 `CMakePresets.json` 中的正式目标。共享预设要求已经配置 Zephyr 环境，本机 `CMakeUserPresets.json` 已准备三个 `*-local` 配置，个人文件不提交。

主要产物为 `zephyr.elf`、`zephyr.bin`、`zephyr.hex` 和 `zephyr.map`。SENTINEL-M 的副板 overlay 由构建脚本和预设自动带入。当前 M 板已配置 OpenOCD runner，构建不会自动烧录，详见 [CLion 与 OpenOCD](../manual/clion-zephyr.md)。

## 工程结构

```text
zephyr/
├─ boards/                  DM MC02 H723 自定义板
├─ targets/                 三个 M 板目标配置；Sentinel 另有 RTC 叠加配置
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
└─ scripts/build-matrix.ps1 三目标构建入口
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
