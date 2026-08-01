# ARBATOS Zephyr 4.4 迁移工程

这里是 ARBATOS 七套机器人固件的 Zephyr 入口。它和原来的
Keil、CubeMX、FreeRTOS 工程并存，不改旧工程的启动文件和工程文件。

当前固定使用 Zephyr `v4.4.0`。`west.yml` 直接锁定该标签，构建系统还会要求
找到 4.4，避免工作区版本悄悄漂移。当前本机验证组合是：

- Zephyr 4.4.0，提交 `684c9e8`
- Zephyr SDK 1.0.1，GNU Arm Embedded 14.3
- west 1.5.0

## 当前结论

七个目标都已把原 `Robotconfig/` 和共享业务源码接进 Zephyr，并在全新构建目录
完成编译和链接。这里的“通过”只代表软件构建闭环，不能代替上电、总线和执行器
验证。

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

准备一个已安装 Zephyr 4.4 模块和 SDK 的 west 工作区，并设置：

```powershell
$env:ZEPHYR_BASE = 'D:\path\to\zephyrproject\zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = 'D:\path\to\zephyr-sdk'
```

构建全部七个目标：

```powershell
pwsh -File D:\ARBATOS\zephyr\scripts\build-matrix.ps1 -Pristine
```

只构建一个或多个目标：

```powershell
pwsh -File D:\ARBATOS\zephyr\scripts\build-matrix.ps1 `
    -Target hero-c,sentinel-m `
    -Pristine
```

若 `west` 或 Ninja 不在当前终端的 `PATH`，可显式传入：

```powershell
pwsh -File D:\ARBATOS\zephyr\scripts\build-matrix.ps1 `
    -Target hero-c `
    -Pristine `
    -West D:\path\to\west.exe `
    -Ninja D:\path\to\ninja.exe
```

产物位于 `zephyr/build-<目标>/zephyr/`，主要文件有：

- `zephyr.elf`：调试和符号分析
- `zephyr.bin`：裸二进制
- `zephyr.hex`：Intel HEX
- `zephyr.map`：内存和链接分析

也可以直接使用 west：

```powershell
west build `
    -s D:\ARBATOS\zephyr `
    -d D:\ARBATOS\zephyr\build-hero-c `
    -b dji_c_f407 `
    -p always `
    -- -DEXTRA_CONF_FILE=D:/ARBATOS/zephyr/targets/hero-c.conf
```

SENTINEL-M 还要带上：

```text
-DDTC_OVERLAY_FILE=D:/ARBATOS/zephyr/targets/sentinel-m.overlay
```

构建脚本已经自动处理该叠加配置。

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
2. 三块板的电池 ADC 通道、分压和换算关系尚无足够可靠的原理图证据，ADC 端口返回
   不可用；电池任务会保守显示 `0 V / 0%` 并保持低压告警，不会伪装成满电。
3. RGB 灯、舵机和发射触发引脚没有可信映射。状态灯接口在无映射时安全无动作；
   舵机和触发接口也不会碰未知 GPIO/PWM。
4. Flash 写入尚未改成 Zephyr 分区和擦写策略，写请求被安全拒绝。致命复位证据目前
   只保留在本次运行内存中，不等价于原备份 SRAM 持久记录。
5. IMU 已恢复原板共同使用的安装矩阵、实际采样间隔和开机静止零偏修正，但 Flash
   写入关闭意味着零偏修正不会跨重启保存。温度稳定、正负方向和姿态仍必须用原始
   数据回放及实车对照验证；状态灯变绿前应保持遥控安全挡并离地测试。
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
