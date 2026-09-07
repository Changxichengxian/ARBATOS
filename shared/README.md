# 共享代码与 Zephyr 接口

`shared/` 放跨板、跨车型复用的控制、协议、驱动和基础算法。车型参数与电机装配在 `Robotconfig/`，板子的引脚和端口在 `boards/`，工程启动入口与显式源码清单在 `projects/`。共享逻辑通过 `g_config`、任务模块或电机能力表接收差异，避免写死车型名。

按接口查看：[算法](#算法兼容层)、[CAN](#can)、[串口与 RS485](#uart-与-rs485)、[USB](#usb-cdc)、[平台与传感器](#平台资源与传感器)、[SD 与副板](#sd-卡与-m-板副板)、[ELRS](#elrs-与-sx1281)。

```text
shared/
|-- application/   控制任务、通信、输入、电机、诊断、日志和校准服务
|-- components/    算法、控制器、设备驱动、通用类型和支持库
|-- zephyr/        当前 Zephyr 适配和兼容层
|-- third_party/   保留独立版本与许可证的第三方源码
`-- hal/           历史 HAL 接口和实现参考
```

`application/services/calibration/` 有零偏状态机 `GyroZeroCali.h`、传统校准流程 `CalibrateTask.c` 和 pitch 补偿 `PitchCali.c`；持久保存是否可用取决于板级后端。运行观察看 `application/services/diagnostics/Watch.c`、`RtProf.c`，SD 日志规则见 [SD 日志说明](../manual/调试与日志.md#日志与复盘)。高频任务太长时，可把只供本任务使用的实现拆进同目录私有 `.inc`，由原 `.c` 包含；入口和主循环留在 `.c`，`.inc` 不进源码清单，也不能被其他 `.c` 包含。真正可复用的逻辑应成为正常 `.c/.h` 模块。拆分后运行：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action check
```

高频任务与故障策略还需对应的主机回归或硬件验证。

## 算法兼容层

`zephyr/port/algorithm/` 用 `AhrsZephyr.c` 与 `ArmMathZephyr.c` 代替旧 ARMCC 的 `AHRS.lib` 和 `arm_cortexM4lf_math.lib`，三块板共用。当前实际调用只有 `arm_sin_f32()` 与 `arm_cos_f32()`，所以 `include/arm_math.h` 只声明 `float32_t` 和这两个函数。构建时将 `shared/zephyr/port/algorithm/include` 放在旧算法头目录之前。

```cmake
shared/zephyr/port/algorithm/AhrsZephyr.c
shared/zephyr/port/algorithm/ArmMathZephyr.c
```

同时保留 `shared/components/algorithm`、`shared/components/support`；不要链接 ARMCC `.lib` 或把旧实现重复加回工程。迁移期三板最小工程曾编译链接，这不能证明与闭源库逐位一致。实车前应回放静止、匀速旋转、快速俯仰、磁力计缺失和加速度异常数据，检查四元数归一化、欧拉角方向、收敛时间和控制允许误差。

## CAN

`zephyr/port/can/` 对接 Zephyr 4.4 CAN：`can-primary` 为 CAN1/FDCAN1，`can-secondary` 为 CAN2/FDCAN2，H723 的 `can-tertiary` 为 FDCAN3。集成时加入 `BspCanZephyr.c` 并启用 `CONFIG_CAN=y`；H723 使用 FD/BRS 时再启用 `CONFIG_CAN_FD_MODE=y`。仲裁速率必须在设备树或 Kconfig 明确配置。

`BspCanTx*()` 返回 0 只代表 Zephyr 接受帧并分配邮箱。发送完成回调才生成 `BspCanTxCompletion`：0 为 `Complete`，停止或取消为 `Aborted`，已定义错误为 `Failed`，其他驱动错误为 `Unknown`。中断回调只更新状态、写固定环形队列并通知接收任务，不分配内存；完成队列满时终态先留在发送槽，之后由轮询接口搬运。

Zephyr 4.4 没有已保证的路径，可在致命异常中绕过内核锁、撤销指定邮箱并有界确认紧急发送。因此 `BspCanFaultTx()` 目前明确失败，`BspCanZephyrFaultTxSupported()` 为 0，`BspCanFaultWaitTxIdle()` 不会伪造完成。投入实车故障停机前，必须有独立硬件断能，或另行实现、审计并实测 bxCAN/FDCAN 原始寄存器紧急发送。`LastErrorCode`、`DataLastErrorCode`、控制器 activity 与 FDCAN error logging count 暂无公共 API，当前返回 0；错误计数和控制器状态来自 `can_get_state()`。

仍需验证多路满载的回调归属与顺序、Classic/FD/BRS、无 ACK/仲裁丢失/Bus-Off、RX 与完成队列满、速率重配中的票据终态，以及断电、看门狗、HardFault 时执行器能由 CAN 外的措施可靠断能。

## UART 与 RS485

`zephyr/port/uart/` 提供 `BspRc`、`BspUsart`，全部缓冲固定，不用堆内存。可用别名 `uart-rc`、`uart-aux`、`uart-referee`、`uart-rs485-0`、`uart-rs485-1`，或由目标配置覆盖：

```c
#define ARB_UART_RC_NODE       DT_NODELABEL(usart3)
#define ARB_UART_AUX_NODE      DT_NODELABEL(usart1)
#define ARB_UART_REFEREE_NODE  DT_NODELABEL(usart6)
#define ARB_UART_RS485_0_NODE  DT_NODELABEL(usart2)
#define ARB_UART_RS485_1_NODE  DT_NODELABEL(uart4)
```

示例只说明写法，实际节点和引脚以目标设备树为准：

- 一个 UART 同时只能承担一个角色；角色缺失返回 `-ENODEV`，不会假装启动成功。
- 优先使用 Zephyr async UART；不可用时，AUX、RS485 退回 IRQ 逐字节接收，RC 与裁判用固定双缓冲，静默 `ARB_UART_IDLE_TIMEOUT_US` 后提交。
- RC 只接受当前共享解析器使用的 18 字节 DJI DBUS 帧，25 字节 SBUS 需先改解析器。
- 发送先复制进本端缓冲，常规长度上限 512 字节，RS485 为 40 字节，超限返回 `-EMSGSIZE`。
- AUX 缓冲写满后的短暂重启窗口不适合未经实测的持续高速流；需要零间隙时应选 async DMA 驱动并测实际吞吐。
- H7 故障锁只能阻止普通任务后续发送，原始紧急发送仍未支持。

## USB CDC

`zephyr/port/usb/` 将旧 `usbd_cdc_if.h` 接到 Zephyr 4.4 新 USB 设备栈，发送用固定 2048 字节环形队列。`CDC_Transmit_FS()` 会完整复制数据但不等待主机完成；未配置、挂起、DTR 未置位或空间不足返回 `USBD_BUSY`，无效指针、过大单包或后端不可用返回 `USBD_FAIL`。每次要么整包入队，要么不入队。断连或 reset 会清空未发数据并计入诊断，`BspUsbCdcGetDiag()` 可读连接、DTR、水位和收发错误计数。

```cmake
shared/zephyr/port/usb/BspZephyrUsbCdc.c
```

```conf
CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_USBD_CDC_ACM_CLASS=y
CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=n
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_UART_LINE_CTRL=y
```

设备树在 `zephyr_udc0` 下只保留一个状态为 `okay` 的 `zephyr,cdc-acm-uart` 节点，因为代码使用 `DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart)`。`BspUsbDeviceInit()` 只调用 `BspUsbCdcInit()`，USB 上下文只能初始化和启用一次。正式产品必须替换测试用或旧工程沿用的 VID、PID、厂商和产品字符串。当前公共工程含 USB 栈，M 板保留 CDC 节点；仍要单独验证 Windows 枚举、DTR、持续发送 BUSY 比例、断线重连和超过 64 字节接收分块。

加入源文件时，也要把 `shared/zephyr/port/usb` 加入头文件搜索目录，以使用当前的 `usbd_cdc_if.h` 兼容接口。

## 平台资源与传感器

`zephyr/port/platform/` 通过 `/arbatos_platform` 描述 GPIO/PWM，属性以 `boards/dts/bindings/arbatos,platform.yaml` 为准：`key-gpios`、`led0-gpios`、`buzzer-pwms`、`servo-pwms`、`shoot-trig-gpios`。`ArbatosPlatformInit()` 缺资源会返回错误，调用者必须处理。M 板 ADC1 通道 4、19 用 16 位和标称 3.3 V 换算，电池电压是索引 0 通道乘 11，精度还需按 VDDA 与分压校准；采样失败返回 `NAN`。A/C 通用 ADC 后端还不能提供真实测量，芯片温度未实现，硬件版本返回 `0xff`。M 板的供电 GPIO、蜂鸣器和舵机是否带载可用仍取决于车型和实测。`BspResetEvidence.c` 使用普通 SRAM，重启会清空，不能提供跨重启的故障证据。

`/arbatos_platform` 当前只绑定上述按键、灯、蜂鸣器、舵机和射击触发资源，**没有 ADC 属性**；不能在该节点填写不存在的 ADC 属性来配置通道。

`zephyr/port/sensors/` 提供 BMI088、IST8310、MPU6500 与 IMU 加热兼容接口，使用 Zephyr SPI、I2C、GPIO、PWM。传感器由 `/arbatos_sensors` 及车型 overlay 选择：A 板 MPU6500/SPI5，磁力计在 AUX 总线且未完成九轴；C 板 BMI088/SPI1 和 IST8310/I2C3；M 板 BMI088/SPI2，默认未绑磁力计。BMI088 是 mode 3，MPU6500 是 mode 0；旧 DMA 接口目前只是同步 SPI 事务，未启用 Zephyr async SPI/DMA。IST8310 旧接口不返回错误，读数为零时需结合 WHO_AM_I 判断。

IMU 加热接口接收旧定时器的 CCR 比较值，**不是百分比**；按 `imu-heater-period-cycles` 换算占空比，当前 A/C/M 板分别为 50、5000、10000。

M 板 IMU 校准区为 `0x080C0000–0x080FFFFF`，两个 128 KiB 扇区各保存一份含 UID、版本、序号和 CRC 的记录；128 KiB 是扇区大小。正式固件只读，`CONFIG_ARBATOS_PREFLIGHT_ONLY` 准备模式才允许写入。HERO-M 使用其车型 `ImuMount.h`，更改安装方向要同时核对旧零偏坐标系。准备模式默认不加热；现有温控目标 40℃、41℃停热、最大 2% 占空比。保存中断电和损坏副本的故障注入尚未测试。

## SD 卡与 M 板副板

`zephyr/port/storage/` 保持 `SdCard`、`SdLog`、仓库 FatFs 与 `SdSpi` 的旧接口。F427 的 `sdmmc1` 可用 Zephyr `disk_access` 磁盘名 `SD`；其他磁盘用 `sd-disk` 别名和 `disk-name`。F407 SPI2、H723 SPI3 保留 `SdSpi.c`，用 `sd-spi` 别名指向带 `reg`、`spi-max-frequency` 和控制器 `cs-gpios` 的 SPI 子设备。未绑定时保持 `STA_NOINIT`，读写返回 `RES_NOTRDY`。

仓库的 `shared/components/support/fatfs/ff.c` 与 `BspZephyrDiskio.c` 必须成对使用，不能同时启用 Zephyr FatFs 适配，以免符号重复。缓存固定大小，SPI 为同步调用且未启用 async DMA；卡移除或传输错误会清掉初始化状态，下一次可重试。同步 SPI 没有单次超时，旧 `timeout_ms` 不能保证总线卡死时返回，恢复依赖板级复位或看门狗。M 板初始化请求 400 kHz，实为约 375 kHz；普通模式 24 MHz，仅 CMD6 成功后可到 48 MHz。HERO-M 已做过 SD 日志和歌曲播放的现场使用，但旧 HAL 的约 3.5 MB/s 数据不能用于评价当前 Zephyr 吞吐。

`zephyr/port/subboard/` 服务 M 板副板：`SubBoardBringupZephyr.c` 通过 I2C 读取 PCF8563 的 0x02..0x08，目标 overlay 要给节点 `subboard-rtc` 别名。设备缺失时返回诊断，`SdLogRtcNow()` 为 0，`SubBoardBringupPoll()` 每秒重试。旧的手动 SCL 脉冲清总线依赖改写 PB8/PB9 复用，当前 Zephyr 端口不这样做；I2C 长期被拉低时应复位副板电源或使用专用恢复方案。

`SubBoardMusic.c`、`MPreflight.c` 和 `MReceiveCheck.c` 分别提供音乐、准备和只接收模式，由工程构建开关及 `ArbatosRuntime.c` 的专用启动分支选择；不能只看车型任务表判断。正式 HERO-M 也可启动音乐服务，其操作与独立测试配置见[测试说明](../tests/README.md#zephyr-hero-m-的-sd-与音乐)。

运行层的故障与启动边界见 [运行层说明](../manual/开发与代码规范.md#程序运行结构)。

## ELRS 与 SX1281

副板计划集成 SX1281，由 M 板通过 SPI 直接运行 ELRS 无线接收。2026-09-08 已引入官方 [ExpressLRS 4.1.0](https://github.com/ExpressLRS/ExpressLRS/releases/tag/4.1.0)，位于 `shared/third_party/ExpressLRS`，以 Git 子模块固定到 `a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`，上游文件未修改。重新克隆仓库后，在仓库根目录取回该版本：

```powershell
git submodule update --init shared/third_party/ExpressLRS
```

**当前只完成源码引入，未移植到 Zephyr，也未加入固件编译或启动。** CMake 的自动头文件收集只扫描项目已有目录，第三方源码不会进入搜索路径。普通 HERO-M 构建不依赖该子模块；SPI 无线收包、对频、失联处理均未实测。上游使用 PlatformIO/Arduino，射频硬件层含 ESP 平台调用，不能直接加入当前 CMake 源码清单就使用。

后续移植主要从 `src/lib/SX1280Driver/`（SX128x 驱动）、`src/lib/FHSS/`（跳频）、`src/lib/OTA/`（空中数据格式）和 `src/src/rx_main.cpp`（接收流程）查看。SX1281 可沿用 SX128x 路线，依据为[上游维护者说明](https://github.com/ExpressLRS/ExpressLRS/discussions/3371)；这不代表已有 H723/Zephyr 成品端口。硬件接线和现有引脚冲突见[开发板说明](../boards/README.md#sx1281-副板与-lcd-口)。

现有 `application/input/ElrsTask.c` 处理外置接收机的串口 CRSF 数据，未实现 SPI 射频驱动；HERO-M 的 `ROBOT_TASK_BUILD_ELRS_LINK` 仍为 0，当前显式源码清单也未加入该任务。未来需完成 SPI、BUSY 等待、中断、微秒定时、跳频同步、对频配置和断连判断，再把有效通道接到 `ManualInputUpdateElrsChannelsGuarded()`，保留现有手动输入选择和失联保护。不能只打开原串口任务来启用 SX1281。

引入的 4.1.0 源码面向 ELRS 4.x；实际接收方案仍需按发射端版本和模式验证。先验证射频芯片通信、收包与失联，再在 SD 日志、IMU 和控制任务同时运行时测延迟。第三方源码保留 GPL-3.0 及各文件自己的声明，详见[第三方材料](../授权与贡献说明.md#第三方材料)。
