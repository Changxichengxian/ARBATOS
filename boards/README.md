# 开发板支持

`boards/` 只说明硬件板本身：芯片、引脚、外设和板级启动。车型的 PID、电机 ID、输入映射和安装位置分别在 `Robotconfig/` 和 `projects/`；删除车型时，A、C、M 板级支持都保留。每块板的 `zephyr/` 放设备树、引脚复用、Flash 分区和调试配置，`boards/dts/` 放共用设备树属性。旧 `bsp/`、`app/` 和 `devices/` 是 HAL 参考，不能直接加入 Zephyr 构建。

| 板 | 芯片 | Zephyr 板名 | 当前状态 |
| --- | --- | --- | --- |
| DJI A F427 | STM32F427IIH | `dji_a_f427` | 图纸和独立构建已核对，未上板 |
| DJI C F407 | STM32F407 | `dji_c_f407` | 已按既有接线恢复，未上板 |
| DM MC02 H7 | STM32H723 | `dm_mc02_h7` | HERO-M 正在使用 M 板与 V2 副板 |

## A 板：DJI A F427

A 板使用 MPU6500 系列接口，磁力计经 IMU 辅助总线接入；现有姿态仍是六轴融合。图纸依据是《RoboMaster 开发板A型 原理图.pdf》10 页，以下页码均为 PDF 阅读器页码，图纸右下角 Sheet 编号比它大 1。

| 资源 | 接线或行为 | 图纸页 |
| --- | --- | --- |
| 主晶振 | PH0/PH1，12 MHz；PLL 提供 168 MHz 与 USB/SDIO 的 48 MHz | 3 |
| CAN1 / CAN2 | PD0/PD1；PB12/PB13 | 2、3、7 |
| IMU SPI5 | PF7/PF8/PF9，PF6 低有效片选，PB8 中断 | 3、6 |
| IMU 加热、蜂鸣器 | PB5/TIM3_CH2 高有效；PH6/TIM12_CH1 高有效 | 2、3、6 |
| SDIO | PC8–PC11、PC12、PD2，4 位总线 | 2、3、5 |
| USB FS | PA11 DM、PA12 DP；PA10 是 ID，PA9 已分配给 PWM，不能当作 VBUS 感知脚 | 2、5、8 |
| 常用串口 | USART3 PD8/PD9；USART6 PG14/PG9；UART8 PE1/PE0 | 3、5、9 |

图纸修正包括：PC14/PC15 未接低速晶振，改用内部 LSI；DBUS 仅 PB7 RX、100000、偶校验、1 停止位；按键 PB2 高电平有效并下拉；绿灯 PF14、红灯 PE11、D1–D8 为 PG1–PG8 且低电平有效；插卡检测 PE15 低电平有效；蓝牙口 PD5/PD6、扩展口 PE8/PE7 默认关闭；调试器目标 F427II，SWD PA13/PA14、软件复位。

仍待确认：新板上电后读 WHO_AM_I，图纸的 MPU6600 标注不足以决定识别值和 SPI 参数；IST8310 位于 IMU AUX I2C，复位 PE2、中断 PE3，辅助总线驱动未完成，不能称九轴可用；PH2–PH5、PG13、PF4/PF5 以及 I2C2、SPI4、PWM、ADC 扩展口尚未接入通用驱动，PF4 也不是整车电池电压。A 板只做过设备树、编译和链接核对，未连接实物或下载固件。

## C 板：DJI C F407

C 板保留 BMI088/IST8310 适配。引脚与时钟来自提交 `951857f` 的既有 C 板定义，没有用 A 板图纸推断 C 板接线。新增 C 板车型前应以实际原理图复核；当前独立工程只确认设备树可编译链接，未做实板验证。

## M 板：DM MC02 H7

M 板供 HERO-M、SENTINEL-M、MINIWHEELEG-M 共用，包括 24 MHz 晶振、三路 FDCAN、SPI2 BMI088、SPI3 SD、I2C1 PCF8563、供电 GPIO、ADC、舵机和蜂鸣器。USART1 保留在板定义中，正式固件关闭串口日志，实际启用还要看车型配置和 overlay。

- SPI6 与 SPI3 共用 PC12，当前关闭 SPI6；PB15/TIM12_CH2 使用 AF2。
- USART2/3 的 RS485 硬件 DE 已配置并读回；收发器极性、时序和全部电机通信仍须按车辆核验。
- 程序限制在前 768 KiB，`0x080C0000–0x080FFFFF` 留给 IMU 校准；下载只更新程序扇区，避免全片擦除。
- 用户于 2026-09-06 确认 HERO-M 使用 M 板加 V2 副板时，底盘和俯仰能够运动。这不覆盖历史 HERO-C 接线，也不代表扩展口、LCD 或长期负载均已验证。

### SX1281 副板与 LCD 口

计划在副板集成 SX1281，由 MC02 的 STM32H723VGT6 通过 SPI 直接驱动。H723 具备所需 SPI 和 GPIO，可作为移植目标；能否满足 ELRS 收包时序还需与现有控制任务一起测试。A 板 F427、C 板 F407 也有 SPI，不能据此判定不能用，但外接引脚、资源占用和运行余量要分别核对，本次不扩展 A/C 板接线。

官方 [LCD 例程的 SPI 配置](https://github.com/dmBots/DM-MC02/blob/master/examples/CtrBoard-H7_LCD/Core/Src/spi.c) 使用 **SPI1**，并非 SPI6；[控制引脚定义](https://github.com/dmBots/DM-MC02/blob/master/examples/CtrBoard-H7_LCD/Core/Inc/main.h) 与当前设备树对应如下。这是 MCU 信号表，不能当作 LCD 插座针序：

| 信号 | MCU 引脚 | SX1281 接入条件 |
| --- | --- | --- |
| SPI1 时钟 / 主机输出 | PB3 / PD7 | 对应 SCK / MOSI |
| SPI1 片选 | PE15 | 对应 NSS，低有效；使用该片选时不再接 LCD |
| SPI1 主机输入 | PB4 | 对应 MISO；设备树已配置，但是否引到 LCD 插座未证实 |
| LCD 背光 / 复位 / 数据命令 | PB10 / PB11 / PD10 | 可评估复用为 GPIO；本次未指定为 SX1281 引脚 |

SX1281 副板需提供 NSS、SCK、MOSI、MISO、NRESET、BUSY、DIO1，以及供电和地。官方 LCD 程序只发送数据，不使用 MISO；[官方原理图](https://github.com/dmBots/DM-MC02/blob/master/drawings/2d/schematics/CtrBoard-H7_V1.0-240124.pdf) 标出了 PB4 的 SPI1_MISO 网络，但没有给出 LCD 插座完整针序与供电连接。因此应先测通插座、确认电压；若未引出 PB4，需要另取 MISO，不能按显示屏的单向 SPI 接线直接接收射频数据。

当前板定义还把 **PB10/PB11 配给已启用的 I2C2**；复用为 NRESET、BUSY 或 DIO1 前，必须确认该总线用途并在目标配置中释放它。SPI3 的 PC10/PC11/PC12 和 PE14 已用于 SD；SPI6 的候选 PC12、PA7 分别与 SD、板载灯冲突，继续保持关闭。此次不改设备树或控制输入，源码版本和待移植范围见[ELRS 说明](../shared/README.md#elrs-与-sx1281)。

## IMU 安装与校准

板级代码只处理原始 IMU 到板载坐标的旋转、零偏写入当前变量、保存后端和启动安全条件。车装方向属于 `Robotconfig/<目标>/安装说明.md`；零偏采样状态机在 `shared/application/services/calibration/GyroZeroCali.h`，不能在各板各写一套。

M 板在 `0x080C0000–0x080FFFFF` 的两个 128 KiB 扇区各保存一份校准记录；记录包含板 UID、版本、序号和 CRC，记录本身不占满扇区。正式运行只读，只有 `CONFIG_ARBATOS_PREFLIGHT_ONLY` 准备模式可保存。A/C 板的持久校准尚未完成。更换安装方向时，要同时核对已有零偏的坐标系、温升和轴向结果。

构建、下载和调试流程见 [Zephyr 开发环境说明](../manual/CLion开发指南.md)。
