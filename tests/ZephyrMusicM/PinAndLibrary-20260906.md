# M 板引脚核对与漏歌修复

## 为什么只有两首

从实板读出根目录 14 个普通文件，其中 12 首歌曲、2 个其他格式文件。10 首歌曲超过旧版 96 字节显示名或 128 字节路径缓冲，最长文件名为 184 个 UTF-8 字节。旧代码直接返回 `-ENAMETOOLONG`，扫描器又忽略返回值，因此只留下短名字的 `hajimi.U8` 和 `YOU.U8`。

现在完整路径保存到 384 字节缓冲，显示名仍为 96 字节但按 UTF-8 边界缩短，打开文件时始终使用完整路径。遍历根目录和最多 4 层普通子目录，容量为 64 首；超长路径、超出容量、不支持的文件和无法进入的子目录会记录原因。不会修改 SD 卡文件。详细歌曲表见 [Playlist-20260906.md](Playlist-20260906.md)。

13:00:24 的实板快照已识别全部 12 首，实际打开播放长文件名歌曲；按键计数为上一首 1 次、下一首 3 次，SD 为 48 MHz、42956 次调用、0 端口错误，音频输出 171087 个采样、0 断供。完整路径从 RAM 表直接读取，保存在 `local/cache/zephyr-music-hardware/20260906/library/tracks.json`，并非仅依赖串口打印。

初次目录日志过多，日志队列报告丢失 8 条消息；最终版本仅输出识别总数、播放歌曲和跳过原因，避免正常扫描的长文件名挤满日志队列。首次串口采集还遇到主机 GBK 输出异常，后续采集改为 UTF-8 增量解码。

## 引脚核对依据

对照用户提供的 `C:/Users/28111/Desktop/dm-mc02-master/图片/H7_管脚标注图.png`、原 HERO-M HAL 初始化，以及本地 STM32H723VGT6 引脚定义 `local/cache/zephyrproject/modules/hal/stm32/dts/st/h7/stm32h723vgtx-pinctrl.dtsi`。

`PinAudit.py` 核对自定义设备树与音乐串口配置中的 38 个复用值，修正后 0 处不一致。其中 `spi2_miso_pc2` 对应厂家表的 `spi2_miso_pc2_c`。这项检查只证明定义一致，不代表全部外设已完成实物通信。

| 项目 | 发现及处理 | 验证边界 |
|---|---|---|
| 蜂鸣器 PB15/TIM12_CH2 | 上一轮已将 AF9 修正为 AF2 | 已实际出声，用户确认 |
| UART10 TX PE3 | AF4 改为 AF11；RX PE2 保持 AF4 | 实板 PE3 复用寄存器确认为 11，未接 UART10 对端通信 |
| SPI6 SCK PC12 | AF8 改为 AF5 | SPI6 继续禁用，与 SD 时钟脚重叠；未启用 2812 |
| USB PA11/PA12 | AF10 改为厂家表要求的模拟模式 | 实板 GPIO 模式均为 3；未测试 M 板 USB 枚举 |
| 两路 RS485 | USART2/USART3 增加 `de-enable`，使 PD4/PB14 自动控制发送方向 | 实板两路 CR3 的 DEM 位均为 1；未对电机发指令 |
| BMI088 SPI2 | 增加软件 NSS，匹配 PC0/PC3 手动片选实现 | 编译通过；本轮未启动 IMU 任务或测量姿态 |

以下已配置的引脚与图和厂家表相符：USART1 PA9/PA10，UART5 PD2，UART7 PE7/PE8，UART8 PE0/PE1，USART2 PD4/PD5/PD6，USART3 PB14/PD8/PD9，CAN1 PD0/PD1、CAN2 PB5/PB6、CAN3 PD12/PD13，SPI2 PB13/PC1/PC2_C，SPI3 PC10/PC11/PC12，TIM3_CH4 PB1，I2C1 PB8/PB9。SD 片选为 PE14，BMI088 片选为 PC0/PC3，板载按钮为 PA15，副板按钮为 PD14/PD15。

另外检查了 PC2_C/PC3_C 的内部模拟开关：实板 `SYSCFG_PMCR=0x03000000`，PC2SO/PC3SO 均为 0，即关闭开关、接通内部路径，未把它误判为缺失配置。

## 图上仍未完整适配的部分

- PC13、PC14 两路输出使能和 PC15 的 5 V 使能没有接入 Zephyr 电源管理；本次没有打开这些输出。
- LCD 的 SPI1 目前仅配置 PB3/SCK 和 PD7/MOSI，图上的 PB4/MISO、PE15/CS 尚未补齐；I2C2 PB10/PB11 也未配置。
- PB8/PB9 的 I2C1 复用有定义，但本轮设备树没有启用 I2C1；不能把此前 HAL 的 RTC 测试当成 Zephyr RTC 已通过。
- PA5/ADC1_CH18 没有接入 Zephyr 电池电压采样，当前 `BspAdc.c` 明确返回 `NAN`。
- 外接 PWM 的 TIM1 PE9/PE13、TIM2 PA0/PA2 未配置输出；TIM2 当前作为音乐采样时钟，后续使用这两个外接 PWM 口需要协调定时器资源。
- UART9 PD14/PD15 被本次副板按钮使用，不能同时启用 UART9。
- 摄像头、QSPI Flash、板载 2812 等扩展功能不在当前已适配范围内。

完整 HERO-M 编译通过不等于运动适配完成。当前实板继续使用音乐模式，未启动底盘、云台、机械臂或加热控制任务。CPU 保持 480 MHz，没有按图片宣传值改成 550 MHz。

## 构建与证据

最终音乐版 FLASH 308348 B、主 RAM 114628 B；完整 HERO-M FLASH 520624 B、主 RAM 237892 B（72.60%）、DTCM 56024 B。最终构建日志为 `local/cache/zephyr-music-library-final.log`、`local/cache/zephyr-formal-pin-audit-final.log`。

引脚逐项结果和寄存器读数：`local/cache/zephyr-music-hardware/20260906/library/pin-audit.json`、`registers.json`。原始完整 Flash 备份仍保留在 `local/cache/zephyr-music-hardware/20260905/flash-before.bin`。

最终版已重新烧录，308348 B Flash 逐字节校验一致，BIN SHA256 为 `1cb9616b1817fae67665590af5bcaac4d27e617b2c7dd44b0c117be94b527c4e`。13:06:43 再次确认 12 首歌，正在播放长文件名的《铁血的奥尔芬斯》歌曲，451865 个采样、0 断供，98567 次 SD 调用、0 端口错误。UART8 收到识别 12 首和播放日志，RAM 日志本轮没有队列丢失；串口启动行仍有缺字，未宣称串口传输已完全无丢字。

最终固件副本、回读校验、快照和串口记录保存在 `local/cache/zephyr-music-hardware/20260906/library-final/`。最终音乐版及完整 HERO-M 均无源码编译警告，`git diff --check` 通过。
