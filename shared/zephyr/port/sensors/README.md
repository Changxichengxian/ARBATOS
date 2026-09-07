# Zephyr 传感器端口

本目录替换旧 HAL 句柄访问，提供 BMI088、IST8310、MPU6500 和 IMU 加热 PWM 的兼容接口。实现只使用 Zephyr 的 `spi`、`i2c`、`gpio`、`pwm` 与内核延时接口。

各板设备树通过 `/arbatos_sensors` 选择实际传感器与加热资源，不要把不同板的属性混在一份模板中。当前默认映射：

| 板 | IMU | 磁力计 | 加热 PWM / 原始周期值 |
| --- | --- | --- | --- |
| DJI A F427 | MPU6500，SPI5，CS PF6，8 MHz | 挂在 MPU AUX 总线，当前九轴接入尚未完成 | TIM3_CH2 / 50 |
| DJI C F407 | BMI088，SPI1，CS PA4/PB0，8 MHz | IST8310，I2C3，复位 PG6 | TIM10_CH1 / 5000 |
| DM MC02 H7 | BMI088，SPI2，CS PC0/PC3，8 MHz | 当前默认未绑定 | TIM3_CH4 / 10000 |

实际属性和引脚以 `boards/<板名>/zephyr/*.dts` 及车型 overlay 为准。BMI088 使用 SPI mode 3，MPU6500 使用 mode 0。加热接口保留旧 CCR 原始值语义；周期值用于把调用者数值换算成占空比，业务任务仍需提供温度与功率限制。

IST8310 的旧接口没有错误返回值，通信失败时读数会是零，调用方应结合 WHO_AM_I 初始化结果判断故障。

MPU6500 的遗留 “DMA” 接口目前完成一笔同步的固定长度 SPI 事务，再由 `finish` 解析缓存；它没有宣称已经使用 Zephyr 异步 SPI/DMA。需要中断并行采样时，应在实板测量后单独接入 `spi_transceive_cb`。

融合任务通过 `ImuFrame.h` 应用安装方向；HERO-M 使用车型自己的 `ImuMount.h`，其余目标保留历史 90° 矩阵。M 板通过 `ImuCalStore.c` 保存校准，A/C 板持久校准仍未完成。新增车辆时应核对方向、零偏和温升。A 板磁力计接在 MPU 的辅助总线上，不能直接套用 C 板 I²C 驱动。

## M 板校准

`ImuCalStore.c` 的存储区为 `0x080C0000–0x080FFFFF`，两份 128 KiB 扇区，程序限制在前 768 KiB。记录检查板 UID、版本、序号与 CRC。正式固件只加载，保存只允许 `CONFIG_ARBATOS_PREFLIGHT_ONLY` 准备模式，运行中保存返回 `-EPERM`。

HERO-M 使用自己的 `ImuMount.h`；更换安装方向需同步核对已有零偏的坐标系，不能只改姿态显示。准备模式默认不加热；现有 M 板温控目标 40℃、超过 41℃停热、最大 2% 占空比。保存、断电加载和轴向实测见 [验证记录](../../../../tests/ZephyrMusicM/Validation.md)，写入中断电及损坏副本的故障注入仍未测试。
