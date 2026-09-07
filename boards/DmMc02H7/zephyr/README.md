# DM MC02 H7 板级定义

本目录包含 STM32H723 的设备树、引脚复用、Flash 分区及 OpenOCD 配置。定义由原工程接线资料建立，经过 M 板与 V2 副板实测修正；当前构建不依赖旧 CubeMX 工程。

基础资源包括 24 MHz 晶振、三路 FDCAN、SPI2 BMI088、SPI3 SD、I2C1 PCF8563、供电 GPIO、ADC、舵机和蜂鸣器。USART1 保留在板定义中，正式固件关闭串口日志。各接口是否启用还要看车型配置和 overlay。

SPI6 与 SPI3 共用 PC12，当前关闭 SPI6。PB15/TIM12_CH2 使用 AF2；USART2/3 的 RS485 硬件 DE 已配置并读回确认，实际收发器极性、时序及全部电机通信仍需按车辆验证。

固件分区限制在前 768 KiB，`0x080C0000–0x080FFFFF` 留作 IMU 校准。下载按程序扇区更新，避免全片擦除；校准写入只在专用准备模式开放。

HERO-M 的整车运行、音乐、RTC、校准及未完成项见 [验证记录](../../../tests/ZephyrMusicM/Validation.md)。板定义不代表所有扩展口、LCD 或长期负载均已验证。
