# Zephyr 平台端口

本目录承接旧 `shared/hal` 的板级边界。板子必须提供 `/arbatos_platform` 属性；端口不会猜测 GPIO、ADC 或 PWM 的接线。缺少启动所需资源时，`ArbatosPlatformInit()` 返回错误，调用方应进入安全状态。

建议属性：`key-gpios`、`led0-gpios`、`buzzer-pwms`、`servo-pwms`、`shoot-trig-gpios`、`adc`、`battery-channel`、`hardware-version-channel`。ADC 量程、分压比与温度换算必须随实板原理图补齐。
当前三块板没有足够证据填写 ADC 映射，所以端口返回无效值；电池任务会把它明确
转换成 `0 V / 0%` 并保持低压告警，避免把未知状态显示成满电。

Zephyr 没有可跨所有 STM32 板保证等价的“致命异常后备份 SRAM 证据”接口；在板级确认 backup SRAM、复位标志与缓存维护前，重启证据只能视作未支持。USB CDC 兼容实现位于 `port/usb/`，使用 Zephyr 4.4 新 USB 设备栈；这里的 `BspUsbDeviceInit()` 只负责调用该后端的初始化入口。
