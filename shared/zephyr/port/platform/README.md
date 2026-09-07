# Zephyr 平台端口

本目录承接 `shared/hal` 的板级接口。板子通过 `/arbatos_platform` 声明 GPIO 和 PWM 资源，属性以 `boards/dts/bindings/arbatos,platform.yaml` 为准：`key-gpios`、`led0-gpios`、`buzzer-pwms`、`servo-pwms`、`shoot-trig-gpios`。不要填写绑定中不存在的 ADC 属性。

## ADC 与输出

- M 板选择 `BspAdcM.c`，读取 ADC1 通道 4、19，16 位采样，按标称 3.3 V 换算；电池电压取索引 0 的通道电压乘 11。精度仍需按实板 VDDA 和分压校准。
- 未启动、通道无效、中断上下文或采样失败时，电压接口返回 `NAN`；芯片温度未实现，返回 `NAN`，硬件版本返回 `0xff`。
- A/C 板通用 ADC 后端尚无可用测量，不能把占位值当成真实电池电压。报警策略还受车型配置影响，不能保证所有车型都以同一方式报警。
- M 板已有供电 GPIO、蜂鸣器和舵机适配；是否实际接入、是否完成负载验证，以车型配置和实测记录为准。

## 启动与诊断

`ArbatosPlatformInit()` 初始化板级资源，缺少必需资源时返回错误，调用方需处理失败。USB CDC 的实现位于 `../usb/`，`BspUsbDeviceInit()` 调用其初始化入口。

`BspResetEvidence.c` 使用普通静态 SRAM，启动时清空，不提供跨重启的故障证据；旧 HAL 备份 SRAM 保留方案不属于当前 Zephyr 功能。完整边界见 [运行层说明](../../../../manual/runtime-architecture.md)。
