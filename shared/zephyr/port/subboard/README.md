# SENTINEL-M 子板初始化（Zephyr）

此端口替换 `Robotconfig/SENTINEL-M/SubBoardBringup.c`，保留 `SubBoardBringupRunOnce()`、`SubBoardBringupPoll()` 和 `SdLogRtcNow()` 公共接口。它通过 Zephyr I2C 读取 PCF8563 的 0x02..0x08 时间寄存器，不直接触碰 STM32 HAL、GPIO 复用或时钟配置。

## DTS 绑定

目标 overlay 需要定义一个 I2C 子节点并给它 `subboard-rtc` 别名，例如：

```dts
aliases {
    subboard-rtc = &pcf8563;
};

&i2c1 {
    pinctrl-0 = <&i2c1_scl_pb8 &i2c1_sda_pb9>;
    pinctrl-names = "default";
    clock-frequency = <I2C_BITRATE_STANDARD>;
    status = "okay";

    pcf8563: rtc@51 {
        compatible = "nxp,pcf8563";
        reg = <0x51>;
    };
};
```

若没有别名或 I2C 设备不可用，端口返回失败诊断并让 `SdLogRtcNow()` 返回 0。`SubBoardBringupPoll()` 每秒重试，方便子板晚于主板上电。

旧实现的手动 SCL 脉冲清总线依赖临时改写 PB8/PB9 复用和重置 I2C1；Zephyr 端口不做这一操作，避免绕过 pinctrl 和其他 I2C 使用者。硬件长期拉低总线时，应由子板电源复位或专用 I2C 恢复方案处理。

## 接入

构建中应以本目录的 `SubBoardBringupZephyr.c` 替换旧的 `Robotconfig/SENTINEL-M/SubBoardBringup.c`，并加入本目录到头文件搜索路径。启用 I2C 驱动及 I2C1 的 DTS/pinctrl；不需要也不应引入 STM32 HAL 的时钟重配代码。
