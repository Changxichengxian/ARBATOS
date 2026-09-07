# Zephyr 传感器端口

本目录替换旧 HAL 句柄访问，提供 BMI088、IST8310、MPU6500 和 IMU 加热 PWM 的兼容接口。实现只使用 Zephyr 的 `spi`、`i2c`、`gpio`、`pwm` 与内核延时接口。

各板设备树必须定义 `/arbatos_sensors`，缺失时会在编译期停止，避免带着错误引脚上车。最低要求如下：

```dts
/ {
    arbatos_sensors {
        bmi088-spi = <&spi1>;
        bmi088-accel-cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;
        bmi088-gyro-cs-gpios = <&gpiob 0 GPIO_ACTIVE_LOW>;
        bmi088-spi-hz = <8000000>;

        ist8310-i2c = <&i2c3>;
        ist8310-reset-gpios = <&gpioa 1 GPIO_ACTIVE_LOW>;

        mpu6500-spi = <&spi5>;
        mpu6500-cs-gpios = <&gpiof 6 GPIO_ACTIVE_LOW>;
        mpu6500-spi-hz = <8000000>;

        imu-heater-pwms = <&pwm3 2 PWM_USEC(1000) PWM_POLARITY_NORMAL>;
        imu-heater-period-cycles = <1000>;
    };
};
```

示例引脚仅展示属性格式，不代表三块板的最终接线；必须根据实物原理图、CubeMX 和示波器结果改写。BMI088 使用 SPI mode 3；F427 的 MPU6500 依据现有 CubeMX 配置使用 SPI mode 0。BMI088 的片选由旧驱动逐字节控制，微秒等待仍使用忙等。IMU 加热 PWM 保留旧 CCR 原始值语义：F407 周期填 5000、F427 填 50、H7 填 10000；传入值只会被限制在周期内。IST8310 的旧接口没有错误返回值；通信失败时读取值会是零，调用方应结合其 WHO_AM_I 初始化结果判定故障。

MPU6500 的遗留 “DMA” 接口目前完成一笔同步的固定长度 SPI 事务，再由 `finish` 解析缓存；它没有宣称已经使用 Zephyr 异步 SPI/DMA。需要中断并行采样时，应在实板测量后单独接入 `spi_transceive_cb`。

融合任务通过 `ImuFrame.h` 应用安装方向；HERO-M 使用车型自己的 `ImuMount.h`，其余目标保留历史 90° 矩阵。M 板通过 `ImuCalStore.c` 保存校准，A/C 板持久校准仍未完成。新增车辆时应核对方向、零偏和温升。A 板磁力计接在 MPU 的辅助总线上，不能直接套用 C 板 I²C 驱动。
