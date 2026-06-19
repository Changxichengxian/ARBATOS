# boards

`boards/` 放“这块硬件板有什么”的内容。这里关心芯片、引脚、外设和板级启动，不关心某台车的 PID 或电机 ID。

## 当前板级目录

| Board | 芯片 | 说明 |
|---|---|---|
| `DjiCF407` | STM32F407 | DJI C 开发板适配 |
| `DjiAF427` | STM32F427 | DJI A 开发板适配 |
| `DmMc02H7` | STM32H723 | 达妙 MC02 H7 开发板适配 |

## 应该放这里

- 板级端口配置：UART、CAN、SPI、I2C、PWM、GPIO。
- 板载设备适配：IMU、蜂鸣器、按键、SD 卡、裁判串口等。
- 板级 IMU 安装差异：传感器安装矩阵、原始数据旋转、温控 PWM、DMA/中断收数。
- 板级启动代码：只有独立板级实验入口才放 `BoardMain.c`、`BoardFreertos.c`。
- 和某块板子强绑定的端口文件，例如 `BspBoardPorts.h`、`BspImuPwmCfg.h`。

## 不应该放这里

- 某台车的 PID、电机 ID、输入映射：放 `Robotconfig/`。
- 可直接打开编译的完整 Keil 工程：放 `projects/`。
- 跨板复用的控制逻辑和协议驱动：放 `shared/`。
- 跨板复用的外设封装：放 `shared/hal/`，这里的 `bsp/` 只放具体板子的端口和配置。

如果一个文件只因为“换板子”才需要改，它通常属于这里；如果只是换车型参数，不应该动这里。

## IMU 和零偏校准边界

各板的 `InsTask.c` 负责把原始 IMU 数据转成板载坐标系，并接入对应的温控、DMA、SPI/I2C 读取方式。陀螺仪零偏采样流程放在 `shared/application/services/calibration/GyroZeroCali.h`，避免三块板各写一套状态机。

板级代码只保留这些差异：

- 原始 gyro/accel 怎么旋转到板载坐标系。
- 零偏 offset 怎么写回当前运行变量。
- 校准结果怎么保存到 Flash。
- 正常上电微调是否允许开始，例如遥控器未连接或处于安全档。

不要把某台车的 PID、电机 ID 或遥控通道写进板级 INS 文件里。
