# RTC、保护逻辑及 HERO-C 对照

## RTC 实板结果

- PCF8563 已通过显式接口设为北京时间 `2026-09-06 15:00:02`，清除低电压标志并启动计数。日期检查包含闰年和每月天数；停止计数或低电压时拒绝向日志提供时间。
- 约40秒后正常读到15:00:41；软件复位后继续走时，新建 `20260906_150044_HERO-M_0013.bin`。
- 用户按提示断开24V和板上USB再重接后，读取到15:02:14，RTC错误0；启动文件为 `20260906_150134_HERO-M_0014.bin`。本轮没有再次校时，说明此次断电保时正常；不等同于长期电池寿命验证。
- 已关闭日志和加热。Flash校准序号2正常加载，重新刷程序前后的两个校准记录逐字节一致。
- 准备固件449532字节读回完全一致，SHA-256 `1ab405abf50fd33dcba3fdf55c66ca3723e885b1740bcc9e849ddcaca1098a20`。正式HERO-M构建532788字节通过；板上仍为静态准备固件。
- 证据：`local/cache/zephyr-preflight-hardware/20260906/rtc/`。以后显式校时可运行 `PreflightProbe.py rtc`，必须使用该固件对应的ELF；普通开机只读取，不改写RTC。

## 保护逻辑检查

`RunSafetyChecks.py` 用本机MSVC编译执行已有生产代码回归，12组全部通过：ManualInputSnapshot、MotorHealth、GimbalFeedbackPolicy、LowCmd、RobotLifecycle、ControlMgr、FaultMgr、ChassisSnapshotPolicy、ShootFaultPolicy、ControlActuatorPolicy、CanTxCompletionPolicy、MotorInstPermit。

覆盖遥控源超时与切换、反馈年龄和接收计数、控制权撤销、旧CAN命令过期、持续禁止输出、故障恢复、执行器写入许可、射击故障禁止等。没有在本轮驱动电机，也没有实测CAN断线后的电机物理停止时间。

IMU失效的现有策略并非所有轴一律停机：云台对超过10ms的数据判为过期，切换时先发布零命令；可靠的yaw编码器可参与降级，无法替代的pitch轴阻断。恢复需要IMU连续稳定200ms且处于安全档。HERO-C和HERO-M的允许编码器降级掩码均为0x01，本轮保留既有行为。

证据：`local/cache/zephyr-preflight-hardware/20260906/safety/results.json`及各组编译/执行日志。最初ControlMgr的MSVC强制包含路径不匹配，改用绝对路径后通过；未因此修改保护实现。

## 保留 HERO-C 基准及 IMU 差异

- 两目标的 `ConfigHardware.inc`、`ConfigTuning.inc`、`ConfigInput.inc` 逐字节相同；电机ID、调参、编码器零位均未修改。内置pitch标定表只存在目标名称注释差异。
- C板BMI088使用SPI1、PA4/PB0片选；M板使用SPI2、PC0/PC3片选，M板原理图和实板读取已核对。两者加热引脚、周期及供电不同，不能直接照搬占空比。
- 旧C/M板代码都写了 `[rawY, -rawX, rawZ]` 变换，目前Zephyr沿用此矩阵。这不能证明两个实物板的参考边和车上安装方向一致；后续按用户动作进行P/Y/R方向实测。
- HERO-M现有任务表比HERO-C少启动服务、主动校准、主机链路、ELRS、状态灯、舵机等任务；这是既有功能差异，本轮未修改。不能把硬件和调参文件相同理解为两目标功能完全一致。
