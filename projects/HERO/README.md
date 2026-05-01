# HERO（RoboMaster 2026 Hero）固件工程

本仓库为 **RoboMaster 英雄机器人** STM32 主控固件工程，基于 **STM32F407IGH6 (STM32F4)** + **FreeRTOS**，包含底盘/云台/射击/裁判系统/姿态解算/在线检测/调试观测等模块。

## 目录


- [工程结构](#工程结构)
- [任务与数据流](#任务与数据流)
- [外设与通信分配](#外设与通信分配)
- [AUX 口调参 + TF/SD 遥测日志](#aux-口调参--tfsd-遥测日志)
- [USB CDC 视觉链路](#usb-cdc-视觉链路)
- [功率限制（底盘限流）](#功率限制底盘限流)
- [本地日志和脚本](#本地日志和脚本)



## 工程结构

- `Core/Inc`、`Core/Src`：CubeMX 生成的 HAL 初始化与中断入口
- `USB_DEVICE/`：CubeMX 生成的 USB CDC 代码
- `Drivers/`、`Middlewares/`：HAL / CMSIS / FreeRTOS / USB 组件
- `MDK-ARM/`：Keil 工程
- `target/HERO/User/application/`：HERO 目标配置和健康监测，包含 `config.c`、`config.h`、`detect_task.c`
- `boards/DJI_C_F407/app/INS_task.c`：C 板 IMU 任务
- `boards/DJI_C_F407/bsp/`：C 板 BSP 适配
- `shared/application/`：共用任务和控制逻辑，包含 `host_link_task.c`、`chassis_control_task.c`、`gimbal_control_task.c`、`shoot.c`、`sdlog_task.c` 等

---

## 任务与数据流

典型数据链路（简化）：

```
USART3(DBUS) -> manual_input -> behaviour -> chassis_control_task / gimbal_control_task
INS(BMI088/IST) -> INS_task -> 角度/角速度/四元数
CAN RX -> CAN_receive -> 电机反馈(measure)

chassis_control_task/gimbal_control_task -> actuator_cmd -> can_command_tx_task -> CAN1(0x200/0x1FF)

USART6(裁判) -> referee_rx_task -> referee.c -> 功率/热量等

USB CDC -> host_link_task(视觉链路收发)
AUX 口（当前 HERO 为 USART1） -> host_link_task(调参/遥测/图传遥控) + elrs_task.c(ELRS/CRSF 接收)
```

---

## 外设与通信分配

### 串口（USART）

- `USART3`（`PC10 TX` / `PC11 RX`）：遥控 SBUS/DBUS（`100000`，Even parity）
- `USART6`（`PG14 TX` / `PG9 RX`）：裁判系统（`115200 8N1`）
- `USART1`（`PA9 TX` / `PB7 RX`）：AUX 口（可换的辅助串口），根据波特率选择：调参/遥测和图传遥控 `921600 8N1`；ELRS/CRSF 接收 `420000 8N1`

> AUX 口实时遥测（JustFloat，一种浮点数连续输出格式）使用 DMA TX（`HAL_UART_Transmit_DMA`），带宽有限需要统一发送周期；TF/SD 会一直记录更详细的运行日志（sdlog）。

### CAN

- **CAN1（指令发送）**：由 `can_command_tx_task` 统一发送
  - 0x200 → 0x201(M1), 0x202(M2), 0x203(Pitch), 0x204(Trigger)
  - 0x1FF → 0x205(M4), 0x206(Yaw), 0x207(M3), 0x208(预留)
- **CAN2**：摩擦轮电机总线

### USB CDC（视觉）

USB 以 CDC ACM（虚拟串口）方式枚举。PC 端需打开串口设备才能收发数据；波特率等参数对 USB 传输本身无影响。

---

## AUX 口调参 + TF/SD 遥测日志

实现位置：`shared/application/host_link_task.c`

### 1) 调参（AUX 口命令）

- 命令格式：`<id>:<value>\r\n`
- `id` 对应 `target/HERO/User/application/config.c` 中每行末尾的 `[ID]` 注释
- 只修改 RAM 中的 `g_config`；**重启会恢复默认值**

示例（具体 ID 以 `config.c` 为准）：

- 开关 AUX 口实时遥测：`241:1`（开）/ `241:0`（关）
- AUX 口遥测周期(ms)：`242:0`（0=auto，额外50%回退，适合无线）
- AUX 口模式判定：波特率为 `420000` 时进入 ELRS/CRSF 接收（不发送遥测）；否则为调参/遥测。
- TF/SD 遥测日志：默认一直记录（只要 TF 挂载且 `sdlog_task` 正常运行），不需要开关/配置。

### 2) AUX 口实时遥测（JustFloat：N*fp32 + INF 尾）

- 帧格式：`N * float32` + `float32(+Inf)` 尾
- 字节序：STM32 小端（IEEE754 `float32`）
- INF 尾（小端字节序）：`00 00 80 7F`
- 这里说的 AUX 口是可换的辅助串口；当前 HERO 板接在 USART1。代码里结构体、枚举和值表都使用 `aux_*`。

#### 默认“全量精简”遥测

在 `target/HERO/User/application/config.c`：

- `aux_telem.channel_num = 0`

此时发送 **内置默认列表**（当前 `N=219` 通道；`channel_map[]` 会被忽略），统一周期由 `aux_telem.period_ms` 控制（0=auto）。

默认列表在“原全量”的基础上精简了 `*_MODE/*_OFFLINE/*_ERR` 等通道，并用 2 个打包通道替代（见下表）；默认列表内容见 `shared/application/host_link_task.c` 的 `aux_telem_default_list[]`。

**PACK_MODE（`AUX_TELEM_SIG_PACK_MODE`）**

十进制打包（以整数解释）：`shoot_mode*100000 + last_chassis_mode*10000 + chassis_mode*1000 + pitch_motor_mode*100 + yaw_motor_mode*10 + gimbal_behaviour`

| 位权 | 字段 | 枚举/取值 |
|---:|---|---|
| 1 | `gimbal_behaviour` | `0:GIMBAL_ZERO_FORCE` `1:GIMBAL_INIT` `2:GIMBAL_CALI` `3:GIMBAL_ANGLE` `4:GIMBAL_MOTIONLESS` |
| 10 | `yaw_motor_mode` | `0:GIMBAL_MOTOR_RAW` `1:GIMBAL_MOTOR_ENCONDE` |
| 100 | `pitch_motor_mode` | `0:GIMBAL_MOTOR_RAW` `1:GIMBAL_MOTOR_ENCONDE` |
| 1000 | `chassis_mode` | `0:CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW` `1:CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW` `2:CHASSIS_VECTOR_NO_FOLLOW_YAW` `3:CHASSIS_VECTOR_RAW` |
| 10000 | `last_chassis_mode` | 同 `chassis_mode` |
| 100000 | `shoot_mode` | `0:SHOOT_STOP` `1:SHOOT_READY_FRIC` `2:SHOOT_READY_BULLET` `3:SHOOT_READY` `4:SHOOT_BULLET` `5:SHOOT_CONTINUE_BULLET` `6:SHOOT_DONE` |

**PACK_OFFLINE（`AUX_TELEM_SIG_PACK_OFFLINE`）**

bitmask（以整数解释），bit=1 表示对应 TOE 离线/错误：

| bit | TOE | 说明 |
|---:|---|---|
| 0 | `DBUS_TOE` | 遥控/DBUS |
| 1 | `CHASSIS_MOTOR1_TOE` | 底盘电机1 |
| 2 | `CHASSIS_MOTOR2_TOE` | 底盘电机2 |
| 3 | `CHASSIS_MOTOR3_TOE` | 底盘电机3 |
| 4 | `CHASSIS_MOTOR4_TOE` | 底盘电机4 |
| 5 | `YAW_GIMBAL_MOTOR_TOE` | yaw电机 |
| 6 | `PITCH_GIMBAL_MOTOR_TOE` | pitch电机 |
| 7 | `TRIGGER_MOTOR_TOE` | 拨弹电机 |
| 8 | `REFEREE_TOE` | 裁判系统 |
| 9 | `RM_IMU_TOE` | RM IMU |
| 10 | `BOARD_GYRO_TOE` | 板载陀螺仪 |
| 11 | `BOARD_ACCEL_TOE` | 板载加速度计 |
| 12 | `BOARD_MAG_TOE` | 板载磁力计 |
| 13 | `OLED_TOE` | OLED |

若 AUX 口丢包/解析异常：优先增大周期（如 `242:40`）或减少通道数（降低单帧字节数）。

新增板载按键遥测：`AUX_TELEM_SIG_BOARD_KEY_DOWN`（1=按下）、`AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT`（去抖计数）。

#### 自定义遥测列表

把 `aux_telem.channel_map[]` 填成“遥测信号 ID 列表”，并设置 `aux_telem.channel_num = <列表长度>`。

- `channel_num=0` 表示使用默认列表（219 通道）
- `0` 是合法的遥测信号 ID（`AUX_TELEM_SIG_SYS_TICK_MS`），不再作为结束符

- 遥测信号枚举：`aux_telem_sig_e`（见 `target/HERO/User/application/config.h`）
- `channel_map` 使用的是 **遥测信号 ID**（不是调参用的 `[ID]`）

### 3) TF/SD 遥测日志（sdlog）

- 写入文件：`0:/sdlog_XXXX.bin`
- 文件序号：优先读取 `0:/sdlog_index.bin`（带 CRC）；若不存在/损坏则回退遍历 `0:/` 找最大号 +1
- 当前格式：v2（块写入；raw 或 LZ4 压缩，lossless；每块带 CRC32(raw) 校验；压缩不划算时会回退为 raw，仅多块头约 20B/块）
- PC 解压为 v1 记录流：在仓库根目录运行 `python tools/sdlog/sdlog_decompress.py sdlog_0000.bin`（输出 `sdlog_0000.bin.raw.bin`）
- Web 可视化：在仓库根目录运行 `python tools/sdlog/sdlog_viewer.py sdlog_0000.bin`（浏览器打开地址，可导出 CSV）
- 记录内容：IMU/PID/云台&底盘 loop/CAN/裁判/视觉等（tag 定义见 `shared/application/sdlog.h`）
- 注意：TF/SD 不会记录 AUX 口的 JustFloat 帧（避免重复/占用写入带宽）

---

## USB CDC 视觉链路

实现位置：`shared/application/host_link_task.c` + `USB_DEVICE/App/usbd_cdc_if.c`

### 板 → PC：`VisionTxFrame`（43B）

- 帧头：`'S','P'`
- 格式：`head[2] + mode(u8) + q[4] + yaw + yaw_vel + pitch + pitch_vel + bullet_speed + bullet_count(u16) + crc16(u16)`
- `sizeof(VisionTxFrame) == 43`

Python 解析示例（小端）：

- `VisionTxFrame = '<2sB9fHH'`

### PC → 板：`VisionToGimbal`（29B）

- 仅在 `len == sizeof(VisionToGimbal)` 时解析（必须一帧一写，不能粘包/拆包）
- `sizeof(VisionToGimbal) == 29`

Python 打包示例（小端）：

- `VisionToGimbal = '<2sB6fH'`

## 功率限制（底盘限流）

底盘功率限制实现：`shared/application/chassis_power_control.c`

- 有裁判系统（USART6 正常在线）时：读取裁判下发的 `chassis_power` / `chassis_power_buffer`，对底盘总电流进行缩放
- 无裁判系统（USART6 离线/未插）时：退化为 `no_judge_total_current_limit` 的固定限流，避免失控

相关参数：`target/HERO/User/application/config.c` 的 `g_config.power.*`

---

## 本地日志和脚本

仓库里现在没有 `analyzelog/` 目录。

- 日志解压脚本：在仓库根目录运行 `python tools/sdlog/sdlog_decompress.py sdlog_0000.bin`
- Web 查看脚本：在仓库根目录运行 `python tools/sdlog/sdlog_viewer.py sdlog_0000.bin`
- 本机输出和临时材料：`local/logs/`
