# 算法接入协议

这份文档定第一版算法串口协议：算法可以给云台目标角，也可以给底盘移动命令。压缩的是包头，不删控制信息。

已接代码：

- 新协议解析：`shared/application/comm/vision/VisionLink.c`
- 外部运动意图缓存：`shared/application/robot/ExternalMotionIntent.c`
- 云台命令入口：`shared/application/gimbal/GimbalControlTask.c`
- 移动命令执行：`shared/application/chassis/ChassisBehaviour.c`
- USB 虚拟串口入口：`projects/*/USB_DEVICE/App/usbd_cdc_if.c`

代码边界：

- `VisionLink.c` 只认识串口包格式，负责解析 `AIM_CMD` 和 `MOVE_CMD`。
- `MOVE_CMD` 会先转换成 `external_motion_intent_t`，再写入 `ExternalMotionIntent.c`。
- 底盘只读取 `external_motion_intent_t`，不直接依赖 `AlgorithmMoveCmd` 这类串口包结构。
- 没有算法链路的目标不需要各自写空实现；统一由 `external_motion_intent` 在无有效命令时返回 false。

## 1. 基本约定

- 小端序。
- `float` 为 32 位 IEEE754。
- 角度单位 rad，角速度 rad/s，角加速度 rad/s^2。
- 平移速度 m/s，平移加速度 m/s^2。
- CRC16 复用工程里的 `append_CRC16_check_sum()` / `verify_CRC16_check_sum()`，初值 `0xffff`，低字节在前。
- 算法只发目标，不直接发电机电流，也不直接发四个轮子的目标转速。

## 2. 紧凑帧格式

外层帧只保留 2 字节包头、1 字节命令号和 CRC16。没有版本号、序号、长度字段；每个命令都是固定长度。

```text
sof0 sof1 cmd payload crc16
```

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| `sof0` | 1 | 固定 `0xA5` |
| `sof1` | 1 | 固定 `0x5A` |
| `cmd` | 1 | 命令号 |
| `payload` | 固定 | 按命令号决定长度 |
| `crc16` | 2 | 从 `sof0` 算到 payload 末尾 |

命令号：

| cmd | 名称 | payload 长度 | 总帧长 | 方向 |
| ---: | --- | ---: | ---: | --- |
| `0x01` | `AIM_CMD` | 28 | 33 | 算法 -> 板端 |
| `0x02` | `MOVE_CMD` | 36 | 41 | 算法 -> 板端 |

解析策略：

- 按 `0xA5 0x5A` 找包头。
- 第 3 字节决定固定帧长。
- CRC 不过就滑动 1 字节继续找包头。
- 旧 `SP` 自瞄包仍然兼容，方便先联调云台。

## 3. AIM_CMD：云台命令

速度、加速度保留。现在云台控制代码主要使用 `yaw_rad / pitch_rad`，但协议里保留速度和加速度，后续可以做前馈或预测。

```c
typedef struct __attribute__((packed)) AlgorithmAimCmd
{
    uint8_t mode;
    uint8_t flags;
    uint16_t timeout_ms;
    float yaw_rad;
    float yaw_vel_radps;
    float yaw_acc_radps2;
    float pitch_rad;
    float pitch_vel_radps;
    float pitch_acc_radps2;
} AlgorithmAimCmd;
```

字段：

| 字段 | 说明 |
| --- | --- |
| `mode` | 0 关闭自瞄；1 控制云台；2 控制云台并请求开火 |
| `flags bit0` | yaw 有效 |
| `flags bit1` | pitch 有效 |
| `flags bit2` | 目标角为绝对角 |
| `timeout_ms` | 建议 100；0 表示板端默认 |
| `yaw_rad` | yaw 目标角 |
| `yaw_vel_radps` | yaw 目标角速度 |
| `yaw_acc_radps2` | yaw 目标角加速度 |
| `pitch_rad` | pitch 目标角 |
| `pitch_vel_radps` | pitch 目标角速度 |
| `pitch_acc_radps2` | pitch 目标角加速度 |

当前代码接入情况：

- 新 `AIM_CMD` 会被转换成旧 `VisionToGimbal`，所以云台现有逻辑能直接吃到 `yaw/pitch/yaw_vel/yaw_acc/pitch_vel/pitch_acc`。
- `GimbalControlTask.c` 目前只把 `yaw_rad` 和 `pitch_rad` 用到目标角里，速度和加速度先保留在包里。
- 旧代码里 pitch 有取负逻辑，实车第一次联调用小角度确认方向。

## 4. MOVE_CMD：底盘移动命令

移动命令先由 `VisionLink.c` 解析，再写入 `ExternalMotionIntent.c`。`ChassisBehaviour.c` 只读取转换后的外部运动意图。它只覆盖底盘目标速度，不绕过后面的运动学、功率限制和电机 PID。

参考其他队伍的做法，这里不让算法直接给四个轮子的目标值。算法只给 `vx/vy/wz`，板端按云台、底盘或场地坐标做转换，再交给原来的底盘控制链路。

```c
typedef struct __attribute__((packed)) AlgorithmMoveCmd
{
    uint8_t mode;
    uint8_t frame;
    uint16_t flags;
    uint16_t timeout_ms;
    uint16_t reserved;
    float vx_mps;
    float vy_mps;
    float wz_radps;
    float yaw_offset_rad;
    float ax_mps2;
    float ay_mps2;
    float wz_acc_radps2;
} AlgorithmMoveCmd;
```

字段：

| 字段 | 说明 |
| --- | --- |
| `mode` | 0 不接管；1 跟随云台；2 不跟随 yaw，直接用 `wz`；3 小陀螺平移；4 停车 |
| `frame` | 0 云台坐标系；1 底盘坐标系；2 场地坐标系 |
| `flags bit0` | `vx_mps / vy_mps` 有效 |
| `flags bit1` | `wz_radps` 有效 |
| `flags bit2` | `yaw_offset_rad` 有效 |
| `flags bit3` | 加速度字段有效 |
| `timeout_ms` | 建议 200；0 表示板端默认 200 |
| `vx_mps` | 前进为正 |
| `vy_mps` | 左移为正 |
| `wz_radps` | 逆时针为正 |
| `yaw_offset_rad` | 跟随云台时的底盘相对云台目标偏角 |
| `ax_mps2` | 前后加速度，用来限制 `vx_mps` 的变化速度 |
| `ay_mps2` | 左右加速度，用来限制 `vy_mps` 的变化速度 |
| `wz_acc_radps2` | 自转角加速度，用来限制 `wz_radps` 的变化速度 |

板端映射：

| `mode` | 当前接入方式 |
| ---: | --- |
| 0 | 清空外部运动接管，不使用算法移动 |
| 1 | 切到 `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW`，覆盖 `vx/vy/yaw_offset` |
| 2 | 切到 `CHASSIS_VECTOR_NO_FOLLOW_YAW`，覆盖 `vx/vy/wz` |
| 3 | 切到小陀螺行为，覆盖 `vx/vy/wz`；如果 `wz` 无效，用板端小陀螺默认速度 |
| 4 | 切到停车，速度清零 |

内部接口：

```c
typedef struct
{
    uint8_t mode;
    uint8_t frame;
    uint16_t flags;
    uint16_t timeout_ms;
    fp32 vx_mps;
    fp32 vy_mps;
    fp32 wz_radps;
    fp32 yaw_offset_rad;
    fp32 ax_mps2;
    fp32 ay_mps2;
    fp32 wz_acc_radps2;
} external_motion_intent_t;
```

`AlgorithmMoveCmd` 是串口协议结构，只在解析层使用；底盘层只接触 `external_motion_intent_t`。后面如果 UART、AUX 或别的链路也要给底盘移动命令，直接写入同一个外部运动意图接口即可。

坐标系处理：

- `frame=0`：算法以云台朝向为前方，适合自瞄边打边移动。
- `frame=1`：算法以车体正前为前方，适合直接控制底盘。
- `frame=2`：算法以 IMU 算出来的场地 yaw 为参考，板端会先转到底盘坐标。
- `mode=1` 跟随云台时，`vx/vy` 最终仍会走底盘原来的跟随云台转换和 yaw 跟随 PID。
- `mode=2/3` 时，`vx/vy/wz` 会直接变成底盘速度目标，再走轮速分解、功率限制和电机 PID。

小陀螺接入：

- `mode=3` 就是小陀螺移动。
- `flags bit0=1` 时，`vx_mps / vy_mps` 有效，板端会保留平移。
- `flags bit1=1` 时，`wz_radps` 有效，算法指定自转速度。
- `flags bit1=0` 时，板端使用当前车型配置里的小陀螺默认自转速度。
- `flags bit3=1` 时，`ax_mps2 / ay_mps2 / wz_acc_radps2` 会限制速度变化，避免小陀螺突然加速。
- 推荐第一版先用云台坐标系：不带加速度限制用 `mode=3, frame=0, flags=0x0003`；带加速度限制用 `mode=3, frame=0, flags=0x000B`。

非全向车说明：

- 麦轮、X-drive 这类全向底盘可以直接响应 `vx/vy/wz`。
- 轮腿平衡车、差速车、舵轮未转到位的底盘这类非全向移动平台，不能像麦轮一样直接侧移。
- 非小陀螺模式下，`vy_mps` 这类横向平移只能通过“车体转向 + 前进/后退”慢慢拼出来，所以横移响应会明显慢。
- 小陀螺模式下车体持续旋转，算法给的平移方向更容易被分解到前后运动里，横向机动会比非小陀螺模式自然一些。
- 如果目标车不是全向底盘，算法端不要假设 `vy_mps` 能瞬时执行；建议降低横移期望，并提高 `timeout_ms` 和轨迹预测容错。

建议算法先用：

- 普通移动：`mode=1, frame=0, flags=0x000F`。
- 小陀螺移动：`mode=3, frame=0, flags=0x000B`。

## 5. 串口链路

现在最方便的是 USB 虚拟串口。物理 UART 后续可以把 AUX UART 收到的字节喂给同一个解析器。

推荐参数：

| 项目 | 建议 |
| --- | --- |
| 物理 UART | 3.3V TTL，TX/RX 交叉，GND 共地 |
| 波特率 | 921600；低速测试可用 115200 |
| 格式 | 8N1，也就是 8 数据位、无校验、1 停止位 |
| 算法频率 | 50-200Hz |

## 6. 安全规则

- 遥控安全档最高优先级，安全档下忽略算法命令。
- 遥控离线、IMU 离线、云台电机离线、底盘电机离线时，算法不能接管。
- 移动命令超时后不再覆盖底盘输入。
- `timeout_ms=0` 时，板端按 200ms 默认超时处理。
- 拒绝 NaN、Inf、过大角度、过大速度这类异常输入。
- `vx/vy/wz` 仍要走板端限幅、功率限制和电机 PID。
- 算法只能请求开火，不能绕过发射安全判断。

第一版限幅建议：

| 项目 | 建议值 |
| --- | --- |
| yaw 单包变化 | 不超过 0.35rad |
| pitch 单包变化 | 不超过 0.20rad |
| 平移速度 | 不超过板端最大速度的 60% |
| 自转速度 | 不超过板端小陀螺速度的 60% |

## 7. 旧自瞄包兼容

旧包仍然能用：

```c
typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2]; // 'S','P'
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal;
```

旧包没有命令号，不能扩展移动控制。后续建议算法统一发新帧。
