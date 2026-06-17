# 算法接入协议

本文按当前代码写，主要看 `shared/application/comm/vision/VisionLink.c`。

现在所有算法串口包都用 2 字节包头。`LS` 是 2 字节，`0xA5 0x5A 0x02` 是 3 字节，所以底盘移动不再用 `0xA5 0x5A + cmd`，改成带类型的 2 字节包头：

- `LS`：云台自瞄和板端瞄准状态。
- `LC`：底盘移动目标和移动反馈。

## 1. 基本约定

- 小端序。
- `float` 为 32 位 IEEE754。
- 角度单位 rad，角速度 rad/s，角加速度 rad/s^2。
- 平移速度 m/s，平移加速度 m/s^2。
- CRC16 使用工程里的 `append_CRC16_check_sum()` / `verify_CRC16_check_sum()`。
- 算法发目标量，不直接发电机电流，也不直接发四个轮子的目标转速。
- 算法只关心板端实际用于控制的反馈，不关心底层来自 IMU、编码器还是融合结果。

## 2. 包总览

| 包 | 方向 | 包头 | payload | 总长 |
| --- | --- | --- | ---: | ---: |
| `GimbalToVision` | 板端 -> 算法 | `'L' 'S'` | 39 | 43 |
| `VisionToGimbal` | 算法 -> 板端 | `'L' 'S'` | 25 | 29 |
| `VisionToChassis` | 算法 -> 板端 | `'L' 'C'` | 16 | 20 |
| `ChassisToVision` | 板端 -> 算法 | `'L' 'C'` | 14 | 18 |

板端接收算法数据时：

- `LS` 按 29 字节 `VisionToGimbal` 解析。
- `LC` 按 20 字节 `VisionToChassis` 解析。

板端发送给算法时：

- `LS` 是 43 字节 `GimbalToVision`。
- `LC` 是 18 字节 `ChassisToVision`。

## 3. 板端瞄准状态：GimbalToVision

板端周期发送瞄准相关反馈、姿态参考和射速信息。

```c
typedef struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2];     // 'L','S'
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision;
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `head` | 固定 `'L' 'S'` |
| `mode` | 0 空闲，1 自瞄，2 小符，3 大符；当前发送侧保持 0 |
| `q[4]` | 板端姿态参考，顺序 `w, x, y, z`；算法不要假设它一定是云台 IMU |
| `yaw` | 板端实际用于瞄准控制的 yaw 反馈 |
| `yaw_vel` | 板端实际用于瞄准控制的 yaw 角速度反馈 |
| `pitch` | 板端实际用于瞄准控制的 pitch 反馈 |
| `pitch_vel` | 板端实际用于瞄准控制的 pitch 角速度反馈 |
| `bullet_speed` | 当前射速估计 |
| `bullet_count` | 17mm 剩余允许发弹量 |
| `crc16` | 整包 CRC16 |

代码里 `yaw/pitch/yaw_vel/pitch_vel` 优先来自 `GimbalState`，也就是云台控制任务实际使用的反馈。具体是 IMU、编码器还是融合值，由各车型底层决定；比如英雄可以用云台 IMU，哨兵如果 IMU 不在云台上，就应该由底层给出编码器或融合后的云台反馈。

`q[4]` 只作为板端姿态参考保留。算法如果要算目标角，优先按 `yaw/pitch` 这组控制反馈对齐，不要把 `q[4]` 固定理解成云台姿态。

`HostLinkTask` 每 2ms 调一次 `VisionLinkPollTx()`。底盘状态有效后，每 5 次里有 1 次会发 `ChassisToVision`，其余时间发 `GimbalToVision`。USB 忙时跳过当次发送。

## 4. 云台自瞄：VisionToGimbal

算法给云台发目标角。速度和加速度字段保留给前馈或预测使用。

```c
typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2];   // 'L','S'
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

字段说明：

| 字段 | 说明 |
| --- | --- |
| `head` | 固定 `'L' 'S'` |
| `mode` | 0 关闭自瞄，1 控制云台，2 控制云台并请求开火 |
| `yaw` | yaw 目标角 |
| `yaw_vel` | yaw 目标角速度 |
| `yaw_acc` | yaw 目标角加速度 |
| `pitch` | pitch 目标角 |
| `pitch_vel` | pitch 目标角速度 |
| `pitch_acc` | pitch 目标角加速度 |
| `crc16` | 整包 CRC16 |

当前云台控制主要使用 `yaw` 和 `pitch`。算法可以继续填速度、加速度，板端后续要加前馈时不用再改包。

## 5. 底盘移动：VisionToChassis

算法发 `VisionToChassis` 控制底盘移动。板端解析后会转换成 `ExternalMotionIntent`，再由底盘控制代码执行。

```c
typedef struct __attribute__((packed)) VisionToChassis
{
    uint8_t head[2];     // 'L','C'
    uint8_t mode;
    uint8_t frame;
    uint8_t flags;
    uint8_t timeout_10ms;
    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
    int16_t yaw_offset_mrad;
    int16_t ax_cmps2;
    int16_t ay_cmps2;
    int16_t wz_acc_mradps2;
    uint16_t crc16;
} VisionToChassis;
```

总长：20 字节。

缩放关系：

| 字段 | 实际值 |
| --- | --- |
| `timeout_10ms` | `0` 表示板端默认 200ms；非 0 表示 `raw * 10ms` |
| `vx_cmps` | `vx_mps = raw * 0.01` |
| `vy_cmps` | `vy_mps = raw * 0.01` |
| `wz_mradps` | `wz_radps = raw * 0.001` |
| `yaw_offset_mrad` | `yaw_offset_rad = raw * 0.001` |
| `ax_cmps2` | `ax_mps2 = raw * 0.01` |
| `ay_cmps2` | `ay_mps2 = raw * 0.01` |
| `wz_acc_mradps2` | `wz_acc_radps2 = raw * 0.001` |

`mode`：

| 值 | 含义 |
| ---: | --- |
| 0 | 不接管移动 |
| 1 | 跟随云台 yaw，使用 `vx/vy/yaw_offset` |
| 2 | 不跟随 yaw，直接使用 `vx/vy/wz` |
| 3 | 小陀螺移动，使用 `vx/vy/wz` |
| 4 | 停车，速度清零 |

`frame`：

| 值 | 坐标系 |
| ---: | --- |
| 0 | 云台坐标系，算法以云台朝向为前方 |
| 1 | 底盘坐标系，算法以车体正前为前方 |
| 2 | 场地坐标系，板端按自己的姿态估计转换到底盘坐标 |

`flags`：

| bit | 含义 |
| ---: | --- |
| 0 | `vx/vy` 有效 |
| 1 | `wz` 有效 |
| 2 | `yaw_offset` 有效 |
| 3 | 加速度字段有效 |

小陀螺接入：

- `mode=3` 表示小陀螺移动。
- `flags bit1=1` 时使用算法给的 `wz`。
- `flags bit1=0` 时使用板端配置里的默认小陀螺速度。
- 推荐起步用 `mode=3, frame=0, flags=0x0B`。

非全向底盘注意：

- 麦轮、X-drive 这类全向底盘可以直接响应 `vx/vy/wz`。
- 轮腿平衡车、差速车、舵轮没转到位的底盘，不能瞬时侧移。
- 非小陀螺模式下，横向平移一般只能由车体旋转和前进后退拼出来，所以 `vy` 响应会慢。

## 6. 底盘移动反馈：ChassisToVision

板端把当前底盘速度和目标速度发给算法。

```c
typedef struct __attribute__((packed)) ChassisToVision
{
    uint8_t head[2];     // 'L','C'
    uint8_t valid;
    uint8_t mode;
    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
    int16_t vx_set_cmps;
    int16_t vy_set_cmps;
    int16_t wz_set_mradps;
    uint16_t crc16;
} ChassisToVision;
```

总长：18 字节。

字段说明：

| 字段 | 说明 |
| --- | --- |
| `valid` | 1 表示底盘状态有效 |
| `mode` | 当前底盘模式 |
| `vx_cmps / vy_cmps / wz_mradps` | 底盘反馈速度 |
| `vx_set_cmps / vy_set_cmps / wz_set_mradps` | 底盘目标速度 |

缩放和 `VisionToChassis` 一样：`cmps * 0.01` 转 m/s，`mradps * 0.001` 转 rad/s。

## 7. 串口建议

| 项目 | 建议 |
| --- | --- |
| 物理 UART | 3.3V TTL，TX/RX 交叉，GND 共地 |
| 波特率 | 921600；低速测试可用 115200 |
| 格式 | 8N1 |
| 算法发送频率 | 自瞄 50-200Hz，移动 20-100Hz |

当前代码走 USB 虚拟串口入口。后面物理 UART 接入时，把收到的字节交给同一个解析函数即可。

## 8. 安全规则

- 遥控安全档优先级最高。
- 遥控离线、云台控制反馈无效、底盘电机离线时，算法不能接管。
- 移动命令超时后不再覆盖底盘输入。
- `timeout_10ms=0` 时，板端按默认 200ms 处理。
- 拒绝 NaN、Inf 和明显过大的速度、角度输入。
- `vx/vy/wz` 仍然要走板端限幅、功率限制和电机 PID。
- 算法只能请求开火，不能绕过发射安全判断。
