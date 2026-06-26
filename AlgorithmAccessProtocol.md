# 算法接入协议

本文按当前代码写，主要看 `shared/application/comm/vision/VisionLink.c`。


- `LS`：云台自瞄和板端瞄准状态。
- `LC`：底盘移动目标和移动反馈。

## 1. 基本约定

- 小端序。
- `float` 为 32 位 IEEE754。
- 角度单位 rad，角速度 rad/s，角加速度 rad/s^2。
- 平移速度 m/s，平移加速度 m/s^2。
- CRC16 初值 `0xffff`，反射多项式 `0x8408`，不取反；覆盖除最后 2 字节外的全部内容。
- 末尾两个 CRC 字节是低字节在前、高字节在后。
- 算法发目标量，不直接发电机电流，也不直接发四个轮子的目标转速。
- 算法只关心板端实际用于控制的反馈，不关心底层来自 IMU、编码器还是融合结果。

### CRC16 函数

对面发包时，先把末尾 2 个字节留出来，按 `len - 2` 算 CRC，再把低字节、高字节依次写到末尾。收包时也按前 `len - 2` 个字节重算，再和末尾 2 个字节比较。

```c
#include <stdint.h>

static uint16_t ArbatosCrc16Calc(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xffffu;

    while (len-- != 0u) {
        crc ^= (uint16_t)(*buf++);

        for (uint8_t i = 0u; i < 8u; i++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

static void ArbatosCrc16Append(uint8_t *buf, uint32_t len)
{
    uint16_t crc;

    if (buf == 0 || len < 2u) {
        return;
    }

    crc = ArbatosCrc16Calc(buf, len - 2u);
    buf[len - 2u] = (uint8_t)(crc & 0xffu);
    buf[len - 1u] = (uint8_t)((crc >> 8) & 0xffu);
}

static uint8_t ArbatosCrc16Verify(const uint8_t *buf, uint32_t len)
{
    uint16_t crc;

    if (buf == 0 || len < 2u) {
        return 0u;
    }

    crc = ArbatosCrc16Calc(buf, len - 2u);
    return (uint8_t)((buf[len - 2u] == (uint8_t)(crc & 0xffu)) &&
                     (buf[len - 1u] == (uint8_t)((crc >> 8) & 0xffu)));
}
```

Python 端可以直接这么写：

```python
def arbatos_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def arbatos_crc16_append(frame_without_crc: bytes) -> bytes:
    crc = arbatos_crc16(frame_without_crc)
    return frame_without_crc + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def arbatos_crc16_verify(frame: bytes) -> bool:
    if len(frame) < 2:
        return False
    crc = arbatos_crc16(frame[:-2])
    return frame[-2] == (crc & 0xFF) and frame[-1] == ((crc >> 8) & 0xFF)
```

对表样例：`arbatos_crc16(b"123456789") == 0x6f91`，末尾应写成 `0x91 0x6f`。

## 2. 包总览

| 包 | 方向 | 包头 | payload | 总长 |
| --- | --- | --- | ---: | ---: |
| `GimbalToVision` | 板端 -> 算法 | `'L' 'S'` | 39 | 43 |
| `VisionToGimbal` | 算法 -> 板端 | `'L' 'S'` | 25 | 29 |
| `VisionToChassis` | 算法 -> 板端 | `'L' 'C'` | 18 | 22 |
| `ChassisToVision` | 板端 -> 算法 | `'L' 'C'` | 14 | 18 |

板端接收算法数据时：

- `LS` 按 29 字节 `VisionToGimbal` 解析。
- `LC` 按 22 字节 `VisionToChassis` 解析。

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

`q[4]` 只作为板端姿态参考保留。算法如果要算目标角，优先按 `yaw/pitch` 这组控制反馈对齐，不要把 `q[4]` 固定理解成云台姿态。具体这块控制板固定在底盘、大 yaw、云台还是轮腿本体上，看对应目标的 `Robotconfig/<TARGET>/MountLayout.md`；统一坐标口径见 `manual/coordinate-frames.md`。

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

总长：22 字节。

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

方向约定：

| 量 | 正方向 |
| --- | --- |
| `vx / ax` | 沿当前 `frame` 的前方；`vx > 0` 表示向前，`vx < 0` 表示向后 |
| `vy / ay` | 沿当前 `frame` 的左方；`vy > 0` 表示向左，`vy < 0` 表示向右 |
| `wz / wz_acc` | 从上往下看，逆时针为正，顺时针为负 |
| `yaw_offset` | 角度正方向和 `wz` 一致；它是跟随云台模式下的目标相对角，不是直接旋转速度 |

`mode`：

| 值 | 含义 |
| ---: | --- |
| 0 | 不接管移动 |
| 1 | 跟随云台 yaw，使用 `vx/vy/yaw_offset` |
| 2 | 不跟随 yaw，直接使用 `vx/vy/wz` |
| 3 | 小陀螺移动，使用 `vx/vy/wz` |
| 4 | 停车，速度清零 |

`frame`：

这里的“前方”就是当前坐标系的 `+x` / `vx > 0` 方向。不同 `frame` 下，“前方”绑定的对象不同：

| 值 | 坐标系 | `vx > 0` 的前方 | 说明 |
| ---: | --- | --- | --- |
| 0 | 云台坐标系 | 云台 yaw 在水平面上的朝向 | 只看 yaw，不看 pitch；可以理解成云台在地面上指向哪里，`vx > 0` 就往哪里走，`vy > 0` 往这个方向的左边走 |
| 1 | 底盘坐标系 | 车体正前方 | 固定在底盘车体上，不跟云台转；车头前方是 `+x`，车体左侧是 `+y` |
| 2 | 场地坐标系 | 板端姿态估计里的场地前方 | 板端会按自己的姿态估计转到底盘坐标；只有算法和板端约好 yaw 零点/场地方向时才建议用 |

当前代码里 `frame=2` 用底盘 `ChassisYaw` 做转换，`ChassisYaw` 来自 `INS yaw - 云台/底盘相对 yaw`。没有磁力计或外部定位时，INS yaw 的零点来自本次上电或融合重置时的姿态，可以近似理解成：如果上电时车体正前对准场地 `+x`，那么后续无论云台 yaw、底盘 yaw 怎么转，场地 `+x` 都保持这个初始方向；板端会把这条固定方向转成当前底盘坐标下的 `vx/vy`。它不是每一刻的当前车头方向，也不自动知道真实赛场坐标；yaw 漂移或重新初始化会改变这个基准。不同车型的控制板固定位置会影响这句话的实际含义，接算法前先看 `Robotconfig/<TARGET>/MountLayout.md`。

如果算法只是想“按相机/云台看见的方向移动”，优先用 `frame=0`。如果算法想直接按车体前后左右给速度，用 `frame=1`。

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
