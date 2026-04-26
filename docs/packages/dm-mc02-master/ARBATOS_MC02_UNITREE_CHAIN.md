# ARBATOS 通过 MC02 接入宇树电机的控制链路建议

## 结论

如果 `MC02` 负责直接通过板载 `RS485` 驱动宇树 `GO-M8010-6`，那么在 ARBATOS 里，宇树电机应该被视为：

- 不是本地主控上的一个 `CAN 电机`
- 也不是 `motor_model_e` 里新增一种电机型号
- 而是一个由 `MC02` 托管的“外部智能关节执行器”

最合适的接入位置是：

`control_hub / subsystem task -> joint goal -> MC02 bridge -> MC02 RS485 -> Unitree motor`

而不是：

`joint_servo -> actuator_cmd -> can_tx_task`

## 现有 ARBATOS 控制链路

你现在仓库里的主链路大致是：

1. `control_hub`
   - 统一输入、仲裁、安全
   - 文件：`shared/User/application/control/control_hub.h`
2. 各子系统 task
   - `chassis_domain.c`
   - `gimbal_domain.c`
   - `shoot_domain.c`
   - `arm_domain.c`
3. 子系统产出 `joint_goal_table_t`
   - 文件：`shared/User/application/robot/joint_goal_table.h`
4. `joint_servo_submit(...)`
   - 汇总 domain goals
   - 文件：`shared/User/application/motor/actuator_cmd.c`
5. 本地执行器链路
   - `actuator_cmd` 共享电流
   - `can_tx_task` 组帧发 RM/DJI 电机

关键点在第 4 步之后。

现有链路在这里把高层目标压缩成了：

- `int16_t current`
- RM 风格 CAN 电流组播

这一步一旦做完，宇树电机需要的：

- `q`
- `dq`
- `tau`
- `kp`
- `kd`

就已经丢了。

所以宇树电机不能接在 `actuator_cmd/can_tx_task` 后面。

## MC02 方案里，应该把 MC02 放在链路哪里

### 正确位置

`MC02` 应该放在“关节目标层”和“总线/驱动层”之间：

1. ARBATOS 主控负责：
   - 行为决策
   - 控制仲裁
   - 关节目标生成
2. MC02 负责：
   - 接收来自 ARBATOS 的关节目标
   - 把目标转换成 Unitree RS485 协议
   - 下发到宇树电机
   - 回收状态并回传给 ARBATOS

所以 `MC02` 在系统里的角色应该是：

- `joint executor board`
- 或者说“关节执行器从控”

而不是“透明串口模块”

### 为什么不要让 ARBATOS 主控直接发宇树原始协议给 MC02

如果主控直接把宇树原始 17B 帧发给 MC02，问题是：

- ARBATOS 高层会被迫知道 Unitree 协议细节
- 高层会被迫知道转子侧/输出侧换算
- 高层会被协议耦合死
- 后面换电机或换从控板会很痛

更好的做法是：

- ARBATOS 发“抽象关节目标”
- MC02 负责转成宇树 RS485 帧

## 推荐的系统分层

### ARBATOS 主控侧

主控只保留抽象层：

- `joint id`
- `mode`
- `position`
- `velocity`
- `torque_ff`
- `kp`
- `kd`

### MC02 从控侧

MC02 负责具体实现层：

- 输出侧 -> 转子侧换算
- RS485 打包
- CRC16 校验
- 在线检测
- 超时处理
- 宇树错误码解析

### 宇树电机侧

只看到合法的 RS485 17B 命令和 16B 回包。

## 现有 `joint_goal_table` 还不够表达宇树混合控制

这是这次接入最关键的一点。

当前 `joint_goal_t` 定义是：

- `mode`
- `position`
- `velocity`
- `current`

它只能表示三选一：

- 电流模式
- 速度模式
- 位置模式

但宇树电机最有价值的控制方式是混合控制：

- `tau`
- `dq`
- `q`
- `kp`
- `kd`

这不是当前 `joint_goal_t` 能完整表达的。

所以从控制链路设计看，有两个方案。

## 方案 A：最小改动接入

适合你先把系统跑通。

### 思路

继续用现有 `joint_goal_table_t`，但只把它当成“简化关节命令”：

- `JOINT_GOAL_MODE_CURRENT`
  - 映射到宇树 `tau`
  - `q=0,dq=0,kp=0,kd=0`
- `JOINT_GOAL_MODE_VELOCITY`
  - 映射到宇树 `dq + kd`
  - `q=0,kp=0,tau=0`
- `JOINT_GOAL_MODE_POSITION`
  - 映射到宇树 `q + dq_limit + kp + kd`
  - `tau=0`

### 优点

- 改动最小
- 高层 task 基本不用大改
- 非常适合先接一个机械臂关节或云台关节

### 缺点

- 用不上宇树完整的混合控制优势
- `kp/kd/tau_ff` 很难由高层精确表达

## 方案 B：正确的长期方案

适合后续正式并入控制链路。

### 思路

扩展 `joint_goal_t`，增加一个混合控制模式，例如：

- `JOINT_GOAL_MODE_HYBRID`

并增加字段：

- `torque_ff`
- `kp`
- `kd`

这样高层就能直接表达宇树电机需要的抽象控制量。

### 推荐修改方向

在 `shared/User/application/robot/joint_goal_table.h` 里扩展：

- `joint_goal_mode_e`
- `joint_goal_t`
- 对应的 set/get helper

然后让需要宇树关节的控制器输出：

- `position`
- `velocity`
- `torque_ff`
- `kp`
- `kd`

MC02 只做执行，不做高层控制决策。

## 我建议你把“插入点”放在这里

### 不要插在 `joint_servo` 后面

因为 `joint_servo` 后面已经是：

- 电流
- CAN 槽位
- RM 电机假设

### 要插在“各 task 生成 joint goal”之后

也就是：

1. `chassis_domain/gimbal_domain/arm_domain/shoot_domain` 先生成 `joint_goal_table_t`
2. 对本地 RM 电机，继续交给 `joint_servo_submit`
3. 对宇树关节，交给新的 `mc02_bridge_task`

从架构上，这一步可以理解为：

- 先统一目标
- 再分不同执行器后端

这才是合理的“控制链路融合”。

## 最推荐的实现结构

### 1. 新增一个“执行器路由层”

建议加一个概念层，不一定叫这个名字，但职责要清晰：

- `joint_executor_router`

它负责：

- 哪些关节是本地 RM/CAN 执行
- 哪些关节是 MC02/Unitree 执行

### 2. 保留现有本地执行器后端

继续保留：

- `joint_servo`
- `actuator_cmd`
- `can_tx_task`

只让它处理本地 DJI/RM 关节。

### 3. 新增 `mc02_bridge_task`

它负责：

- 周期读取宇树相关 joint goals
- 打包发给 MC02
- 接收 MC02 回传状态
- 更新共享反馈

### 4. MC02 上再做 `unitree_executor_task`

MC02 自己负责：

- 接收 ARBATOS 主控的抽象关节目标
- 输出侧量 -> 转子侧量换算
- RS485 发宇树命令
- 解析宇树反馈
- 回传状态

## 主控和 MC02 之间建议用什么链路

推荐优先级：

1. `CAN/CANFD`
2. UART
3. USB

### 为什么优先 CAN

因为：

- ARBATOS 本来就是实时控制系统
- MC02 自带 `3 路 CANFD`
- 主控板本身也有 CAN
- CAN 更适合周期命令和状态回传
- 错误处理、丢帧、超时语义更好做

而 MC02 的板载 `RS485` 应该只留给宇树电机，不建议主控也来抢这条总线。

## 主控 <-> MC02 的消息应该长什么样

不要传宇树原始协议，建议传抽象包。

### 下行：主控发给 MC02

建议按“输出轴物理量”定义：

```c
typedef struct
{
    uint8_t joint_id;
    uint8_t mode;      // disable/current/velocity/position/hybrid
    uint8_t enable;
    uint8_t reserved;

    float position_rad;
    float velocity_rad_s;
    float torque_ff_nm;
    float kp;
    float kd;

    uint32_t seq;
} arbatos_mc02_joint_cmd_t;
```

### 上行：MC02 发给主控

```c
typedef struct
{
    uint8_t joint_id;
    uint8_t online;
    uint8_t motor_mode;
    uint8_t motor_error;

    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_deg_c;

    uint32_t tick_ms;
} arbatos_mc02_joint_state_t;
```

## 为什么一定要传“输出轴量”

因为从控制链路分工上讲：

- ARBATOS 高层做的是机器人关节控制
- 它应该只关心输出轴
- `6.33` 减速比、转子侧命令、CRC16、帧头这些都该由 MC02 管

也就是说：

- ARBATOS 传 `output-side`
- MC02 转 `rotor-side`

不要把 Unitree 的硬件特性上卷到 ARBATOS 高层。

## MC02 上应该承担哪些控制逻辑

### 应该放在 MC02 上的

- 输出轴 -> 转子侧换算
- `GO-M8010-6` 协议 pack/unpack
- CRC16
- RS485 收发时序
- 回包超时
- 宇树错误码处理
- 电机在线状态机

### 不建议放在 MC02 上的

- 行为决策
- 模式仲裁
- 控制资源分配
- 机器人级安全状态机

这些仍然应该由 ARBATOS 主控的 `control_hub` 和各 subsystem task 管。

## 在 ARBATOS 现代码里，最合理的改动点

### 第一层：joint 抽象

重点文件：

- `shared/User/application/robot/joint_goal_table.h`

如果你要长期用宇树混合控制，应该先从这里改。

### 第二层：执行器路由

重点位置：

- `shared/User/application/motor/actuator_cmd.c`

这里现在做的是“domain goal -> 本地电流输出重建”。
后续可以拆成：

- 本地 RM 执行器重建
- 外部 MC02 执行器转发

### 第三层：反馈回流

现在本地反馈是：

- `CAN_receive.c`

后续你需要增加一条并行反馈路径：

- `mc02_feedback.c` 或类似模块

负责把 MC02 回传的宇树状态映射成 ARBATOS 的统一关节状态。

## 如果你现在就要落第一版，我建议这样做

### 第一阶段

不要先改整个控制链路。

先做：

1. 为宇树关节单独定义一个 `mc02_bridge_task`
2. 让它直接读取某个 subsystem 的 goal
3. 发给 MC02
4. MC02 控宇树

### 第二阶段

等这条链路稳定后，再抽象成：

- 通用外部执行器后端
- 通用关节状态回流

### 最容易先接的子系统

优先级建议：

1. `arm/manipulation`
2. `gimbal`
3. `chassis`

因为：

- 机械臂关节最适合位置/速度混合控制
- 风险最小
- 不会像底盘那样要求多关节强同步

## 最终建议

如果你现在的目标是“把宇树通过 MC02 融入 ARBATOS 控制链路”，最稳的答案是：

1. 把 `MC02` 定义成“外部关节执行器板”，不是透明串口桥
2. 在 ARBATOS 里，插入点放在 `joint goal` 层，不放在 `CAN current` 层
3. 主控和 MC02 之间传“抽象关节目标/状态”，不要传宇树原始帧
4. 输出轴到转子侧的换算，全部放到 MC02
5. 第一版先用现有 `joint_goal_table` 做简化映射
6. 长期要用好宇树混合控制，就扩展 `joint_goal_t`

一句话概括：

`ARBATOS 负责“想让关节做什么”，MC02 负责“怎么用 RS485 驱动宇树把这件事做出来”。`
