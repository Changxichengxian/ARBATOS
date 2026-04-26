# ARBATOS 中接入 Unitree GO-M8010-6 的建议方案

## 结论

不要把 `GO-M8010-6` 当成 ARBATOS 现有的 DJI/RM CAN 电机去接。

它和 ARBATOS 现在这套电机链路的核心区别是：

- ARBATOS 现有主链路是 `joint_servo -> actuator_cmd -> can_tx_task -> CAN 0x200/0x1FF 组播电流帧`
- `GO-M8010-6` 官方协议是 `RS-485/4 Mbps/主从问答`，不是 CAN 电调
- ARBATOS 现有电机模型本质上都是“发电流，收 8B 反馈”
- `GO-M8010-6` 是“发 17B 位置/速度/力矩/刚度/阻尼命令，收 16B 状态帧”

所以最稳的接法是：

1. 保留现有 CAN 电机链路不动
2. 给 `GO-M8010-6` 单独开一条 `RS-485` 执行器链路
3. 在 ARBATOS 里把它当成“独立智能关节”，而不是“另一种 CAN 电机型号”

## 为什么不能直接塞进现有 `motor_model`

现有实现默认了几个前提：

- 反馈来自 CAN 标准帧，见 `shared/User/application/can/CAN_receive.c`
- 控制量是 `int16_t current`，见 `shared/User/application/motor/actuator_cmd.c`
- CAN 发送按 `0x200/0x1FF` 两组 4 路打包，见 `shared/User/application/can/can_tx_task.c`
- 电机模型只描述 `CAN ID base + current limit + reduction ratio`，见 `shared/User/application/motor/motor_model_db.c`

`GO-M8010-6` 不满足这些前提：

- 通信总线不是 CAN，而是 `RS-485`
- 通信速率是 `4 Mbps`
- 协议是单电机点对点命令/应答，不是 4 路电流组播
- 控制接口是 `tau / dq / q / kp / kd`
- 状态反馈是 `torque / speed / position / temp / error`

如果强行塞进 `motor_model`，后面会把 `can_tx_task`、`CAN_receive`、`actuator_cmd`、`motor_measure_t` 全部扭坏。

## 官方资料里对接入最关键的点

你放进仓库的资料里，和 ARBATOS 接入最相关的信息如下：

- `GO-M8010-6_Motor_Data_User_Manual_V1.0.pdf`
  - 工作电压：`12~30 VDC`，推荐 `24 VDC`
  - 通信：`High Speed 485`
  - 波特率：`4 Mbit/s`
  - 控制频率：`6 KHz`
  - 减速比：`1:6.33`
  - 反馈量：`Torque / Angle / Angular Velocity / Temperature`
- `GO-M8010-6_Motor_User_Manual_V1.0.pdf`
  - 主机发 `17B` 命令，电机回 `16B` 状态
  - 命令头：`0xFE 0xEE`
  - 反馈头：`0xFD 0xEE`
  - CRC：`CRC16-CCITT`
  - 串口格式：`8N1`
- `unitree_actuator_sdk/README.md`
  - 官方示例明确按“转子侧”填写 `q/dq/kp/kd`
  - 输出侧控制量要按减速比换算

注意：

- 手册表格里的中英文描述对 “joint/output/rotor” 有翻译混用
- 但 SDK README 和示例代码都明确要求按转子侧发送
- 在 ARBATOS 里接入时，应以官方 SDK 示例的换算方式为准

## 推荐的硬件接法

### 总线与电源

- 控制板串口：优先用 `UART8`
  - 板级文件：`boards/DJI_A_F427/Core/Src/usart.c`
  - 当前 `UART8 (PE1/PE0)` 没有被 ARBATOS 业务占用
- 外挂一个 `RS-485` 收发器
  - 优先选带自动收发方向切换的模块，这样 MCU 不需要额外 `DE/RE` 管脚
  - 如果你用普通 `MAX3485/SP3485`，再额外找一个 GPIO 控 `DE/RE`
- 电源按宇树手册走 `24V`
- 多电机串接时，总线两端加 `120 ohm` 终端电阻

### 为什么推荐 UART8

当前 F427 板上的串口占用大致是：

- `USART1`：遥控/ELRS/SBUS
- `UART7`：裁判系统
- `USART6`：shared 里保留了 USART6 referee 路径
- `UART8`：当前未被 ARBATOS 主流程占用

因此第一版接入，最适合挂在 `UART8`，避免和现有输入链路打架。

## 推荐的软件架构

### 方案总览

建议新增一条并行链路：

1. `bsp_unitree_rs485`
   - 基于 `UART8`
   - 负责发 17B、收 16B
   - 负责半双工时序或方向控制
2. `unitree_go_m8010_6_proto`
   - 负责命令打包、状态解包、CRC16
3. `unitree_motor_task`
   - 周期发送命令并收反馈
   - 维护在线状态、错误码、最新反馈
4. `unitree_joint_adapter`
   - 把 ARBATOS 内部控制目标映射成 `tau/dq/q/kp/kd`

### 为什么要单独 task

因为 `GO-M8010-6` 是主从式问答，流程更像：

1. 准备单个关节命令
2. 发串口帧
3. 等回包
4. 校验 CRC
5. 更新状态

这和现有 CAN 的“先写共享电流，后统一组帧广播”不是一类东西。

## 在 ARBATOS 里最适合先挂到哪里

### 最推荐：先作为 `CARRIER` 的机械臂独立关节

理由：

- `CARRIER` 已经有机械臂控制域：`shared/User/application/robot/arm_domain.c`
- 不会影响底盘 `chassis_domain`
- 不会打穿云台/发射现有稳定链路
- 宇树电机自带位置/速度/力矩混控，比“夹爪纯电流”更适合机械臂关节

### 不推荐的第一版接法

- 不建议先替换底盘电机
  - 现有底盘是 4 路 CAN 同步电流控制
  - Unitree 逐个关节问答式链路不适合第一版直接进底盘闭环
- 不建议先替换云台 yaw/pitch
  - 现有云台控制周期很快，且围绕 RM 电机反馈结构写死较多
- 不建议先放进 `motor_model_e` 里冒充 CAN 电机

## 和现有控制层的对接方式

### 现有 ARBATOS 控制层是什么样

共享层现在是：

- `chassis_domain.c` 产生底盘轮目标
- `gimbal_domain.c` 产生 yaw/pitch 目标
- `shoot_domain.c` 产生拨盘/摩擦轮目标
- `arm_domain.c` 产生机械臂目标
- 这些目标统一提交到 `joint_servo_submit(...)`
- `joint_servo` 最终生成 CAN 电流

### Unitree 接入时该怎么接

建议不要让 `joint_servo` 直接给 Unitree 发串口。

建议做成两层：

1. 高层仍然产出“关节目标”
2. Unitree 专用适配层把目标翻成 `tau/dq/q/kp/kd`

也就是：

- `joint_goal_table` 继续作为高层统一接口
- 但 `GO-M8010-6` 不走 `actuator_cmd/can_tx_task`
- 它走自己的 `unitree_motor_task`

## 目标量换算建议

设减速比 `r = 6.33`

若 ARBATOS 高层使用的是输出轴量，则发给宇树时按下面换算：

- `q_rotor = q_out * r`
- `dq_rotor = dq_out * r`
- `tau_rotor = tau_out / r`
- `kp_rotor = kp_out / (r * r)`
- `kd_rotor = kd_out / (r * r)`

其中：

- `q` 单位：`rad`
- `dq` 单位：`rad/s`
- `tau` 单位：`N*m`

`tau_rotor = tau_out / r` 这一条是按减速器力矩关系做的工程换算，官方 README 没直接写这一行，但它已经明确了命令量按转子侧填写。

## 控制模式映射建议

### 速度模式

如果高层给的是“某关节目标速度”：

- `tau = 0`
- `q = 0`
- `kp = 0`
- `dq = dq_out * 6.33`
- `kd` 给一个小正数

这最适合先做“机械臂升降/开合”的第一版调通。

### 位置模式

如果高层给的是“某关节目标角度”：

- `q = q_out * 6.33`
- `dq = dq_limit * 6.33`
- `kp`、`kd` 走输出侧设计后再换算到转子侧
- `tau` 做前馈，可先从 `0` 开始

### 零力/阻尼模式

对应手册里的：

- 零力模式：`tau=0, dq=0, q=0, kp=0, kd=0`
- 阻尼模式：`tau=0, dq=0, q=0, kp=0, kd>0`

这两个模式非常适合作为 ARBATOS 的“安全上电模式/失能模式”。

## 第一版最小落地路径

### Phase 1: 先在 PC 上把电机单独跑通

直接用你放进仓库的官方 SDK：

- `docs/packages/宇树_GO-M8010-6/unitree_actuator_sdk/example/example_goM8010_6_motor.cpp`
- `docs/packages/宇树_GO-M8010-6/unitree_actuator_sdk/example/changeID.cpp`

先确认：

- 电机 ID 是你想要的值
- 已切回 motor mode，不是 factory mode
- 速度模式能正常转
- 回包正常、温度正常、无错误码

### Phase 2: MCU 侧只做协议通

在 ARBATOS 第一版里，只做这几个动作：

1. `UART8` 改成 `4 Mbps, 8N1`
2. 接 `RS-485` 模块
3. 发固定速度命令
4. 收固定格式回包
5. 校验 `CRC16-CCITT`
6. 把 `q/dq/temp/error` 打到调试口或日志

到这一步先不要接入 `joint_servo`

### Phase 3: 接到机械臂子系统

建议优先做“单关节速度控制”：

- 让 `arm_domain` 或新建的 `unitree_joint_task` 读取机械臂控制目标
- 把目标速度映射成 `dq`
- 把 `open/close/off` 或 `lift up/down/off` 映射成 `velocity / damping / zero torque`

这样改动面最小，最容易先让关节动起来。

### Phase 4: 再做位置模式和统一抽象

等速度模式稳定后，再做：

- 输出轴角度闭环
- 限位保护
- 失联超时
- 错误码处理
- 和 `joint_goal_table` 更深的统一

## 你在 ARBATOS 里应该改哪些地方

如果按上面的推荐做，建议修改点如下：

- 板级
  - `boards/DJI_A_F427/Core/Src/usart.c`
  - 把 `UART8` 配成 `4 Mbps`
  - 如需方向控制，再补 GPIO
- BSP
  - 新增 `bsp_unitree_rs485.c/.h`
  - 风格参考：
    - `target/CARRIER/User/bsp/boards/bsp_uart_dispatch.c`
    - `shared/User/bsp/boards/bsp_usart.c`
- 协议层
  - 新增 `unitree_go_m8010_6_proto.c/.h`
  - 负责 pack/unpack + CRC
- 应用层
  - 新增 `unitree_motor_task.c/.h`
  - 周期发命令、收反馈、维护状态
- 机械臂层
  - 先在 `shared/User/application/robot/arm_domain.c` 旁边挂一层适配
  - 或者单独建 `unitree_joint_adapter.c/.h`

## 一个更贴近 ARBATOS 现风格的建议

你现在代码里的控制风格，是“高层目标统一，底层执行器各走各的传输”。

所以 `GO-M8010-6` 最适合按下面这个思路并入：

- 高层统一：继续用 `joint_goal_table`
- 底层分流：
  - DJI/RM 电机 -> `actuator_cmd + can_tx_task`
  - Unitree 电机 -> `unitree_motor_task`

也就是说，把“统一”放在目标层，不要强行放在总线层。

这是最符合 ARBATOS 现架构的接法。

## 我给你的最终建议

如果你的目标是“尽快在 ARBATOS 里把这颗宇树电机用起来”，就按下面顺序做：

1. 用官方 SDK 在上位机把 `GO-M8010-6` 单独跑通
2. 在 F427 板上用 `UART8 + RS-485` 做最小串口收发
3. 在 ARBATOS 里新增 `Unitree` 独立任务，不要先改 CAN 链路
4. 第一版只把它接成 `CARRIER` 的一个机械臂关节
5. 先上速度模式，再上位置/混合控制

如果你后面要我继续往下做，下一步最值得直接落代码的是：

- 先补 `UART8 RS-485 BSP`
- 再补 `GO-M8010-6` 协议打包/解包
- 最后补一个最小 `unitree_motor_task`

这条路径风险最低，也最符合 ARBATOS 现在的结构。
