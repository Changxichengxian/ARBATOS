# ARBATOS 通过 MC02 接宇树电机

## 结论

MC02 在这条链里是“宇树执行板”，不是透明串口转接板。

推荐链路：

`高层关节目标 -> MC02 Unitree executor -> MC02 板载 RS485 -> GO-M8010-6`

不推荐链路：

`joint_servo -> actuator_cmd -> can_command_tx_task`

## 协议基线

这份链路说明现在和本地参考工程对齐，按下面这套来：

- 下发 `34` 字节
- 回包 `78` 字节
- 包头 `0xFE 0xEE`
- 校验 `CRC32`

旧版 `17B / 16B / CRC16` 说法，这里不再使用。

参考源：

- [local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Inc/motor_msg.h](/C:/Users/28111/Desktop/ARBATOS/local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Inc/motor_msg.h)
- [local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Src/motor_msg.c](/C:/Users/28111/Desktop/ARBATOS/local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Src/motor_msg.c)

## 现在仓库里的实现边界

这次代码已经补到下面这一步：

- RM/CAN 主链不动
- 非 RM 节点在 CAN 收发链里不再被静默按 RM 处理
- MC02 侧统一走 `arm_task`，机械臂上层由 `arm_motion` 负责，底层由 `unitree_motor_driver` 负责 Unitree 驱动
- MC02 的 `USART2/USART3` 可以切到 `4 Mbps`
- 监控面可以看到 Unitree 电机状态

但还没有把宇树正式并进：

- 多关节 `joint_goal_table`
- 完整机械臂域
- 多电机调度

## 为什么要这么分层

原因很直接：

- ARBATOS 高层关心的是关节目标
- MC02 负责把关节目标翻成宇树协议
- `6.33` 减速比、转子侧量、CRC32、帧收发时序都留在 MC02 这层处理

这样主控不用知道宇树帧长、校验和字节布局，后面换执行器也更好收口。
