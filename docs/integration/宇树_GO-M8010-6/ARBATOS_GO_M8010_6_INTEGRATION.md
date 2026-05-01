# ARBATOS 接入宇树 GO-M8010-6

## 结论

`GO-M8010-6` 现在按下面这条链接进 ARBATOS：

- RM / DJI 电机继续走 `actuator_cmd -> can_command_tx_task -> CAN 0x200/0x1FF`
- 宇树电机现在走 `MC02 板载 RS485 -> arm_task -> arm_motion -> unitree_motor_driver`

不要再把它当成“另一种 CAN 电机型号”塞进 `g_config.motor.*`。

这次接入以仓库里的本地参考工程为准：

- [local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Inc/motor_msg.h](/C:/Users/28111/Desktop/ARBATOS/local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Inc/motor_msg.h)
- [local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Src/motor_msg.c](/C:/Users/28111/Desktop/ARBATOS/local/reference/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/unitree-go-m-8010-6-communication-stm32-f411-project-using-nucleo-board-main/Core/Src/motor_msg.c)

## 协议基线

当前统一按这套来：

- 主机下发 `34` 字节命令帧
- 电机回 `78` 字节状态帧
- 包头用 `0xFE 0xEE`
- 校验用 `CRC32`
- 参考工程里的 CRC 算法按整 `uint32_t` 字做，发包校验前 `28` 字节，回包校验前 `72` 字节

旧文档里写的 `17B / 16B / CRC16-CCITT`，这版不再采用。

## 代码落点

这次改动后，相关入口是这些：

- [can_command_tx_task.c](/C:/Users/28111/Desktop/ARBATOS/shared/application/can_command_tx_task.c)
  - 非 RM 节点不再静默丢掉
  - 现在会走扩展口 `can_tx_process_extra_item(...)`
  - 没有扩展处理时，会记任务错误计数
- [shared/application/CAN_receive.c](/C:/Users/28111/Desktop/ARBATOS/shared/application/CAN_receive.c)
  - 固定 `motor_measure_t` 只给 RM 组播链用
  - 非 RM 节点会先走 `CAN_rx_process_extra_frame(...)`
  - 处理不了时会记接收链错误，不再硬按 RM 8 字节格式解
- [shared/application/arm_task.c](/C:/Users/28111/Desktop/ARBATOS/shared/application/arm_task.c)
  - `arm_task` 里接入 `arm_motion -> unitree_motor_driver`
  - 负责 RS485 4M、34B 发包、78B 回包、CRC32、在线超时
- [boards/DM_MC02_H7/app/board_freertos.c](/C:/Users/28111/Desktop/ARBATOS/boards/DM_MC02_H7/app/board_freertos.c)
  - `ARM_FAMILY_UNIFIED` 会起统一 `arm_task`
- [boards/DM_MC02_H7/bsp/bsp_usart.c](/C:/Users/28111/Desktop/ARBATOS/boards/DM_MC02_H7/bsp/bsp_usart.c)
  - 新增 RS485 口动态切波特率

## 当前这版能做什么

这是最小可用 bringup 版，不是完整机械臂域接入版。

已经有的：

- MC02 侧用板载 `USART2/USART3 RS485`
- 配置项 `arm.j0_unitree[800-809]`
- 默认按输出轴量输入，再在任务里换到转子侧
- 键盘调试：
  - `G` 正转
  - `Shift + G` 反转
  - 没按键时给阻尼或零力，取决于 `hold_kd`
  - 如果开了 `g_arm_deadman_hold_ctrl`，还要同时按 `Ctrl`
- 监控面能看到：
  - 在线状态
  - 电机模式
  - 错误码
  - 温度
  - 最近回包时间
  - 收发计数
  - CRC 错误计数

还没做的：

- 多电机总线调度
- `joint_goal_table` 到 Unitree 混控量的正式适配
- 机械臂域直接下发 `q/dq/tau/kp/kd`

## 配置说明

`arm.j0_unitree` 现在这几个字段有效：

- `800 enable`
- `801 rs485_port`
- `802 motor_id`
- `803 baudrate`
- `804 control_period_ms`
- `805 rx_timeout_ms`
- `806 reduction_ratio`
- `807 key_speed_rad_s`
- `808 hold_kd`
- `809 drive_kd`

推荐起步值：

- `enable = 1`
- `rs485_port = 0`
- `motor_id = 0`
- `baudrate = 4000000`
- `control_period_ms = 5`
- `rx_timeout_ms = 30`
- `reduction_ratio = 6.33`

## 现阶段建议

如果你下一步是要把它真正接进机械臂控制层，建议顺序还是这三个：

1. 先把单电机速度模式跑稳
2. 再补 `joint goal -> Unitree 命令` 适配
3. 最后做多关节调度和状态回流
