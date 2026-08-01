# ARBATOS Zephyr CAN 后端

这个目录把 `shared/hal/BspCan.h` 接到 Zephyr 4.4 CAN 控制器接口。三块板都使用
设备树别名：

- `can-primary` -> CAN1/FDCAN1
- `can-secondary` -> CAN2/FDCAN2
- `can-tertiary` -> FDCAN3（仅 MC02 H723）

## 发送完成语义

`BspCanTx*()` 返回 0 只表示 `can_send()` 已经接受帧并分配发送邮箱。所有发送都带
Zephyr 完成回调；受跟踪发送只有在回调到达后才生成 `BspCanTxCompletion`：

- 回调错误为 0：`Complete`
- 已停止/取消：`Aborted`
- Zephyr 明确定义的发送错误：`Failed`
- 未识别的驱动错误：`Unknown`

回调使用的上下文、RX 队列和完成队列都是固定容量静态对象。中断回调只更新状态、
写环形队列和通知 `CanRxTask`，不会分配内存，也不会把发送提交延后到工作队列。
完成环形队列暂时满时，终态保留在原发送槽，`BspCanTxCompletionPoll()` 或
`BspCanTxCompletionPop()` 会继续搬运。

## 故障环境限制

Zephyr 4.4 公共 CAN 接口没有“致命异常里绕过内核锁、撤销指定邮箱、直接操作原始
寄存器并有界确认上总线”的保证。因此：

- 正常任务上下文中，`BspCanFaultLock()` 与最终驱动提交串行，返回后永久阻止
  后续普通发送；异常/中断上下文会立即原子锁定，随后由故障链直接复位；
- `BspCanFaultTx()` 明确返回失败；
- `BspCanZephyrFaultTxSupported()` 明确返回 0；
- `BspCanFaultWaitTxIdle()` 不会伪造已经发送完成。

上层在启用 Zephyr 实车故障停机前，必须提供独立的硬件断能路径，或者分别为 bxCAN
和 STM32 FDCAN 编写、审计并实测专用的原始寄存器紧急发送实现。

## Zephyr 配置要求

集成方需要把 `BspCanZephyr.c` 加入应用，并启用 `CONFIG_CAN=y`。H723 使用 FD/BRS
时还需要 `CONFIG_CAN_FD_MODE=y`。三个控制器的仲裁速率必须在设备树或 Kconfig
明确设为实车所需值；不要依赖 Zephyr 默认速率。

`_smoke` 是只检查本后端和 Zephyr 驱动能否共同编译、链接的最小应用。当前已使用
Zephyr 4.4.0 和 Zephyr SDK 1.0.1 完整编译通过 `dji_c_f407`、`dji_a_f427`、
`dm_mc02_h7` 三块板；它不替代总线和执行器实测。

## 必须上硬件验证

1. F407/F427 两路 bxCAN 同时满载时，三个硬件邮箱的回调归属和完成顺序。
2. H723 三路 FDCAN 同时发送，Classic CAN、FD、BRS 三种标志组合。
3. 无 ACK、仲裁丢失、错误被动、Bus-Off、停机重配时的回调错误码。
4. RX 环形队列满时丢帧计数，以及 `CanRxTask` 高负载唤醒不丢通知。
5. 完成队列故意堵满后的延后搬运和票据唯一性。
6. `BspCanFdSetDataBitrate()` 停止/启动期间所有在途票据的终态。
7. 断电、看门狗和 HardFault 路径确实由 CAN 之外的措施把执行器带到安全状态。

`LastErrorCode`、`DataLastErrorCode`、控制器 activity 和 FDCAN error logging count
没有对应的 Zephyr 公共 API，目前明确返回 0。TX/RX 错误计数和控制器状态来自
`can_get_state()`。
