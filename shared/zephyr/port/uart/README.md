# Zephyr UART port

这个目录实现了共享层的 `BspRc`、`BspUsart` 串口接口，使用固定缓冲区，不使用堆内存。

## 绑定

目标 DTS 可用下列别名自动绑定角色：`uart-rc`、`uart-aux`、`uart-referee`、`uart-rs485-0`、`uart-rs485-1`。也可以在编译前用角色宏覆盖，例如：

```c
#define ARB_UART_RC_NODE       DT_NODELABEL(usart3)
#define ARB_UART_AUX_NODE      DT_NODELABEL(usart1)
#define ARB_UART_REFEREE_NODE  DT_NODELABEL(usart6)
#define ARB_UART_RS485_0_NODE  DT_NODELABEL(usart2)
#define ARB_UART_RS485_1_NODE  DT_NODELABEL(usart3)
```

外部角色宏优先于 DTS 别名，通常由目标专用配置头提供。未绑定角色会明确返回 `-ENODEV`；RC/裁判接收会保留错误诊断，绝不静默假装已启动。一个 UART 只能分给一个运行中的角色。

## 行为边界

- 优先用 Zephyr async UART：裁判和 SBUS 以 `ARB_UART_IDLE_TIMEOUT_US` 模拟空闲线分包；AUX 的 `RX_RDY` 被转换成旧接口的 IDLE/TC 事件。
- async API 不可用时，AUX 和 RS485 自动退回 IRQ 逐字节接收；RC 与裁判也会逐字节写入固定双缓冲，并在最后一个字节静默 `ARB_UART_IDLE_TIMEOUT_US` 后由工作队列提交完整块。中断内不分配、不等待。
- IRQ 回退中，RC 块必须严格等于共享层 `BSP_RC_SBUS_FRAME_LENGTH` 才会提交；溢出、串口错误和长度不符都会丢弃并计入诊断。当前共享层实际固定为 18 字节 DJI DBUS 帧，不能在不修改共享解析器的情况下把真正 25 字节 SBUS 帧交给它。
- AUX 的 async 接收使用调用方提供的固定缓冲区。缓冲区写满后会报告 TC 并重新开始接收，存在一个很短的重启窗口；对必须零间隙的持续高速流，目标应启用支持 async UART 的 DMA 驱动并评估实际吞吐。
- 发送数据都会复制到本端固定缓冲区后再提交；长度超过 512 字节（RS485 为 40 字节）会返回 `-EMSGSIZE`。
- H7 普通 RS485 发送和故障锁共用最终提交边界；正常任务中故障锁返回后不会再提交
  新帧。异常上下文只做原子锁定并交给上层立即复位，原始紧急发送仍明确不支持。

Zephyr 配置需要启用 UART 中断接口。若目标驱动同时支持 async UART，可进一步打开异步接口以使用驱动级空闲超时；当前三块 STM32 板的完整构建走 IRQ 加延时工作项回退路径。
