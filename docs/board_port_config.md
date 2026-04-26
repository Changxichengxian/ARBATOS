# 板级口子配置

结论先说：

- 现在和串口用途有关的东西，平时只看 `target/*/User/application/bsp_board_port_usage.h`。
- 这个文件上半段是参考说明，方便接线和对照 CubeMX / Keil。
- 这个文件下半段的宏才是真正生效的绑定关系。
- `boards/*/User/bsp/boards/bsp_board_layout.h` 只保留会被代码直接用到的固定资源，比如按键、蜂鸣器、IMU 相关句柄。
- 运行参数还是继续放 `config.c` / `config.h`。

## 日常怎么用

### 1. 看板子口子

打开对应 target 的 `bsp_board_port_usage.h`。

先看文件头部注释，这里会写：

- 板子有哪些串口
- 每个口的 TX / RX / DE 在哪
- 哪些口有硬件反相
- 哪些口带 DMA
- 哪些口实际上更适合固定用途
- 关键引脚在哪，比如 KEY

这部分只是说明，改了不会影响编译结果。

### 2. 改当前用途

还是改同一个 `bsp_board_port_usage.h`。

直接改下面这些宏：

- `BSP_BOARD_RC_UART_HANDLE`
- `BSP_BOARD_RC_UART_IRQn`
- `BSP_BOARD_RC_DMA_HANDLE`（有的板子有，有的没有）
- `BSP_BOARD_AUX_UART_HANDLE`
- `BSP_BOARD_REFEREE_UART_HANDLE`
- `BSP_BOARD_RS485_PORT0_UART_HANDLE`
- `BSP_BOARD_RS485_PORT1_UART_HANDLE`

比如想把裁判口换走，就改 `BSP_BOARD_REFEREE_UART_HANDLE` 对应到哪个 `huartx`。

如果接收机口也跟着换了，再一起改它的 `IRQn` 和 `DMA` 宏。

### 3. 改运行参数

这些还是在原来的地方改：

- 波特率
- PID
- 控制周期
- 其他运行时参数

不要再把“这个口现在给谁用”塞回 `config.c`。

## 兼容老代码

老代码还是继续包含 `bsp_board_ports.h`。

只是现在这个头文件基本只做一件事：

- 把 `bsp_board_layout.h`
- 和 target 本地的 `bsp_board_port_usage.h`

一起带进来。

所以业务代码里现有这些宏还能继续用：

- `BSP_BOARD_RC_UART_HANDLE`
- `BSP_BOARD_AUX_UART_HANDLE`
- `BSP_BOARD_REFEREE_UART_HANDLE`

## 现在这套的实际好处

- 平时只进一个文件，入口更集中
- 接线和改用途在同一处看，来回切文件少很多
- 上面是“看板子”，下面是“改分配”，脑子里更顺
- 板子固定资源还留在板级头文件里，不会和口子分配混在一起
