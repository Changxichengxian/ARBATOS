# HERO-M 只接收检查

后续用户已把pitch拨码改为5，软件同步改为CAN2/编号5，并补齐SD原始CAN记录，见 [SD日志补齐](ReceiveSd-20260906.md)。下文为首版接收固件的历史证据，不能据此认定新版已完成烧录。

## 配置与边界

- 按用户要求，把HERO-M的pitch显式改为CAN2，型号3510、ID 3及其他控制参数保留。
- 当前配置的第三个摩擦轮也在CAN2、ID 3，存在反馈ID 0x203冲突。实板诊断已报告1个路由冲突；等待用户确认实际摩擦轮数量、总线与ID，不擅自更改其他电机。
- `hero-m-receive.conf`叠加于HERO-M准备版。仅启动输入接收、CAN反馈解码、IMU及准备版SD记录，不创建底盘、云台、发射或电机发送线程。
- CAN保留硬件ACK（收到帧后的协议应答），所有应用CAN发送在公共入口返回失败。辅助串口及RS485应用发送同样返回失败；串口控制台和日志发送关闭。诊断通过DAP只读采集。
- 加热默认关闭；沿用已确认的HERO-M轴向及Flash零偏。主机没有打开加热。
- CAN帧复用正式`BspCanRxPop`、`CAN_rx_process_frame`，遥控复用正式`BspRcSbusRxPop`、`ManualInputOnSbusFrame`。原始CAN帧另外按总线和ID保留，避免配置错配遮住实际收到的内容。

## 编译与烧录证据

- 只接收版及正式HERO-M编译通过，无编译警告或错误。正式版只编译，没有烧入。
- 烧录前整片1MiB Flash独立读取两次一致，备份SHA-256：`1dadbc6d7af28aeec4d1884d50e9b4d984d24155a9f490567869085d27b7e92b`。
- 接收固件466072字节，全文读回一致，SHA-256：`71ee22d1f3b9ebdc079278562ecc3d2f2b233b06e97fb95325f58d06129af443`。
- 反汇编核对：CAN发送入口仅计数并返回失败，不存在硬件发送调用；辅助串口发送入口直接返回-EPERM。这是软件发送边界检查，未用示波器测CAN TX引脚。
- 原始证据：`local/cache/zephyr-preflight-hardware/20260906/rx-only/`；构建日志`local/cache/receive-build.log`和`receive-formal-build.log`。

## 当前实板接收结果

初始连续快照：三个CAN控制器均初始化成功，发送请求0，接收帧0，收发错误计数0；遥控串口初始化无错误，但尚未收到帧。IMU持续更新，加热0。尚不能据此判定CAN及遥控接收通过，等待用户接线、供电并做动作。

主机工具：`ReceiveProbe.py --elf <配套elf> --output <新目录> --seconds 600`。
固件保留64个不同总线/ID的统计；1MHz DAP为避免跨100ms发布周期，主机每帧读前16项。若读满16项会明确标记`frameTableMayBeTruncated`，不能把截断结果称为完整总线清单。

只允许一个DAP进程连接；创建采集目录中的`stop`文件可结束采集，固件继续保持只接收状态。
