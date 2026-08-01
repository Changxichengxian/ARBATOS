# ARBATOS Zephyr USB CDC 兼容层

这个目录把历史 CubeMX `usbd_cdc_if.h` 接口接到 Zephyr 4.4 的新 USB 设备栈
CDC ACM 设备上。发送使用固定 2048 字节环形队列，不分配应用层动态内存。

## 行为

- `CDC_Transmit_FS()` 在返回前复制完整数据，但不会等待主机发送完成。
- 主机未配置、USB 挂起、DTR 未置位或队列剩余空间不足时返回 `USBD_BUSY`。
- 无效指针、单次长度超过整个队列或后端不可用时返回 `USBD_FAIL`。
- 每次调用要么完整入队，要么完全不入队，不会出现半包。
- UART IRQ 每次最多取 64 字节 RX 数据，立即交给
  `VisionLinkRxCallback()`。
- 断连和 USB reset 会清空尚未发送的数据，并计入诊断。
- `BspUsbCdcGetDiag()` 可读取连接、DTR、队列水位、收发字节和错误计数。

这里的 UART 指 Zephyr 把 CDC ACM 暴露成的虚拟串口，不占用板上的实体串口。

## CMake 接入

加入源文件：

```cmake
zephyr/port/usb/BspZephyrUsbCdc.c
```

加入头文件目录：

```cmake
zephyr/port/usb
```

`BspUsbDeviceInit()` 应改为调用：

```c
(void)BspUsbCdcInit();
```

不要再由 `BspUsbDeviceInit()` 直接调用 `usbd_enable()`，USB 上下文只能初始化和
启用一次。

## prj.conf 接入

```conf
CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_USBD_CDC_ACM_CLASS=y
CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=n
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_UART_LINE_CTRL=y
```

正式固件还应设置自己的 `CONFIG_USB_DEVICE_VID`、`CONFIG_USB_DEVICE_PID`、
`CONFIG_USB_DEVICE_MANUFACTURER` 和 `CONFIG_USB_DEVICE_PRODUCT`。Zephyr 默认的
VID/PID 只供测试，不能直接当作量产设备身份。

## 设备树接入

在每块板的 `zephyr_udc0` 下加入一个 CDC ACM 节点：

```dts
&zephyr_udc0 {
    cdc_acm_uart0: cdc_acm_uart0 {
        compatible = "zephyr,cdc-acm-uart";
    };
};
```

当前代码使用 `DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart)`，所以只能放一个状态为
`okay` 的 CDC ACM 节点。

设备描述符延续旧 CubeMX 工程的 `VID=0x0483`、`PID=0x5740`、
`STMicroelectronics / STM32 Virtual ComPort`，避免已有上位机识别发生无意义
变化。这组标识属于 ST；若固件作为独立产品发布，必须替换为产品方合法取得的
VID/PID 和厂商字符串。

## 已做编译验证

使用 Zephyr 4.4 和 Zephyr SDK 1.0.1，`_smoke` 示例已在以下目标完成编译和链接：

- `dm_mc02_h7/stm32h723xx`
- `dji_c_f407/stm32f407xx`
- `dji_a_f427/stm32f427xx`

## 实机检查

编译通过只说明接口和 USB 控制器能组合。上板后还需要检查 Windows 枚举、串口
DTR、持续发送时的 BUSY 比例、断开重连、连续小包和超过 64 字节的接收分块。
