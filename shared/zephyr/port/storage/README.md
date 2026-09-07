# Zephyr 存储端口

此目录让共享层的 `SdCard`、`SdLog`、仓库自带 FatFs 和 `SdSpi` 继续使用原有接口：`disk_initialize`、`disk_read`、`disk_write`、`disk_ioctl` 与 `BspSdSpiPort`。

## 后端选择

- F427 的 SDMMC1：`sdmmc1` 状态为 `okay` 时，默认使用 Zephyr `disk_access` 的磁盘名 `SD`。
- 任何 Zephyr 磁盘设备：DTS 添加 `sd-disk` 别名，别名节点须有 `disk-name` 属性；该名字交给 `disk_access_*`。
- F407 SPI2、H723 SPI3：保留仓库 `SdSpi.c` 协议层，DTS 添加 `sd-spi` 别名，指向一个 SPI 子设备节点。该节点需要正常的 `reg`、`spi-max-frequency`，且其 SPI 控制器以 `cs-gpios` 提供片选。`BspZephyrSdSpiPort.c` 用同步 Zephyr SPI API 和该片选实现字节、块传输。

没有磁盘或 SPI 绑定时，初始化明确保持 `STA_NOINIT`，读写返回 `RES_NOTRDY`，不会误报挂载成功。

## 集成约束

仓库 `shared/components/support/fatfs/ff.c` 与本目录的 `BspZephyrDiskio.c` 是一组：应启用它们，且不要同时启用 Zephyr 自己的 FatFs 适配层，否则 `f_*` 或 `disk_*` 符号会重复。源码选择由 `projects/CMakeLists.txt` 和显式清单负责，设备绑定由板级设备树及车型 overlay 提供。

缓存和协议层均为固定大小静态内存；SPI 接口调用同步 Zephyr SPI API，接收用虚拟发送块：M 板每块 512 字节，其他板每块 64 字节。这里并未启用异步 DMA。SD 卡移除或任意传输错误会清除初始化状态，后续 `disk_initialize()` 可重试。

Zephyr 同步 SPI API 没有逐次传输超时参数，因此旧 `timeout_ms` 接口保留但由驱动完成时间决定；对总线卡死的恢复仍需要板级复位或看门狗策略。

## M 板频率和已测范围

SPI3 在初始化阶段请求 400 kHz，96 MHz 内核时钟下实际为 375 kHz；普通模式为 24 MHz，仅 CMD6 高速查询和切换成功后使用 48 MHz。同步接口仍依赖底层驱动实现，不能只凭请求频率推断实际吞吐。

当前 HERO-M 有 Zephyr SD 日志与歌曲播放实测，见 [验证记录](../../../../tests/ZephyrMusicM/Validation.md)。`tests/SdBenchM` 的 152 MiB、约 3.5 MB/s 数据来自旧 HAL 独立固件，不能作为当前 Zephyr 固件的吞吐率。
