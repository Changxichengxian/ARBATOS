# M 板副板 SD 独立测试固件

用途：先把 DM MC02 / STM32H723 + 第二版副板的 SPI3 存储性能测清楚，后续再把验证过的方案接到 HERO-M / Zephyr 日志中。此固件使用最小 FreeRTOS + FatFs，与正式机器人启动入口分开；不包含机器人任务，不初始化 CAN、电机、IMU、遥控或执行器。RTC 和按钮也不参与本轮测试。

当前交付已经通过 Keil 编译、电脑上的模拟协议测试，以及 2026-09-05 的实物测试：Horco CMSIS-DAP 连接 M 板，10 档全部通过，累计写入并读回校验 152 MiB。48 MHz 下两轮 32 MiB 写入约 3.49—3.53 MB/s，纯读取约 1.74 MB/s。详见 [实测记录](HardwareResult-20260905.md)。这些结果只验证此板此卡的独立 SD 固件，HERO-M 适配及 Zephyr 运动问题仍未验证。

## 硬件和频率

- M 板：STM32H723，沿用之前副板实测的 480 MHz 主频配置。
- SD：SPI3，PC10=SCK、PC11=MISO、PC12=MOSI，PE14=CS。
- SPI123 时钟 96 MHz，测试 6 / 12 / 24 / 48 MHz。任何实际频率与目标不符都会停止。
- 48 MHz 只在 24 MHz 下完成 CMD6 查询、切换及状态 CRC16 校验后运行；明确不支持则记录跳过，通信或切换异常则停止。
- 本版没有 DMA；mode 2 使用 SPI 的 FIFO（外设内部缓冲区）阈值 4，让 HAL 写入路径按 32 位搬运。读入仍是 HAL 阻塞式接收，不能把这版称为 DMA 吞吐率测试。

## 自动测试内容

| 档位 | SPI | 接收方式 | 读取命令 | 文件大小 |
| --- | --- | --- | --- | --- |
| 0 | 6 MHz | 原 64 字节 HAL | CMD17 单块 | 8 MiB |
| 1 | 12 MHz | 原 64 字节 HAL | CMD17 单块 | 8 MiB |
| 2 | 24 MHz | 原 64 字节 HAL | CMD17 单块 | 8 MiB |
| 3 | 24 MHz | 整 512 字节 HAL | CMD17 单块 | 8 MiB |
| 4 | 24 MHz | 整 512 字节 HAL | CMD18 连续多块 | 8 MiB |
| 5 | 24 MHz | 整 512 字节 HAL + FIFO | CMD18 连续多块 | 8 MiB |
| 6 | 48 MHz | 整 512 字节 HAL + FIFO | CMD18 连续多块 | 8 MiB |
| 7—9 | 成功档中同量写读总耗时最短的配置 | 同最佳档 | 同最佳档 | 每轮 32 MiB |

写入保留原 H723 的 CMD25 多块机制。档 0—2 的接收分成 64 字节，写入保持原来的整扇区 HAL，避免人为拖慢旧版基准。

应用每次调用 FatFs 读写 32 KiB，每档创建全新的 `SBxxxxxx.BIN` 文件，写完同步、关闭，再重开读回，逐个 32 位字比较全部数据。每轮内容与位置相关，防止旧缓存、错地址或短读被误判为成功。

- `writeUs`：所有 `f_write` 耗时；`syncUs`：同步和写文件关闭耗时；写速度包含这两项。
- `readUs`：所有 `f_read` 耗时；`verifyUs`：生成预期数据和比较耗时，单独列出。
- `maxWriteUs / maxReadUs`：单次 32 KiB 调用的最大等待，后续做日志缓存时要看它，不能只看平均速度。
- 档 9 额外检查卡返回的每个接收块 CRC16，因此其 `readUs` 包含 CRC 计算开销；只用于加强校验，不与前两轮直接比较速度。
- 之前的 0.91 MB/s 包含校验，本版纯读取速度需要与 `readUs + verifyUs` 的口径区别开。数据量和块大小也变了，旧报告不是本轮严格对照。

完整一轮最多写 152 MiB 并全部读回；同一时刻最大临时文件 32 MiB。启动时要求至少 64 MiB 空间。不格式化，不覆盖已有同名文件，不直接写裸扇区。成功的测试文件会删除；任何读写、底层传输或校验错误都会停止并保留文件及 RAM 现场，不继续在异常链路上清理目录。重新复位会开始新一轮。

## 历史构建

该独立测试使用的 Keil 生成器和工程模板已从主线删除。保留这里的测试源码、协议检查和实测结果供查阅；要重现 2026-09-05 的独立固件，请在另一份检出目录使用历史提交 `951857f`，其中有 `tests/SdBenchM/Build.py` 及 `projects/`。不要把历史测试固件当作当前 HERO-M 固件。

以下结果读取说明依赖当时生成的 `local/cache/sd-bench-m/firmware.json`，不属于当前固件的编译入口。

## 接板后读取结果

先确认连接的是目标 M 板，备份其**当前**完整 1 MiB Flash。不要直接拿上一轮的旧备份覆盖当前程序。烧录本次 HEX 后复位即可开始；测试只需板子、副板、SD 卡和调试器，执行机构断电。

用 `python -m pyocd list` 查实际调试器 ID，然后执行：

```powershell
python -X utf8 D:\ARBATOS\tests\SdBenchM\ReadResult.py --manifest D:\ARBATOS\local\cache\sd-bench-m\firmware.json --probe 实际调试器ID --watch 600 --output D:\ARBATOS\local\cache\sd-bench-m\measurement
```

读取工具仅附加读取 RAM，不暂停、不复位、不烧写。输出 `sd-bench-ram.bin`、JSON 和 CSV。`phase=4` 为本轮完成，`phase=5` 为失败；`cases[i].result=0` 才是该档成功，`result=1` 表示 48 MHz 被跳过，未执行档 `targetHz=0`。整体完成不代表 48 MHz 一定通过。

失败时看 `currentCase`、`error`、`activeFile`、`fsResult`、`mismatchOffset` 和传输错误计数。卡上空间不足是 `-328`，时钟配置不符是 `-326`。恢复程序时使用本轮新备份并读回校验完整 Flash 哈希；是否恢复启动，需结合当时接车状态决定。

## 协议检查

`ProtocolTest.c` 直接编译实际 SD 驱动，通过模拟端口检查 CMD17、CMD18/CMD12 正常及失败清理、CMD6 支持/拒绝/busy/非法命令、CRC 正确与损坏的处理。它不模拟 STM32 外设、卡内部时延、布线、FatFs 或真实断电行为。

运行 `HostChecks.py` 可在本机 VS2022 Build Tools 下复跑协议测试，同时核对结果解码格式。运行日志保存在构建目录的 `host-tests` 内。

SD 协议依据为 SD Association 的 Physical Layer Simplified Specification，第 4.3.10 节（CMD6）与第 7 章（SPI 模式）；[官方规范入口](https://www.sdcard.org/downloads/pls/)。SPI 搬运方式以仓库自带 `stm32h7xx_hal_spi.c` 的实现为准。
