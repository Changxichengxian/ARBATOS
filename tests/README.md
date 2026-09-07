# 测试说明

测试资料分为三类：板级独立编译、历史 SD 台架实测、正式 Zephyr HERO-M 的 SD 与音乐功能记录。命令和结果都标明适用条件；编译、历史台架和实车结果不能互相替代。

## 板级独立编译

`tests/Boards` 保留 A、C 板在旧车型删除后仍能独立使用的构建检查。A 板同时编译 SDMMC 与 MPU6500，并在编译时检查晶振、DBUS、按键、指示灯和 SD 检测配置；C 板检查原有外设定义和共用按键适配。

准备 Zephyr 4.4 与 SDK 后，在仓库根目录执行：

```powershell
west build -s tests/Boards -b dji_a_f427 -d local/build/board-check-dji_a_f427 -- -DEXTRA_CONF_FILE=a-sd.conf
west build -s tests/Boards -b dji_c_f407 -d local/build/board-check-dji_c_f407
```

2026-09-08 两个工程均编译、链接通过：A 板 FLASH 33,292 B、RAM 6,720 B；C 板 FLASH 27,704 B、RAM 5,440 B。本机完整日志为 `local/build/board-check-*.log`。这只证明独立工程可编译，未烧录、未做实板或整车验证。

## M 板副板 SD 历史台架

`tests/SdBenchM` 保留 2026-09-05 的 HAL / FreeRTOS 独立测试源码、协议检查工具和实测资料；它不属于当前 Zephyr 正式构建入口。测试对象为一块 STM32H723 + V2 副板和一张卡，SPI3 使用 PC10/PC11/PC12、片选 PE14，未使用 DMA。

10 档全部完成，累计写入并读回校验 152 MiB。48 MHz 下两轮 32 MiB 写入约 3.49～3.53 MB/s，纯读取约 1.74 MB/s；第三轮增加软件 CRC16 后读取为 0.738 MB/s，不能直接与前两轮比较。连续多块读取将同为 24 MHz 的纯读取从 0.946 MB/s 提升到 1.698 MB/s，FIFO 档约 1.744 MB/s。48 MHz 继续提高后读取几乎不变，后续应量化 HAL 轮询开销并验证 DMA 或批量接收，不应直接归因于 PCB。

历史固件、清单与板上镜像确认匹配后，以下命令只读取 RAM，不复位或烧写：

```powershell
python -X utf8 tests/SdBenchM/ReadResult.py --manifest local/cache/sd-bench-m/firmware.json --probe 实际调试器ID --watch 600 --output local/cache/sd-bench-m/measurement
```

`phase=4` 为完成，`phase=5` 为失败；每档 `result=0` 为成功，`result=1` 为高速档跳过。总体完成不保证 48 MHz 一定执行。`ProtocolTest.c` 只检查 CMD17、CMD18/CMD12、CMD6 与错误清理；`HostChecks.py` 依赖历史清单；`ReadResult.py` 只读。若复现旧独立固件，应在单独目录检出 `951857f`，先备份当前完整 1 MiB Flash 并记录哈希，执行机构断电，结束后恢复本轮备份并读回核对。

该数据只是当时 HAL 固件、该板和该卡的结果，不能当成现有 Zephyr 吞吐率，也未覆盖所有卡、长时间运行、反复掉电、DMA 或整车日志负载。原始统计、Flash 备份与恢复核对材料保留在 `local/cache/sd-bench-m/measurements/20260905-192435/`。

## Zephyr HERO-M 的 SD 与音乐

正式 HERO-M 使用 SPI3 PC10/PC11/PC12、PE14 片选。M 板请求 48 MHz；只有 CMD6 查询和高速切换成功后才启用，否则保持 24 MHz。初始化请求 400 kHz，96 MHz 内核时钟下实际分频约 375 kHz。完整 HERO-M 默认含音乐服务；音乐专用配置只用于独立检查，和正式整车镜像分目录保存。

音乐扫描根目录和最多 4 层普通子目录，最多 64 首；支持无文件头、12 kHz、无符号 8 位单声道 `.u8`，以及 RIFF/WAVE PCM 的 8/16 位、单/双声道 WAV（最高 48 kHz）。服务只读歌曲，不转换、覆盖或删除卡内文件。PD14 单击上一首、PD15 单击下一首，任一按钮双击播放/停止；均为低电平按下，20 ms 消抖、350 ms 双击窗口。M 板用 TIM5 定时采样、TIM12_CH2/PB15 输出 PWM、8 KiB 环形缓冲；PB15 必须为 AF2。

音乐专用构建命令：

```powershell
$env:ZEPHYR_BASE = 'D:/ARBATOS/local/cache/zephyrproject/zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = 'D:/ARBATOS/local/cache/zephyr-sdk'
$env:PATH = 'C:/Program Files/CMake/bin;D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts;' + $env:PATH
$env:CMAKE_BUILD_PARALLEL_LEVEL = '2'
$env:ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr'
& 'D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts/west.exe' build -s projects -d local/build/hero-m-music -b dm_mc02_h7 -- '-DZephyr-sdk_DIR=D:/ARBATOS/local/cache/zephyr-sdk/cmake' '-DEXTRA_CONF_FILE=D:/ARBATOS/projects/HERO-M/prj.conf;D:/ARBATOS/projects/HERO-M/music.conf' '-DEXTRA_DTC_OVERLAY_FILE=D:/ARBATOS/projects/HERO-M/music.overlay' '-DCMAKE_MAKE_PROGRAM=D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts/ninja.exe'
```

正式整车构建使用 `tools/build.ps1 -Project HERO-M`，输出为 `local/build/hero-m`；正式 `flash`/`debug` 会拒绝音乐、准备和只接收配置。2026-09-06 已确认当时 M 板接线的 HERO-M 整车运动正常，且实际听到 `.u8` 播放。后续报警/按钮调整版的实体单击、双击行为仍待补验收；WAV 只完成编译与代码检查，尚未用真实 WAV 文件验收。上述结果不代表当前提交、其他接线、其他外设或长期负载已经验证。

调试工具 `BoardProbe.py` 面向 Horco CMSIS-DAP `486655686570` 与 STM32H723：`backup` 独立读两次完整 1 MiB Flash 并比较，`verify` 比较 Flash 与 `.bin`，`snapshot` 只读 RAM，`reset` 启动固件；`next`/`previous` 只写音乐服务的软件请求，不能代替实体按键测试。UART8 测试使用 115200、8N1、PE1/TX 接调试器 RX、PE0/RX 接调试器 TX 并共地；当时为 COM6，复现时以设备管理器实际端口为准。
