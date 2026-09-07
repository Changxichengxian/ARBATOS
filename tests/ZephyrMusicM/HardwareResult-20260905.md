# 2026-09-05 Zephyr HERO-M 音乐模式实板记录

后续更正：2026-09-06 用户反馈无声，查出 PB15 错配 AF9，H723 应使用 AF2。本文采样计数只证明软件运行，不能证明蜂鸣器出声；修正后用户才确认听到歌曲。以下旧版 BIN/HEX/ELF 已保存在 `local/cache/zephyr-music-hardware/20260906/before-af2-fix/`，常用构建目录已更新为修正版。

## 已确认结果

硬件为 STM32H723 M 板和第二版副板，Horco CMSIS-DAP `486655686570`。实板运行正式 Zephyr 存储、音频和歌曲服务，附加 `CONFIG_ARBATOS_MUSIC_ONLY=y`；控制任务没有启动。

- 卡根目录发现 `hajimi.U8` 和 `YOU.U8` 两首可播放歌曲；14 个目录条目，2 个普通文件扩展不受支持。原始文件未改动。
- 20:20:42 快照：正在播放 `hajimi.U8`；请求和实际采样率均 12000 Hz；累计输出 62971 个采样；音频欠载 0；SD 调用 16154 次，端口错误 0。
- 20:22:30 快照：仍在播放 `hajimi.U8`；累计输出 1367820 个采样；音频欠载 0；SD 调用 299566 次，端口错误 0；歌曲服务错误 0。
- CMD6 高速切换结果为 0，SD 端口为 48000000 Hz。PLL1 Q 配为 96 MHz，SPI 分频 2；CPU 保持 480 MHz。初始化实际为 375 kHz；默认速度兼容档为 24 MHz。
- 实板是固件输出验证，未录音或独立评价听感。PD14/PD15 在上述快照中均释放，实体按键计数均为 0；已请用户实际按键确认。

本次没有重新测正式 Zephyr 的大块写入吞吐，不能把此前独立 Keil 测试的约 3.5 MB/s 写入数字直接当作本次 Zephyr 成绩。

## 修复与中间故障

1. 首次运行在 `log_process_thread_func()` 的 `log_backend_count_get() > 0` 断言停止。此前配置开启日志线程，却没有日志后端。新增 `ArbLogRam.c` 后端后正常启动，并保留可由 DAP 读取的日志。
2. 原通用 SPI 路径在 48 MHz 首次单字节接收等待中停止。对比已通过的 Keil 配置，发现协议层手动管理 CS 后，Zephyr 仍按硬件 NSS 配置；同时 PC10/PC12 输出速度为默认低速。
3. 最终明确使用软件 NSS，将 PC10/PC12 配为 `high-speed`，恢复普通轮询收发后正常读取和播放。中间尝试的 FIFO/中断路径发生超时，已撤回；不能声称该路径已验证。
4. 调试过程中发现芯片睡眠会影响 DAP 访问，曾通过 APB-AP 的 DBGMCU 恢复调试时钟；音乐验证配置启用 `CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP`。
5. 20:24 左右尝试通过 DAP 发软件切歌请求时出现 WAIT/FAULT 响应，随后无法稳定连接目标。切歌请求是否送达未确认，不能计为软件切歌或实体按键通过。保留已烧录的音乐固件，未再覆盖 Flash。
6. 最后恢复尝试仍可经 APB-AP 读取芯片身份 `0x10016483`，调试控制值为 `0x70003f`，重新写入后主核访问仍报 AP#0 FAULT。因此只能确认 DAP 与调试外设仍可通信，不能将本次断连归因为睡眠配置，也未确认最后的播放状态。

## 产物与验证

当前已烧录音乐模式：`local/cache/zephyr-hero-m-music/zephyr/zephyr.hex`。

- BIN 长度 306600 字节，实际 Flash 逐字节一致。
- BIN SHA256：`dd3c1b0bf4d1fb81664b34957eb6dad186f0a8d892b0361caf24d215222e1ebe`。
- HEX SHA256：`98d3ab5b732ee3251b9932bd89912bf89bb1ae5adc36688c2bf36a024e7d6ef1`。
- ELF SHA256：`c4feb400b6be24617b4444d4f68d91f13d4f74a74bfd1fc8fbf39dd4b5d401fb`。

完整 HERO-M：`local/cache/zephyr-hero-m-formal/zephyr/zephyr.hex`，FLASH 519680 B、主 RAM 211396 B / 320 KiB、DTCM 56024 B / 128 KiB。完整配置编译通过，未启动其控制任务做运动验证。

跨板构建：HERO-C 主 RAM 123716 B / 128 KiB（94.39%）；INFANTRY-A 主 RAM 111940 B / 192 KiB（56.94%）。两者均无源码编译告警，SD 频率未改成 48 MHz。SD 协议的现有 C 主机测试通过。

测试前完整 Flash 独立备份两次一致：`local/cache/zephyr-music-hardware/20260905/flash-before.bin`，SHA256 `54a6f96a295b14352ad9559bd3b3310230f54032672acbef4faa0d1518f2621b`。

主要原始记录在 `local/cache/zephyr-music-hardware/20260905/`：`pins/verify.json`、`sample9/snapshot.json`、`sample10/snapshot.json`。当前保留音乐模式；原固件备份可供需要时恢复。
