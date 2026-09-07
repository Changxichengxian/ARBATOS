# M 板副板 SD 历史独立测试

这里保留 2026-09-05 的 HAL / FreeRTOS 独立台架源码、协议检查工具和实测结果。它不属于当前 Zephyr 正式构建入口。HERO-M 后续已完成 Zephyr SD、音乐和整车运动测试，见 [Zephyr 验证记录](../ZephyrMusicM/Validation.md)。

## 历史结果

M 板 STM32H723 + V2 副板，SPI3 PC10/PC11/PC12、PE14 片选。独立测试的 10 档全部通过，累计写入并读回校验 152 MiB；48 MHz 下两轮 32 MiB 写入约 3.49～3.53 MB/s，纯读取约 1.74 MB/s。

这是该板该卡在旧 HAL 固件中的测量，未使用 DMA。不能当成当前 Zephyr 的吞吐率。频率、数据块、计时和校验口径见 [原始实测报告](HardwareResult-20260905.md)。

## 保留的工具

- `ProtocolTest.c`：用模拟端口检查实际 SD 协议代码的 CMD17、CMD18/CMD12、CMD6 和错误清理，不模拟 STM32 外设、卡内部时延或真实断电。
- `HostChecks.py`：使用 VS2022 Build Tools 运行协议和结果解码检查；还依赖历史 `local/cache/sd-bench-m/firmware.json`。没有该清单时不能仅凭克隆仓库直接运行完整检查。
- `ReadResult.py`：根据历史清单读取测试固件的 RAM，输出原始数据、JSON 和 CSV；只附加读取，不复位或烧写。

历史固件已运行且清单与板上镜像匹配时，才使用：

```powershell
python -X utf8 tests/SdBenchM/ReadResult.py --manifest local/cache/sd-bench-m/firmware.json --probe 实际调试器ID --watch 600 --output local/cache/sd-bench-m/measurement
```

`phase=4` 表示完成，`phase=5` 表示失败；每档 `result=0` 才是成功，`result=1` 表示高速档跳过。总体完成不保证 48 MHz 一定执行。

## 如需复现独立固件

旧 Keil 构建器 `Build.py` 和工程模板已从主线移除，可在单独目录检出 `951857f` 重现。测试固件启动后会创建测试文件、写入并读回，复位会重新开始一轮；不要用它替换现车固件作为日常入口。

临时上板前备份当前完整 1 MiB Flash 并记录哈希，执行机构断电；结束后恢复本轮备份并读回核对。不要拿上一轮旧备份覆盖当前程序。

协议参考：[SD Association 规范](https://www.sdcard.org/downloads/pls/)，Physical Layer Simplified Specification 第 4.3.10 节和第 7 章。
