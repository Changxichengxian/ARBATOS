# 工具说明

`tools/` 放本地辅助脚本。正式入口是 Zephyr 4.4 的检查、构建、下载和调试；它不替代实车调试。

## 正式构建和检查

默认构建 `HERO-M`，产物在 `local/build/<target>/`：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project SENTINEL-M
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project all -Pristine
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check -Project all
```

默认并行数为 2，可用 `-BuildRoot`、`-West`、`-Ninja` 指定本机路径，用 `-Jobs` 指定并行数。`-Action check` 调用 `CheckZephyr.py`，检查 Zephyr 的 CMake 源码清单、正式目标、板级配置和 overlay 引用；它不读取 `.uvprojx`，不需要 Keil，也不代替编译或实车验证。`flash` 与 `debug` 会连接、复位或写入硬件；CLion 预设和 OpenOCD 的用法见[CLion 开发指南](../manual/CLion开发指南.md)。

已移除的旧工具、工程检查和构建脚本可用 `git show 951857f:<path>` 或 `zephyr` 分支的 `6bdf19e` 查阅。

## 主机回归测试

以下脚本直接编译相应生产代码做主机回归；它们不连接车辆，也不能替代编译和实车验证。

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestInputReferee.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestManualInputSnapshot.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestElrsInput.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestStateStore.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestLowCmd.ps1
```

上面覆盖 DBUS、裁判系统 CRC 与拆帧、手动输入快照、ELRS/CRSF、StateStore 双缓冲，以及 LowCmd 安全输出边界。故障管理、电机反馈、控制域生命周期与安全输出边界使用：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestFaultMgr.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestMotorHealth.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestArmFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestShootFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestMotorAxisFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestUnitreeMotorPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestChassisSnapshotPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestControlMgr.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestChassisCtrl.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestShootCtrl.ps1
```

这些脚本要求 Zig 提供 C 编译器；未安装时会明确报错。

## SD 日志

- `tools/sdlog/SdLogViewer.py`：打开 SD 日志网页查看器，可导出 tag、字段和未知记录 CSV。
- `tools/sdlog/SdLogDecompress.py`：去掉当前格式日志的 LZ4 块压缩，输出仍为当前格式。

使用流程见[调试与日志](../manual/调试与日志.md#日志与复盘)。

## MP3 转蜂鸣器 U8

把 `.mp3` 放入 `tools/Mp3ToU8/`，双击 `ConvertMp3ToU8.cmd`，或把一个或多个文件拖到它上面。结果写入该目录的 `U8/`，为无文件头、12 kHz、单声道、无符号 8 位 PCM；现有蜂鸣器配置按这个格式播放。

在 `tools/Mp3ToU8/` 目录运行等效转换命令：

```powershell
.\ffmpeg.exe -y -i "input.mp3" -vn -ac 1 -ar 12000 -af "acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95" -c:a pcm_u8 -f u8 ".\U8\input.U8"
```

该目录的 `ffmpeg.exe` 只服务此转换流程。`.mp3` 输入不纳入 Git，`U8/` 的输出保留。

## PID 单环自动整定

`PidAutotune/ArbatosPidAutotune.py` 通过既有 `UART1 tune` 文本协议观察并尝试整定单个环。`ps`/`pa` 为俯仰速度环/角度环，`ys`/`ya` 为水平云台速度环/角度环，`cf` 为底盘跟随环，`cm` 为底盘电机速度环。固件命令为 `at <目标>`、`at period 20`、`at off`，输出为 8 通道 JustFloat：`timestamp_ms, setpoint, input, output, error, kp, ki, kd`，尾部为 `INF`。

```powershell
python tools\PidAutotune\ArbatosPidAutotune.py --port COM5 --target ps --window 120 --rounds 8 --mode heuristic
```

它默认切入单任务运行编排；加 `--operation none` 才保持现有运行编排。先调内环再调外环，每轮使用相近动作；激励不足时工具会跳过这一轮。依赖：`pip install pyserial`。
