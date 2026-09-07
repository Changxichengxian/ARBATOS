# Zephyr M 板 SD 与歌曲播放

使用正式 Zephyr HERO-M 的存储和蜂鸣器实现，附加音乐启动配置，首次上板不启动 CAN、底盘、云台、加热或舵机任务。完整 HERO-M 默认包含音乐服务。2026-09-06 用户已确认当前 M 板接线的 HERO 整车运动正常，包括俯仰，详见 [实车运行记录](NormalOutput-20260906.md)；音乐专用配置仍仅用于独立验证。

## 使用

- SD：SPI3 PC10/PC11/PC12，片选 PE14。仅 M 板请求 48 MHz；CMD6 高速查询和切换成功后才启用，不支持时保留 24 MHz。初始化请求 400 kHz，96 MHz 内核时钟实际分频为 375 kHz。
- 音乐：扫描根目录和最多 4 层普通子目录，最多 64 首，按目录遍历顺序播放，播完自动下一首；不进入隐藏、系统或点号开头的目录。
- 完整路径最多 383 字节，显示名最多 95 字节；长显示名按 UTF-8 边界缩短，打开歌曲仍使用完整路径。路径、容量、目录深度或文件格式超出支持范围时，UART8 会输出原因。
- PD14 单击上一首，PD15 单击下一首；任一按钮双击播放/停止。均低电平按下，20 ms 消抖，350 ms 双击窗口，首尾循环。
- 开机扫描歌曲后保持静音待机；停止时选歌不播放，双击从所选歌曲开头播放。播放中单击立即切换，正常播完自动下一首。
- `.u8`：无文件头、12 kHz、无符号 8 位单声道。
- `.wav`：RIFF/WAVE PCM，8/16 位、单/双声道，最高 48 kHz；双声道合并为单声道。未加入 MP3 解码。
- 音乐服务只读歌曲，不转换、覆盖或删除卡内文件。

M 板用 TIM5 定时采样，TIM12_CH2/PB15 输出 PWM，8 KiB 环形缓冲。静音中点为 128，音量围绕中点缩放；中断仅更新占空比。原来按系统节拍提交工作项的播放方式无法达到 12 kHz，已从 M 板实现中替换。

H723 的 PB15 必须使用 AF2。2026-09-06 纠正原 AF9 配置后，用户确认实际听到歌曲；此前只有软件计数，不能算作声音验收。音乐测试版通过 UART8 输出启动和歌曲日志：115200、8N1，PE1/TX 接调试器 RX，PE0/RX 接调试器 TX，共地；本机对应 COM6。

## 本机构建

在 `D:/ARBATOS` 的 PowerShell 中执行；SDK 缓存可能需要当前账户具备访问权限：

```powershell
$env:ZEPHYR_BASE = 'D:/ARBATOS/local/cache/zephyrproject/zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = 'D:/ARBATOS/local/cache/zephyr-sdk'
$env:PATH = 'D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts;' + $env:PATH
& 'D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts/west.exe' build -s projects -d local/cache/zephyr-hero-m-music -b dm_mc02_h7 -- '-DZephyr-sdk_DIR=D:/ARBATOS/local/cache/zephyr-sdk/cmake' '-DEXTRA_CONF_FILE=D:/ARBATOS/projects/HERO-M/prj.conf;D:/ARBATOS/projects/HERO-M/music.conf' '-DEXTRA_DTC_OVERLAY_FILE=D:/ARBATOS/projects/HERO-M/music.overlay' '-DCMAKE_MAKE_PROGRAM=D:/ARBATOS/local/cache/zephyrproject/.venv/Scripts/ninja.exe'
```

完整 HERO-M 使用 `hero-m.conf`，不附加 `hero-m-music.conf`，本次输出目录为 `local/cache/zephyr-hero-m-formal`。

## 调试与结果

`BoardProbe.py` 使用 Horco CMSIS-DAP `486655686570`、STM32H723。`backup` 独立读两次完整 1 MiB Flash 并比较；`verify` 比较实际 Flash 与 `.bin`；`snapshot` 只读 RAM，不暂停播放。`reset` 启动固件。`next`/`previous` 写音乐服务的软件请求，仅用于检查切歌流程，不代表实体按键测试。

调试符号：`SubBoardMusicDiagData`、`SubBoardMusicTracks`、`BuzzerPcmDiag`、`SdSpiPortDiag`、`SdSpiHighSpeedResult`、`ArbLogRam`。内存日志循环覆盖旧记录，无需连接串口或调试器才能继续运行。音乐模式保留芯片睡眠期间的调试访问。

`BoardProbe.py` 默认按当前 64 首、384 字节路径解析歌曲表；读取旧版 32 首、128 字节路径的 ELF 时需传 `--track-capacity 32 --track-path-bytes 128`。`PinAudit.py --output <json>` 对照本机 STM32H723VGT6 官方引脚定义检查 38 个复用映射，不代表对应外设已经完成实物通信测试。

实板记录见 [HardwareResult-20260905.md](HardwareResult-20260905.md)。`.u8` 已实板播放；WAV 目前完成编译和代码检查，尚未用实际 WAV 文件验收。
