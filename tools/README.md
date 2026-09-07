# 工具说明

`tools/` 放可复用工具源码。日常双击根目录的 `打开工程.cmd` 或 `编译固件.cmd` 即可；命令行继续用 `tools/build.ps1`。

## 这些工具是做什么的

| 位置 | 用途 | 是否需要 |
| --- | --- | --- |
| `build.ps1` | 统一检查、编译、下载和调试入口 | 日常开发保留 |
| `build/` | 启动入口、实际构建、工程检查、固件版本信息生成 | 构建配套，通常不用逐个打开 |
| `config/RobotConfigGen.py` | 从 `Robotconfig/<车型>/RobotConfig.toml` 生成构建、任务和 profile 输入 | 新增车型或检查声明时使用 |
| `tests/` | 电脑上的自动回归测试和所需测试代码 | 修改控制、输入、故障保护时使用，保留 |
| `sdlog/` | 查看、解压和导出 SD 日志 | 实车调试使用 |
| `Mp3ToU8/` | 把 MP3 转成蜂鸣器播放格式 | 可选，音乐功能使用 |
| `PidAutotune/` | 通过串口辅助整定 PID | 可选，需要对应固件和实车条件 |
| `sim/`、`mujoco/`、`WheelLegLqr/` | 配置压力估算、轮腿物理仿真、轮腿参数计算 | 可选，轮腿开发时使用 |

原来顶层的 25 个 `Test*.ps1` 都是测试入口，已集中到 `tests/`。它们通常会编译一小段真实控制代码并检查异常输入、失联、限幅等行为，不是固件运行时要加载的 25 个程序。生成的可执行文件、模型和缓存放 `local/build/` 或 `local/cache/`。仿真用法集中在[仿真说明](仿真说明.md)。

## 正式构建和检查

默认构建 `HERO-M`，产物在 `local/build/<target>/`：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project SENTINEL-M
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project all -Pristine
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check -Project all
```

默认并行数为 2，可用 `-BuildRoot`、`-West`、`-Ninja` 指定本机路径，用 `-Jobs` 指定并行数。`-Action check` 调用 `build/CheckZephyr.py`，检查 Zephyr 的 CMake 源码清单、正式目标、板级配置和 overlay 引用；它不读取 `.uvprojx`，不需要 Keil，也不代替编译或实车验证。`flash` 与 `debug` 会连接、复位或写入硬件；CLion 预设和 OpenOCD 的用法见[CLion 开发指南](../manual/CLion开发指南.md)。

`build/build-matrix.ps1` 是 `build.ps1` 调用的实际编译实现。CMake 自动调用 `build/GenBuildInfo.py`，为每个构建目录刷新固件版本和源码指纹；不变时不重写版本头。`build/GenBuildInfo.ps1` 仅保留手动兼容入口，使用条件见[日志说明](../manual/调试与日志.md#日志与复盘)。

车型列表来自 `Robotconfig/*/RobotConfig.toml`，脚本会自动发现，无需修改 PowerShell 映射表。可以单独查看或检查声明：

新增车型可运行 `pwsh -File tools/build.ps1 -Action new -Project NEW -From HERO-M`，再运行 `pwsh -File tools/build.ps1 -Action presets -Project all`，为 CLion 补齐本机配置。这两个入口自动使用项目的 Python 环境。

```powershell
python .\tools\config\RobotConfigGen.py list --json
python .\tools\config\RobotConfigGen.py check --target HERO-M
python .\tools\config\RobotConfigGen.py new --target NEW --from HERO-M
python .\tools\config\RobotConfigGen.py presets --target all
```

生成出的 `robot.conf`、`RobotTarget*.h/.inc/.cmake` 和 `robot-config.json` 都在构建目录，不能手工修改。`robot-config.json` 可用来核对选择了哪些服务、依赖、固定栈和优先级。新算法控制器放在 `shared/controllers/<name>/`，由其 `Controller.toml` 的参数默认值和范围约束；车型只在 TOML 选择控制器并按电机实例名装配。控制器不直接写总线，输出仍要经过 `LowCmd` 许可、急停和限幅。

已移除的旧工具、工程检查和构建脚本可用 `git show 951857f:<path>` 或 `zephyr` 分支的 `6bdf19e` 查阅。

## 主机回归测试

统一入口会依次运行测试；可以只列出清单或选择一个测试。这里的 `tools/tests/` 在电脑上验证代码；仓库根目录的 `tests/` 保存板级独立测试工程。主机测试不连接车辆，也不能替代固件编译和实车验证。

```powershell
pwsh -NoProfile -File .\tools\tests\RunTests.ps1 -List
pwsh -NoProfile -File .\tools\tests\RunTests.ps1
pwsh -NoProfile -File .\tools\tests\RunTests.ps1 -Name LowCmd
pwsh -NoProfile -File .\tools\tests\TestLowCmd.ps1
```

| 测试类别 | 检查内容 |
| --- | --- |
| 遥控、裁判、图传、服务输入 | 拆帧、校验、输入快照和数据失效；ELRS 项只验证串口 CRSF，不验证 SX1281 射频 |
| LowCmd、CAN 完成状态、电机输出许可 | 输出命令、发送结果、是否允许电机输出 |
| 故障管理、电机健康、机械臂/射击故障、复位记录 | 故障进入、保持、恢复与边界条件 |
| 控制域、机器人生命周期、状态存储、单写者规则 | 启动和切换顺序、状态一致性 |
| 底盘、云台反馈、轮腿输出、射击控制 | 各控制模块的计算、反馈和输出限制 |
| 宇树电机策略 | 宇树驱动相关的控制策略 |
| `TestCheckZephyr.py` | 工程检查器能否识别缺文件、错误车型和非法源码引用 |
| `TestBuildInfo.py`、`TestSdLogViewerPowerMeter.py` | 版本变化、子模块、无变化增量构建，以及功率计/模型/源码状态解码 |
| `PowerMeter`、`CanFaultPort`、`Rs485FaultPort`、`ResetEvidencePort`、`RobotFaultZephyr` | 功率计协议与队列，以及真实异常处理源码在模拟寄存器下的行为 |

C 回归脚本要求 Zig 提供编译器；统一入口也能找到 WinGet 安装但尚未加入当前 PATH 的 Zig。工程检查器回归使用 Python。缺少依赖或测试失败会返回失败，不跳过后冒充通过。测试源码和模拟接口也在 `tests/`，删除它们会让对应入口失效。

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
