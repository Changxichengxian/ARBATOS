# tools

`tools/` 放本地辅助脚本。它不替代 Keil，也不替代实车调试，主要用来在改共享代码后先做一轮低成本检查；现在也提供 GCC/CMake 命令行构建入口。

## 构建信息

Keil 工程的 `BeforeMake` 会调用：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ..\..\..\tools\GenBuildInfo.ps1
```

这个脚本生成 `shared/generated/build_info_autogen.h`，把 Git 提交、dirty 状态和编译时间带进固件。生成文件被 `.gitignore` 忽略，不需要手工提交。

## 一键检查

在仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\CheckAll.ps1
```

当前会检查：

- 所有 `projects/*/MDK-ARM/*.uvprojx` 能被解析。
- Keil 工程里列出的源码文件是否真实存在。
- Include Path 里本地目录是否存在；Keil Pack 提供的目录如果仓库里没有，会先作为警告。
- 每个工程是否只引用自己的 `Robotconfig/<TARGET>`。
- `Robotconfig/<TARGET>/RobotConfig.c` 里的 profile 是否和工程包含的任务源码、任务创建入口匹配。
- `tools/**/*.py` 是否有 Python 语法错误。
- 文档里是否还残留几类已经确认过时的路径或 MIT 轮腿描述。

它不会真的调用 Keil Rebuild，也不会默认跑完整 GCC 编译。要做真实编译验证，走下面的 `tools/build.ps1 -Action gcc-build`。

GitHub 上的 `.github/workflows/check-all.yml` 也会跑同一个脚本。也就是说，PR 或推送到主分支时，至少会自动检查工程引用、profile 和文档旧路径这些低级断裂。

如果想连未跟踪的本地文档也扫一遍，可以加 `-AllText`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\CheckAll.ps1 -AllText
```

## 输入与裁判协议回归

改 DBUS 内容校验或裁判系统拆帧后，可在仓库根目录运行轻量主机测试：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestInputReferee.ps1
```

它会用 Zig 自带的 C 编译器直接编译生产用的 DBUS 解码、裁判 CRC/拆帧和数据更新代码，覆盖坏拨杆、越界通道、
CRC8/CRC16 错误、分段收包、未知新命令和 1 字节 payload。

统一手动输入快照的来源选择、同代映射、双 bank 发布、读端超时安全化和 tick 回绕回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestManualInputSnapshot.ps1
```

该测试直接编译生产用 `ManualInput.c` 与 `ControlInput.c`，还会在候选计算和 Watch 副作用中注入新来源，验证旧候选不会覆盖新帧、发布副作用不会逆序，读取本身也不会重新计算或发布。

ELRS 严格链路证据、CRSF 重同步、映射通道校验、DMA/逐字节中断批裁决和 stop/restart 会话隔离回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestElrsInput.ps1
```

该测试直接编译生产用 `ElrsTask.c`、`ManualInput.c` 与 `ControlInput.c`。产品策略要求先收到新鲜的 0x14 Link Statistics，再接收一帧新的 0x16；只发送 0x16 的泛 CRSF 设备不会成为控制来源。

StateStore 双缓冲与快照时间信息的主机回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestStateStore.ps1
```

LowCmd 批量清除、持续局部禁写、优先级和急停锁边界的主机回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestLowCmd.ps1
```

故障范围、稳定恢复和电机反馈新鲜度的主机回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestFaultMgr.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestMotorHealth.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestArmFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestShootFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestMotorAxisFaultPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestUnitreeMotorPolicy.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestChassisSnapshotPolicy.ps1
```

控制管理器的资源预留、更新重入、保护停机、诊断和调试结构布局回归，以及底盘和 Shoot 控制域生命周期回归：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestControlMgr.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestChassisCtrl.ps1
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\TestShootCtrl.ps1
```

Zig 目前只是这些本地主机测试的可选依赖，常规
`CheckAll.ps1` 和 GitHub 检查不会静默增加这个要求；未安装时脚本会明确报错。

## 构建入口

统一入口是：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action probe
```

KEIL 路线仍然直接打开 `projects/<TARGET>/MDK-ARM/<TARGET>.uvprojx`。这条路线用于本地调试、下载和继续使用 uVision 工程。

GCC/CMake 路线从同一个 `.uvprojx` 生成，不单独手写一套工程清单：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project all
```

生成内容放在 `build/gcc/<TARGET>/`，包括 CMake 工程、GNU 启动文件和链接脚本。这个目录被 Git 忽略，源码、头文件路径、宏、启动文件和 scatter 文件变化后重新生成即可，不要手改生成出来的 CMake 文件。

## SD 日志工具

- `tools/sdlog/SdLogViewer.py`：打开 SD 日志网页查看器，支持导出 tag、字段和未知记录 CSV。
- `tools/sdlog/SdLogDecompress.py`：去掉当前格式日志里的 LZ4 块压缩，输出仍然是当前格式。

使用流程见 `../manual/sdlog.md`。
