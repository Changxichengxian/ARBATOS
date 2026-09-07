# tools

`tools/` 放本地辅助脚本。正式入口是 Zephyr 4.4 的检查、构建、下载和调试；它不替代实车调试。

## 正式构建和检查

默认构建 `HERO-M`；默认输出目录是 `local/build/<target>/`：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
```

构建一个目标、全量构建或从干净目录重建：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project SENTINEL-M
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project all -Pristine
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action flash -Project HERO-M
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action debug -Project HERO-M
```

`-Action check` 调用 `CheckZephyr.py`，检查 Zephyr 的 CMake 源码清单、正式目标、板级配置和 overlay 引用；它不读取 `.uvprojx`，不要求安装 Keil，也不代替编译或实车验证：

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check -Project all
```

构建可指定 `-BuildRoot <目录>`、`-West <west 路径>`、`-Ninja <ninja 路径>` 和 `-Jobs <并行数>`；默认并行数为 2。`flash` 和 `debug` 会连接、复位或写入硬件。CLion 预设和 OpenOCD 使用方式见 `../manual/clion-zephyr.md`；界面构建和硬件调试仍需分别验收。

已移除的旧工具、工程检查和构建脚本如需恢复，使用 `git show 951857f:<path>` 或查看 `zephyr` 分支的 `6bdf19e`。

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

Zig 目前只是这些本地主机测试的可选依赖；未安装时相应脚本会明确报错。

## SD 日志工具

- `tools/sdlog/SdLogViewer.py`：打开 SD 日志网页查看器，支持导出 tag、字段和未知记录 CSV。
- `tools/sdlog/SdLogDecompress.py`：去掉当前格式日志里的 LZ4 块压缩，输出仍然是当前格式。

使用流程见 `../manual/sdlog.md`。
