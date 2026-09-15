# CLion 开发指南

当前车型包括 M 板的 HERO-M、SENTINEL-M、MINIWHEELEG-M，以及 DJI A F427 的 INFANTRY-TIAN-A。A 板车型当前只开放基础控制链，默认未放行电机且未上板。A、C、M 三种开发板支持都保留在 boards，与车型分开管理。后续开发提交到 main。旧 Keil 工程、GCC 转换器与旧 A/C 板车型已从主线移除，历史保存在提交 951857f 和 zephyr 分支中。

## 根目录快速入口

- 双击 `打开工程.cmd`：自动查找 PATH 或 JetBrains 安装记录中的 CLion，打开 `projects`；保留 IDE 当前配置，本机目前启用 `hero-m-local`。
- 双击 `编译固件.cmd`：按菜单选择车型，回车默认 HERO-M；结束后窗口保留编译结果。也可在终端运行 `.\编译固件.cmd -Project INFANTRY-TIAN-A`。

三个车型共享一个 CMake 工程，不必为每台车复制一份 CLion 工程。单独编译某车型不会修改 IDE 当前选择；在 CLion 中切换时使用下面对应的 `*-local` 配置。入口按 [CLion 官方命令行用法](https://www.jetbrains.com/help/clion/opening-files-from-command-line.html)传入项目目录，不改 `.idea` 或本机预设。

入口需要 PowerShell 7（`pwsh`）。若 CLion 使用便携安装且未登记位置，可把 `clion64.exe` 所在目录加入 PATH，或通过 `CLION_EXE` 环境变量指定完整文件路径。仅核对入口、不打开 IDE 或编译时，运行 `.\打开工程.cmd -Check` 或 `.\编译固件.cmd -Check -Project HERO-M`。

## 还要安装什么

这台电脑已有 CLion 2026.2.2、Zephyr 4.4、Zephyr SDK、Python/West、CMake、Ninja、OpenOCD 和 ARM GDB。当前流程不需要 STM32CubeCLT、CubeMX、CubeIDE 或 Keil。CubeCLT 是 ST 提供的另一套编译/烧录工具包；这里直接使用现有 Zephyr SDK。

先在 PowerShell 中检查一次工具路径：

```powershell
cd D:\ARBATOS
pwsh -NoProfile -File .\tools\build.ps1 -Action probe
```

需要下载或调试时，再接好现有 CMSIS-DAP 调试器：USB 接电脑，SWDIO、SWCLK、GND、目标电压参考与 M 板匹配，M 板正常供电。打开工程和编译不需要接板。Windows 能识别 CMSIS-DAP 即可，不必因为使用 CLion 再装一遍驱动；只有 OpenOCD 报找不到调试器时才检查 USB 线和实际驱动。

本机工具目录是 `local/cache/` 下的本地环境，未提交 Git。以后换电脑需按 [Zephyr 官方安装说明](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)准备 Zephyr 4.4、SDK 和 Python 依赖，再创建本机 CMakeUserPresets.json；只克隆此仓库不会自动带上 SDK。

## 在 CLion 里编译

1. 打开 `D:\ARBATOS\projects`，使用现有 CMake 工程入口。
2. 设置 → 构建、执行、部署 → CMake：启用 `hero-m-local`，停用自动生成的 `Debug`。如果刚改了预设，执行重新加载 CMake 项目。
3. 在构建目标中选择 `zephyr_final` 或全部目标，点击“构建项目”。不要用 Run/Debug 按钮代替编译按钮。
4. 成功后产物在 `D:\ARBATOS\local\build\hero-m\zephyr\`：`zephyr.elf` 带调试符号，`.hex`/`.bin` 是烧录镜像。

个人预设已设置本机 SDK、Python、Ninja 路径。CLion 的初始普通 `Debug` 配置找不到 Zephyr，并不表示这些工具尚未安装。工程编译器由 Zephyr SDK 选择，不需要改用 CubeCLT 的编译器。

另外两个车型分别使用 `sentinel-m-local`、`miniwheeleg-m-local`，输出目录分别为 `local/build/sentinel-m`、`local/build/miniwheeleg-m`。新增车型后先运行 `python .\tools\config\RobotConfigGen.py presets --target all`；它只补齐缺失的 `CMakeUserPresets.json` 项，不覆盖已有个人预设。一次只启用当前需要的配置。

默认构建并行数为 2。若 CLion 自己的 Build options 中指定了更大的 `-j`，改为 `-j2`，避免界面设置覆盖限制。正常修改只用增量构建，不需要每次清缓存或重编所有车型。

## 最短的编译、下载流程

也可以直接在 CLion 下方的 PowerShell 终端运行同一份脚本：

```powershell
cd D:\ARBATOS
# 检查三个目标的工程文件，不编译
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
# 编译当前英雄车，默认并行数为 2
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project HERO-M
# 接好 M 板与 CMSIS-DAP 后，指定板上当前固件对应的旧 ELF，再下载刚编好的固件并校验
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M `
    -RunningElf D:\固件存档\上次烧录\zephyr.elf -ProbeSerial 你的CMSIS-DAP序列号
```

`flash` 就是把固件写进主控。此操作会暂停/复位主控并启动新程序，先断开执行机构动力或按台架条件准备。它不自动重新编译，所以改代码后先执行 build，再执行 flash。

HERO-M 的正式入口会从 `-RunningElf` 取得停写邮箱地址和 256 位构建指纹。运行中只读取 4 字节邮箱标识和 32 字节构建指纹；两者都匹配后才请求日志任务写完内存记录、同步并关闭文件。收到成功确认后先暂停主控，再完整核对板上旧固件，核对通过后才烧录。旧固件没有构建指纹或停写邮箱时，先物理移除 SD 卡，再明确使用 `-SdRemoved`。脚本不会退化成只检查邮箱标识，也不会把“旧固件没有邮箱”推断成“没有插卡”。SENTINEL-M、MINIWHEELEG-M 尚未接入这套邮箱，也必须物理移除 SD 卡并传 `-SdRemoved` 才允许从正式入口烧录。

脚本固定使用 OpenOCD 和当前 M 板配置：CMSIS-DAP、SWD、500 kHz。只允许单个车型，拒绝 `flash -Project all`。HERO-M 的保护流程会保存烧录前后的校准区并完整回读程序区；结果默认放在带时间的 `local/flash/hero-m-*` 目录。HERO-M 从 `0x080C0000` 起的校准区须保持不变。`-RunningElf` 必须是板上当前正在运行固件的原始 ELF，且要包含与板上固件一致的构建指纹，不能拿准备烧入的新 ELF 代替。单独调用 `quiesce_running_firmware(...)` 只执行短身份核对和日志停写确认，主控继续运行，供取卡前检查；它不会在运行中批量读取 Flash。

换车型时把 `HERO-M` 改为所需车型名。`INFANTRY-TIAN-A` 的本机预设通过 `tools/build.ps1 -Action presets -Project INFANTRY-TIAN-A` 生成，名称为 `infantry-tian-a-local`；该 A 板目标本次只做软件验证。`-BuildRoot` 可指定另外的输出根目录；编译和下载必须使用同一个根目录。

## 在 CLion 图形界面下载和调试

这是一次性配置，后续直接选它运行：

1. 设置 → 构建、执行、部署 → 嵌入式开发：把 OpenOCD 可执行文件指向下面表中的 `openocd.exe`。
2. 在所选工具链的 Debugger 项中选择自定义 GDB，填写下面的 ARM GDB 路径。
3. 运行 → 编辑配置 → 添加 **OpenOCD Download & Run**。
4. 名称可填 `HERO-M / CMSIS-DAP`，构建目标选 `zephyr_final`，对应 `hero-m-local`；ELF 使用下面列出的英雄车文件，Board config 使用仓库的 `openocd.cfg`。
5. 编译通过、接板并准备好之后，点 Run 下载运行，或点 Debug 下载并进入调试。可以在源码左侧加断点，用 Continue、Step Over、Step Into 查看变量和执行流程。

CLion 自带的 **OpenOCD Download & Run** 目前不会请求 SD 日志任务停写。M 板只在 SD 卡已物理移除时使用这个下载入口；插卡运行时从终端调用上面的受保护 `flash` 流程。

| 设置 | 本机路径 |
| --- | --- |
| OpenOCD | `D:\ARBATOS\local\cache\zephyr-sdk\hosttools\openocd\bin\openocd.exe` |
| ARM GDB | `D:\ARBATOS\local\cache\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gdb.exe` |
| Board config | `D:\ARBATOS\boards\DmMc02H7\zephyr\support\openocd.cfg` |
| HERO-M ELF | `D:\ARBATOS\local\build\hero-m\zephyr\zephyr.elf` |
| OpenOCD 脚本搜索目录 | `D:\ARBATOS\local\cache\zephyr-sdk\hosttools\openocd\share\openocd\scripts` |

如果 OpenOCD 报 `Can't find interface/cmsis-dap.cfg`，为 OpenOCD 设置 `OPENOCD_SCRIPTS` 环境变量指向表中的脚本搜索目录。若默认 GDB 端口 3333 被占用，先结束另一份正在使用同一调试器的会话。CLion、Keil、pyOCD 等不要同时占用同一调试器。

普通断点会暂停主控上的控制任务；电机带载运行时不要随意打断点。当前新下载配置还没有连接实物验收，无线调试链路的断点可靠性也未验证。

## 从终端调试

已经执行过 flash、板上固件与本次 ELF 一致时：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action debug -Project HERO-M
```

这个入口启动 OpenOCD 与 SDK 的 ARM GDB，不重复下载，但可能复位/暂停主控。进入 GDB 后可输入：

```text
break main
continue
next
print 变量名
quit
```

图形界面的 OpenOCD Download & Run 与终端 debug 二选一使用。前者提供 CLion 的断点和变量窗口，后者直接进入 GDB 命令界面。

## 当前迁移状态与验证范围

Keil 项目、CubeMX 工程副本、ARMCC 二进制库、旧转换工具和四个旧车型已移除。A、C、M 板的设备树、板级头文件和现用适配全部保留；共享算法在 `shared/zephyr/port/algorithm/`。工程统一在 `projects/`，新产物统一在 `local/build/`，根目录不再保留旧 `build`、`out`、`zephyr` 或 `Open*.cmd`。

目录整理后 HERO-M 正式构建与本机 CMake 预设构建通过，A/C 独立板级工程编译链接通过，工程检查的 5 项回归通过。这些是软件验证。HERO-M 先前有整车运动实测；目录整理后的固件未再次上车，CLion 图形界面下载/调试也仍待实物验证。

## 官方参考

- [JetBrains：Zephyr](https://www.jetbrains.com/help/clion/zephyr.html)
- [JetBrains：OpenOCD Download & Run](https://www.jetbrains.com/help/clion/openocd-support.html)
- [JetBrains：CMake 预设](https://www.jetbrains.com/help/clion/cmake-presets.html)
- [ST：STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)

## 工程目录

`projects/` 负责构建和启动，车型参数与板级支持分别保留。

```text
boards/
  DjiAF427/zephyr/       A 板设备树、引脚、调试配置
  DjiCF407/zephyr/       C 板设备树、引脚、调试配置
  DmMc02H7/zephyr/       M 板设备树、引脚、调试配置
  dts/                  共用设备树属性
shared/
  application/          共用机器人控制逻辑
  components/           算法、协议、设备
  zephyr/               共用系统适配、外设接口、兼容层
projects/
  CMakeLists.txt        统一工程入口
  CMakePresets.json     CLion 与命令行共用预设
  cmake/                显式源码清单
  src/                  main 与任务启动
Robotconfig/            每车型 RobotConfig.toml、参数、接线和安装方向
local/build/            所有新编译产物，不提交 Git
```

新增车型时，按[车型配置](../Robotconfig/README.md#新增车型)生成 `Robotconfig/<车型>` 并在其中完成声明。A/C 独立编译检查见[测试说明](../tests/README.md)。板级支持与当前外设缺项见[开发板说明](../boards/README.md)。
