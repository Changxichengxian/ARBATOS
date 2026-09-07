# CLion 开发指南

当前整车目标为 HERO-M、SENTINEL-M、MINIWHEELEG-M，主板均为 DM MC02 H7 / STM32H723。A、C、M 三种开发板支持都保留在 boards，与车型分开管理。后续开发提交到 main。旧 Keil 工程、GCC 转换器与 A/C 板车型已从主线移除，历史保存在提交 951857f 和 zephyr 分支中。

## 还要安装什么

这台电脑已有 CLion 2026.2.2、Zephyr 4.4、Zephyr SDK、Python/West、CMake、Ninja、OpenOCD 和 ARM GDB。当前流程不需要 STM32CubeCLT、CubeMX、CubeIDE 或 Keil。CubeCLT 是 ST 提供的另一套编译/烧录工具包；这里直接使用现有 Zephyr SDK。

先在 PowerShell 中检查一次工具路径：

```powershell
cd D:\ARBATOS
pwsh -NoProfile -File .\tools\build.ps1 -Action probe
```

还需要接好现有 CMSIS-DAP 调试器：USB 接电脑，SWDIO、SWCLK、GND、目标电压参考与 M 板匹配，M 板正常供电。Windows 能识别 CMSIS-DAP 即可，不必因为使用 CLion 再装一遍驱动；只有 OpenOCD 报找不到调试器时才检查 USB 线和实际驱动。

本机工具目录是 `local/cache/` 下的本地环境，未提交 Git。以后换电脑需按 [Zephyr 官方安装说明](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)准备 Zephyr 4.4、SDK 和 Python 依赖，再创建本机 CMakeUserPresets.json；只克隆此仓库不会自动带上 SDK。

## 在 CLion 里编译

1. 打开 `D:\ARBATOS\projects`，使用现有 CMake 工程入口。
2. 设置 → 构建、执行、部署 → CMake：启用 `hero-m-local`，停用自动生成的 `Debug`。如果刚改了预设，执行重新加载 CMake 项目。
3. 在构建目标中选择 `zephyr_final` 或全部目标，点击“构建项目”。不要用 Run/Debug 按钮代替编译按钮。
4. 成功后产物在 `D:\ARBATOS\local\build\hero-m\zephyr\`：`zephyr.elf` 带调试符号，`.hex`/`.bin` 是烧录镜像。

个人预设已设置本机 SDK、Python、Ninja 路径。CLion 的初始普通 `Debug` 配置找不到 Zephyr，并不表示这些工具尚未安装。工程编译器由 Zephyr SDK 选择，不需要改用 CubeCLT 的编译器。

另外两个车型分别使用 `sentinel-m-local`、`miniwheeleg-m-local`，输出目录分别为 `local/build/sentinel-m`、`local/build/miniwheeleg-m`。一次只启用当前需要的配置。

默认构建并行数为 2。若 CLion 自己的 Build options 中指定了更大的 `-j`，改为 `-j2`，避免界面设置覆盖限制。正常修改只用增量构建，不需要每次清缓存或重编所有车型。

## 最短的编译、下载流程

也可以直接在 CLion 下方的 PowerShell 终端运行同一份脚本：

```powershell
cd D:\ARBATOS
# 检查三个目标的工程文件，不编译
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
# 编译当前英雄车，默认并行数为 2
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project HERO-M
# 接好 M 板与 CMSIS-DAP 后，下载刚编好的固件并校验
pwsh -NoProfile -File .\tools\build.ps1 -Action flash -Project HERO-M
```

`flash` 就是把固件写进主控。此操作会暂停/复位主控并启动新程序，先断开执行机构动力或按台架条件准备。它不自动重新编译，所以改代码后先执行 build，再执行 flash。

脚本固定使用 OpenOCD 和当前 M 板配置：CMSIS-DAP、SWD、500 kHz。只允许单个车型，拒绝 `flash -Project all`。按镜像所在扇区更新，不执行全片擦除；HERO-M 从 `0x080C0000` 起的校准区须保留。

换车型时把 `HERO-M` 改为 `SENTINEL-M` 或 `MINIWHEELEG-M`，其余步骤相同。`-BuildRoot` 可指定另外的输出根目录；编译和下载必须使用同一个根目录。

## 在 CLion 图形界面下载和调试

这是一次性配置，后续直接选它运行：

1. 设置 → 构建、执行、部署 → 嵌入式开发：把 OpenOCD 可执行文件指向下面表中的 `openocd.exe`。
2. 在所选工具链的 Debugger 项中选择自定义 GDB，填写下面的 ARM GDB 路径。
3. 运行 → 编辑配置 → 添加 **OpenOCD Download & Run**。
4. 名称可填 `HERO-M / CMSIS-DAP`，构建目标选 `zephyr_final`，对应 `hero-m-local`；ELF 使用下面列出的英雄车文件，Board config 使用仓库的 `openocd.cfg`。
5. 编译通过、接板并准备好之后，点 Run 下载运行，或点 Debug 下载并进入调试。可以在源码左侧加断点，用 Continue、Step Over、Step Into 查看变量和执行流程。

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
  HERO-M/prj.conf       英雄构建配置；测试模式也在此目录
  SENTINEL-M/           哨兵构建配置与副板配置
  MINIWHEELEG-M/        小轮腿构建配置
Robotconfig/            车型参数、控制配置、安装方向
local/build/            所有新编译产物，不提交 Git
```

新增车型时，按[车型配置](../Robotconfig/README.md#新增车型)补齐目标选择、源码清单、预设和工具车型表。A/C 独立编译检查见[测试说明](../tests/README.md)。板级支持与当前外设缺项见[开发板说明](../boards/README.md)。
