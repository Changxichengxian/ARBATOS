# CLion、Zephyr 与 OpenOCD 工作流

## 当前主线

从 2026-09-07 起，开发和后续提交统一在 `main`。`zephyr` 分支停留在 `6bdf19e`，保留探索与实车适配历史；HERO-M 的成果已快进合入 `main`。当前实车是 M 板与第二版副板，旧 HERO-C 接线不适用。

正式构建直接读取 `zephyr/CMakeLists.txt`、`zephyr/cmake/ArbatosLegacy.cmake` 和 `zephyr/targets/`，不解析 `.uvprojx`，不要求安装 Keil、ARMCC 或 Keil 设备包。文件名中的 Legacy 表示复用原有机器人业务代码，不表示构建仍依赖 Keil。

本次先解除正常开发入口对 Keil 的依赖。旧 `projects/`、板级参考代码、Keil 工程和离线诊断工具仍留在 Git 中；正式工作流不使用它们的工程清单。清理历史文件需另行核对仍被 Zephyr 引用的头文件和共享代码。

## 几种工具分别负责什么

| 层次 | 本项目现在使用什么 | 作用 |
| --- | --- | --- |
| 编辑和工程界面 | CLion | 写代码、补全、管理构建配置、查看调试结果；与 Keil uVision 属于同一层 |
| 固件平台 | Zephyr 4.4 | 在主控上提供线程、驱动和运行环境 |
| 构建入口 | West、CMake、Ninja | 选择车型与源码，组织编译；CLion 可以调用它们 |
| 编译和链接 | Zephyr SDK 中的 GNU 工具链 | 把 C/C++ 源码变成 ELF、HEX、BIN 固件 |
| 烧录和调试连接 | OpenOCD + GDB | OpenOCD 接调试器和芯片；GDB 处理断点、单步、变量等调试命令 |
| 调试硬件 | 当前 CMSIS-DAP | 通过 SWD 连接电脑与 M 板 |

CLion 与 Keil 是可选的工程界面；OpenOCD 可以被 CLion 或命令行共同调用。CLI 只是命令行界面的缩写，不是另一种 IDE。

## 工程已经准备了什么

- `zephyr/CMakePresets.json`：七个正式车型的共享 CMake 配置，不包含个人安装路径。
- `zephyr/CMakeUserPresets.json`：本机生成、Git 忽略的 `hero-m-local` 配置，已填入现有 SDK、Python 和 Ninja 路径。
- `tools/build.ps1`：CLion 外部工具或终端均可调用的构建、检查和工具探测入口。
- `zephyr/boards/dm_mc02_h7/support/openocd.cfg`：M 板 CMSIS-DAP、SWD、500 kHz 配置，不内置烧录、擦除或启动命令；M 板默认 runner 已选择 OpenOCD。
- 编译输出在 `out/zephyr/<target>/`，与旧实测证据及旧构建目录分开。CLion 的 `.idea`、个人预设和输出目录均不提交。

本机已安装 CLion 2026.2.2，并由用户完成非商业激活。工程已导入；个人配置已启用 `hero-m-local`、停用初始桌面 `Debug` 配置。七个车型通过实际构建，HERO-M 另通过新入口的全新构建，OpenOCD 配置解析通过。CLion 内点击构建及硬件断点、烧录尚未验收。

## 先验证工具和构建

在仓库根目录运行：

```powershell
pwsh -NoProfile -File .\tools\build.ps1 -Action probe
pwsh -NoProfile -File .\tools\build.ps1 -Action check -Project all
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project HERO-M
pwsh -NoProfile -File .\tools\build.ps1 -Action build -Project all -BuildRoot .\out\zephyr-all
```

默认 `build` 使用正式 HERO-M 配置，不附加音乐专用、静态准备或只接收测试配置。默认并行数为 2，可用 `-Jobs` 调整。`-Pristine` 会将旧输出移到该构建根目录的 `.pristine-backups` 后重新构建；也可选择新的 `-BuildRoot`。

只检查工程不编译可直接运行 `python tools/CheckZephyr.py --project all`。它检查 Zephyr 源清单、目标/板级配置及 SENTINEL-M 的 overlay 引用；不覆盖原 `CheckAll.ps1` 的全部检查。默认 CI 运行此检查及独立回归；历史检查可手动选择，当前仍有旧路径命名规则与 Zephyr 文件名不兼容的问题。控制逻辑回归与真实编译应另行运行。

## CLion 工程配置

本机已准备可重复的 CMake 预设入口：

1. 在 CLion 中打开 `D:\ARBATOS\zephyr`，选择作为 CMake 工程打开。
2. 在 CMake 配置里启用 `hero-m-local`、停用自动生成的 `Debug`。该配置来自不提交的个人预设；其他机器可配置自己的环境后启用共享 `hero-m`。首次打开时默认 `Debug` 找不到 Zephyr，不代表 SDK 缺失。
3. 构建目标选 `zephyr_final` 或默认全部目标。不要在运行配置中选择旧 `projects/HERO-C/MDK-ARM` 工程。
4. 固件与符号文件为 `D:\ARBATOS\out\zephyr\hero-m\zephyr\zephyr.elf`，同时生成 `.bin` 和 `.hex`。`compile_commands.json` 在构建目录，可供代码索引使用。

需要复现预设时，在 `zephyr` 目录执行：

```powershell
cmake --preset hero-m-local
cmake --build --preset hero-m-local
```

共享预设需要 `ZEPHYR_BASE` 指向 Zephyr 源码，`ZEPHYR_SDK_INSTALL_DIR` 指向 SDK，Python、CMake 和 Ninja 可被找到。本机个人预设已填入这些信息；不把个人路径提交给其他人。

CLion 2026.2 也提供内置 West 支持。可按官方说明把同一 `zephyr/CMakeLists.txt` 转为 West 工程，选择板 `dm_mc02_h7`、本机 `west.exe` 和上述构建目录，并保持 `EXTRA_CONF_FILE` 指向 `zephyr/targets/hero-m.conf`。本次使用 CMake 预设入口，未配置原生 West 界面。

## 本机工具路径

| 工具 | 当前已准备路径 |
| --- | --- |
| West | `D:\ARBATOS\local\cache\zephyrproject\.venv\Scripts\west.exe` |
| Python | `D:\ARBATOS\local\cache\zephyrproject\.venv\Scripts\python.exe` |
| Ninja | `D:\ARBATOS\local\cache\zephyrproject\.venv\Scripts\ninja.exe` |
| Zephyr 源码 | `D:\ARBATOS\local\cache\zephyrproject\zephyr` |
| Zephyr SDK | `D:\ARBATOS\local\cache\zephyr-sdk` |
| C 编译器 | `D:\ARBATOS\local\cache\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gcc.exe` |
| C++ 编译器 | `D:\ARBATOS\local\cache\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-g++.exe` |
| GDB | `D:\ARBATOS\local\cache\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gdb.exe` |
| OpenOCD | `D:\ARBATOS\local\cache\zephyr-sdk\hosttools\openocd\bin\openocd.exe` |

CLion 的代码编辑功能不要求固件使用它自带的桌面编译器；此工程应使用 Zephyr SDK 提供的 ARM 工具链。

## 后续烧录和调试

构建按钮只生成文件。CLion 的 Run/Debug、OpenOCD Download & Run 或 West 的 flash/debug 会连接硬件，并可能暂停、复位、烧录或启动主控，不能当作单纯的代码检查。

本轮不运行下面的硬件命令。后续确认接的是当前 M 板、关闭遥控并按实车条件准备后，可显式执行：

```powershell
# 由 West 写入镜像后校验；不要追加全片擦除选项。
west flash -d .\out\zephyr\hero-m --runner openocd --verify
west debug -d .\out\zephyr\hero-m --runner openocd
```

CLion 的 OpenOCD 配置使用上述 `openocd.exe`、SDK GDB、本工程的 `.cfg` 和实际 `.elf`。原生 West 配置的 Flash options 可填 `--runner openocd --verify`，Debug options 可填 `--runner openocd`。

500 kHz 来自当前无线调试链路的成功记录。新模板尚未做实物回归；此前批量调试访问出现过失真，不能仅凭配置可解析认定无线连接或实时断点可靠。普通断点会暂停整块主控的控制任务。

HERO-M 的校准区从 `0x080C0000` 起。正式固件必须保持分区边界；不使用 `mass_erase` 或全片擦除。当前模板没有内置擦除动作，具体写入由 West/CLion 对所选镜像发起。

## 保留的旧工具

- `legacy-check`：原 `tools/CheckAll.ps1`，仍针对旧工程检查，不能拿它替代 Zephyr 构建结果。
- `legacy-manifest`、`legacy-gcc`、`legacy-gcc-build`：显式调用旧 `.uvprojx` 解析与 GCC 转换流程。
- `tests/SdBenchM/Build.py`：旧独立 SD 测试固件仍使用 Keil，仅用于复现该次测试；正式 HERO-M 编译不调用它。

这些历史入口的保留不要求日常安装 Keil。后续删除历史工程前，需继续核对工具和公共头文件引用。

## 参考

- [JetBrains：Zephyr 与 West](https://www.jetbrains.com/help/clion/zephyr.html)
- [JetBrains：CMake Presets](https://www.jetbrains.com/help/clion/cmake-presets.html)
- [JetBrains：OpenOCD](https://www.jetbrains.com/help/clion/openocd-support.html)
