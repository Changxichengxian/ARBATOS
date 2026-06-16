# tools

`tools/` 放本地辅助脚本。它不替代 Keil，也不替代实车调试，主要用来在改共享代码后先做一轮低成本检查；现在也提供 GCC/CMake 命令行构建入口。

## 构建信息

Keil 工程的 `BeforeMake` 会调用：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ..\..\..\tools\gen_build_info.ps1
```

这个脚本生成 `shared/generated/build_info_autogen.h`，把 Git 提交、dirty 状态和编译时间带进固件。生成文件被 `.gitignore` 忽略，不需要手工提交。

## 一键检查

在仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\check_all.ps1
```

当前会检查：

- 所有 `projects/*/MDK-ARM/*.uvprojx` 能被解析。
- Keil 工程里列出的源码文件是否真实存在。
- Include Path 里本地目录是否存在；Keil Pack 提供的目录如果仓库里没有，会先作为警告。
- 每个工程是否只引用自己的 `Robotconfig/<TARGET>`。
- `Robotconfig/<TARGET>/config.c` 里的 profile 是否和工程包含的任务源码、任务创建入口匹配。
- `tools/**/*.py` 是否有 Python 语法错误。
- 文档里是否还残留几类已经确认过时的路径或 MIT 轮腿描述。

它不会真的调用 Keil Rebuild，也不会默认跑完整 GCC 编译。要做真实编译验证，走下面的 `tools/build.ps1 -Action gcc-build`。

GitHub 上的 `.github/workflows/check-all.yml` 也会跑同一个脚本。也就是说，PR 或推送到主分支时，至少会自动检查工程引用、profile 和文档旧路径这些低级断裂。

如果想连未跟踪的本地文档也扫一遍，可以加 `-AllText`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\check_all.ps1 -AllText
```

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

- `tools/sdlog/sdlog_viewer.py`：打开 SD 日志网页查看器，支持导出 tag、字段和未知记录 CSV。
- `tools/sdlog/sdlog_decompress.py`：去掉当前格式日志里的 LZ4 块压缩，输出仍然是当前格式。

使用流程见 `../manual/sdlog.md`。
