# tools

`tools/` 放本地辅助脚本。它不替代 Keil，也不替代实车调试，主要用来在改共享代码后先做一轮低成本检查。

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

现在它还不会真的调用 Keil 编译。等本地命令行编译路径确认后，可以把代表工程编译接到这个脚本后面。

GitHub 上的 `.github/workflows/check-all.yml` 也会跑同一个脚本。也就是说，PR 或推送到主分支时，至少会自动检查工程引用、profile 和文档旧路径这些低级断裂。

如果想连未跟踪的本地文档也扫一遍，可以加 `-AllText`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\check_all.ps1 -AllText
```
