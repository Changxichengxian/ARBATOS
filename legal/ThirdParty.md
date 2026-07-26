# 第三方代码和资料边界

先看结论：根目录 `LICENSE` 只覆盖 ARBATOS 自有代码和文档。第三方代码、厂商 SDK、参考工程、二进制工具和外部资料仍按它们自己的许可证执行。

## 以原始许可证为准的内容

下面这些位置里的内容，优先看各自的文件头、目录内 `LICENSE`、`COPYING`、`NOTICE` 或上游说明：

- `projects/*/Drivers/`
- `projects/MINIWHEELEG-M/Drivers/CMSIS/` 中随工程分发的 ARM CMSIS Core 与 ST STM32H7xx Device 头文件，许可证见目录内的 `LICENSE.txt` 和 `Package_license.md`
- `projects/*/Middlewares/`
- `shared/components/algorithm/*.lib`
- `tools/Mp3ToU8/ffmpeg.exe`
- `tools/Mp3ToU8/FFMPEG_LICENSE.txt`
- `local/reference/`
- `local/docs/04_原厂资料包/`
- 任何带独立版权头、许可证文本、来源声明或上游归属说明的文件

## 怎么判断

如果某个文件满足下面任意一点，就不要简单当成 ARBATOS 自有 `Apache-2.0` 内容：

- 文件头已经写了别的版权或许可证
- 同目录附带了独立 `LICENSE`、`COPYING`、`NOTICE`
- 它明显是厂商 SDK、第三方库、外部例程、参考工程、二进制工具或打包资料

## 贡献时的要求

- 引入第三方内容时，要保留原来的版权和许可证信息。
- 不确定归属时，先在 PR 里说明来源，不要直接删改别人的许可证。
- 如果第三方许可证要求附带源码、NOTICE 或修改说明，提交时要一起补齐。
- 如果某个目录以后做了更细的第三方清单，以那份清单为准。

## 重要提醒

- Apache-2.0 只改变 ARBATOS 自有部分的授权方式。
- 根许可证不会覆盖或替换第三方文件自己的许可证。
- 使用、分发固件或工具时，需要同时满足 ARBATOS 和相关第三方组件的许可证要求。
