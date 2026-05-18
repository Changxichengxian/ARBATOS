# 第三方代码和资料边界

先说结论：仓库根目录的 `PolyForm-Noncommercial-1.0.0` 只覆盖 Licensor 自己有权按该协议发布的那部分内容，不会把第三方代码、厂商资料或别人的许可证自动改掉。

## 以原始许可证为准的内容

下面这些位置里的内容，原则上优先按它们各自的文件头、目录内 `LICENSE`、`NOTICE` 或上游说明执行：

- `projects/*/Drivers/`
- `projects/*/Middlewares/`
- `local/docs/04_原厂资料包/`
- `tools/mp3_to_u8/`
- `local/reference/`
- 任何带独立版权头、许可证文本、来源声明或上游归属说明的文件

## 怎么判断

如果某个文件同时满足下面任意一点，就不要把它简单当成 `PolyForm-Noncommercial-1.0.0`：

- 文件头已经写了别的版权或许可证
- 同目录附带了独立 `LICENSE`、`COPYING`、`NOTICE`
- 它明显是厂商 SDK、第三方库、外部例程、参考工程或打包资料

## 贡献时的要求

- 引入第三方内容时，要保留原来的版权和许可证信息
- 不确定归属时，先在 PR 里说明来源，不要直接删改别人的许可证
- 如果某个目录以后做了更细的第三方清单，以那份清单为准

## 重要提醒

根目录 `LICENSE` 里的 `Licensed Work` 已经排除了这类第三方内容。也就是说：

- 你不能把别人的代码当成 Licensor 自己的代码去单独卖授权
- 你也不能因为仓库根目录放了 `PolyForm-Noncommercial-1.0.0`，就假定所有文件都跟着一起变成 `PolyForm-Noncommercial-1.0.0`
