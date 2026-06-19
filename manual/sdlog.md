# SD 日志和复盘

SD 日志的目标不是“把所有东西都写下来”，而是让一次上车能被复盘：这版固件是谁编的、用的什么配置、跑了哪些状态、什么时候掉线或超时。

## 当前格式

当前主线只保留一种 SD 日志格式：

- 文件头：`sdlog_file_header_t`
- 数据块：`sdlog_block_header_t + block bytes`
- 块内容：变长整数记录流
- 记录：`dt_ms + tag + len + payload`

旧的 v1 / v2 / v3 命名已经删掉，不再为了没实测过的格式保留解析分支。文件头也不再写数字格式版本。

如果 payload 自身需要演进，按 tag 规则处理：

- 兼容新增字段：追加字段。
- 不兼容改字段：提升 payload version，或者分配新 tag。
- 不要复用同一个 tag 表示两种不兼容结构。

## BUILD_INFO

每个日志启动时会写 `SDLOG_TAG_BUILD_INFO`。解析后能看到：

- `target`
- `board`
- `git_sha`
- `build_dirty`
- `build_date`
- `build_time`
- `schema_version`
- `config_size`
- `config_crc32`
- `task_module_count`
- `task_module_mask`
- `high_rate_div`
- `compression_enabled`

Keil 工程的 `BeforeMake` 会运行：

```text
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ..\..\..\tools\GenBuildInfo.ps1
```

GCC/CMake 路线也会在 `tools/build.ps1 -Action gcc` 和 `tools/build.ps1 -Action gcc-build` 开始时运行同一个脚本。

它生成 `shared/generated/build_info_autogen.h`，这个文件被 `.gitignore` 忽略，不会把每次编译时间提交进 Git。日志里的 `git_sha` 保留 16 位 Git 编号，`build_dirty` 标记这次固件是不是来自带改动的工作区，配合 `config_crc32` 可以把“哪版代码、哪份配置、哪次构建”对上。

因此，只要 SD 日志能解析出 `BUILD_INFO`，这次固件就有基本发布追溯。`build_dirty = 1` 需要在复盘里说明，但它不是“仓库缺少发布纪律”的扣分点。

## 查看日志

启动网页查看器：

```powershell
python tools\sdlog\SdLogViewer.py sdlog_0001.bin
```

常用导出：

- 单个 tag CSV：网页里的 `Export tag CSV`
- 单个字段 CSV：网页里的 `Export field CSV`
- 未知记录 CSV：网页里的 `Export unknown CSV`

未知记录 CSV 会保留：

- tick
- tag
- payload 长度
- payload CRC32
- payload 十六进制内容

这保证解析器没跟上新 tag 时，原始数据不会直接丢。

## 解压日志

如果固件开启 LZ4 块压缩，可以先解压成当前格式的未压缩文件：

```powershell
python tools\sdlog\SdLogDecompress.py sdlog_0001.bin
```

默认输出：

```text
sdlog_0001.bin.uncompressed.bin
```

这个脚本只去掉块压缩，不会转换成旧格式。

## 留基线日志

每个真正上车的阶段都应该留一组基线。推荐目录：

```text
local/logs/sdlog/<TARGET>/<YYYY-MM-DD>/
```

最少留这些信息：

```text
Target：
Board：
Git：
dirty：
电池：
硬件改动：
测试动作：
参数改动：
日志文件：
现象：
结论：
```

推荐至少覆盖：

- 启动后静止 10 秒。
- 遥控输入变化。
- 云台单轴动作。
- 底盘低速动作。
- CAN 反馈在线 / 掉线情况。
- `RT_PROFILER` 和 `SYS_STATS`。
- 轮腿实验时的 MIT 配置和状态。

只有 `META` 或只有 `BUILD_INFO` 的文件不能当运行基线，它只能证明文件创建成功。

## 新增 tag 前检查

新增 SD 日志 tag 前先问：

1. 这个数据是否真的需要高频记录？
2. 能不能降频或打包成 base stream？
3. payload 里字段单位是否清楚？
4. 解析器是否同步更新？
5. 有没有样例日志或合成测试能覆盖？

高频控制任务里新增日志时，还要考虑复制成本、环形缓冲占用和最坏耗时。日志不是直接写文件，但仍然会复制数据并进入短临界区。
