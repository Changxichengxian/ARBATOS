# ARBATOS 桌面客户端

这是仓库内的本机工程工具：查看和编辑某一车型的现有配置文件、枚举与连接串口、显示串口曲线，以及为单一车型执行检查、编译和确认后的烧录。界面借鉴了 [Betaflight App](https://github.com/betaflight/betaflight-configurator) 的紧凑工具布局；没有复制其源码、资源或协议实现。

客户端不连接云端。桌面进程启动一个仅绑定 `127.0.0.1` 的 Python 服务，页面通过本次启动的随机会话令牌访问它；服务再调用仓库已有的生成器、构建脚本和串口服务。

## 启动

在仓库根目录双击 [打开客户端.cmd](../打开客户端.cmd)。它会：

1. 优先使用 `local/cache/zephyrproject/.venv/Scripts/python.exe`，没有时使用 PATH 中的 Python；需要 Python 3.12 或更高版本。
2. 首次缺少依赖时安装 `client/backend/requirements.txt` 中的 `pyserial`、`tomlkit` 与 `PyYAML`。
3. 首次缺少桌面运行程序时运行 `npm ci` 和 `npm run setup:desktop`，再运行 `npm run build`。需要 Node.js 24 和 npm；首次安装需要联网。
4. 启动 Electron 桌面窗口及本机服务。

首次安装也可手动执行：

```powershell
cd D:\ARBATOS\client
python -m pip install -r .\backend\requirements.txt
npm ci
npm run setup:desktop
npm run build
npm start
```

`npm start` 需要先有 `dist/`，并从 `client/` 目录运行。需要指定 Python 时，可先设置 `ARBATOS_PYTHON` 为 Python 可执行文件的完整路径。

## 目录和职责

| 位置 | 职责 |
| --- | --- |
| `src/` | Vue 3 页面；只通过 `src/api.ts` 调用本机 RPC。 |
| `desktop/` | Electron 窗口、服务启动与关闭确认。 |
| `backend/` | 车型、文件、任务和串口 RPC；不向外网开放端口。 |
| `scripts/test-backend.mjs` | 启动后端单元测试的 npm 入口。 |

接口按用途分为 `workspace.summary`、`robot.*`、`file.*`、`serial.*` 和 `job.*`。页面不直接读取任意路径，也不会用静态示例数据伪装真实串口或构建状态。

## 车型配置和文件

选择车型后，配置页可编辑控制器、服务和功能开关；完整配置可在文件页打开 `RobotConfig.toml`。保存前会带上读取时的修订号；外部修改造成冲突时后端会拒绝写入，不覆盖对方内容。修改可先做“检查修改”，保存后再编译。

“新建车型”复制所选模板，并把新副本设为单电机模式且不选择任何电机。模板保持原状；新车核对接线后在 `ConfigOperation.inc` 显式启用输出。

文件页只列出后端允许的、该车型目录中的现有文本文件。常见入口仍是：

| 内容 | 文件 |
| --- | --- |
| 车型能力、控制器和服务选择 | `Robotconfig/<车型>/RobotConfig.toml` |
| 电机型号、编号、总线和实例绑定 | `Robotconfig/<车型>/ConfigHardware.inc` |
| PID、限幅与运行参数 | `Robotconfig/<车型>/ConfigTuning.inc` |

客户端可以编辑这些现有文件，但在线参数保存通路还没有接入。串口连接用于查看和手动发送文本；它不会把 PID 或其他控制参数直接写入正在运行的固件。

## 串口与曲线

客户端默认只枚举本机 COM 口，不会自动打开或发送。连接时选择 `text`、`csv` 或 `justfloat`，以及 1 至 16 个通道；连接期间格式和通道数锁定。文本发送固定使用 LF 行结尾。

- `text`：按文本行显示。
- `csv`：每行 N 个逗号分隔的数值，或首列时间加 N 个数值，N 为选择的通道数。
- `justfloat`：每帧 N 个小端 32 位浮点数，末尾为 `00 00 80 7f`；N 为选择的通道数。
- CSV 离线回放：只接受最多 5 MB、100000 行、1 至 16 个数值通道的文件。可选首列时间表头为 `time`、`time_s`、`timestamp`、`tick_ms` 或 `time_ms`；后两者自动转换为秒。普通列名表头也可用。空值、非数值、列数不一致和倒退的时间会被拒绝。

导入离线 CSV 前必须先断开真实串口，回放界面会明确标注“离线数据”。它不代表设备当前在线。

## 编译与烧录

“检查”只检查当前车型的配置和工程引用；“增量编译”使用该车型现有构建目录；“重新编译”使用全量构建流程。客户端一次只对选中的车型启动任务，日志保留最近一段输出。配置或文件有未保存修改时，客户端会阻止开始构建，避免编译磁盘上的旧内容。

烧录不会直接开始。先请求产物计划，显示精确车型、固件路径、构建时刻和 SHA-256；确认后才启动下载。后端还会核对构建产物、当前源码状态和车型匹配。下载沿用仓库 `tools/build.ps1` 的正式 OpenOCD 参数，其中包含 `--no-erase --verify`：保留校准区，且写入后校验；不要改成整片擦除后再把这项保护当作仍然存在。

客户端的编译、计划和本机服务已做电脑端检查；真实电机、下载器、校准数据和整车硬件尚未用此客户端完成实测。烧录和上车仍按 [调试与日志](../manual/调试与日志.md) 的硬件准备步骤执行。

## 开发与测试

```powershell
cd D:\ARBATOS\client
npm run build          # Vue 类型检查和生产构建
npm run dev            # 仅启动 Vite 开发服务；不启动本机 RPC
npm run test:backend   # Python 后端单元测试
pwsh -NoProfile -File ..\tools\client\StartClient.ps1 -BuildOnly
```

以上命令验证页面构建或后端逻辑，不能替代实际串口、下载器和车辆测试。

开发界面时，先在一个终端启动后端：

```powershell
python .\backend\server.py --port 8765 --dev-origin http://127.0.0.1:8766 --session-file ..\local\cache\client\dev-session.json
```

再在另一个终端运行 `npm run dev`。浏览器打开 `http://127.0.0.1:8766/#token=<dev-session.json 中的 token>`；令牌只用于当前本机服务，文件在退出后移除。Vite 将 `/api` 转发给后端。改前端端口时同步修改后端 `--dev-origin`；改后端端口时设置前端环境变量 `ARBATOS_API_URL`。

界面调用 `POST /api/rpc`，请求为 `{ "method": "robot.get", "params": { "target": "HERO-M" } }`，通过 `Authorization: Bearer <token>` 验证会话。成功返回 `{ "ok": true, "result": ... }`，失败返回 `{ "ok": false, "error": { "code": ..., "message": ... } }`。后端独立校验路径、修订号和参数；更换前端时继续使用这份接口，不需要重写配置生成或下载逻辑。
