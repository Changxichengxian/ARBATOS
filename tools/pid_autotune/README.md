# PID Autotune Helper

先说限制：这版工具现在只适合一次盯一个环，不做双环一起联调。

它复用了仓库里现成的 `UART1 tune` 文本命令口，在单片机侧额外补了一条“自动整定专用”数据流，主机侧工具再按窗口做：

- 收样
- 打分
- 记录当前最好的一组
- 明显变差就回退
- 再给下一轮参数

## 现在支持的目标

- `ps`：pitch 速度环
- `pa`：pitch 角度环
- `ys`：yaw 速度环
- `ya`：yaw 角度环
- `cf`：底盘跟随环
- `cm`：底盘电机速度环（4 个轮子的平均窗口）

## 固件侧命令

这几个命令走的还是原来的 `UART1 tune` 小文本协议。

- `at ps`
- `at pa`
- `at ys`
- `at ya`
- `at cf`
- `at cm`
- `at period 20`
- `at off`

打开后，固件会在 UART1 上输出固定 8 通道的 JustFloat 数据：

```text
timestamp_ms, setpoint, input, output, error, kp, ki, kd
```

尾巴还是 `INF`，和现有 JustFloat 一样，只是通道数固定成了 8。

## 推荐用法

先把 UART1 切到 tune 模式，再跑主机工具。

```powershell
python tools\pid_autotune\arbatos_pid_autotune.py --port COM5 --target ps
```

更完整一点的例子：

```powershell
python tools\pid_autotune\arbatos_pid_autotune.py `
  --port COM5 `
  --target ps `
  --window 120 `
  --rounds 8 `
  --mode heuristic
```

如果你想接 OpenAI 兼容接口：

```powershell
python tools\pid_autotune\arbatos_pid_autotune.py `
  --port COM5 `
  --target ps `
  --mode llm `
  --api-base http://127.0.0.1:8000/v1 `
  --api-key sk-demo `
  --model your-model-name
```

## 测试模式

工具默认会自动切到一个更适合单环观察的测试模式：

- `ps` / `pa` -> `TEST_MODE_PITCH_ONLY`
- `ys` / `ya` -> `TEST_MODE_YAW_ONLY`
- `cf` / `cm` -> `TEST_MODE_CHASSIS_ONLY`

如果你不想让工具改测试模式，加 `--test-mode none`。

## 现阶段建议

- 双环先只调内环，再调外环。
- `cm` 最适合做直线或单一方向动作，不适合复杂混合运动。
- 每轮尽量做相似的动作，不然窗口之间很难公平比较。
- 如果这一轮激励太小，工具会直接跳过，不会乱改参数。

## 依赖

```powershell
pip install pyserial
```
