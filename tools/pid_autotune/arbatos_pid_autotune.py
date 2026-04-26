#!/usr/bin/env python3
"""
ARBATOS PID autotune helper.

This tool talks to the existing UART1 tuning port in "tune mode" and reuses
the same small-text command channel that firmware already understands.

The embedded side now exposes a compact 8-channel JustFloat stream for one
selected loop:
    timestamp_ms, setpoint, input, output, error, kp, ki, kd

The host side keeps the logic intentionally small and conservative:
    - collect one tuning window
    - score the response
    - keep best-so-far
    - rollback on clear regression
    - generate the next PID with either heuristics or an optional LLM
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Iterable, List, Sequence

try:
    import serial as pyserial  # type: ignore
except ImportError:  # pragma: no cover - runtime environment dependent
    pyserial = None


TAIL_WORD = 0x7F800000
TAIL_BYTES = struct.pack("<I", TAIL_WORD)
FRAME_FLOATS = 9
FRAME_BYTES = FRAME_FLOATS * 4
HISTORY_LIMIT = 5

TARGET_TEST_MODE = {
    "ps": 4,
    "pa": 4,
    "ys": 2,
    "ya": 2,
    "cf": 1,
    "cm": 1,
}
TEST_MODE_PARAM_ID = 244

TARGET_LABEL = {
    "ps": "pitch speed",
    "pa": "pitch angle",
    "ys": "yaw speed",
    "ya": "yaw angle",
    "cf": "chassis follow",
    "cm": "chassis motor speed",
}


def fmean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


@dataclass
class Sample:
    timestamp_ms: float
    setpoint: float
    input_value: float
    output: float
    error: float
    p: float
    i: float
    d: float


@dataclass
class PID:
    p: float
    i: float
    d: float


@dataclass
class Metrics:
    avg_error: float
    max_error: float
    steady_state_error: float
    overshoot: float
    zero_crossings: int
    status: str
    reference_abs: float
    setpoint_span: float
    input_span: float


@dataclass
class HistoryItem:
    round_index: int
    pid: PID
    metrics: Metrics
    analysis: str


@dataclass
class Suggestion:
    pid: PID
    action: str
    analysis: str
    fallback_used: bool = False
    llm_raw: str = ""


@dataclass
class SessionState:
    current_pid: PID | None = None
    best_pid: PID | None = None
    best_metrics: Metrics | None = None
    best_round: int | None = None
    history: List[HistoryItem] = field(default_factory=list)
    stable_rounds: int = 0
    rollback_count: int = 0


class JustFloatParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, chunk: bytes) -> list[Sample]:
        if chunk:
            self._buffer.extend(chunk)

        samples: list[Sample] = []
        while True:
            tail_index = self._buffer.find(TAIL_BYTES)
            if tail_index < 0:
                if len(self._buffer) > FRAME_BYTES:
                    del self._buffer[: len(self._buffer) - (FRAME_BYTES - 1)]
                break

            frame_start = tail_index - (FRAME_BYTES - 4)
            if frame_start < 0:
                del self._buffer[: tail_index + 4]
                continue

            if frame_start > 0:
                del self._buffer[:frame_start]
                tail_index -= frame_start

            if len(self._buffer) < FRAME_BYTES:
                break

            frame = bytes(self._buffer[:FRAME_BYTES])
            del self._buffer[:FRAME_BYTES]

            try:
                values = struct.unpack("<9f", frame)
            except struct.error:
                continue

            if not math.isinf(values[-1]):
                continue

            sample = Sample(
                timestamp_ms=float(values[0]),
                setpoint=float(values[1]),
                input_value=float(values[2]),
                output=float(values[3]),
                error=float(values[4]),
                p=float(values[5]),
                i=float(values[6]),
                d=float(values[7]),
            )
            samples.append(sample)

        return samples


class SerialTunerBridge:
    def __init__(self, port: str, baud: int, read_timeout: float) -> None:
        if pyserial is None:
            raise RuntimeError("缺少 pyserial。先执行: pip install pyserial")
        self.port = port
        self.baud = baud
        self.serial = pyserial.Serial(port, baud, timeout=read_timeout)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def flush_input(self) -> None:
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

    def write_command(self, command: str, pause_s: float = 0.04) -> None:
        payload = (command.strip() + "\n").encode("ascii", errors="ignore")
        self.serial.write(payload)
        self.serial.flush()
        if pause_s > 0.0:
            time.sleep(pause_s)

    def set_test_mode(self, value: int) -> None:
        self.write_command(f"{TEST_MODE_PARAM_ID}:{value}")

    def arm_autotune_target(self, target: str, period_ms: int) -> None:
        self.write_command("at off")
        self.write_command(f"at period {period_ms}")
        self.write_command(f"at {target}")

    def disarm_autotune_target(self) -> None:
        self.write_command("at off")

    def set_pid(self, target: str, pid: PID) -> None:
        self.write_command(f"{target} kp {pid.p:.6f}")
        self.write_command(f"{target} ki {pid.i:.6f}")
        self.write_command(f"{target} kd {pid.d:.6f}")

    def read_chunk(self, max_bytes: int = 256) -> bytes:
        return self.serial.read(max_bytes)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARBATOS UART1 PID autotune helper")
    parser.add_argument("--port", required=True, help="串口号，比如 COM5")
    parser.add_argument("--baud", type=int, default=230400, help="UART1 tune 波特率")
    parser.add_argument(
        "--target",
        choices=sorted(TARGET_LABEL),
        required=True,
        help="本轮要整定的单环目标",
    )
    parser.add_argument(
        "--test-mode",
        default="auto",
        help="测试模式编号，默认 auto；传 none 表示不改",
    )
    parser.add_argument("--period-ms", type=int, default=20, help="固件输出窗口周期")
    parser.add_argument("--window", type=int, default=120, help="每轮采样点数")
    parser.add_argument("--rounds", type=int, default=8, help="最多轮数")
    parser.add_argument(
        "--read-timeout",
        type=float,
        default=0.1,
        help="串口 read timeout（秒）",
    )
    parser.add_argument(
        "--window-timeout",
        type=float,
        default=12.0,
        help="单轮收样超时（秒）",
    )
    parser.add_argument(
        "--min-excitation",
        type=float,
        default=0.02,
        help="最小激励阈值，太小就跳过本轮",
    )
    parser.add_argument(
        "--stable-rounds",
        type=int,
        default=2,
        help="连续稳定多少轮后提前停",
    )
    parser.add_argument(
        "--mode",
        choices=("heuristic", "llm"),
        default="heuristic",
        help="建议来源：规则 or LLM",
    )
    parser.add_argument("--api-base", default="", help="OpenAI 兼容接口根地址")
    parser.add_argument("--api-key", default="", help="OpenAI 兼容接口 key")
    parser.add_argument("--model", default="", help="模型名")
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.2,
        help="LLM 温度，默认 0.2",
    )
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def score_metrics(metrics: Metrics) -> float:
    status_penalty = {
        "OSCILLATING": 12.0,
        "OVERSHOOTING": 8.0,
        "SLOW_RESPONSE": 5.0,
        "UNEXCITED": 25.0,
    }.get(metrics.status, 0.0)
    return (
        metrics.avg_error
        + metrics.steady_state_error * 1.2
        + metrics.overshoot * 0.6
        + status_penalty
    )


def is_better(candidate: Metrics, baseline: Metrics) -> bool:
    return score_metrics(candidate) + 1e-6 < score_metrics(baseline)


def should_rollback(current: Metrics, best: Metrics) -> bool:
    if not is_better(best, current):
        return False
    if best.status == "STABLE" and current.status != "STABLE":
        return True

    avg_regression = current.avg_error > max(best.avg_error * 1.5, best.avg_error + 0.02)
    steady_regression = current.steady_state_error > max(
        best.steady_state_error * 1.8,
        best.steady_state_error + 0.01,
    )
    overshoot_regression = current.overshoot > best.overshoot + 1.0
    return avg_regression or steady_regression or overshoot_regression


def is_good_enough(metrics: Metrics) -> bool:
    ref = max(metrics.reference_abs, 1e-3)
    return (
        metrics.status == "STABLE"
        and metrics.steady_state_error <= max(ref * 0.03, 0.005)
        and metrics.avg_error <= max(ref * 0.05, 0.008)
        and metrics.overshoot <= 3.0
    )


def count_zero_crossings(values: Sequence[float]) -> int:
    count = 0
    last_sign = 0
    for value in values:
        sign = 1 if value > 0 else (-1 if value < 0 else 0)
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            count += 1
        last_sign = sign
    return count


def calc_metrics(samples: Sequence[Sample], min_excitation: float) -> Metrics:
    if not samples:
        raise ValueError("empty sample window")

    errors = [sample.error for sample in samples]
    abs_errors = [abs(value) for value in errors]
    setpoints = [sample.setpoint for sample in samples]
    inputs = [sample.input_value for sample in samples]

    setpoint_span = (max(setpoints) - min(setpoints)) if setpoints else 0.0
    input_span = (max(inputs) - min(inputs)) if inputs else 0.0

    dominant_setpoint = max(setpoints, key=lambda value: abs(value))
    direction = 1.0 if dominant_setpoint >= 0.0 else -1.0
    reference_abs = max(abs(dominant_setpoint), max(abs(value) for value in inputs), 1e-6)

    signed_inputs = [direction * value for value in inputs]
    signed_targets = [direction * value for value in setpoints]
    overshoot = max(0.0, max(signed_inputs) - max(signed_targets)) / reference_abs * 100.0

    steady_len = max(1, len(samples) // 5)
    steady_state_error = fmean(abs_errors[-steady_len:])
    avg_error = fmean(abs_errors)
    max_error = max(abs_errors)
    zero_crossings = count_zero_crossings(errors)

    excited = max(setpoint_span, input_span, max_error) >= min_excitation
    status = "STABLE"
    if not excited:
        status = "UNEXCITED"
    elif zero_crossings > max(2, len(samples) // 4):
        status = "OSCILLATING"
    elif overshoot > 5.0:
        status = "OVERSHOOTING"
    elif avg_error > max(reference_abs * 0.12, 0.01) and steady_state_error > max(reference_abs * 0.06, 0.006):
        status = "SLOW_RESPONSE"

    return Metrics(
        avg_error=avg_error,
        max_error=max_error,
        steady_state_error=steady_state_error,
        overshoot=overshoot,
        zero_crossings=zero_crossings,
        status=status,
        reference_abs=reference_abs,
        setpoint_span=setpoint_span,
        input_span=input_span,
    )


def fmt_pid(pid: PID) -> str:
    return f"P={pid.p:.6f} I={pid.i:.6f} D={pid.d:.6f}"


def build_history_text(history: Sequence[HistoryItem]) -> str:
    if not history:
        return "无历史记录，本轮视作第一轮。"
    lines = []
    for item in history[-HISTORY_LIMIT:]:
        lines.append(
            f"Round {item.round_index}: {fmt_pid(item.pid)} | "
            f"status={item.metrics.status} avg={item.metrics.avg_error:.6f} "
            f"steady={item.metrics.steady_state_error:.6f} "
            f"overshoot={item.metrics.overshoot:.2f}% | {item.analysis}"
        )
    return "\n".join(lines)


def heuristic_suggestion(pid: PID, metrics: Metrics) -> Suggestion:
    proposal = PID(pid.p, pid.i, pid.d)

    if metrics.status == "OSCILLATING":
        proposal.p *= 0.82
        proposal.i *= 0.88
        proposal.d *= 1.15
        return Suggestion(
            pid=proposal,
            action="DAMP_OSCILLATION",
            analysis="检测到振荡，保守降低 P/I，并略增 D。",
            fallback_used=True,
        )

    if metrics.status == "OVERSHOOTING":
        proposal.p *= 0.88
        proposal.i *= 0.92
        proposal.d *= 1.12
        return Suggestion(
            pid=proposal,
            action="REDUCE_OVERSHOOT",
            analysis="检测到超调，先压 P/I，再加一点阻尼。",
            fallback_used=True,
        )

    if metrics.status == "SLOW_RESPONSE":
        proposal.p *= 1.18
        if metrics.steady_state_error > max(metrics.reference_abs * 0.05, 0.01):
            proposal.i *= 1.12
        return Suggestion(
            pid=proposal,
            action="BOOST_RESPONSE",
            analysis="响应偏慢，优先加 P；稳态误差还偏大时再轻加 I。",
            fallback_used=True,
        )

    if metrics.status == "UNEXCITED":
        return Suggestion(
            pid=proposal,
            action="HOLD",
            analysis="这一轮激励太小，先不改参数，重新做一轮更明显的动作。",
            fallback_used=True,
        )

    proposal.p *= 1.04
    proposal.i *= 1.03
    return Suggestion(
        pid=proposal,
        action="FINE_TUNE",
        analysis="系统基本稳定，做小步细调。",
        fallback_used=True,
    )


def guardrail_pid(current: PID, candidate: PID) -> tuple[PID, list[str]]:
    limits = {
        "p": {"min": 0.0, "max": 5000.0, "ratio": 2.0},
        "i": {"min": 0.0, "max": 500.0, "ratio": 2.5},
        "d": {"min": 0.0, "max": 500.0, "ratio": 2.5},
    }

    notes: list[str] = []
    result = PID(candidate.p, candidate.i, candidate.d)
    for key in ("p", "i", "d"):
        current_value = max(0.0, getattr(current, key))
        raw_value = max(0.0, getattr(result, key))
        cfg = limits[key]
        bounded = clamp(raw_value, cfg["min"], cfg["max"])
        if current_value > 0.0:
            max_step = min(cfg["max"], current_value * cfg["ratio"])
            if bounded > max_step:
                bounded = max_step
                notes.append(f"{key.upper()} 增幅过大，已限到 {bounded:.6f}")
        if bounded != raw_value and all(not note.startswith(key.upper()) for note in notes):
            notes.append(f"{key.upper()} 已从 {raw_value:.6f} 调整到 {bounded:.6f}")
        setattr(result, key, bounded)
    return result, notes


def _extract_json_object(text: str) -> dict[str, Any] | None:
    stripped = text.strip()
    candidates: list[str] = []
    if stripped.startswith("{") and stripped.endswith("}"):
        candidates.append(stripped)

    fence_start = stripped.find("{")
    fence_end = stripped.rfind("}")
    if fence_start >= 0 and fence_end > fence_start:
        candidates.append(stripped[fence_start : fence_end + 1])

    for candidate in candidates:
        try:
            data = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(data, dict):
            return data
    return None


def _join_llm_content(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts: list[str] = []
        for item in content:
            if isinstance(item, dict):
                text = item.get("text")
                if isinstance(text, str):
                    parts.append(text)
        return "".join(parts)
    return ""


def ask_llm(
    api_base: str,
    api_key: str,
    model: str,
    temperature: float,
    target: str,
    current_pid: PID,
    metrics: Metrics,
    history_text: str,
) -> Suggestion:
    prompt = f"""
你在帮一个真实硬件回路做保守 PID 调参。

目标回路: {TARGET_LABEL[target]}
当前 PID: {fmt_pid(current_pid)}
当前指标:
- status: {metrics.status}
- avg_error: {metrics.avg_error:.6f}
- steady_state_error: {metrics.steady_state_error:.6f}
- overshoot: {metrics.overshoot:.2f}
- zero_crossings: {metrics.zero_crossings}
- setpoint_span: {metrics.setpoint_span:.6f}
- input_span: {metrics.input_span:.6f}

历史:
{history_text}

要求:
1. 真实硬件，必须保守。
2. 只给下一轮建议，不要一次跳太大。
3. 优先稳定性，其次再提速。
4. 如果本轮激励不足，也可以建议保持不变。

只输出 JSON:
{{
  "analysis_summary": "一句话说明",
  "tuning_action": "动作名",
  "p": 0.0,
  "i": 0.0,
  "d": 0.0
}}
""".strip()

    base = api_base.rstrip("/")
    if not base.endswith("/chat/completions"):
        base = base + "/chat/completions"

    payload = {
        "model": model,
        "temperature": temperature,
        "messages": [
            {
                "role": "system",
                "content": "你是保守的嵌入式控制工程师，只输出 JSON。",
            },
            {"role": "user", "content": prompt},
        ],
    }

    request = urllib.request.Request(
        base,
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            body = response.read().decode("utf-8", errors="ignore")
    except (urllib.error.URLError, TimeoutError) as exc:
        raise RuntimeError(f"LLM 请求失败: {exc}") from exc

    try:
        parsed = json.loads(body)
    except json.JSONDecodeError as exc:
        raise RuntimeError("LLM 返回不是合法 JSON") from exc

    choices = parsed.get("choices")
    if not isinstance(choices, list) or not choices:
        raise RuntimeError("LLM 返回里没有 choices")

    message = choices[0].get("message", {})
    content = _join_llm_content(message.get("content"))
    data = _extract_json_object(content)
    if data is None:
        raise RuntimeError("LLM 内容里没有可解析的 JSON 对象")

    try:
        suggestion_pid = PID(
            p=float(data.get("p", current_pid.p)),
            i=float(data.get("i", current_pid.i)),
            d=float(data.get("d", current_pid.d)),
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError("LLM 返回的 P/I/D 不是数字") from exc

    return Suggestion(
        pid=suggestion_pid,
        action=str(data.get("tuning_action", "LLM_SUGGEST")),
        analysis=str(data.get("analysis_summary", "LLM 给出了一轮建议。")),
        fallback_used=False,
        llm_raw=content,
    )


def choose_suggestion(
    args: argparse.Namespace,
    target: str,
    current_pid: PID,
    metrics: Metrics,
    history_text: str,
) -> Suggestion:
    if args.mode != "llm":
        return heuristic_suggestion(current_pid, metrics)
    if not args.api_base or not args.api_key or not args.model:
        raise RuntimeError("LLM 模式需要同时提供 --api-base --api-key --model")
    try:
        return ask_llm(
            api_base=args.api_base,
            api_key=args.api_key,
            model=args.model,
            temperature=args.temperature,
            target=target,
            current_pid=current_pid,
            metrics=metrics,
            history_text=history_text,
        )
    except RuntimeError as exc:
        fallback = heuristic_suggestion(current_pid, metrics)
        fallback.analysis = f"{fallback.analysis}（LLM 失败，已回退到规则：{exc}）"
        fallback.fallback_used = True
        return fallback


def collect_window(
    bridge: SerialTunerBridge,
    parser: JustFloatParser,
    window_size: int,
    timeout_s: float,
) -> list[Sample]:
    samples: list[Sample] = []
    deadline = time.monotonic() + timeout_s
    while len(samples) < window_size:
        if time.monotonic() >= deadline:
            raise TimeoutError("本轮收样超时，请确认 UART1 在 tune 模式且正在输出 autotune 流。")
        chunk = bridge.read_chunk()
        if not chunk:
            continue
        for sample in parser.feed(chunk):
            samples.append(sample)
            if len(samples) >= window_size:
                break
    return samples


def resolve_test_mode(args: argparse.Namespace) -> int | None:
    raw = str(args.test_mode).strip().lower()
    if raw in {"", "none", "skip"}:
        return None
    if raw == "auto":
        return TARGET_TEST_MODE.get(args.target)
    return int(raw)


def print_metrics(round_index: int, metrics: Metrics) -> None:
    print(
        f"[Round {round_index}] status={metrics.status} "
        f"avg={metrics.avg_error:.6f} steady={metrics.steady_state_error:.6f} "
        f"overshoot={metrics.overshoot:.2f}% zero_crossings={metrics.zero_crossings} "
        f"set_span={metrics.setpoint_span:.6f} input_span={metrics.input_span:.6f}"
    )


def main() -> int:
    args = parse_args()
    parser = JustFloatParser()
    state = SessionState()
    bridge: SerialTunerBridge | None = None

    try:
        bridge = SerialTunerBridge(args.port, args.baud, args.read_timeout)
        print(f"串口: {args.port} @ {args.baud}")
        print(f"目标: {args.target} ({TARGET_LABEL[args.target]})")

        test_mode = resolve_test_mode(args)
        if test_mode is not None:
            bridge.set_test_mode(test_mode)
            print(f"已切到测试模式 {test_mode}")

        bridge.flush_input()
        bridge.arm_autotune_target(args.target, args.period_ms)
        bridge.flush_input()
        parser.clear()
        print("自动整定数据流已打开。")
        print("现在开始做尽量一致的单环动作，每轮会自动评估一次。")

        for round_index in range(1, args.rounds + 1):
            samples = collect_window(bridge, parser, args.window, args.window_timeout)
            current_from_stream = PID(
                p=samples[-1].p,
                i=samples[-1].i,
                d=samples[-1].d,
            )
            if state.current_pid is None:
                state.current_pid = current_from_stream

            metrics = calc_metrics(samples, args.min_excitation)
            print_metrics(round_index, metrics)

            if metrics.status == "UNEXCITED":
                print("这一轮激励太小，先不改参数，继续下一轮。")
                continue

            if state.best_metrics is None or (
                metrics.status == "STABLE" and is_better(metrics, state.best_metrics)
            ):
                state.best_metrics = metrics
                state.best_pid = PID(current_from_stream.p, current_from_stream.i, current_from_stream.d)
                state.best_round = round_index
                print(f"记录新的最佳结果: round {round_index} {fmt_pid(state.best_pid)}")

            if state.best_metrics is not None and state.best_pid is not None and should_rollback(metrics, state.best_metrics):
                print(
                    f"本轮明显劣化，回退到 round {state.best_round} 的最佳参数: "
                    f"{fmt_pid(state.best_pid)}"
                )
                bridge.set_pid(args.target, state.best_pid)
                state.current_pid = PID(state.best_pid.p, state.best_pid.i, state.best_pid.d)
                state.rollback_count += 1
                parser.clear()
                bridge.flush_input()
                time.sleep(0.4)
                continue

            if is_good_enough(metrics):
                state.stable_rounds += 1
            else:
                state.stable_rounds = 0

            if state.stable_rounds >= args.stable_rounds:
                print(f"连续 {state.stable_rounds} 轮达到可用标准，提前结束。")
                break

            assert state.current_pid is not None
            history_text = build_history_text(state.history)
            suggestion = choose_suggestion(
                args=args,
                target=args.target,
                current_pid=state.current_pid,
                metrics=metrics,
                history_text=history_text,
            )

            safe_pid, notes = guardrail_pid(state.current_pid, suggestion.pid)
            print(f"建议: {suggestion.action} -> {fmt_pid(safe_pid)}")
            print(f"原因: {suggestion.analysis}")
            if notes:
                print("护栏:", "; ".join(notes))

            history_item = HistoryItem(
                round_index=round_index,
                pid=PID(state.current_pid.p, state.current_pid.i, state.current_pid.d),
                metrics=metrics,
                analysis=suggestion.analysis,
            )
            state.history.append(history_item)
            if len(state.history) > HISTORY_LIMIT:
                state.history = state.history[-HISTORY_LIMIT:]

            if (
                abs(safe_pid.p - state.current_pid.p) < 1e-9
                and abs(safe_pid.i - state.current_pid.i) < 1e-9
                and abs(safe_pid.d - state.current_pid.d) < 1e-9
            ):
                print("本轮建议等于当前参数，不下发。")
                continue

            bridge.set_pid(args.target, safe_pid)
            state.current_pid = PID(safe_pid.p, safe_pid.i, safe_pid.d)
            parser.clear()
            bridge.flush_input()
            time.sleep(0.35)

        print("结束。")
        if state.current_pid is not None:
            print("当前 PID:", fmt_pid(state.current_pid))
        if state.best_pid is not None:
            print(f"最佳 PID: round {state.best_round} {fmt_pid(state.best_pid)}")
        print(f"回退次数: {state.rollback_count}")
        return 0
    except KeyboardInterrupt:
        print("用户中断。")
        return 130
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if bridge is not None:
            try:
                bridge.disarm_autotune_target()
            finally:
                bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
