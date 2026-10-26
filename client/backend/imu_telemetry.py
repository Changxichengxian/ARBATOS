"""解析板载 IMU 遥测；按设备时间与解缠角度计算短窗口零漂。"""

from __future__ import annotations

import math
import re
from collections import deque
from typing import Any, Callable


class ImuTelemetry:
    names = (
        "ms", "state", "id", "error", "samples", "seq", "flags", "temp_mC", "target_mC",
        "roll_mdeg", "pitch_mdeg", "yaw_mdeg", "gx_mdps", "gy_mdps", "gz_mdps",
        "ax_mmps2", "ay_mmps2", "az_mmps2", "bx_udps", "by_udps", "bz_udps", "mapX", "mapY", "mapZ",
    )

    def __init__(self, clock: Callable[[], float], window_seconds: float = 180.0, max_records: int = 1800):
        self.clock = clock
        self.window = max(1.0, window_seconds)
        self.max_records = max(20, max_records)
        self.reset()

    def reset(self):
        self.last: dict[str, Any] | None = None
        self.history: deque[dict[str, Any]] = deque(maxlen=self.max_records)
        self.extended_ms: int | None = None
        self.unwrap: dict[str, int] = {}
        self.accepted = self.dropped = self.invalid = self.restarts = 0

    def reset_statistics(self):
        # 清统计不会伪装成设备重启，也不丢掉用于校准应答的最近状态。
        self.history.clear()
        self.unwrap.clear()
        self.dropped = self.invalid = 0

    @staticmethod
    def forward(new: int, old: int) -> int | None:
        delta = (new - old) & 0xFFFFFFFF
        return delta if delta < 0x80000000 else None

    @staticmethod
    def valid(data: dict[str, Any]) -> bool:
        if not all(0 <= data[key] <= 0xFFFFFFFF for key in ("ms", "seq", "id")):
            return False
        if not (0 <= data["state"] <= 6 and 0 <= data["flags"] <= 63 and
                0 <= data["samples"] <= 30000 and -(2**31) <= data["error"] < 2**31):
            return False
        limits = {
            "temp_mC": 150000, "target_mC": 150000,
            "roll_mdeg": 360000, "pitch_mdeg": 360000, "yaw_mdeg": 360000,
            "gx_mdps": 4000000, "gy_mdps": 4000000, "gz_mdps": 4000000,
            "ax_mmps2": 200000, "ay_mmps2": 200000, "az_mmps2": 200000,
            "bx_udps": 6000000, "by_udps": 6000000, "bz_udps": 6000000,
        }
        if any(abs(data[key]) > bound for key, bound in limits.items()):
            return False
        axes = [data[key] for key in ("mapX", "mapY", "mapZ")]
        if sorted(abs(axis) for axis in axes) != [1, 2, 3]:
            return False
        inversions = sum(abs(axes[i]) > abs(axes[j]) for i in range(3) for j in range(i + 1, 3))
        return (-1 if inversions % 2 else 1) * math.prod(1 if axis > 0 else -1 for axis in axes) == 1

    def feed(self, text: str, received: float) -> bool:
        if not text.startswith("@IMU2,"):
            return False
        parts = text.strip().split(",")
        if len(parts) != 26 or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", parts[1]):
            self.invalid += 1
            return False
        # 整数先限定文本长度，避免畸形长数字转浮点或超量计算。
        if any(not re.fullmatch(r"-?[0-9]{1,11}", item) for item in parts[2:]):
            self.invalid += 1
            return False
        data: dict[str, Any] = dict(zip(self.names, map(int, parts[2:])))
        data["target"] = parts[1]
        if not self.valid(data) or not math.isfinite(received):
            self.invalid += 1
            return False

        previous = self.last
        advance = self.forward(data["ms"], previous["ms"]) if previous else None
        reboot = bool(previous and data["target"] == previous["target"] and advance is None)
        if reboot:
            self.restarts += 1
        changed = bool(previous and (
            data["target"] != previous["target"] or data["seq"] != previous["seq"] or
            (data["flags"] & 17) != (previous["flags"] & 17) or
            any(data[key] != previous[key] for key in ("mapX", "mapY", "mapZ")) or
            (data["state"] in (1, 2, 3)) != (previous["state"] in (1, 2, 3))))
        if advance == 0 and not changed:
            return False
        if advance is None or changed:
            self.history.clear()
            self.unwrap.clear()
            self.extended_ms = data["ms"]
        else:
            self.extended_ms += advance
            if advance >= 250:
                # 固件约 100 ms 一帧；这里只有间隙次数，没有可推导的精确丢包数。
                self.dropped += 1
            if advance >= 1000:
                self.history.clear()
                self.unwrap.clear()
        data["deviceMs"] = self.extended_ms
        data["receivedAt"] = received
        for axis in ("roll", "pitch", "yaw"):
            raw = data[f"{axis}_mdeg"]
            old = self.unwrap.get(axis)
            unwrapped = raw if old is None else old + (raw - old + 180000) % 360000 - 180000
            self.unwrap[axis] = unwrapped
            data[f"{axis}Unwrapped"] = unwrapped
        self.last = data
        self.accepted += 1
        if data["flags"] & 1 and data["state"] not in (1, 2, 3):
            self.history.append(data)
        while self.history and data["deviceMs"] - self.history[0]["deviceMs"] > self.window * 1000:
            self.history.popleft()
        return True

    def _observation(self) -> dict[str, Any] | None:
        if len(self.history) < 2:
            return None
        first = self.history[0]
        times = [(row["deviceMs"] - first["deviceMs"]) / 1000 for row in self.history]
        elapsed = times[-1]
        if elapsed <= 0:
            return None
        mean_time = sum(times) / len(times)
        variance = sum((value - mean_time) ** 2 for value in times)
        if variance <= 0:
            return None
        drift = {}
        for axis in ("roll", "pitch", "yaw"):
            values = [row[f"{axis}Unwrapped"] / 1000 for row in self.history]
            mean_value = sum(values) / len(values)
            drift[axis] = sum((t - mean_time) * (v - mean_value) for t, v in zip(times, values)) / variance * 60
        return {
            "seconds": elapsed,
            "angleDriftDegPerMin": drift,
            "gyroMeanDegPerSec": {
                axis: sum(row[f"g{axis}_mdps"] for row in self.history) / len(self.history) / 1000
                for axis in ("x", "y", "z")
            },
        }

    def snapshot(self) -> dict[str, Any]:
        last = dict(self.last) if self.last else None
        age = self.clock() - last["receivedAt"] if last else 0
        fresh = bool(last and 0 <= age <= 3)
        healthy = bool(last and fresh and last["flags"] & 1)
        observing = bool(healthy and last["state"] not in (1, 2, 3))
        return {
            "last": last, "fresh": fresh, "healthy": healthy,
            "safeToStart": bool(healthy and last["flags"] & 2 and last["state"] not in (1, 2, 3)),
            "observation": self._observation() if observing else None,
            "accepted": self.accepted, "dropped": self.dropped, "invalid": self.invalid, "restarts": self.restarts,
        }
