"""接收机遥测文本帧的有界解析与会话快照。"""

from __future__ import annotations

import math
import re
from typing import Any, Callable


_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_MARKER = re.compile(r"@(RC|ELRS|UART|INPUT),([^\r\n]*)")
_UINT32 = 0xFFFFFFFF


class ReceiverTelemetry:
    """只保存当前串口会话内经完整校验的接收机遥测。"""

    def __init__(self, clock: Callable[[], float]) -> None:
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        self._channels: list[int] | None = None
        self._rc: dict[str, Any] | None = None
        self._link: dict[str, Any] | None = None
        self._uart: dict[str, Any] | None = None
        self._input: dict[str, Any] | None = None
        self._last_ms: dict[str, int] = {}
        self._rc_reboot_candidate: tuple[int, int] | None = None

    def snapshot(self) -> dict[str, Any]:
        return {
            "channels": list(self._channels) if self._channels is not None else None,
            "rc": dict(self._rc) if self._rc is not None else None,
            "link": dict(self._link) if self._link is not None else None,
            "uart": dict(self._uart) if self._uart is not None else None,
            "input": self._copy_input(),
        }

    def _copy_input(self) -> dict[str, Any] | None:
        if self._input is None:
            return None
        result = dict(self._input)
        result["channels"] = list(result["channels"])
        result["switches"] = list(result["switches"])
        return result

    def feed(self, text: str, received_at: float | None = None) -> None:
        cleaned = _ANSI.sub("", text)
        if "booting" in cleaned.lower():
            self.reset()
            return
        marker = _MARKER.search(cleaned)
        if marker is None:
            return
        tag, payload = marker.groups()
        values = self._integers(payload)
        if values is None:
            return
        received = self._clock() if received_at is None else received_at
        if not isinstance(received, (int, float)) or not math.isfinite(received):
            return
        parser = {
            "RC": self._feed_rc,
            "ELRS": self._feed_link,
            "UART": self._feed_uart,
            "INPUT": self._feed_input,
        }[tag]
        parser(values, float(received))

    @staticmethod
    def _integers(payload: str) -> list[int] | None:
        parts = payload.split(",")
        if not parts or any(not part or not re.fullmatch(r"[+-]?\d+", part.strip()) for part in parts):
            return None
        try:
            return [int(part.strip()) for part in parts]
        except ValueError:
            return None

    def _begin(self, tag: str, values: list[int], count: int) -> bool:
        if len(values) != count or not self._uint(values[0]):
            return False
        last = self._last_ms.get(tag)
        if last is not None and values[0] < last:
            self.reset()
        self._last_ms[tag] = values[0]
        return True

    @staticmethod
    def _uint(value: int) -> bool:
        return 0 <= value <= _UINT32

    @staticmethod
    def _bit(value: int) -> bool:
        return value in (0, 1)

    def _feed_rc(self, values: list[int], received: float) -> None:
        if len(values) != 20:
            return
        ms, frame_ms, valid, sequence, *channels = values
        if not (self._uint(ms) and self._uint(frame_ms) and self._bit(valid) and self._uint(sequence)
                and all(0 <= channel <= 2_047 for channel in channels)):
            return
        last_ms = self._last_ms.get("RC")
        if last_ms is not None and ms < last_ms:
            # UART 文本有序到达时，设备时间回退代表固件重启。首条新会话文本
            # 可能仍是重启前滞留的有效帧，先将其作为失效候选，等 ms 与序号都
            # 前进后才恢复有效输入。
            self.reset()
            self._last_ms["RC"] = ms
            self._rc_reboot_candidate = (ms, sequence)
            self._rc = {
                "ms": ms, "frameMs": frame_ms, "valid": False, "sequence": sequence,
                "receivedAt": received,
            }
            return
        if self._rc_reboot_candidate is not None:
            candidate_ms, candidate_sequence = self._rc_reboot_candidate
            if ms <= candidate_ms or not self._sequence_newer(sequence, candidate_sequence):
                return
            self._rc_reboot_candidate = None
        if self._rc is not None and sequence == self._rc["sequence"]:
            # 同一帧号的失效状态可能由固件在帧超时后主动上报。保留最后一帧
            # 的到达时刻，避免重复文本让页面误判为有新的遥控输入。
            if self._rc["valid"] and not valid:
                self._rc.update({"ms": ms, "frameMs": frame_ms, "valid": False})
            return
        if (self._rc is not None and ms >= self._last_ms.get("RC", ms)
                and not self._sequence_newer(sequence, self._rc["sequence"])):
            return
        if not self._begin("RC", values, 20):
            return
        # 固件用序号零表示尚未收过原始帧；初始化数组不应显示为测得的通道零值。
        self._channels = None if sequence == 0 and frame_ms == 0 and not valid else channels
        self._rc = {
            "ms": ms, "frameMs": frame_ms, "valid": bool(valid), "sequence": sequence,
            "receivedAt": received,
        }

    @staticmethod
    def _sequence_newer(candidate: int, current: int) -> bool:
        """按 uint32 序号判断新帧，允许计数器从最大值回绕到零。"""
        delta = (candidate - current) & _UINT32
        return 0 < delta <= (_UINT32 >> 1)

    def _feed_link(self, values: list[int], received: float) -> None:
        if len(values) != 12:
            return
        (ms, port_active, state, lq, rc_frames, stats_frames, published, crc_errors,
         length_errors, overflows, restarts, stats_timeouts) = values
        if not (self._uint(ms) and self._bit(port_active) and 0 <= state <= 255 and 0 <= lq <= 100
                and all(self._uint(value) for value in values[4:])):
            return
        if not self._begin("ELRS", values, 12):
            return
        self._link = {
            "ms": ms, "portActive": bool(port_active), "state": state, "lq": lq,
            "rcFrames": rc_frames, "statsFrames": stats_frames, "published": published,
            "crcErrors": crc_errors, "lengthErrors": length_errors, "overflows": overflows,
            "restarts": restarts, "statsTimeouts": stats_timeouts, "receivedAt": received,
        }

    def _feed_uart(self, values: list[int], received: float) -> None:
        if len(values) != 8 or not all(self._uint(value) for value in values):
            return
        if not self._begin("UART", values, 8):
            return
        ms, irq_bytes, errors, overrun, framing, parity, noise, last_error = values
        self._uart = {
            "ms": ms, "irqBytes": irq_bytes, "errors": errors, "overrun": overrun,
            "framing": framing, "parity": parity, "noise": noise, "lastError": last_error,
            "receivedAt": received,
        }

    def _feed_input(self, values: list[int], received: float) -> None:
        if len(values) != 11:
            return
        ms, online, source, active_mask, *rest = values
        channels, switches = rest[:5], rest[5:]
        if not (self._uint(ms) and self._bit(online) and 0 <= source <= 255 and self._uint(active_mask)
                and all(-32_768 <= channel <= 32_767 for channel in channels)
                and all(0 <= switch <= 255 for switch in switches)):
            return
        if not self._begin("INPUT", values, 11):
            return
        self._input = {
            "ms": ms, "online": bool(online), "source": source, "activeMask": active_mask,
            "channels": channels, "switches": switches, "receivedAt": received,
        }
