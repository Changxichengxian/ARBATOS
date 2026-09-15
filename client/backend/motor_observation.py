"""步兵车 UART 诊断中的被动电机观测快照。"""

from __future__ import annotations

import math
import re
from typing import Any, Callable


_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_MOTOR = re.compile(r"(?:^|\s)MOTOR\s+(?P<name>[^\s]+)(?P<fields>(?:\s+[^\s=]+=[^\s]+)+)\s*$")
_LIFECYCLE = re.compile(r"(?:^|\s)INFANTRY-TIAN-A(?P<fields>(?:\s+[^\s=]+=[^\s]+)+)\s*$")
_INFANTRY_HEADER = re.compile(r"(?:^|\s)INFANTRY-TIAN-A(?:\s|$)")
_MOTOR_UNAVAILABLE = re.compile(r"(?:^|\s)MOTOR\s+snapshot=unavailable(?:\s|$)")
_FIELD = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[^\s]+)")


class MotorObservation:
    """只解析已到达的 UART 文本；从不打开串口、发送命令或读取配置。"""

    source = "infantry_diag_motor"
    _HOST_FRESH_MS = 3_000
    _FEEDBACK_FRESH_MS = 200

    def __init__(self, clock: Callable[[], float]) -> None:
        self._clock = clock
        self._generation = 0
        self.reset()

    def reset(self) -> None:
        self._generation += 1
        self._motors: dict[str, dict[str, Any]] = {}
        self._lifecycle: dict[str, Any] | None = None

    def feed(self, text: str, received_at: float | None = None) -> None:
        received = self._received_at(received_at)
        if received is None:
            return
        cleaned = _ANSI.sub("", text)
        if "booting" in cleaned.lower():
            self.reset()
            return
        if _INFANTRY_HEADER.search(cleaned) is not None:
            lifecycle = _LIFECYCLE.search(cleaned)
            if lifecycle is None or not self._feed_lifecycle(lifecycle.group("fields"), received):
                # 诊断头已出现却没有可信安全态，不能保留最近一帧的可手转条件。
                self.reset()
            return
        if _MOTOR_UNAVAILABLE.search(cleaned) is not None:
            # 固件已经明确无法取得反馈快照，旧的在线和转速均不可继续使用。
            self.reset()
            return
        motor = _MOTOR.search(cleaned)
        if motor is not None:
            self._feed_motor(motor.group("name"), motor.group("fields"), received)

    def snapshot(self) -> dict[str, Any]:
        # 没有来源数据时不读取时钟，避免让只含接收机遥测的会话产生额外副作用。
        now = self._now() if self._motors or self._lifecycle is not None else 0.0
        motors = [self._copy_motor(item, now) for item in self._motors.values()]
        motors.sort(key=lambda item: (item["configuredBus"], item["configuredRouteId"], item["name"]))
        safety = self._copy_safety(now)
        hand_turn_candidate_ready = bool(
            safety is not None and safety["fresh"] and not safety["outputAllowed"]
            and any(motor["fresh"] for motor in motors)
        )
        return {
            "generation": self._generation,
            "available": bool(motors),
            "source": self.source if motors else None,
            "motors": motors,
            "safety": safety,
            # 诊断中没有编码器 ECD/绝对角度；只能依据新鲜 rpm 观察手转瞬间。
            "encoderAvailable": False,
            "motionEvidence": "rpm",
            # 仅说明数据和诊断输出许可已满足候选条件，不能证明物理接线或转向。
            "handTurnCandidateReady": hand_turn_candidate_ready,
        }

    def _feed_motor(self, name: str, raw_fields: str, received: float) -> None:
        fields = self._fields(raw_fields)
        if fields is None:
            return
        required = ("id", "bus", "cfg_fb", "online", "last_fb", "count", "age", "rpm", "fb_cur",
                    "cmd", "mode", "cmd_cur")
        if any(key not in fields for key in required):
            return
        values = {key: self._number(fields[key]) for key in required}
        if any(value is None for value in values.values()):
            return
        if not (0 <= values["id"] <= 255 and 1 <= values["bus"] <= 255
                and values["online"] in (0, 1) and values["cmd"] in (0, 1)
                and 0 <= values["cfg_fb"] <= 0x1FFFFFFF and 0 <= values["last_fb"] <= 0x1FFFFFFF
                and 0 <= values["count"] <= 0xFFFFFFFF and 0 <= values["age"] <= 0xFFFFFFFF):
            return
        self._motors[name] = {
            "name": name,
            # id 是 route->motorId 的固件内部实例编号，bus 是固件路由总线；
            # 两者都不是从反馈帧自动识别出的电机节点号或总线探测结果。
            "configuredRouteId": values["id"],
            "configuredBus": values["bus"],
            "configuredFeedbackId": values["cfg_fb"],
            # last_fb 是固件实际记录的最近接收反馈帧标识。
            "observedFeedbackId": values["last_fb"],
            "online": bool(values["online"]),
            "feedbackCount": values["count"],
            "diagnosticFeedbackAgeMs": values["age"],
            "rpm": values["rpm"],
            "feedbackCurrent": values["fb_cur"],
            "commandActive": bool(values["cmd"]),
            "commandMode": values["mode"],
            "commandCurrent": values["cmd_cur"],
            "receivedAt": received,
        }

    def _feed_lifecycle(self, raw_fields: str, received: float) -> bool:
        fields = self._fields(raw_fields)
        if fields is None or "output" not in fields or "uptime" not in fields:
            return False
        output = self._number(fields["output"])
        uptime = self._number(fields["uptime"])
        if output not in (0, 1) or uptime is None or not 0 <= uptime <= 0xFFFFFFFF:
            return False
        if self._lifecycle is not None and uptime < self._lifecycle["hardwareUptimeMs"]:
            # 诊断源重启后，旧会话的 ID、速度和安全态都不能继续使用。
            self.reset()
        elif self._lifecycle is not None and not self._lifecycle["outputAllowed"] and output == 1:
            # 即使前端没轮询到短暂的输出允许，也必须丢弃此前的手转候选。
            self.reset()
        self._lifecycle = {
            "outputAllowed": bool(output),
            "hardwareUptimeMs": uptime,
            "receivedAt": received,
        }
        return True

    @staticmethod
    def _fields(text: str) -> dict[str, str] | None:
        fields = {match.group("key"): match.group("value") for match in _FIELD.finditer(text)}
        return fields or None

    @staticmethod
    def _number(text: str) -> int | None:
        try:
            return int(text, 0)
        except ValueError:
            return None

    def _copy_motor(self, motor: dict[str, Any], now: float) -> dict[str, Any]:
        result = dict(motor)
        age_ms = self._age_ms(now, motor["receivedAt"])
        result["ageMs"] = age_ms
        result["fresh"] = bool(motor["online"] and age_ms <= self._HOST_FRESH_MS
                               and motor["diagnosticFeedbackAgeMs"] <= self._FEEDBACK_FRESH_MS)
        return result

    def _copy_safety(self, now: float) -> dict[str, Any] | None:
        if self._lifecycle is None:
            return None
        result = dict(self._lifecycle)
        result["ageMs"] = self._age_ms(now, result["receivedAt"])
        result["fresh"] = result["ageMs"] <= self._HOST_FRESH_MS
        return result

    def _received_at(self, received_at: float | None) -> float | None:
        value = self._now() if received_at is None else received_at
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return None
        return float(value)

    def _now(self) -> float:
        value = self._clock()
        return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else 0.0

    @staticmethod
    def _age_ms(now: float, received_at: float) -> int:
        return max(0, int(round((now - received_at) * 1000)))
