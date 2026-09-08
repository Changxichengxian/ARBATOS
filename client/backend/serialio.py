"""桌面客户端串口读写服务。默认只枚举，不会自动打开或发送。"""

from __future__ import annotations

import math
import re
import struct
import threading
import time
from collections import deque
from typing import Any, Callable, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - 部署缺包时给出可读错误
    serial = None
    list_ports = None


_COM = re.compile(r"COM[1-9][0-9]*\Z", re.IGNORECASE)
_FORMATS = {"text", "csv", "justfloat"}
_TAIL = b"\x00\x00\x80\x7f"


class SerialError(ValueError):
    """串口参数、连接或协议错误。"""


class SerialService:
    def __init__(
        self,
        serial_factory: Callable[..., Any] | None = None,
        port_provider: Callable[[], Iterable[Any]] | None = None,
        *,
        max_records: int = 4000,
        max_line_bytes: int = 4096,
        max_send_bytes: int = 1024,
        max_sends_per_second: int = 20,
    ) -> None:
        self._serial_factory = serial_factory or (serial.Serial if serial is not None else None)
        self._port_provider = port_provider or (list_ports.comports if list_ports is not None else None)
        self._max_records = max(20, int(max_records))
        self._max_line_bytes = max(64, int(max_line_bytes))
        self._max_send_bytes = max(1, int(max_send_bytes))
        self._max_sends_per_second = max(1, int(max_sends_per_second))
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._serial: Any = None
        self._reader: threading.Thread | None = None
        self._opening = False
        self._closed = False
        self._generation = 0
        self._connected = False
        self._port: str | None = None
        self._baud: int | None = None
        self._format: str | None = None
        self._channels = 1
        self._rx_bytes = 0
        self._tx_bytes = 0
        self._error: str | None = None
        self._seq = 0
        self._lines: deque[dict[str, Any]] = deque(maxlen=self._max_records)
        self._samples: deque[dict[str, Any]] = deque(maxlen=self._max_records)
        self._dropped_through = 0
        self._buffer = bytearray()
        self._send_times: deque[float] = deque()

    def dispatch(self, method: str, params: dict[str, Any] | None = None) -> Any:
        params = params or {}
        if method == "serial.ports":
            return self._ports()
        if method == "serial.open":
            return self._open(params)
        if method == "serial.close":
            return self._close()
        if method == "serial.status":
            return self._status()
        if method == "serial.read":
            return self._read(params)
        if method == "serial.send":
            return self._send(params)
        raise SerialError(f"不支持的串口方法：{method}")

    def shutdown(self) -> None:
        with self._lock:
            self._closed = True
        self._close()

    def _ports(self) -> list[dict[str, str]]:
        if self._port_provider is None:
            raise SerialError("缺少 pyserial，无法枚举串口")
        result: list[dict[str, str]] = []
        try:
            ports = self._port_provider()
        except Exception as exc:
            raise SerialError(f"枚举串口失败：{exc}") from exc
        for item in ports:
            device = str(getattr(item, "device", ""))
            if not _COM.fullmatch(device):
                continue
            result.append({
                "device": device,
                "description": str(getattr(item, "description", "")),
                "hwid": str(getattr(item, "hwid", "")),
            })
        return result

    def _open(self, params: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._closed:
                raise SerialError("串口服务已经关闭")
        port = params.get("port")
        baud = params.get("baud")
        data_format = params.get("format")
        channels = params.get("channels")
        if not isinstance(port, str) or not _COM.fullmatch(port):
            raise SerialError("串口号无效")
        available = {item["device"].upper() for item in self._ports()}
        if port.upper() not in available:
            raise SerialError("只能打开当前实际枚举到的 COM 口")
        if isinstance(baud, bool) or not isinstance(baud, int) or not 300 <= baud <= 4_000_000:
            raise SerialError("波特率必须是 300 到 4000000 的整数")
        if data_format not in _FORMATS:
            raise SerialError("格式只能是 text、csv 或 justfloat")
        if isinstance(channels, bool) or not isinstance(channels, int) or not 1 <= channels <= 16:
            raise SerialError("通道数必须是 1 到 16 的整数")
        if self._serial_factory is None:
            raise SerialError("缺少 pyserial，无法打开串口")
        with self._lock:
            if self._closed:
                raise SerialError("串口服务已经关闭")
            if self._connected or self._opening:
                raise SerialError("已有串口连接，请先关闭")
            old_stop, old_handle, old_reader = self._stop, self._serial, self._reader
            old_stop.set()
            self._serial = None
            self._reader = None
            self._connected = False
            self._opening = True
            self._generation += 1
            generation = self._generation
            self._reset_session(port, baud, data_format, channels)
            stop = self._stop
        self._finish_handle(old_handle, old_reader)
        try:
            handle = self._serial_factory(
                port=port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=0.5,
            )
        except Exception as exc:
            with self._lock:
                if self._generation == generation:
                    self._error = str(exc)
                    self._opening = False
            raise SerialError(f"打开串口失败：{exc}") from exc
        with self._lock:
            cancelled = self._closed or self._generation != generation or not self._opening
            if not cancelled:
                self._serial = handle
                self._connected = True
                self._opening = False
                self._reader = threading.Thread(
                    target=self._reader_loop,
                    args=(generation, handle, stop),
                    name="arbatos-serial-reader",
                    daemon=False,
                )
                self._reader.start()
                return self._status_locked()
        try:
            handle.close()
        except Exception:
            pass
        raise SerialError("串口打开已取消，服务正在关闭")

    def _reset_session(self, port: str, baud: int, data_format: str, channels: int) -> None:
        self._stop = threading.Event()
        self._port = port
        self._baud = baud
        self._format = data_format
        self._channels = channels
        self._rx_bytes = 0
        self._tx_bytes = 0
        self._error = None
        self._seq = 0
        self._lines.clear()
        self._samples.clear()
        self._dropped_through = 0
        self._buffer.clear()
        self._send_times.clear()

    def _close(self) -> dict[str, Any]:
        with self._lock:
            stop, handle, reader = self._stop, self._serial, self._reader
            stop.set()
            self._generation += 1
            self._opening = False
            self._connected = False
            self._serial = None
            self._reader = None
        error = self._finish_handle(handle, reader)
        with self._lock:
            if error:
                self._error = error
            return self._status_locked()

    @staticmethod
    def _finish_handle(handle: Any, reader: threading.Thread | None) -> str | None:
        error = None
        if handle is not None:
            try:
                handle.close()
            except Exception as exc:
                error = str(exc)
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=2.0)
        return error

    def _status(self) -> dict[str, Any]:
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> dict[str, Any]:
        return {
            "connected": self._connected,
            "port": self._port,
            "baud": self._baud,
            "format": self._format,
            "rxBytes": self._rx_bytes,
            "txBytes": self._tx_bytes,
            "error": self._error,
        }

    def _read(self, params: dict[str, Any]) -> dict[str, Any]:
        after = params.get("after", 0)
        if isinstance(after, bool) or not isinstance(after, int) or after < 0:
            raise SerialError("after 必须是非负整数")
        with self._lock:
            return {
                "status": self._status_locked(),
                "lines": [dict(item) for item in self._lines if item["seq"] > after],
                "samples": [dict(item) for item in self._samples if item["seq"] > after],
                "cursor": self._seq,
                "dropped": after < self._dropped_through,
            }

    def _send(self, params: dict[str, Any]) -> dict[str, Any]:
        text = params.get("text")
        ending = params.get("lineEnding")
        if not isinstance(text, str):
            raise SerialError("发送内容必须是文本")
        endings = {"LF": b"\n", "CRLF": b"\r\n", "none": b""}
        if ending not in endings:
            raise SerialError("换行方式只能是 LF、CRLF 或 none")
        payload = text.encode("utf-8") + endings[ending]
        if len(payload) > self._max_send_bytes:
            raise SerialError(f"单次发送不能超过 {self._max_send_bytes} 字节")
        now = time.monotonic()
        with self._lock:
            if self._closed:
                raise SerialError("串口服务已经关闭")
            while self._send_times and now - self._send_times[0] >= 1.0:
                self._send_times.popleft()
            if len(self._send_times) >= self._max_sends_per_second:
                raise SerialError("发送过快，请稍后再试")
            if not self._connected or self._serial is None:
                raise SerialError("串口未连接")
            handle = self._serial
            generation = self._generation
            self._send_times.append(now)
        try:
            with self._write_lock:
                written = handle.write(payload)
                if written is None:
                    written = len(payload)
                if int(written) != len(payload):
                    raise OSError("串口只写入了部分数据")
        except Exception as exc:
            self._disconnect_error(exc, generation, handle)
            raise SerialError(f"串口发送失败：{exc}") from exc
        with self._lock:
            if self._generation != generation or self._serial is not handle or not self._connected:
                raise SerialError("发送期间串口已关闭")
            self._tx_bytes += len(payload)
            return self._status_locked()

    def _reader_loop(self, generation: int, handle: Any, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                chunk = handle.read(4096)
            except Exception as exc:
                if not stop.is_set():
                    self._disconnect_error(exc, generation, handle)
                break
            if not chunk:
                stop.wait(0.005)
                continue
            if not isinstance(chunk, (bytes, bytearray)):
                self._disconnect_error(TypeError("串口返回了非字节数据"), generation, handle)
                break
            received = time.time()
            with self._lock:
                if self._generation != generation or self._serial is not handle or stop.is_set():
                    break
                self._rx_bytes += len(chunk)
                self._feed_locked(bytes(chunk), received)

    def _disconnect_error(self, exc: Exception, generation: int, handle: Any) -> None:
        with self._lock:
            if self._generation == generation and self._serial is handle:
                self._error = str(exc)
                self._connected = False
                self._stop.set()
        try:
            handle.close()
        except Exception:
            pass

    def _feed_locked(self, chunk: bytes, received: float) -> None:
        self._buffer.extend(chunk)
        if self._format == "justfloat":
            self._feed_justfloat_locked(received)
        else:
            self._feed_text_locked(received)

    def _feed_text_locked(self, received: float) -> None:
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) > self._max_line_bytes:
                    del self._buffer[: len(self._buffer) - self._max_line_bytes]
                    self._error = "收到的文本行过长，已丢弃前部数据"
                return
            raw = bytes(self._buffer[:newline]).rstrip(b"\r")
            del self._buffer[: newline + 1]
            if len(raw) > self._max_line_bytes:
                raw = raw[-self._max_line_bytes :]
                self._error = "收到的文本行过长，已截断"
            text = raw.decode("utf-8", errors="replace")
            self._append_record_locked(self._lines, {"time": received, "text": text})
            if self._format == "csv":
                self._parse_csv_locked(text, received)

    def _parse_csv_locked(self, text: str, received: float) -> None:
        parts = [part.strip() for part in text.split(",")]
        if len(parts) not in {self._channels, self._channels + 1}:
            return
        try:
            numbers = [float(part) for part in parts]
        except ValueError:
            return
        if not all(math.isfinite(value) for value in numbers):
            return
        sample_time = received
        if len(numbers) == self._channels + 1:
            sample_time = numbers[0]
            numbers = numbers[1:]
        self._append_record_locked(self._samples, {"time": sample_time, "values": numbers})

    def _feed_justfloat_locked(self, received: float) -> None:
        payload_bytes = self._channels * 4
        frame_bytes = payload_bytes + len(_TAIL)
        while True:
            tail = self._buffer.find(_TAIL)
            if tail < 0:
                if len(self._buffer) > frame_bytes * 4:
                    del self._buffer[: len(self._buffer) - (frame_bytes - 1)]
                    self._error = "JustFloat 数据过长且没有帧尾，已重新同步"
                return
            start = tail - payload_bytes
            if start < 0:
                del self._buffer[: tail + len(_TAIL)]
                continue
            if start > 0:
                del self._buffer[:start]
                tail -= start
            if len(self._buffer) < frame_bytes:
                return
            payload = bytes(self._buffer[:payload_bytes])
            del self._buffer[:frame_bytes]
            try:
                values = list(struct.unpack(f"<{self._channels}f", payload))
            except struct.error:
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            self._append_record_locked(self._samples, {"time": received, "values": values})

    def _append_record_locked(self, target: deque[dict[str, Any]], item: dict[str, Any]) -> None:
        if len(target) == target.maxlen and target:
            self._dropped_through = max(self._dropped_through, int(target[0]["seq"]))
        self._seq += 1
        item["seq"] = self._seq
        target.append(item)
