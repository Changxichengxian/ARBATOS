from __future__ import annotations

import queue
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serialio import SerialError, SerialService


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.input = queue.Queue()
        self.writes = []
        self.closed = False

    def read(self, size):
        if self.closed:
            return b""
        try:
            item = self.input.get(timeout=0.02)
        except queue.Empty:
            return b""
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, payload):
        if self.closed:
            raise OSError("closed")
        self.writes.append(payload)
        return len(payload)

    def close(self):
        self.closed = True


class Factory:
    def __init__(self):
        self.instances = []

    def __call__(self, **kwargs):
        instance = FakeSerial(**kwargs)
        self.instances.append(instance)
        return instance


class BlockingFactory:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.instance = None

    def __call__(self, **kwargs):
        self.entered.set()
        self.release.wait(1)
        self.instance = FakeSerial(**kwargs)
        return self.instance


def ports():
    return [
        SimpleNamespace(device="COM7", description="USB UART", hwid="USB VID:PID=1234:5678"),
        SimpleNamespace(device="loop://", description="测试端口", hwid="n/a"),
    ]


def wait_cursor(service: SerialService, cursor: int = 1) -> dict:
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        result = service.dispatch("serial.read", {})
        if result["cursor"] >= cursor or result["status"]["error"]:
            return result
        time.sleep(0.005)
    raise AssertionError("串口读取线程没有产生数据")


class SerialServiceTest(unittest.TestCase):
    def test_enumerates_only_com_and_does_not_open_by_default(self):
        factory = Factory()
        service = SerialService(factory, ports)
        self.assertEqual(service.dispatch("serial.ports"), [{
            "device": "COM7", "description": "USB UART", "hwid": "USB VID:PID=1234:5678"
        }])
        self.assertFalse(service.dispatch("serial.status")["connected"])
        self.assertEqual(factory.instances, [])

    def test_text_chunks_and_send_are_bounded_and_explicit(self):
        factory = Factory()
        service = SerialService(factory, ports, max_send_bytes=8, max_sends_per_second=2)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"hel")
        port.input.put(b"lo\r\nworld\n")
        result = wait_cursor(service, 2)
        self.assertEqual([line["text"] for line in result["lines"]], ["hello", "world"])
        self.assertEqual(result["status"]["rxBytes"], 13)
        sent = service.dispatch("serial.send", {"text": "go", "lineEnding": "CRLF"})
        self.assertEqual(port.writes, [b"go\r\n"])
        self.assertEqual(sent["txBytes"], 4)
        with self.assertRaisesRegex(SerialError, "超过"):
            service.dispatch("serial.send", {"text": "12345678", "lineEnding": "LF"})
        service.shutdown()

    def test_csv_uses_optional_timestamp_and_rejects_nonfinite(self):
        factory = Factory()
        service = SerialService(factory, ports)
        service.dispatch("serial.open", {"port": "COM7", "baud": 230400, "format": "csv", "channels": 2})
        factory.instances[-1].input.put(b"12.5,1,2\n3,4\nnan,8\nbad\n")
        result = wait_cursor(service, 6)
        self.assertEqual(len(result["samples"]), 2)
        self.assertEqual(result["samples"][0]["time"], 12.5)
        self.assertEqual(result["samples"][0]["values"], [1.0, 2.0])
        self.assertEqual(result["samples"][1]["values"], [3.0, 4.0])
        self.assertEqual(len(result["lines"]), 4)
        service.shutdown()

    def test_justfloat_handles_split_frames_noise_and_nonfinite_values(self):
        factory = Factory()
        service = SerialService(factory, ports)
        service.dispatch("serial.open", {"port": "COM7", "baud": 230400, "format": "justfloat", "channels": 2})
        port = factory.instances[-1]
        valid = struct.pack("<2f", 1.25, -2.5) + b"\x00\x00\x80\x7f"
        invalid = struct.pack("<2f", float("nan"), 9.0) + b"\x00\x00\x80\x7f"
        port.input.put(b"noise" + valid[:5])
        port.input.put(valid[5:] + invalid)
        result = wait_cursor(service)
        self.assertEqual(len(result["samples"]), 1)
        for actual, expected in zip(result["samples"][0]["values"], [1.25, -2.5]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result["lines"], [])
        service.shutdown()

    def test_close_and_reopen_clear_old_records_and_counters(self):
        factory = Factory()
        service = SerialService(factory, ports)
        params = {"port": "COM7", "baud": 115200, "format": "text", "channels": 1}
        service.dispatch("serial.open", params)
        factory.instances[-1].input.put(b"old\n")
        self.assertEqual(wait_cursor(service)["lines"][0]["text"], "old")
        service.dispatch("serial.close")
        service.dispatch("serial.open", params)
        result = service.dispatch("serial.read", {})
        self.assertEqual(result["cursor"], 0)
        self.assertEqual(result["lines"], [])
        self.assertEqual(result["status"]["rxBytes"], 0)
        service.shutdown()

    def test_disconnect_reopen_and_concurrent_close(self):
        factory = Factory()
        service = SerialService(factory, ports)
        params = {"port": "COM7", "baud": 115200, "format": "text", "channels": 1}
        service.dispatch("serial.open", params)
        factory.instances[-1].input.put(OSError("device removed"))
        deadline = time.monotonic() + 1
        while service.dispatch("serial.status")["connected"] and time.monotonic() < deadline:
            time.sleep(0.005)
        status = service.dispatch("serial.status")
        self.assertFalse(status["connected"])
        self.assertIn("device removed", status["error"])
        self.assertTrue(service.dispatch("serial.open", params)["connected"])
        self.assertEqual(len(factory.instances), 2)
        threads = [threading.Thread(target=lambda: service.dispatch("serial.close")) for _ in range(3)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def test_rejects_virtual_port_and_rate_limits_send(self):
        factory = Factory()
        service = SerialService(factory, ports, max_sends_per_second=1)
        with self.assertRaisesRegex(SerialError, "串口号无效"):
            service.dispatch("serial.open", {"port": "loop://", "baud": 115200,
                                               "format": "text", "channels": 1})
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        service.dispatch("serial.send", {"text": "a", "lineEnding": "none"})
        with self.assertRaisesRegex(SerialError, "发送过快"):
            service.dispatch("serial.send", {"text": "b", "lineEnding": "none"})
        service.shutdown()

    def test_shutdown_during_open_closes_late_handle_and_is_permanent(self):
        factory = BlockingFactory()
        service = SerialService(factory, ports)
        params = {"port": "COM7", "baud": 115200, "format": "text", "channels": 1}
        errors = []

        def opening():
            try:
                service.dispatch("serial.open", params)
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=opening)
        thread.start()
        self.assertTrue(factory.entered.wait(1))
        service.shutdown()
        factory.release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertTrue(factory.instance.closed)
        self.assertFalse(service.dispatch("serial.status")["connected"])
        self.assertIsInstance(errors[0], SerialError)
        with self.assertRaisesRegex(SerialError, "已经关闭"):
            service.dispatch("serial.open", params)
        with self.assertRaisesRegex(SerialError, "已经关闭"):
            service.dispatch("serial.send", {"text": "x", "lineEnding": "none"})


if __name__ == "__main__":
    unittest.main()
