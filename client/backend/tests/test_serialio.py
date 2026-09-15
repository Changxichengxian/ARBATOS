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
    def test_receiver_initial_snapshot_does_not_fabricate_zero_channels(self):
        factory = Factory()
        service = SerialService(factory, ports)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"@RC,100,0,0,0," + b",".join([b"0"] * 16) + b"\n")
        receiver = wait_cursor(service)["receiver"]
        self.assertIsNone(receiver["channels"])
        self.assertFalse(receiver["rc"]["valid"])
        port.input.put(b"@RC,101,101,0,1," + b",".join([b"0"] * 16) + b"\n")
        receiver = wait_cursor(service, 2)["receiver"]
        self.assertEqual(receiver["channels"], [0] * 16)
        service.shutdown()

    def test_receiver_telemetry_accepts_split_prefixed_frames(self):
        factory = Factory()
        clock = iter([1000.25, 1000.5, 1000.75, 1001.0]).__next__
        service = SerialService(factory, ports, clock=clock)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"\x1b[32m<inf> bench: @RC,100000,90000,1,5,172,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\r\n@ELRS,11,1,2,99,3,4,5,6,7,8,9,10\n@UART,12,13,14,15,16,17,18,19\n@IN")
        port.input.put(b"PUT,13,1,2,3,1000,1500,2000,1200,1800,1,2\n")
        result = wait_cursor(service, 4)
        receiver = result["receiver"]
        self.assertEqual(receiver["channels"], [172, 992, 1811, 0, 2047] + [1024] * 11)
        self.assertEqual(receiver["rc"], {"ms": 100000, "frameMs": 90000, "valid": True, "sequence": 5, "receivedAt": 1000.25})
        self.assertEqual(receiver["link"]["lq"], 99)
        self.assertEqual(receiver["uart"]["lastError"], 19)
        self.assertEqual(receiver["input"]["channels"], [1000, 1500, 2000, 1200, 1800])
        self.assertEqual(receiver["input"]["switches"], [1, 2])
        self.assertEqual(service.dispatch("serial.status")["receiver"], receiver)
        service.shutdown()

    def test_receiver_rejects_bad_frames_and_duplicate_rc_sequence(self):
        factory = Factory()
        service = SerialService(factory, ports, clock=lambda: 1.0)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"@RC,100000,90000,1,8,172,992,1811,0,2047,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n")
        self.assertEqual(wait_cursor(service)["receiver"]["rc"]["receivedAt"], 1.0)
        port.input.put(b"@RC,100001,99999,0,8,1500,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n@RC,100002,1,1,8,1500,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n@RC,100003,1,1,7,1500,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n@ELRS,100004,1,2,101,3,4,5,6,7,8,9,10\n")
        result = wait_cursor(service, 5)
        self.assertEqual(result["receiver"]["channels"][0], 172)
        self.assertEqual(result["receiver"]["rc"]["receivedAt"], 1.0)
        self.assertFalse(result["receiver"]["rc"]["valid"])
        self.assertEqual(result["receiver"]["rc"]["frameMs"], 99999)
        self.assertIsNone(result["receiver"]["link"])
        service.shutdown()

    def test_receiver_clears_on_boot_ms_rollback_and_close(self):
        factory = Factory()
        service = SerialService(factory, ports)
        params = {"port": "COM7", "baud": 115200, "format": "text", "channels": 1}
        service.dispatch("serial.open", params)
        port = factory.instances[-1]
        port.input.put(b"@ELRS,100,1,2,99,3,4,5,6,7,8,9,10\n@UART,100,13,14,15,16,17,18,19\n")
        self.assertIsNotNone(wait_cursor(service, 2)["receiver"]["link"])
        port.input.put(b"*** Booting Zephyr OS\n@RC,1,4,1,1,1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n")
        result = wait_cursor(service, 4)
        self.assertIsNone(result["receiver"]["link"])
        self.assertIsNotNone(result["receiver"]["rc"])
        port.input.put(b"@RC,0,4,1,2,1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015\n")
        result = wait_cursor(service, 5)
        self.assertEqual(result["receiver"]["rc"]["sequence"], 2)
        service.dispatch("serial.close")
        self.assertIsNone(service.dispatch("serial.status")["receiver"]["rc"])
        service.shutdown()

    def test_receiver_accepts_rc_sequence_uint32_wrap(self):
        factory = Factory()
        service = SerialService(factory, ports)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"@RC,10,9,1,4294967294,172,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n@RC,11,10,1,1,173,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n")
        receiver = wait_cursor(service, 2)["receiver"]
        self.assertEqual(receiver["rc"]["sequence"], 1)
        self.assertEqual(receiver["channels"][0], 173)
        service.shutdown()

    def test_receiver_ms_rollback_requires_a_later_frame_before_valid(self):
        factory = Factory()
        service = SerialService(factory, ports)
        service.dispatch("serial.open", {"port": "COM7", "baud": 115200, "format": "text", "channels": 1})
        port = factory.instances[-1]
        port.input.put(b"@RC,100,90,1,8,172,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n@RC,101,91,0,8,172,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n@RC,1,1,1,1,173,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n")
        receiver = wait_cursor(service, 3)["receiver"]
        self.assertFalse(receiver["rc"]["valid"])
        self.assertIsNone(receiver["channels"])
        port.input.put(b"@RC,2,2,1,2,174,992,1811,0,2047,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024\n")
        receiver = wait_cursor(service, 4)["receiver"]
        self.assertTrue(receiver["rc"]["valid"])
        self.assertEqual(receiver["channels"][0], 174)
        service.shutdown()
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
        port = factory.instances[-1]
        port.input.put(b"@ELRS,10,1,2,99,3,4,5,6,7,8,9,10\n")
        self.assertIsNotNone(wait_cursor(service)["receiver"]["link"])
        port.input.put(OSError("device removed"))
        deadline = time.monotonic() + 1
        while service.dispatch("serial.status")["connected"] and time.monotonic() < deadline:
            time.sleep(0.005)
        status = service.dispatch("serial.status")
        self.assertFalse(status["connected"])
        self.assertIn("device removed", status["error"])
        self.assertIsNone(status["receiver"]["link"])
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
