"""编译并运行真实 C 测试逻辑的模拟回归，同时检查报告解析。"""
import os
import binascii
import io
from pathlib import Path
import struct
import subprocess
import unittest

import CanBench


class ReportTest(unittest.TestCase):
    def test_uart_crc_and_size(self):
        raw = bytearray(5292)
        struct.pack_into('<15I', raw, 0, 0x43424d31, 1, 0, 0, 1, 0, 0, *([0] * 8))
        message = (f'CB1_BEGIN 0 5292 {binascii.crc32(raw):08x}\n'.encode() +
                   raw.hex().encode() + b'\nCB1_END\n')
        self.assertEqual(CanBench.serial_report(io.BytesIO(message), 0)['ready'], 1)
        self.assertEqual(CanBench.serial_report(io.BytesIO(b'CB1_STATUS partial' + message), 0)['ready'], 1)
        class SplitSerial:
            def __init__(self):
                self.parts = iter([b'partial', message[:13], message[13:].split(b'\n')[0] + b'\n',
                                   raw.hex().encode()[:19], b'', raw.hex().encode()[19:] + b'\n', b'CB1_END\n'])

            def readline(self):
                return next(self.parts)

        self.assertEqual(CanBench.serial_report(SplitSerial(), 0)['ready'], 1)
        with self.assertRaises(ValueError):
            CanBench.serial_report(io.BytesIO(message.replace(b'314d4243', b'304d4243')), 0)

    def test_decode_and_failure_counts(self):
        raw = bytearray(5292)
        struct.pack_into('<15I', raw, 0, 0x43424d31, 1, 0, 0, 1, 2, 0, *([0] * 8))
        struct.pack_into('<10I', raw, 180, 1, 1, 0xfc, 1, 1, 1, 0, 600, 1, 1)
        struct.pack_into('<10I', raw, 220, 1, 2, 0xfc, 20, 20, 20, 0, 100000, 0, 2)
        decoded = CanBench.decode(raw)
        decoded['test'] = 'transitions'
        row = CanBench.summarize([decoded])['transitions'][0]
        self.assertEqual(row['success'], 1)
        self.assertEqual(row['tests'], 2)
        self.assertEqual(row['latency_median_us'], 600)
        self.assertEqual(row['attempts_max'], 20)

    def test_observed_load_and_no_duplicate_windows(self):
        window = dict(zip(CanBench.WINDOW, [1000, 1000000, 1000, 1000, 1000, 0, 1000, 111000, 135000, 0, 0, 0, 0]))
        records = [{'test': 'passive', 'windows': [window], 'trials': []},
                   {'test': 'transitions', 'windows': [window], 'trials': []}]
        rows = CanBench.summarize(records)['load_windows']
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['observed_load_min_percent'], 22.2)
        self.assertEqual(rows[0]['observed_load_max_percent'], 27.0)


if __name__ == '__main__':
    here = Path(__file__).parent
    build = CanBench.ROOT / 'local/build/can-bench-host'
    build.mkdir(parents=True, exist_ok=True)
    zig = Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe').resolve()
    env = os.environ.copy()
    env['ZIG_GLOBAL_CACHE_DIR'] = str(build / 'zig-global')
    env['ZIG_LOCAL_CACHE_DIR'] = str(build / 'zig-local')
    exe = build / 'CanBenchRegression.exe'
    subprocess.run([str(zig), 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(here / 'stubs'), str(here / 'TestCanBench.c'), '-o', str(exe)],
                   check=True, env=env)
    subprocess.run([str(exe)], check=True)
    unittest.main()
