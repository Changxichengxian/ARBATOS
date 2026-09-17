import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location('record', Path(__file__).resolve().parents[1] / 'ImuCalRecord.py')
record = importlib.util.module_from_spec(spec)
spec.loader.exec_module(record)


class RecordTest(unittest.TestCase):
    def setUp(self):
        self.snapshot = {'sd_quiesce_confirmed': True, 'g_robot_lifecycle': {'output_allowed': 0},
                         'InsGyroCaliState': {'boot_adjust_finished': 1}, 'InsHeaterStable': 1,
                         'InsTemp': 48, 'ImuPwmLastError': 0, 'uid_words': [1, 2, 3],
                         'InsGyroOffset': [.01, -.02, .003]}
        self.uid = struct.pack('>III', 3, 2, 1)

    def test_roundtrip_and_uid_order(self):
        data = record.from_snapshot(self.snapshot, 7)
        self.assertEqual(len(data), 64)
        result = record.decode_record(data, self.uid)
        self.assertEqual(result['sequence'], 7)
        self.assertAlmostEqual(result['bias'][1], -.02)

    def test_torn_and_wrong_board(self):
        data = record.from_snapshot(self.snapshot, 7)
        for i in range(44):
            corrupt = bytearray(data)
            corrupt[i] ^= 1
            with self.assertRaises(ValueError):
                record.decode_record(corrupt, self.uid)
        with self.assertRaises(ValueError):
            record.decode_record(data, b'\0' * 12)
        with self.assertRaises(ValueError):
            record.decode_record(data[:40], self.uid)

    def test_reject_unstable_active_and_invalid_bias(self):
        for key, value in [('InsHeaterStable', 0), ('InsTemp', 45), ('ImuPwmLastError', -1),
                           ('sd_quiesce_confirmed', False), ('InsGyroOffset', [float('nan'), 0, 0])]:
            s = dict(self.snapshot, **{key: value})
            with self.assertRaises(ValueError):
                record.from_snapshot(s, 1)
        self.snapshot['g_robot_lifecycle']['output_allowed'] = 1
        with self.assertRaises(ValueError):
            record.from_snapshot(self.snapshot, 1)


if __name__ == '__main__':
    unittest.main()
