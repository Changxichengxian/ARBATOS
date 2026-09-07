import importlib.util
import math
import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("sdlog_viewer", ROOT / "tools" / "sdlog" / "SdLogViewer.py")
VIEWER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = VIEWER
SPEC.loader.exec_module(VIEWER)


class PowerMeterDecodeTest(unittest.TestCase):
    def test_source_state_keeps_build_identity(self):
        state = b"dirty:" + b"a" * 64
        self.assertEqual(VIEWER.extract_records(0, 0x0058, state)[0][3]["source_state"], state.decode())
        for invalid in (b"", b"bad\0state", b"\xff", b"a" * 481):
            self.assertIsNone(VIEWER.extract_records(0, 0x0058, invalid))

    def test_power_meter_batch_uses_sample_ticks_and_unsigned_raw(self):
        head = struct.pack("<BBBBHHII", 1, 2, 1, 0, 0x212, 0, 3, 4)
        samples = struct.pack("<IIHHfff", 100, 7, 2450, 65535, 24.5, 655.35, 16056.075)
        samples += struct.pack("<IIHHfff", 101, 8, 2400, 200, 24.0, 2.0, 48.0)
        rows = VIEWER.extract_records(999, 0x0056, head + samples)
        self.assertEqual([row[0] for row in rows], [100, 101])
        self.assertEqual(rows[0][3]["raw_current"], 65535)
        self.assertAlmostEqual(rows[0][3]["power_w"], 16056.075, places=2)

    def test_power_meter_rejects_bad_length_count_and_version(self):
        head = struct.pack("<BBBBHHII", 1, 1, 1, 0, 0x212, 0, 0, 0)
        self.assertIsNone(VIEWER.extract_records(0, 0x0056, head))
        bad_count = struct.pack("<BBBBHHII", 1, 17, 1, 0, 0x212, 0, 0, 0)
        self.assertIsNone(VIEWER.extract_records(0, 0x0056, bad_count))
        bad_version = struct.pack("<BBBBHHII", 2, 1, 1, 0, 0x212, 0, 0, 0) + bytes(28)
        self.assertIsNone(VIEWER.extract_records(0, 0x0056, bad_version))

    def test_chassis_model_valid_and_invalid(self):
        payload = struct.pack("<BBHII4i4ff", 1, 1, 0x0005, 250, 9, 1, -2, 3, -4, 10.0, 20.0, 30.0, 40.0, 123.5)
        rows = VIEWER.extract_records(0, 0x0057, payload)
        self.assertEqual(rows[0][0], 250)
        self.assertEqual(rows[0][3]["current_cmd_1"], -2)
        self.assertEqual(rows[0][3]["active_mask"], 0x0005)
        self.assertTrue(rows[0][3]["model_valid"])
        invalid = struct.pack("<BBHII4i4ff", 1, 0, 0, 251, 10, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, math.nan)
        self.assertFalse(VIEWER.extract_records(0, 0x0057, invalid)[0][3]["model_valid"])


if __name__ == "__main__":
    unittest.main()
