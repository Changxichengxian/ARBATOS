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


class HistoricalRecordDecodeTest(unittest.TestCase):
    def test_profiler_ids_include_wheelleg_loop(self):
        payload = struct.pack('<B3xB3x6I',1,7,10,100,204000,2000,5000,1)
        ds = VIEWER.Dataset('test',0,16)
        ds.add_record(1000,0x004E,payload)
        self.assertIn('RT_PROFILER.SDLOG_BLOCK_WRITE',ds.series)
        ds = VIEWER.Dataset('legacy',0,16)
        ds.profiler_names = VIEWER.RT_PROFILER_LEGACY_NAMES
        ds.add_record(1000,0x004E,payload)
        self.assertIn('RT_PROFILER.SDLOG_SYNC',ds.series)

    def test_bench_stop_preserves_capture_time_and_first_reason(self):
        payload = struct.pack("<4B10If56f", 1, 1, 1, 0, 501, 1503, 2, 1503, 4, 12,
                              128, 3, 0, 3, 37.5, *[float(i) for i in range(56)])
        rows = VIEWER.extract_records(1600, 0x0059, payload)
        self.assertEqual(rows[0][0], 1503)
        self.assertEqual(rows[0][1], "WHEELLEG_BENCH_STOP")
        fields = rows[0][3]
        self.assertEqual(fields["source_name"], "WHEEL_FEEDBACK_REJECT")
        self.assertEqual(fields["reject_value"], 37.5)
        self.assertEqual(fields["sd_dropped"], 3)
        self.assertEqual(fields["target_roll"], 22)
        self.assertEqual(fields["wheel_target_L"], 47)
        self.assertEqual(fields["acceleration"], 55)
        for bad in (payload[:-1], b"\x02"+payload[1:], payload[:1]+b"\x02"+payload[2:]):
            self.assertIsNone(VIEWER.extract_records(0, 0x0059, bad))

    def test_manual_raw_channels_and_switches(self):
        payload = struct.pack("<4B16h4B3hH", 1, 1, 1, 5, *range(16), 3, 1, 0, 0, -2, 0, 4, 0)
        fields = VIEWER.extract_records(20, 0x46, payload)[0][3]
        self.assertEqual((fields["ch3"], fields["sw0"], fields["sw1"], fields["mouse_x"]), (3,3,1,-2))

    def test_bench_integrity_does_not_treat_crc_as_no_loss(self):
        ds = VIEWER.Dataset("test",0,16)
        self.assertFalse(ds.bench_integrity()["available"])
        for seq in (10,11,13):
            payload = struct.pack("<4B10If56f",1,0,1,0,seq,seq*3,1,33,4,2,128,
                                  int(seq==13),0,3,36.,*[0.]*56)
            ds.add_record(seq*3,0x59,payload)
        result = ds.bench_integrity()
        self.assertEqual(result["missing_frames"],1)
        self.assertEqual(result["sd_dropped_during_window"],1)
        self.assertEqual(result["missing_stop_sequences"],[1])
        self.assertFalse(result["complete_within_recorded_window"])

    def test_motor_diag_temperature_and_historical_layouts(self):
        for version in (1, 2, 3, 4):
            head = struct.pack("<BBH", version, 6, 0)
            if version >= 2:
                head += struct.pack("<8I", *range(8))
            entries = []
            for role in range(6):
                values = [role, role + 1] + [0] * 16 + [0] * 4
                values += [101 + role] * (6 if version >= 3 else 5) + [0.25] * 9
                fmt = "<18B4H6I9f" if version >= 3 else "<18B4H5I9f"
                if version >= 4:
                    fmt += "3B"
                    values += [30 + role, 40 + role, int(role != 5)]
                entries.append(struct.pack(fmt, *values))
            payload = head + b"".join(entries)
            rows = VIEWER.extract_records(123, 0x0053, payload)
            motors = {row[3]["role_name"]: row[3] for row in rows if "role_name" in row[3]}
            self.assertEqual(len(motors), 6)
            self.assertAlmostEqual(motors["LW"]["fb_torque_nm"], 0.25)
            if version >= 3:
                self.assertEqual(motors["LW"]["tx_id_count"], 103)
            if version == 4:
                self.assertEqual(motors["LW"]["fb_mos_temperature_c"], 32)
                self.assertEqual(motors["LW"]["fb_coil_temperature_c"], 42)
                self.assertTrue(math.isnan(motors["RW"]["fb_mos_temperature_c"]))
                self.assertFalse(motors["RW"]["fb_temperature_valid"])
            else:
                self.assertNotIn("fb_coil_temperature_c", motors["LW"])
        self.assertIsNone(VIEWER.extract_records(0, 0x0053, payload[:-1]))
        self.assertIsNone(VIEWER.extract_records(0, 0x0053, b"\x03" + payload[1:]))

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
