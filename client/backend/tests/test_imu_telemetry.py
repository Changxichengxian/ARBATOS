from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_telemetry import ImuTelemetry


def frame(ms: int, seq: int, *, flags: int = 3, state: int = 0, request_id: int = 0) -> str:
    values = [ms, state, request_id, 0, 0, seq, flags, 25000, 40000,
              1000 + (ms % 10000), 2000, 3000, 1000, -2000, 3000, 0, 0, 9810, 1, 2, 3, 1, 2, 3]
    return "@IMU2,HERO-M," + ",".join(map(str, values))


class ImuTelemetryTest(unittest.TestCase):
    def test_accepts_valid_frame_and_computes_window_metrics(self):
        now = [1.0]
        telemetry = ImuTelemetry(lambda: now[0])
        self.assertTrue(telemetry.feed(frame(1000, 10), now[0]))
        now[0] = 2.0
        self.assertTrue(telemetry.feed(frame(1100, 10), now[0]))
        snapshot = telemetry.snapshot()
        self.assertTrue(snapshot["healthy"])
        self.assertTrue(snapshot["safeToStart"])
        self.assertAlmostEqual(snapshot["observation"]["angleDriftDegPerMin"]["roll"], 60.0)
        self.assertAlmostEqual(snapshot["observation"]["gyroMeanDegPerSec"]["x"], 1.0)

    def test_rejects_malformed_mapping_and_tracks_drops_and_restart(self):
        telemetry = ImuTelemetry(lambda: 1.0)
        self.assertFalse(telemetry.feed("@IMU2,0,1", 1.0))
        self.assertTrue(telemetry.feed(frame(100, 10), 1.0))
        self.assertTrue(telemetry.feed(frame(400, 10), 1.1))
        self.assertTrue(telemetry.feed(frame(1, 1), 1.2))
        snapshot = telemetry.snapshot()
        self.assertEqual(snapshot["invalid"], 1)
        self.assertEqual(snapshot["dropped"], 1)
        self.assertEqual(snapshot["restarts"], 1)

    def test_accepts_uint32_wrap_and_avoids_stale_health(self):
        now = [1.0]
        telemetry = ImuTelemetry(lambda: now[0])
        self.assertTrue(telemetry.feed(frame(0xFFFFFFFE, 0xFFFFFFFE), now[0]))
        self.assertTrue(telemetry.feed(frame(1, 1), now[0] + .1))
        now[0] = 4.2
        snapshot = telemetry.snapshot()
        self.assertFalse(snapshot["fresh"])
        self.assertFalse(snapshot["healthy"])

    def test_constant_flash_sequence_and_real_target_accept_normal_stream(self):
        telemetry = ImuTelemetry(lambda: 10.0)
        for ms in range(1000, 2000, 100):
            self.assertTrue(telemetry.feed(frame(ms, 3), 10.0))
        snapshot = telemetry.snapshot()
        self.assertEqual(snapshot["accepted"], 10)
        self.assertEqual(snapshot["dropped"], 0)
        self.assertEqual(snapshot["last"]["target"], "HERO-M")
        self.assertAlmostEqual(snapshot["observation"]["seconds"], .9)

    def test_angle_wrap_and_timestamp_wrap_use_device_time(self):
        telemetry = ImuTelemetry(lambda: 10.0)
        for ms, angle in ((0xFFFFFFCE, 179999), (50, -179999)):
            parts = frame(ms, 3).split(",")
            parts[13] = str(angle)  # yaw 毫度
            self.assertTrue(telemetry.feed(",".join(parts), 10.0))
        result = telemetry.snapshot()
        self.assertEqual(result["restarts"], 0)
        self.assertAlmostEqual(result["observation"]["seconds"], .1)
        self.assertAlmostEqual(result["observation"]["angleDriftDegPerMin"]["yaw"], 1.2)

    def test_epoch_changes_clear_observation_without_counting_as_reboot(self):
        telemetry = ImuTelemetry(lambda: 10.0)
        telemetry.feed(frame(100, 3), 10)
        telemetry.feed(frame(200, 3), 10)
        self.assertIsNotNone(telemetry.snapshot()["observation"])
        telemetry.feed(frame(300, 4, state=4), 10)
        self.assertIsNone(telemetry.snapshot()["observation"])
        self.assertEqual(telemetry.snapshot()["restarts"], 0)
        telemetry.feed(frame(400, 4, flags=19), 10)
        self.assertIsNone(telemetry.snapshot()["observation"])
        telemetry.feed(frame(500, 4, flags=19), 10)
        self.assertIsNotNone(telemetry.snapshot()["observation"])
        telemetry.feed(frame(600, 4, state=2, flags=19), 10)
        self.assertIsNone(telemetry.snapshot()["observation"])
        self.assertFalse(telemetry.snapshot()["safeToStart"])

    def test_stale_unhealthy_and_long_gap_do_not_supply_drift(self):
        now = [10.0]
        telemetry = ImuTelemetry(lambda: now[0])
        telemetry.feed(frame(100, 3), 10)
        telemetry.feed(frame(200, 3), 10)
        now[0] = 14
        self.assertIsNone(telemetry.snapshot()["observation"])
        now[0] = 9
        self.assertFalse(telemetry.snapshot()["fresh"])
        now[0] = 15
        telemetry.feed(frame(5200, 3), 15)
        self.assertIsNone(telemetry.snapshot()["observation"])
        telemetry.feed(frame(5300, 3, flags=0), 15)
        self.assertFalse(telemetry.snapshot()["healthy"])

    def test_extreme_fields_and_reflection_rejected_without_losing_last_sample(self):
        telemetry = ImuTelemetry(lambda: 10.0)
        telemetry.feed(frame(100, 3), 10)
        for index, value in ((2, "4294967296"), (14, "9" * 400), (11, "nan"),
                             (7, "-1"), (25, "-3"), (20, "7000000")):
            parts = frame(200, 3).split(",")
            parts[index] = value
            self.assertFalse(telemetry.feed(",".join(parts), 10))
        self.assertEqual(telemetry.snapshot()["last"]["ms"], 100)

    def test_reset_statistics_keeps_device_and_reboot_counter(self):
        telemetry = ImuTelemetry(lambda: 10.0, max_records=20)
        telemetry.feed(frame(5000, 3), 10)
        telemetry.feed(frame(100, 3), 10)
        self.assertEqual(telemetry.snapshot()["restarts"], 1)
        telemetry.reset_statistics()
        self.assertEqual(telemetry.snapshot()["last"]["ms"], 100)
        self.assertEqual(telemetry.snapshot()["restarts"], 1)
        for ms in range(200, 5000, 100):
            telemetry.feed(frame(ms, 3), 10)
        self.assertEqual(len(telemetry.history), 20)


if __name__ == "__main__":
    unittest.main()
