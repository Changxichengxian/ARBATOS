import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ZIG = Path(os.environ["LOCALAPPDATA"]) / "Microsoft/WinGet/Links/zig.exe"


class ImuAttitudeTest(unittest.TestCase):
    def test_real_attitude_and_filters(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "attitude.exe"
            paths = [
                "shared/zephyr/port/sensors",
                "shared/zephyr/port/algorithm/include",
                "shared/components/algorithm",
                "shared/components/support",
            ]
            sources = [
                "tools/tests/ImuAttitudeRegression.c",
                "shared/zephyr/port/sensors/ImuAttitude.c",
                "shared/zephyr/port/algorithm/AhrsZephyr.c",
                "shared/components/algorithm/UserLib.c",
            ]
            cmd = [str(ZIG), "cc", "-std=c99", "-O0"]
            cmd += [arg for path in paths for arg in ("-I", str(ROOT / path))]
            cmd += [str(ROOT / path) for path in sources] + ["-lm", "-o", str(exe)]
            subprocess.run(cmd, check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
