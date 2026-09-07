import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
CHECK = REPO / "tools" / "CheckZephyr.py"


class CheckZephyrTest(unittest.TestCase):
    def run_check(self, root=REPO, project="all"):
        return subprocess.run(
            [sys.executable, str(CHECK), "--root", str(root), "--project", project, "--json"],
            capture_output=True, text=True, check=False,
        )

    def fixture(self):
        directory = tempfile.TemporaryDirectory()
        root = Path(directory.name) / "ARBATOS"
        shutil.copytree(REPO / "zephyr", root / "zephyr")
        return directory, root

    def test_repository_all_targets_pass(self):
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        payload = json.loads(result.stdout)
        self.assertTrue(payload["ok"])
        self.assertEqual(len(payload["checked_projects"]), 7)

    def test_missing_formal_config_fails(self):
        directory, root = self.fixture()
        with directory:
            (root / "zephyr" / "targets" / "hero-m.conf").unlink()
            result = self.run_check(root, "HERO-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing file", result.stdout)

    def test_old_startup_source_in_manifest_fails(self):
        directory, root = self.fixture()
        with directory:
            manifest = root / "zephyr" / "cmake" / "ArbatosLegacy.cmake"
            manifest.write_text(manifest.read_text(encoding="utf-8") + "\nshared/hal/startup_stm32f4xx.s\n", encoding="utf-8")
            result = self.run_check(root, "HERO-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("forbidden legacy entry", result.stdout)

    def test_missing_sentinel_overlay_fails(self):
        directory, root = self.fixture()
        with directory:
            (root / "zephyr" / "targets" / "sentinel-m.overlay").unlink()
            result = self.run_check(root, "SENTINEL-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("sentinel-m.overlay", result.stdout)


if __name__ == "__main__":
    unittest.main()
