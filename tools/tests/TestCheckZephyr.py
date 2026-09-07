import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
CHECK = REPO / "tools" / "build" / "CheckZephyr.py"


class CheckZephyrTest(unittest.TestCase):
    def run_check(self, root=REPO, project="all"):
        return subprocess.run(
            [sys.executable, str(CHECK), "--root", str(root), "--project", project, "--json"],
            capture_output=True, text=True, check=False,
        )

    def fixture(self):
        directory = tempfile.TemporaryDirectory()
        root = Path(directory.name) / "ARBATOS"
        tracked = subprocess.check_output(
            ["git", "-C", str(REPO), "ls-files", "--cached", "--others", "--exclude-standard",
             "-z", "projects", "shared", "Robotconfig", "boards"],
        ).decode("utf-8").split("\0")
        for relative in filter(None, tracked):
            source = REPO / relative
            if not source.is_file():
                continue
            destination = root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        baseline = self.run_check(root)
        self.assertEqual(baseline.returncode, 0, baseline.stderr + baseline.stdout)
        return directory, root

    def test_repository_all_targets_pass(self):
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        payload = json.loads(result.stdout)
        self.assertTrue(payload["ok"])
        self.assertEqual(len(payload["checked_projects"]), 3)

    def test_missing_formal_config_fails(self):
        directory, root = self.fixture()
        with directory:
            (root / "projects" / "HERO-M" / "prj.conf").unlink()
            result = self.run_check(root, "HERO-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing file", result.stdout)

    def test_old_startup_source_in_manifest_fails(self):
        directory, root = self.fixture()
        with directory:
            manifest = root / "projects" / "cmake" / "ArbatosLegacy.cmake"
            manifest.write_text(manifest.read_text(encoding="utf-8") + "\nshared/hal/startup_stm32f4xx.s\n", encoding="utf-8")
            result = self.run_check(root, "HERO-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("forbidden legacy entry", result.stdout)

    def test_missing_sentinel_overlay_fails(self):
        directory, root = self.fixture()
        with directory:
            (root / "projects" / "SENTINEL-M" / "app.overlay").unlink()
            result = self.run_check(root, "SENTINEL-M")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("app.overlay", result.stdout)

    def test_unused_board_support_is_still_required(self):
        directory, root = self.fixture()
        with directory:
            (root / "boards" / "DjiCF407" / "zephyr" / "board.yml").unlink()
            result = self.run_check(root)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("DjiCF407", result.stdout)


if __name__ == "__main__":
    unittest.main()
