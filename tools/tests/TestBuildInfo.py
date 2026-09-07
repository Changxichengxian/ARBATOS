import os
import re
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "build" / "GenBuildInfo.py"


class GenBuildInfoTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="arbatos 构建信息 ")
        self.repo = Path(self.temp.name) / "仓库 空格"
        self.repo.mkdir()
        self.output = Path(self.temp.name) / "构建 输出" / "build_info_autogen.h"
        self.git("init")
        self.git("config", "user.email", "test@example.invalid")
        self.git("config", "user.name", "Test")
        (self.repo / "tracked.c").write_text("int x;\n", encoding="utf-8")
        self.git("add", "tracked.c")
        self.git("commit", "-m", "initial")

    def tearDown(self):
        self.temp.cleanup()

    def git(self, *args):
        subprocess.run(["git", "-C", str(self.repo), *args], check=True, stdout=subprocess.DEVNULL)

    def generate(self, env=None):
        subprocess.run(
            [sys.executable, str(SCRIPT), "--repo", str(self.repo), "--output", str(self.output)],
            check=True,
            env=env,
            stdout=subprocess.DEVNULL,
        )
        return self.output.read_text(encoding="ascii")

    @staticmethod
    def macro(content, name):
        match = re.search(rf'^#define {name} "(.*)"$', content, flags=re.MULTILINE)
        return match.group(1) if match else None

    def test_clean_dirty_and_unchanged_write(self):
        clean = self.generate()
        self.assertIn('ARBATOS_BUILD_DIRTY 0u', clean)
        self.assertRegex(self.macro(clean, "ARBATOS_BUILD_DATE"), r"^\d{4}-\d{2}-\d{2}$")
        self.assertRegex(self.macro(clean, "ARBATOS_BUILD_TIME"), r"^\d{2}:\d{2}:\d{2}$")
        stamp = self.output.stat().st_mtime_ns
        time.sleep(0.02)
        self.generate()
        self.assertEqual(stamp, self.output.stat().st_mtime_ns)
        (self.repo / "tracked.c").write_text("int changed;\n", encoding="utf-8")
        self.assertIn('ARBATOS_BUILD_DIRTY 1u', self.generate())
        self.git("add", "tracked.c")
        self.assertIn('ARBATOS_BUILD_DIRTY 1u', self.generate())
        self.git("commit", "-m", "change tracked")
        committed = self.generate()
        self.assertIn('ARBATOS_BUILD_DIRTY 0u', committed)
        self.assertNotEqual(clean, committed)
        (self.repo / "new_source.c").write_text("int new_file;\n", encoding="utf-8")
        self.assertIn('ARBATOS_BUILD_DIRTY 1u', self.generate())

    def test_dirty_source_state_tracks_each_content_change(self):
        (self.repo / "tracked.c").write_text("int first_change;\n", encoding="utf-8")
        first = self.generate()
        first_state = self.macro(first, "ARBATOS_SOURCE_STATE")
        first_stamp = self.output.stat().st_mtime_ns
        time.sleep(0.02)
        self.generate()
        self.assertEqual(first_stamp, self.output.stat().st_mtime_ns)
        (self.repo / "tracked.c").write_text("int second_change;\n", encoding="utf-8")
        second = self.generate()
        self.assertIn('ARBATOS_BUILD_DIRTY 1u', second)
        self.assertNotEqual(first_state, self.macro(second, "ARBATOS_SOURCE_STATE"))
        self.assertNotEqual(first_stamp, self.output.stat().st_mtime_ns)

    def test_generation_time_is_not_head_commit_time(self):
        (self.repo / "tracked.c").write_text("int old_commit;\n", encoding="utf-8")
        self.git("add", "tracked.c")
        env = os.environ.copy()
        env["GIT_AUTHOR_DATE"] = "2001-02-03T04:05:06+0000"
        env["GIT_COMMITTER_DATE"] = "2001-02-03T04:05:06+0000"
        subprocess.run(
            ["git", "-C", str(self.repo), "commit", "-m", "old dated commit"],
            check=True,
            env=env,
            stdout=subprocess.DEVNULL,
        )
        generated = self.generate()
        self.assertNotEqual(self.macro(generated, "ARBATOS_BUILD_DATE"), "2001-02-03")
        self.assertNotEqual(self.macro(generated, "ARBATOS_BUILD_TIME"), "04:05:06")

    def test_ignored_file_and_missing_git(self):
        (self.repo / ".gitignore").write_text("local/\n", encoding="utf-8")
        self.git("add", ".gitignore")
        self.git("commit", "-m", "ignore local")
        (self.repo / "local").mkdir()
        (self.repo / "local" / "generated.h").write_text("generated", encoding="utf-8")
        self.assertIn('ARBATOS_BUILD_DIRTY 0u', self.generate())
        env = os.environ.copy()
        env["PATH"] = ""
        self.assertIn('ARBATOS_GIT_SHA "unknown"', self.generate(env))

    def test_cli_uses_utf8_for_chinese_output_path(self):
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "cp1252"
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--repo", str(self.repo), "--output", str(self.output)],
            check=False,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode("utf-8", errors="replace"))
        output = result.stdout.decode("utf-8")
        self.assertIn(str(self.output.resolve()), output)

    def test_submodule_revision_and_dirty_state(self):
        child = Path(self.temp.name) / "child"
        child.mkdir()
        subprocess.run(["git", "init", str(child)], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(["git", "-C", str(child), "config", "user.email", "test@example.invalid"], check=True)
        subprocess.run(["git", "-C", str(child), "config", "user.name", "Test"], check=True)
        (child / "child.c").write_text("int child;\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(child), "add", "child.c"], check=True)
        subprocess.run(["git", "-C", str(child), "commit", "-m", "child"], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(
            ["git", "-C", str(self.repo), "-c", "protocol.file.allow=always", "submodule", "add", str(child), "vendor/child"],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        self.git("commit", "-m", "add child")
        clean = self.generate()
        self.assertIn("ARBATOS_GIT_SUBMODULES", clean)
        (self.repo / "vendor" / "child" / "child.c").write_text("int child_changed;\n", encoding="utf-8")
        first_dirty = self.generate()
        self.assertIn('ARBATOS_BUILD_DIRTY 1u', first_dirty)
        first_state = self.macro(first_dirty, "ARBATOS_SOURCE_STATE")
        (self.repo / "vendor" / "child" / "child.c").write_text("int child_changed_again;\n", encoding="utf-8")
        second_dirty = self.generate()
        self.assertNotEqual(first_state, self.macro(second_dirty, "ARBATOS_SOURCE_STATE"))


if __name__ == "__main__":
    unittest.main()
