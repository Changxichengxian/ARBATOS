from __future__ import annotations

import hashlib
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jobs import JobError, Jobs


def make_root(root: Path, state: str = "dirty:test-state") -> Path:
    (root / "tools" / "config").mkdir(parents=True)
    (root / "tools" / "build").mkdir(parents=True)
    (root / "Robotconfig" / "HERO-M").mkdir(parents=True)
    (root / "boards" / "TestBoard" / "zephyr").mkdir(parents=True)
    sdk = root / "local" / "cache" / "zephyr-sdk"
    (sdk / "gnu" / "arm-zephyr-eabi" / "bin").mkdir(parents=True)
    (sdk / "hosttools" / "openocd" / "bin").mkdir(parents=True)
    (sdk / "hosttools" / "openocd" / "share" / "openocd" / "scripts").mkdir(parents=True)
    (sdk / "gnu" / "arm-zephyr-eabi" / "bin" / "arm-zephyr-eabi-gdb.exe").write_bytes(b"test")
    (sdk / "hosttools" / "openocd" / "bin" / "openocd.exe").write_bytes(b"test")
    (root / "Robotconfig" / "HERO-M" / "RobotConfig.toml").write_text("board = 'test'\n", encoding="utf-8")
    (root / "tools" / "build.ps1").write_text("# fixture\n", encoding="utf-8")
    (root / "tools" / "config" / "RobotConfigGen.py").write_text(
        "def target_identity(root, requested):\n"
        "    from pathlib import Path\n"
        "    root = Path(root)\n"
        "    found = [p for p in (root / 'Robotconfig').iterdir() "
        "if p.is_dir() and p.name.lower() == str(requested).lower() "
        "and (p / 'RobotConfig.toml').is_file()]\n"
        "    if len(found) != 1: raise ValueError('找不到车型')\n"
        "    return dict(name=found[0].name, preset=found[0].name.lower())\n"
        "def read_toml(path): return {'board': 'test_board'}\n"
        "BOARDS = {'test_board': {'directory': 'boards/TestBoard'}}\n",
        encoding="utf-8",
    )
    (root / "tools" / "build" / "GenBuildInfo.py").write_text(
        f"def build_values(root):\n    return ('abc', 1, '', {state!r})\n",
        encoding="utf-8",
    )
    return root


def make_artifact(root: Path, state: str = "dirty:test-state", target: str = "HERO-M") -> Path:
    build = root / "local" / "build" / "hero-m"
    zephyr = build / "zephyr"
    generated = build / "generated"
    zephyr.mkdir(parents=True)
    generated.mkdir()
    (build / "CMakeCache.txt").write_text("ARBATOS_ROBOT:STRING=HERO-M\n", encoding="utf-8")
    (zephyr / "runners.yaml").write_text(
        "runners:\n- openocd\n"
        "flash-runner: openocd\n"
        "debug-runner: openocd\n"
        "config:\n"
        f"  board_dir: {(root / 'boards/TestBoard/zephyr').as_posix()}\n"
        "  elf_file: zephyr.elf\n"
        "  hex_file: zephyr.hex\n"
        "  bin_file: zephyr.bin\n"
        f"  gdb: {(root / 'local/cache/zephyr-sdk/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb.exe').as_posix()}\n"
        f"  openocd: {(root / 'local/cache/zephyr-sdk/hosttools/openocd/bin/openocd.exe').as_posix()}\n"
        "  openocd_search:\n"
        f"    - {(root / 'local/cache/zephyr-sdk/hosttools/openocd/share/openocd/scripts').as_posix()}\n"
        "args:\n"
        "  openocd:\n"
        "    - --cmd-load\n"
        "    - flash write_image erase\n"
        "    - --cmd-verify\n"
        "    - verify_image\n",
        encoding="utf-8",
    )
    (zephyr / ".config").write_text(f'CONFIG_ARBATOS_ROBOT_NAME="{target}"\n', encoding="utf-8")
    (generated / "build_info_autogen.h").write_text(
        '#define ARBATOS_BUILD_DATE "2026-09-08"\n'
        '#define ARBATOS_BUILD_TIME "12:34:56"\n'
        f'#define ARBATOS_SOURCE_STATE "{state}"\n',
        encoding="ascii",
    )
    state_bytes = state.encode("ascii")
    (zephyr / "zephyr.elf").write_bytes(b"elf\0" + state_bytes + b"\0")
    record = bytes([len(state_bytes), 0, 0, 0]) + state_bytes
    checksum = bytes([(-sum(record)) & 0xFF])
    (zephyr / "zephyr.hex").write_text(
        ":" + (record + checksum).hex().upper() + "\n:00000001FF\n", encoding="ascii"
    )
    (zephyr / "zephyr.bin").write_bytes(b"bin\0" + state_bytes + b"\0")
    return zephyr / "zephyr.hex"


def wait_done(jobs: Jobs, job_id: str) -> dict:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        result = jobs.dispatch("job.get", {"id": job_id})
        if result["status"] != "running":
            return result
        time.sleep(0.005)
    raise AssertionError("任务没有结束")


class JobsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = make_root(Path(self.temp.name))

    def tearDown(self):
        self.temp.cleanup()

    def test_build_command_is_fixed_and_logs_are_incremental(self):
        calls = []

        def runner(command, cwd, emit, cancel):
            calls.append((command, cwd))
            emit("第一行\n")
            emit("第二行")
            return 0

        jobs = Jobs(self.root, runner=runner)
        started = jobs.dispatch("job.start", {"target": "HERO-M", "action": "build"})
        done = wait_done(jobs, started["id"])
        self.assertEqual(done["status"], "succeeded")
        self.assertEqual(done["exitCode"], 0)
        self.assertEqual([line["text"] for line in done["lines"]], ["第一行", "第二行"])
        self.assertEqual(jobs.dispatch("job.get", {"id": done["id"], "after": 1})["lines"], [
            {"seq": 2, "text": "第二行"}
        ])
        self.assertEqual(calls[0][0][-5:], ["build", "-Project", "HERO-M", "-Jobs", "2"])
        self.assertEqual(calls[0][1], self.root.resolve())
        jobs.shutdown()

    def test_only_one_job_and_cancel_waits_for_runner(self):
        entered = threading.Event()

        def runner(command, cwd, emit, cancel):
            entered.set()
            while not cancel.wait(0.005):
                pass
            return 9

        jobs = Jobs(self.root, runner=runner)
        job = jobs.dispatch("job.start", {"target": "HERO-M", "action": "check"})
        self.assertTrue(entered.wait(1))
        self.assertTrue(jobs.is_busy())
        with self.assertRaisesRegex(JobError, "已有任务"):
            jobs.dispatch("job.start", {"target": "HERO-M", "action": "build"})
        jobs.dispatch("job.cancel", {"id": job["id"]})
        done = wait_done(jobs, job["id"])
        self.assertEqual(done["status"], "cancelled")
        self.assertIsNone(done["exitCode"])
        self.assertFalse(jobs.is_busy())
        jobs.shutdown()

    def test_shutdown_cancels_running_job(self):
        entered = threading.Event()

        def runner(command, cwd, emit, cancel):
            entered.set()
            cancel.wait(1)
            return 1

        jobs = Jobs(self.root, runner=runner)
        job = jobs.dispatch("job.start", {"target": "HERO-M", "action": "rebuild"})
        self.assertTrue(entered.wait(1))
        jobs.shutdown()
        self.assertEqual(jobs.dispatch("job.get", {"id": job["id"]})["status"], "cancelled")

    def test_flash_plan_and_confirmation_bind_exact_artifact(self):
        image = make_artifact(self.root)
        calls = []

        def runner(command, cwd, emit, cancel):
            calls.append(command)
            return 0

        jobs = Jobs(self.root, runner=runner)
        plan = jobs.dispatch("job.plan_flash", {"target": "HERO-M"})
        self.assertTrue(plan["ready"])
        self.assertEqual(plan["sha"], hashlib.sha256(image.read_bytes()).hexdigest())
        self.assertEqual(plan["buildTime"], "2026-09-08 12:34:56")
        self.assertEqual(len(plan["artifactRevision"]), 64)
        with self.assertRaisesRegex(JobError, "固件已变化"):
            jobs.dispatch("job.start", {"target": "HERO-M", "action": "flash",
                          "confirmation": {"target": "HERO-M", "artifactRevision": "old"}})
        job = jobs.dispatch("job.start", {"target": "HERO-M", "action": "flash",
                            "confirmation": {"target": "HERO-M", "artifactRevision": plan["artifactRevision"]}})
        self.assertEqual(wait_done(jobs, job["id"])["status"], "succeeded")
        self.assertIn("flash", calls[0])
        jobs.shutdown()

    def test_flash_confirmation_changes_when_elf_changes(self):
        make_artifact(self.root)
        jobs = Jobs(self.root, runner=lambda *args: 0)
        old = jobs.dispatch("job.plan_flash", {"target": "HERO-M"})
        (self.root / "local/build/hero-m/zephyr/zephyr.elf").write_bytes(b"changed elf dirty:test-state")
        new = jobs.dispatch("job.plan_flash", {"target": "HERO-M"})
        self.assertNotEqual(old["artifactRevision"], new["artifactRevision"])
        with self.assertRaisesRegex(JobError, "固件已变化"):
            jobs.dispatch("job.start", {"target": "HERO-M", "action": "flash",
                          "confirmation": {"target": "HERO-M", "artifactRevision": old["artifactRevision"]}})
        jobs.shutdown()

    @unittest.skipUnless(os.name == "nt", "Windows 客户端使用文件共享锁")
    def test_flash_keeps_confirmed_files_read_only_until_runner_finishes(self):
        make_artifact(self.root)
        blocked = []

        def runner(command, cwd, emit, cancel):
            try:
                (self.root / "local/build/hero-m/zephyr/zephyr.hex").write_text("changed", encoding="ascii")
            except OSError:
                blocked.append(True)
            return 0

        jobs = Jobs(self.root, runner=runner)
        plan = jobs.dispatch("job.plan_flash", {"target": "HERO-M"})
        job = jobs.dispatch("job.start", {"target": "HERO-M", "action": "flash",
                            "confirmation": {"target": "HERO-M", "artifactRevision": plan["artifactRevision"]}})
        self.assertEqual(wait_done(jobs, job["id"])["status"], "succeeded")
        self.assertEqual(blocked, [True])
        jobs.shutdown()

    def test_flash_plan_rejects_runner_path_and_argument_tampering(self):
        for old, new, reason in (
            ("hex_file: zephyr.hex", "hex_file: OTHER-TARGET.hex", "hex_file"),
            ("board_dir:", "board_dir: C:/outside #", "开发板目录"),
            ("flash write_image erase", "flash write_image OTHER-TARGET.hex", "OpenOCD"),
        ):
            with self.subTest(new=new), tempfile.TemporaryDirectory() as directory:
                root = make_root(Path(directory))
                make_artifact(root)
                path = root / "local/build/hero-m/zephyr/runners.yaml"
                content = path.read_text(encoding="utf-8")
                if old == "board_dir:":
                    content = "\n".join(
                        "  " + new if line.strip().startswith("board_dir:") else line
                        for line in content.splitlines()
                    ) + "\n"
                else:
                    content = content.replace(old, new)
                path.write_text(content, encoding="utf-8")
                plan = Jobs(root, runner=lambda *args: 0).dispatch("job.plan_flash", {"target": "HERO-M"})
                self.assertFalse(plan["ready"])
                self.assertIn(reason, plan["reason"])

    def test_flash_plan_rejects_refreshed_header_with_old_images(self):
        make_artifact(self.root, state="dirty:old-build")
        header = self.root / "local/build/hero-m/generated/build_info_autogen.h"
        header.write_text(
            '#define ARBATOS_BUILD_DATE "2026-09-08"\n'
            '#define ARBATOS_BUILD_TIME "12:35:00"\n'
            '#define ARBATOS_SOURCE_STATE "dirty:test-state"\n',
            encoding="ascii",
        )
        plan = Jobs(self.root, runner=lambda *args: 0).dispatch("job.plan_flash", {"target": "HERO-M"})
        self.assertFalse(plan["ready"])
        self.assertIn("构建版本", plan["reason"])

    def test_flash_plan_rejects_duplicate_inline_runner_sections(self):
        for extra in (
            "config: {board_dir: C:/outside, elf_file: zephyr.elf, "
            "hex_file: C:/OTHER.hex, bin_file: zephyr.bin}\n",
            "args: {openocd: [--cmd-load, 'flash write_image C:/OTHER.hex']}\n",
            '"config": {board_dir: C:/outside, elf_file: zephyr.elf, '
            'hex_file: C:/OTHER.hex, bin_file: zephyr.bin}\n',
        ):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory() as directory:
                root = make_root(Path(directory))
                make_artifact(root)
                path = root / "local/build/hero-m/zephyr/runners.yaml"
                path.write_text(path.read_text(encoding="utf-8") + extra, encoding="utf-8")
                plan = Jobs(root, runner=lambda *args: 0).dispatch("job.plan_flash", {"target": "HERO-M"})
                self.assertFalse(plan["ready"])
                self.assertIn("烧录配置", plan["reason"])

    def test_flash_plan_rejects_new_config_with_old_completed_images(self):
        make_artifact(self.root)
        config = self.root / "local/build/hero-m/zephyr/.config"
        newest_image = max(
            (self.root / "local/build/hero-m/zephyr" / name).stat().st_mtime_ns
            for name in ("zephyr.elf", "zephyr.hex", "zephyr.bin")
        )
        os.utime(config, ns=(newest_image + 1_000_000_000, newest_image + 1_000_000_000))
        plan = Jobs(self.root, runner=lambda *args: 0).dispatch("job.plan_flash", {"target": "HERO-M"})
        self.assertFalse(plan["ready"])
        self.assertIn("可能未完成", plan["reason"])

    def test_flash_plan_rejects_domains_and_external_openocd_search(self):
        for change, reason in (("domains", "domains.yaml"), ("search", "搜索目录")):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                root = make_root(Path(directory))
                make_artifact(root)
                if change == "domains":
                    (root / "local/build/hero-m/domains.yaml").write_text("default: other\n", encoding="utf-8")
                else:
                    runners = root / "local/build/hero-m/zephyr/runners.yaml"
                    content = runners.read_text(encoding="utf-8")
                    expected = (root / "local/cache/zephyr-sdk/hosttools/openocd/share/openocd/scripts").as_posix()
                    runners.write_text(content.replace(expected, "C:/outside/openocd/scripts"), encoding="utf-8")
                plan = Jobs(root, runner=lambda *args: 0).dispatch("job.plan_flash", {"target": "HERO-M"})
                self.assertFalse(plan["ready"])
                self.assertIn(reason, plan["reason"])

    def test_targets_are_discovered_per_request_and_bad_toml_does_not_block_startup(self):
        (self.root / "Robotconfig/BROKEN").mkdir()
        (self.root / "Robotconfig/BROKEN/RobotConfig.toml").write_text("this is bad = [", encoding="utf-8")
        calls = []
        jobs = Jobs(self.root, runner=lambda command, *args: calls.append(command) or 0)
        (self.root / "Robotconfig/NEW-M").mkdir()
        (self.root / "Robotconfig/NEW-M/RobotConfig.toml").write_text("board = 'test'\n", encoding="utf-8")
        job = jobs.dispatch("job.start", {"target": "NEW-M", "action": "check"})
        self.assertEqual(wait_done(jobs, job["id"])["status"], "succeeded")
        self.assertIn("NEW-M", calls[0])
        jobs.shutdown()

    def test_flash_plan_rejects_stale_wrong_target_and_dedicated_mode(self):
        for change, reason in (("source", "源码"), ("target", "车型"), ("mode", "专用模式")):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                root = make_root(Path(directory))
                make_artifact(root, state="dirty:old" if change == "source" else "dirty:test-state",
                              target="SENTINEL-M" if change == "target" else "HERO-M")
                if change == "mode":
                    config = root / "local/build/hero-m/zephyr/.config"
                    config.write_text(config.read_text(encoding="utf-8") + "CONFIG_ARBATOS_RECEIVE_ONLY=y\n",
                                      encoding="utf-8")
                jobs = Jobs(root, runner=lambda *args: 0)
                plan = jobs.dispatch("job.plan_flash", {"target": "HERO-M"})
                self.assertFalse(plan["ready"])
                self.assertIn(reason, plan["reason"])
                jobs.shutdown()

    def test_invalid_target_and_action_never_reach_runner(self):
        def forbidden(*args):
            self.fail("不应调用 runner")

        jobs = Jobs(self.root, runner=forbidden)
        with self.assertRaisesRegex(JobError, "车型无效"):
            jobs.dispatch("job.start", {"target": "../bad", "action": "build"})
        with self.assertRaisesRegex(JobError, "动作无效"):
            jobs.dispatch("job.start", {"target": "HERO-M", "action": "anything"})
        jobs.shutdown()


if __name__ == "__main__":
    unittest.main()
