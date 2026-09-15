import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = Path(__file__).with_name("FlashBench.py")
SPEC = importlib.util.spec_from_file_location("flash_bench", MODULE_PATH)
FLASH_BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FLASH_BENCH)


class FlashBenchSafetyTest(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(serial="probe", speed=500, sd_removed=False,
                                    pause_timeout_ms=1000)
        self.symbols = {
            "g_sdLogFlashMailboxGuard": (0x08012340, 4),
            "g_sdLogFlashBuildFingerprint": (0x08012350, 32),
            "g_sdLogFlashPauseRequest": (0x24000100, 4),
            "g_sdLogFlashPauseReady": (0x24000104, 4),
        }
        self.fingerprint = bytes(range(32))

    def test_short_identity_readback_does_not_halt_or_write_target(self):
        with tempfile.TemporaryDirectory() as directory:
            expected_guard = Path(directory) / "guard.bin"
            observed_guard = Path(directory) / "guard-read.bin"
            expected_fingerprint = Path(directory) / "fingerprint.bin"
            observed_fingerprint = Path(directory) / "fingerprint-read.bin"
            expected_guard.write_bytes(FLASH_BENCH.MAILBOX_GUARD.to_bytes(4, "little"))
            expected_fingerprint.write_bytes(self.fingerprint)
            commands = FLASH_BENCH.identity_commands(
                "probe", 500, [
                    (0x08012340, expected_guard, observed_guard),
                    (0x08012350, expected_fingerprint, observed_fingerprint),
                ])
        dumps = [command for command in commands if command.startswith("dump_image")]
        self.assertEqual(len(dumps), 2)
        self.assertTrue(dumps[0].endswith(" 4"))
        self.assertTrue(dumps[1].endswith(" 32"))
        self.assertFalse(any(command == "halt" or command == "reset halt" for command in commands))
        self.assertFalse(any(command.startswith("mww") for command in commands))
        self.assertFalse(any(command.startswith("verify_image") for command in commands))

    def test_ready_precedes_halt_and_bulk_read(self):
        with tempfile.TemporaryDirectory() as directory:
            expected = Path(directory) / "full.bin"
            observed = Path(directory) / "full-read.bin"
            expected.write_bytes(bytes(4096))
            commands = FLASH_BENCH.pause_and_halt_commands(
                "probe", 500, self.symbols, self.fingerprint, 1000,
                [(FLASH_BENCH.FLASH_START, expected, observed)])
        request_index = commands.index("mww 0x24000100 0x53445051")
        ready_index = next(i for i, command in enumerate(commands)
                           if "did not confirm" in command)
        halt_index = commands.index("halt")
        bulk_index = next(i for i, command in enumerate(commands)
                          if command.startswith("dump_image"))
        self.assertLess(request_index, ready_index)
        self.assertLess(ready_index, halt_index)
        self.assertLess(halt_index, bulk_index)
        self.assertFalse(any(command.startswith("dump_image")
                             for command in commands[:halt_index]))

    def test_all_short_identity_checks_precede_mailbox_write(self):
        commands = FLASH_BENCH.pause_commands(
            self.symbols, self.fingerprint, 1000)
        request_index = commands.index("mww 0x24000100 0x53445051")
        fingerprint_checks = [i for i, command in enumerate(commands)
                              if "build fingerprint mismatch" in command]
        self.assertEqual(len(fingerprint_checks), 8)
        self.assertTrue(all(index < request_index for index in fingerprint_checks))

    def test_removed_card_mode_never_touches_mailbox(self):
        self.args.sd_removed = True
        commands = FLASH_BENCH.build_commands(
            self.args, "candidate.bin", 123, "before.bin", "after.bin", "readback.bin")
        reset_index = commands.index("reset halt")
        self.assertFalse(any("0x53445051" in command for command in commands))
        self.assertFalse(any(command.startswith("verify_image") for command in commands[:reset_index]))

    def test_pause_failure_cannot_reach_halt(self):
        commands = FLASH_BENCH.pause_commands(
            self.symbols, self.fingerprint, 1000)
        failure_index = next(i for i, command in enumerate(commands)
                             if "failed to sync and close" in command)
        ready_index = next(i for i, command in enumerate(commands)
                           if "did not confirm" in command)
        halt_index = commands.index("halt")
        self.assertLess(failure_index, halt_index)
        self.assertLess(ready_index, halt_index)

    def test_pause_only_preparation_leaves_target_running(self):
        commands = FLASH_BENCH.pause_commands(
            self.symbols, self.fingerprint, 1000, halt_after=False)
        self.assertFalse(any(command in ("halt", "reset halt") for command in commands))
        self.assertTrue(any("did not confirm" in command for command in commands))

    def test_identity_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            expected = Path(directory) / "expected.bin"
            observed = Path(directory) / "observed.bin"
            expected.write_bytes(b"old firmware")
            observed.write_bytes(b"different")
            with self.assertRaisesRegex(RuntimeError, "does not match ELF"):
                FLASH_BENCH.compare_identity_segments([
                    (FLASH_BENCH.FLASH_START, expected, observed)])

    def test_short_read_failure_never_starts_pause_or_bulk_read(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            image = bytes([0xff]) * 0x200
            guard_offset = 0x40
            fingerprint_offset = 0x50
            image = bytearray(image)
            image[guard_offset:guard_offset + 4] = FLASH_BENCH.MAILBOX_GUARD.to_bytes(4, "little")
            image[fingerprint_offset:fingerprint_offset + 32] = self.fingerprint
            segments = [(0x08012300, bytes(image))]
            with mock.patch.object(FLASH_BENCH, "require_hero_m_config"), \
                    mock.patch.object(FLASH_BENCH, "load_running_elf",
                                      return_value=(self.symbols, segments)), \
                    mock.patch.object(FLASH_BENCH, "run_openocd",
                                      side_effect=RuntimeError("short read failed")) as run:
                with self.assertRaisesRegex(RuntimeError, "short read failed"):
                    FLASH_BENCH.quiesce_and_verify_running_firmware(
                        Path(directory), output, "probe", 500,
                        Path(directory) / "running.elf")
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[1].name, "running-short-identity.cfg")
            self.assertFalse((output / "sd-log-quiesce-halt.cfg").exists())

    def test_different_build_fingerprint_is_rejected_before_pause(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            image = bytearray(bytes([0xff]) * 0x200)
            image[0x40:0x44] = FLASH_BENCH.MAILBOX_GUARD.to_bytes(4, "little")
            image[0x50:0x70] = self.fingerprint
            segments = [(0x08012300, bytes(image))]

            def write_different_short_identity(root, cfg, log_path, timeout):
                del root, cfg, log_path, timeout
                (output / "running-short-readback-0.bin").write_bytes(
                    FLASH_BENCH.MAILBOX_GUARD.to_bytes(4, "little"))
                different = bytearray(self.fingerprint)
                different[-1] ^= 0xff
                (output / "running-short-readback-1.bin").write_bytes(different)

            with mock.patch.object(FLASH_BENCH, "require_hero_m_config"), \
                    mock.patch.object(FLASH_BENCH, "load_running_elf",
                                      return_value=(self.symbols, segments)), \
                    mock.patch.object(FLASH_BENCH, "run_openocd",
                                      side_effect=write_different_short_identity) as run:
                with self.assertRaisesRegex(RuntimeError, "does not match ELF"):
                    FLASH_BENCH.quiesce_and_verify_running_firmware(
                        Path(directory), output, "probe", 500,
                        Path(directory) / "running.elf")
            self.assertEqual(run.call_count, 1)
            self.assertFalse((output / "sd-log-quiesce-halt.cfg").exists())

    def test_old_elf_without_build_fingerprint_is_rejected(self):
        elf = MODULE_PATH.parents[2] / "local/bench/20260911/sd-crc-quiesce/zephyr.elf"
        if not elf.is_file():
            self.skipTest("old firmware ELF is unavailable")
        with self.assertRaisesRegex(RuntimeError, "g_sdLogFlashBuildFingerprint"):
            FLASH_BENCH.load_running_elf(elf)

    def test_real_elf_reconstructs_exact_objcopy_binary(self):
        root = MODULE_PATH.parents[2]
        pairs = [
            root / "local/build/hero-m-bench/zephyr/zephyr.elf",
            root / "local/build/hero-m/zephyr/zephyr.elf",
        ]
        checked = 0
        for elf in pairs:
            binary = elf.with_suffix(".bin")
            if not elf.is_file() or not binary.is_file():
                continue
            try:
                _, segments = FLASH_BENCH.load_running_elf(elf)
            except RuntimeError as error:
                if "g_sdLogFlashBuildFingerprint" in str(error):
                    continue
                raise
            self.assertEqual(segments, [(FLASH_BENCH.FLASH_START, binary.read_bytes())])
            checked += 1
        if checked == 0:
            self.skipTest("no real Zephyr ELF/bin pair is available")


if __name__ == "__main__":
    unittest.main()
