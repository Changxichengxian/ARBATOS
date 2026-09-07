"""配置入口不能重新带入旧启动代码，未完成的整板移植应提前拒绝。"""

import importlib.util
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("robot_config_boundaries", REPO / "tools/config/RobotConfigGen.py")
GEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)


class RobotConfigBoundariesTest(unittest.TestCase):
    def test_unported_application_boards_fail_before_source_resolution(self):
        for board in ("dji_a_f427", "dji_c_f407"):
            with self.subTest(board=board), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                target = root / "Robotconfig/NEW"
                target.mkdir(parents=True)
                (target / "RobotConfig.toml").write_text(f'schema = 1\nboard = "{board}"\n', encoding="utf-8")
                self.assertEqual(GEN.targets(root)[0]["board"], board)
                with self.assertRaisesRegex(GEN.ConfigError, "完整机器人运行栈尚未迁移"):
                    GEN.resolve(root, "NEW")

    def test_source_policy_preserves_zephyr_adapters_and_blocks_old_platform(self):
        for source in ("shared/hal/BspCan.c", "boards/DmMc02H7/bsp/BspBoard.c",
                       "projects/HERO-M/Core/startup.c", "shared/vendor/startup.s"):
            with self.subTest(source=source):
                self.assertIsNotNone(GEN.FORBIDDEN_SOURCE.search(source))
        self.assertIsNone(GEN.FORBIDDEN_SOURCE.search("shared/zephyr/port/platform/BspFlash.c"))


if __name__ == "__main__":
    unittest.main()
