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
    def test_real_a_target_allows_only_seven_ported_tasks(self):
        target = GEN.resolve(REPO, "INFANTRY-TIAN-A")
        self.assertEqual(target["board"], "dji_a_f427")
        self.assertEqual(
            {row["symbol"] for row in target["tasks"]},
            {"RC_SBUS", "HEALTH_MONITOR", "CAN_COMMAND_TX", "CAN_FEEDBACK_RX",
             "CLASSIC_CHASSIS", "SINGLE_GIMBAL", "IMU"})
        self.assertEqual(target["board_info"]["cpu_hz"], 168000000)

    def test_real_a_target_rejects_unported_tasks(self):
        path = REPO / "Robotconfig/INFANTRY-TIAN-A/RobotConfig.toml"
        text = path.read_text(encoding="utf-8")
        cases = {
            "SDLOG": text.replace('"HEALTH_MONITOR"]', '"HEALTH_MONITOR", "SDLOG"]'),
            "ELRS_LINK": text.replace('"HEALTH_MONITOR"]', '"HEALTH_MONITOR", "ELRS_LINK"]'),
            "ARM": text + '\n[controllers.arm]\ntype = "standard"\n',
        }
        for name, config_text in cases.items():
            with self.subTest(task=name), self.assertRaisesRegex(GEN.ConfigError, "未迁移模块"):
                GEN.resolve(REPO, "INFANTRY-TIAN-A", config_text)

    def test_a_board_allows_only_the_ported_runtime_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "Robotconfig/NEW"
            (root / "shared/application/robot").mkdir(parents=True)
            target.mkdir(parents=True)
            (root / "shared/application/robot/RobotTaskCatalog.def").write_text(
                'ROBOT_TASK(RC_SBUS, 1, "module.rc_sbus", "task.rc_sbus", RobotModuleKindInput, '
                '0u, 0u, 512u, 256u, RobotModulePriorityAboveNormal, '
                'ROBOT_MODULE_FLAG_HAS_TASK, RcSbus, "RcSbusTask.h", RcSbusTask, 1u, "", '
                '"@target/DetectTask.c")\n',
                encoding="utf-8")
            (target / "RobotConfig.c").write_text("", encoding="utf-8")
            (target / "DetectTask.c").write_text("", encoding="utf-8")
            (target / "RobotConfig.toml").write_text(
                'schema = 1\nboard = "dji_a_f427"\nservices = ["RC_SBUS"]\n'
                '[build]\nsources = ["RobotConfig.c", "DetectTask.c"]\n',
                encoding="utf-8")
            self.assertEqual(GEN.resolve(root, "NEW")["board"], "dji_a_f427")

    def test_c_board_still_fails_before_source_resolution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "Robotconfig/NEW"
            target.mkdir(parents=True)
            (target / "RobotConfig.toml").write_text('schema = 1\nboard = "dji_c_f407"\n', encoding="utf-8")
            self.assertEqual(GEN.targets(root)[0]["board"], "dji_c_f407")
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
