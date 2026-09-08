import importlib.util
import shutil
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("workspace", REPO / "client/backend/workspace.py")
WORKSPACE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKSPACE)


class WorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "repo"
        shutil.copytree(REPO / "Robotconfig", self.root / "Robotconfig")
        shutil.copytree(REPO / "shared", self.root / "shared")
        self.workspace = WORKSPACE.Workspace(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def get(self, target="HERO-M"):
        return self.workspace.dispatch("robot.get", {"target": target})

    def test_real_targets_parse_and_summary_contract(self):
        (self.root / "Robotconfig/EMPTY").mkdir()
        summary = self.workspace.dispatch("workspace.summary", {})
        self.assertTrue({"HERO-M", "SENTINEL-M", "MINIWHEELEG-M"}.issubset({row["name"] for row in summary["targets"]}))
        self.assertNotIn("EMPTY", {row["name"] for row in summary["targets"]})
        self.assertTrue(summary["controllers"])
        for name in ("HERO-M", "SENTINEL-M", "MINIWHEELEG-M"):
            detail = self.get(name)
            self.assertTrue(detail["validation"]["ok"])
            self.assertIsInstance(detail["resolved"], dict)

    def test_invalid_robot_update_does_not_change_file(self):
        detail = self.get()
        path = self.root / "Robotconfig/HERO-M/RobotConfig.toml"
        before = path.read_bytes()
        bad = dict(detail["config"])
        bad["board"] = "bad-board"
        with self.assertRaisesRegex(ValueError, "未支持"):
            self.workspace.dispatch("robot.update", {"target": "HERO-M", "revision": detail["revision"], "config": bad})
        self.assertEqual(path.read_bytes(), before)

    def test_broken_toml_can_be_read_and_repaired(self):
        path = self.root / "Robotconfig/HERO-M/RobotConfig.toml"
        original = path.read_text(encoding="utf-8")
        path.write_text("[broken\n", encoding="utf-8")
        detail = self.get()
        self.assertIsNone(detail["config"])
        self.assertFalse(detail["validation"]["ok"])
        raw = self.workspace.dispatch("file.read", {"target": "HERO-M", "name": "RobotConfig.toml"})
        self.assertIn("[broken", raw["content"])
        repaired = self.workspace.dispatch("file.write", {"target": "HERO-M", "name": "RobotConfig.toml",
                                                            "revision": raw["revision"], "content": original})
        self.assertNotEqual(repaired["revision"], raw["revision"])
        self.assertTrue(self.get()["validation"]["ok"])

    def test_validate_rejects_candidate_board_and_unmigrated_service(self):
        detail = self.get()
        invalid_board = dict(detail["config"])
        invalid_board["board"] = "bad-board"
        result = self.workspace.dispatch("robot.validate", {"target": "HERO-M", "config": invalid_board,
                                                              "revision": detail["revision"]})
        self.assertFalse(result["ok"])
        self.assertIn("未支持", result["errors"][0])
        (self.root / "Robotconfig/SENTINEL-M/RobotConfig.toml").write_text("[broken\n", encoding="utf-8")
        self.assertTrue(self.workspace.dispatch("robot.validate", {"target": "HERO-M", "config": detail["config"]})["ok"])
        summary = self.workspace.dispatch("workspace.summary", {})
        self.assertFalse(next(item for item in summary["services"] if item["symbol"] == "ELRS_LINK")["available"])

    def test_comment_bom_and_crlf_are_preserved(self):
        path = self.root / "Robotconfig/HERO-M/RobotConfig.toml"
        original = path.read_text(encoding="utf-8")
        original = original.replace('services = ["RC_SBUS",', 'services = [\n    "RC_SBUS", # 保留数组内说明\n   ')
        path.write_bytes(b"\xef\xbb\xbf" + "# 保留这一行\r\n".encode("utf-8") + original.replace("\n", "\r\n").encode("utf-8"))
        detail = self.get()
        config = detail["config"]
        config["profile"] = "custom"
        saved = self.workspace.dispatch("robot.update", {"target": "HERO-M", "revision": detail["revision"], "config": config})
        data = path.read_bytes()
        self.assertTrue(data.startswith(b"\xef\xbb\xbf"))
        self.assertIn("# 保留这一行", data.decode("utf-8-sig"))
        self.assertIn('"RC_SBUS", # 保留数组内说明', data.decode("utf-8-sig"))
        self.assertIn(b"\r\n", data)
        self.assertEqual(saved["config"]["profile"], "custom")

    def test_path_escape_and_external_edit_conflict(self):
        with self.assertRaisesRegex(ValueError, "超出"):
            self.workspace.dispatch("file.read", {"target": "HERO-M", "name": "../README.md"})
        detail = self.get()
        path = self.root / "Robotconfig/HERO-M/RobotConfig.toml"
        path.write_bytes(path.read_bytes() + "\n# 外部编辑\n".encode("utf-8"))
        with self.assertRaisesRegex(ValueError, "外部修改"):
            self.workspace.dispatch("robot.update", {"target": "HERO-M", "revision": detail["revision"], "config": detail["config"]})

    def test_file_write_keeps_bom_and_newline(self):
        path = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        path.write_bytes(b"\xef\xbb\xbf" + "/* 中文 */\r\nint a;\r\n".encode("utf-8"))
        detail = self.workspace.dispatch("file.read", {"target": "HERO-M", "name": "ConfigTuning.inc"})
        result = self.workspace.dispatch("file.write", {"target": "HERO-M", "name": "ConfigTuning.inc", "revision": detail["revision"], "content": "/* 中文 */\nint b;\n"})
        self.assertEqual((result["encoding"], result["newline"]), ("UTF-8-BOM", "CRLF"))
        self.assertEqual(path.read_bytes(), b"\xef\xbb\xbf" + "/* 中文 */\r\nint b;\r\n".encode("utf-8"))

    def test_file_write_rejects_nul(self):
        detail = self.workspace.dispatch("file.read", {"target": "HERO-M", "name": "ConfigTuning.inc"})
        with self.assertRaisesRegex(ValueError, "NUL"):
            self.workspace.dispatch("file.write", {"target": "HERO-M", "name": "ConfigTuning.inc",
                                                     "revision": detail["revision"], "content": "a\0b"})

    def test_create_never_overwrites(self):
        result = self.workspace.dispatch("robot.create", {"name": "NEWBOT", "source": "HERO-M"})
        self.assertEqual(result["name"], "NEWBOT")
        with self.assertRaisesRegex(ValueError, "不覆盖"):
            self.workspace.dispatch("robot.create", {"name": "NEWBOT", "source": "HERO-M"})


if __name__ == "__main__":
    unittest.main()
