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
        catalog = self.root / "boards" / "DmMc02H7" / "PortCatalog.toml"
        catalog.parent.mkdir(parents=True)
        shutil.copy2(REPO / "boards" / "DmMc02H7" / "PortCatalog.toml", catalog)
        c_catalog = self.root / "boards/DjiCF407/PortCatalog.toml"
        c_catalog.parent.mkdir(parents=True)
        shutil.copy2(REPO / "boards/DjiCF407/PortCatalog.toml", c_catalog)
        a_catalog = self.root / "boards/DjiAF427/PortCatalog.toml"
        a_catalog.parent.mkdir(parents=True)
        shutil.copy2(REPO / "boards/DjiAF427/PortCatalog.toml", a_catalog)
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

    def test_validate_rejects_candidate_board_and_reports_external_elrs(self):
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
        self.assertTrue(next(item for item in summary["services"] if item["symbol"] == "ELRS_LINK")["available"])

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

    def test_vehicle_setup_catalog_and_wheelleg_gimbal_shoot_create(self):
        catalog = self.workspace.dispatch("vehicle_setup.catalog", {})
        boards = {item["id"]: item for item in catalog["boards"]}
        self.assertEqual(catalog["schema"], 1)
        self.assertTrue(boards["dm_mc02_h7"]["available"])
        self.assertNotIn("a_classic_single", boards["dm_mc02_h7"]["templates"])
        self.assertTrue(boards["dji_c_f407"]["available"])
        self.assertEqual(boards["dji_c_f407"]["templates"], ["c_classic_single"])
        mechanisms = {item["id"]: item for item in catalog["mechanisms"]}
        gimbal_group = mechanisms["single_gimbal"]["motor_groups"][0]
        self.assertEqual(gimbal_group["defaults_by_board"]["dm_mc02_h7"][0],
                         {"model": "MOTOR_MODEL_6020", "id": 1, "bus": "CAN2"})
        self.assertEqual(gimbal_group["defaults_by_board"]["dji_a_f427"][0]["bus"], "CAN1")
        pwm_group = mechanisms["pwm_servos"]["motor_groups"][0]
        self.assertEqual(pwm_group["defaults"], [])
        self.assertEqual(pwm_group["empty_label"], "不启用")
        result = self.workspace.dispatch("robot.create", {"name": "WLGIMBAL", "setup": {
            "board": "dm_mc02_h7", "template": "wheelleg",
            "mechanisms": ["wheelleg", "single_gimbal", "rm_shoot"], "radio": "dji_sbus",
            "power": {"limit_mode": "Auto", "power_limit": 90, "no_judge_total_current_limit": 36000}}})
        self.assertEqual(result["based_on"], "MINIWHEELEG-M")
        self.assertTrue(result["validation"]["ok"])
        self.assertTrue(result["setup"]["direction_check_required"])
        config = result["detail"]["config"]
        self.assertEqual(config["controllers"]["wheelleg"]["type"], "mit")
        self.assertEqual(config["controllers"]["gimbal"]["type"], "single")
        self.assertEqual(config["controllers"]["shoot"]["type"], "rm")
        self.assertEqual(result["setup"]["power"]["limitMode"], "Auto")
        self.assertTrue(result["setup"]["commissioning_required"])
        self.assertFalse(result["setup"]["motion_ready"])
        self.assertTrue(any("独立 IMU" in item for item in result["setup"]["verification_required"]))
        tuning = (self.root / "Robotconfig/WLGIMBAL/ConfigTuning.inc").read_text(encoding="utf-8")
        self.assertIn(".limitMode = ChassisPowerLimitAuto", tuning)
        self.assertIn(".power_limit = 90.0f", tuning)
        hardware = (self.root / "Robotconfig/WLGIMBAL/ConfigHardware.inc").read_text(encoding="utf-8")
        for value in (".can_id = 1u, .can_bus = 2u", ".can_id = 2u, .can_bus = 2u",
                      ".can_id = 3u, .can_bus = 2u"):
            self.assertIn(value, hardware)
        self.assertEqual(hardware.count(".control_mode = MOTOR_CONTROL_MODE_MIT"), 6)
        self.assertIn(".fric_speed_rpm = 4000.0f", tuning)
        self.assertIn(".max_joint_torque_nm = 3.0f", tuning)
        input_text = (self.root / "Robotconfig/WLGIMBAL/ConfigInput.inc").read_text(encoding="utf-8")
        operation = (self.root / "Robotconfig/WLGIMBAL/ConfigOperation.inc").read_text(encoding="utf-8")
        self.assertIn(".active_source = MANUAL_INPUT_SRC_DBUS", input_text)
        self.assertIn(".mode = ROBOT_RUN_MODE_SINGLE_MOTOR", operation)
        self.assertIn(".target_motor = (uint8_t)MotorCount", operation)
        note = (self.root / "Robotconfig/WLGIMBAL/安装说明.md").read_text(encoding="utf-8")
        self.assertIn("编译通过只说明配置和代码能组成固件", note)
        self.assertIn("单板 IMU 的物理归属不能由向导推断", note)

    def test_vehicle_setup_supported_combinations_apply_real_axes(self):
        cases = (
            ("WLGIMBALONLY", "wheelleg", ["wheelleg", "single_gimbal"], False),
            ("CLASSICFULL", "classic_single", ["classic_chassis", "single_gimbal", "rm_shoot"], True),
            ("DUALFULL", "classic_dual_yaw", ["classic_chassis", "dual_yaw_gimbal", "rm_shoot"], True),
        )
        for name, template, mechanisms, has_shoot in cases:
            result = self.workspace.dispatch("robot.create", {"name": name, "setup": {
                "board": "dm_mc02_h7", "template": template, "mechanisms": mechanisms,
                "radio": "elrs_crsf", "motors": {"chassis": ["MOTOR_MODEL_3510", "", "", ""]}
                if "classic_chassis" in mechanisms else {"wheelleg": ["", "", "", "", "", ""]}}})
            self.assertTrue(result["validation"]["ok"])
            config = result["detail"]["config"]
            self.assertEqual(config["services"].count("ELRS_LINK"), 1)
            self.assertNotIn("RC_SBUS", config["services"])
            self.assertEqual(config["ports"]["elrs_crsf"]["port"], "uart5")
            input_text = (self.root / "Robotconfig" / name / "ConfigInput.inc").read_text(encoding="utf-8")
            hardware = (self.root / "Robotconfig" / name / "ConfigHardware.inc").read_text(encoding="utf-8")
            self.assertIn(".active_source = MANUAL_INPUT_SRC_ELRS", input_text)
            if has_shoot:
                self.assertIn(".model = MOTOR_MODEL_2006, .can_id = 3u, .can_bus = 2u", hardware)
                self.assertEqual(config["controllers"]["shoot"]["type"], "rm")
                self.assertIn(".model = MOTOR_MODEL_3510, .can_id = 1u, .can_bus = 1u", hardware)
                expected_gimbal = "dual_yaw" if "dual_yaw_gimbal" in mechanisms else "single"
                self.assertEqual(config["controllers"]["gimbal"]["type"], expected_gimbal)
            else:
                self.assertNotIn("shoot", config["controllers"])
                self.assertIn(".model = MOTOR_MODEL_2006, .can_id = 0u, .can_bus = 2u", hardware)

    def test_vehicle_setup_shoot_without_gimbal_creates_independent_task(self):
        result = self.workspace.dispatch("robot.create", {"name": "SHOOTONLY", "setup": {
            "board": "dm_mc02_h7", "template": "classic_single",
            "mechanisms": ["classic_chassis", "rm_shoot"], "radio": "dji_sbus"}})
        self.assertTrue(result["validation"]["ok"])
        config = result["detail"]["config"]
        self.assertEqual(config["controllers"], {"chassis": {"type": "classic"}, "shoot": {"type": "rm"}})
        selected = {task["symbol"] for task in result["detail"]["resolved"]["tasks"]}
        self.assertIn("SHOOT_RM", selected)
        self.assertFalse(selected & {"SINGLE_GIMBAL", "DUAL_YAW_GIMBAL", "CONTROL_GIMBAL"})
        operation = (self.root / "Robotconfig/SHOOTONLY/ConfigOperation.inc").read_text(encoding="utf-8")
        self.assertIn(".mode = ROBOT_RUN_MODE_SINGLE_MOTOR", operation)
        self.assertIn(".target_motor = (uint8_t)MotorCount", operation)

    def test_vehicle_setup_string_models_and_collision_rollback(self):
        result = self.workspace.dispatch("robot.create", {"name": "CUSTOMMOTOR", "setup": {
            "board": "dm_mc02_h7", "template": "classic_single",
            "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"], "radio": "dji_sbus",
            "motors": {"chassis": ["MOTOR_MODEL_3510", "", "", ""],
                       "shoot": ["", "", "MOTOR_MODEL_3508"]}}})
        self.assertTrue(result["validation"]["ok"])
        hardware = (self.root / "Robotconfig/CUSTOMMOTOR/ConfigHardware.inc").read_text(encoding="utf-8")
        self.assertIn(".model = MOTOR_MODEL_3510, .can_id = 1u, .can_bus = 1u", hardware)
        self.assertIn(".model = MOTOR_MODEL_3508, .can_id = 3u, .can_bus = 2u", hardware)

        with self.assertRaisesRegex(ValueError, "ID 冲突"):
            self.workspace.dispatch("robot.create", {"name": "COLLISION", "setup": {
                "board": "dm_mc02_h7", "template": "classic_single",
                "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"], "radio": "dji_sbus",
                "motors": {"gimbal": [{"model": "MOTOR_MODEL_3508", "id": 1, "bus": "CAN2"}, ""]}}})
        self.assertFalse((self.root / "Robotconfig/COLLISION").exists())

    def test_vehicle_setup_rejects_unsupported_model_and_can_enable_pwm(self):
        with self.assertRaisesRegex(ValueError, "不支持型号"):
            self.workspace.dispatch("robot.create", {"name": "BADMODEL", "setup": {
                "board": "dm_mc02_h7", "template": "classic_single",
                "mechanisms": ["classic_chassis"], "radio": "dji_sbus",
                "motors": {"chassis": ["MOTOR_MODEL_DM_J4310_2EC_V12"]}}})
        self.assertFalse((self.root / "Robotconfig/BADMODEL").exists())

        result = self.workspace.dispatch("robot.create", {"name": "WITHPWM", "setup": {
            "board": "dm_mc02_h7", "template": "classic_single",
            "mechanisms": ["classic_chassis", "pwm_servos"], "radio": "dji_sbus",
            "motors": {"pwm_servos": ["PWM_SERVO", "", "PWM_SERVO", ""]}}})
        self.assertTrue(result["validation"]["ok"])
        self.assertIn("SERVO", result["detail"]["config"]["services"])
        hardware = (self.root / "Robotconfig/WITHPWM/ConfigHardware.inc").read_text(encoding="utf-8")
        self.assertIn(".configured = 1u", hardware)
        self.assertEqual(hardware.count(".enabled = 1u"), 2)

    def test_a_board_setup_uses_existing_supported_subset_and_stays_safe(self):
        catalog = self.workspace.dispatch("vehicle_setup.catalog", {})
        board = next(item for item in catalog["boards"] if item["id"] == "dji_a_f427")
        self.assertTrue(board["available"])
        self.assertEqual(board["templates"], ["a_classic_single"])
        result = self.workspace.dispatch("robot.create", {"name": "ANEWBOT", "setup": {
            "board": "dji_a_f427", "template": "a_classic_single",
            "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"], "radio": "dji_sbus"}})
        self.assertTrue(result["validation"]["ok"])
        self.assertEqual(result["based_on"], "INFANTRY-TIAN-A")
        self.assertEqual(result["detail"]["config"]["profile"], "infantry")
        self.assertEqual(result["detail"]["config"]["ports"]["rc_sbus"], "usart1")
        operation = (self.root / "Robotconfig/ANEWBOT/ConfigOperation.inc").read_text(encoding="utf-8")
        tuning = (self.root / "Robotconfig/ANEWBOT/ConfigTuning.inc").read_text(encoding="utf-8")
        hardware = (self.root / "Robotconfig/ANEWBOT/ConfigHardware.inc").read_text(encoding="utf-8")
        self.assertIn("ROBOT_RUN_MODE_SINGLE_MOTOR", operation)
        self.assertIn(".yaw_middle_ecd = 8186", tuning)
        self.assertIn(".model = MOTOR_MODEL_3508, .can_id = 3u, .can_bus = 1u", hardware)
        self.assertIn(".model = MOTOR_MODEL_2006, .can_id = 7u, .can_bus = 2u", hardware)
        elrs = self.workspace.dispatch("robot.create", {"name": "ABADELRS", "setup": {
            "board": "dji_a_f427", "template": "a_classic_single",
            "mechanisms": ["classic_chassis"], "radio": "elrs_crsf"}})
        self.assertEqual(elrs["detail"]["config"]["ports"]["elrs_crsf"]["port"], "uart8")

    def test_imu_mount_reads_and_updates_only_right_handed_mapping(self):
        detail = self.workspace.dispatch("imu.mount", {"target": "HERO-M"})
        self.assertEqual(detail["axes"], ["+X", "+Y", "+Z"])
        self.assertTrue(detail["configured"])
        self.assertTrue(detail["requiresBuild"])
        saved = self.workspace.dispatch("imu.mount.update", {
            "target": "HERO-M", "axes": ["+Y", "-X", "+Z"], "revision": detail["revision"]})
        self.assertEqual(saved["axes"], ["+Y", "-X", "+Z"])
        self.assertNotEqual(saved["revision"], detail["revision"])
        mount = (self.root / "Robotconfig/HERO-M/ImuMount.h").read_text(encoding="utf-8")
        self.assertIn("ROBOT_IMU_FRAME_VERSION 3u", mount)
        with self.assertRaisesRegex(ValueError, "右手"):
            self.workspace.dispatch("imu.mount.update", {
                "target": "HERO-M", "axes": ["+X", "+Y", "-Z"], "revision": saved["revision"]})
        with self.assertRaisesRegex(ValueError, "外部修改"):
            self.workspace.dispatch("imu.mount.update", {
                "target": "HERO-M", "axes": ["+X", "+Y", "+Z"], "revision": detail["revision"]})

    def test_imu_mount_preserves_format_and_rejects_extra_logic(self):
        detail = self.workspace.dispatch("imu.mount", {"target": "SENTINEL-M"})
        self.assertTrue(detail["configured"])
        self.assertEqual(detail["axes"], ["+Y", "-X", "+Z"])
        path = self.root / "Robotconfig/SENTINEL-M/ImuMount.h"
        original = path.read_text(encoding="utf-8")
        path.write_bytes(b"\xef\xbb\xbf" + original.replace("\n", "\r\n").encode("utf-8"))
        reread = self.workspace.dispatch("imu.mount", {"target": "SENTINEL-M"})
        self.workspace.dispatch("imu.mount.update", {
            "target": "SENTINEL-M", "axes": ["+X", "+Y", "+Z"], "revision": reread["revision"]})
        saved = path.read_bytes()
        self.assertTrue(saved.startswith(b"\xef\xbb\xbf"))
        self.assertIn(b"\r\n", saved)
        self.assertIn("实际安装仍需实车确认", saved.decode("utf-8-sig"))
        bad = saved.replace(b"    out[2] = raw[2];", b"    out[2] = raw[2];\r\n    out[2] += 0.0f;")
        path.write_bytes(bad)
        blocked = self.workspace.dispatch("file.read", {"target": "SENTINEL-M", "name": "ImuMount.h"})
        with self.assertRaisesRegex(ValueError, "只能包含"):
            self.workspace.dispatch("imu.mount.update", {
                "target": "SENTINEL-M", "axes": ["+Y", "-X", "+Z"], "revision": blocked["revision"]})
        with self.assertRaises(ValueError):
            self.workspace.dispatch("imu.mount.update", {
                "target": "HERO-M", "axes": [["+X"], "+Y", "+Z"], "revision": detail["revision"]})

    def test_imu_mount_reports_external_hi14_as_unsupported(self):
        detail = self.get("HERO-M")
        config = detail["config"]
        config["ports"]["external_imu"] = {"port": "usart10", "baud": 115200, "protocol": "hipnuc_hi14"}
        self.workspace.dispatch("robot.update", {"target": "HERO-M", "revision": detail["revision"], "config": config})
        mount = self.workspace.dispatch("imu.mount", {"target": "HERO-M"})
        self.assertTrue(mount["unsupported"])
        self.assertIn("HI14", mount["reason"])
        with self.assertRaisesRegex(ValueError, "HI14"):
            self.workspace.dispatch("imu.mount.update", {
                "target": "HERO-M", "axes": ["+Y", "-X", "+Z"], "revision": "0" * 64})

    def test_classic_setup_power_round_trip_and_wheelleg_rejects_it(self):
        result = self.workspace.dispatch("robot.create", {"name": "CLASSICPOWER", "setup": {
            "board": "dm_mc02_h7", "template": "classic_single",
            "mechanisms": ["classic_chassis", "single_gimbal"], "radio": "dji_sbus",
            "power": {"limit_mode": "Model", "power_limit": 75}}})
        self.assertEqual(result["setup"]["power"], {"limitMode": "Model", "power_limit": 75})
        text = (self.root / "Robotconfig/CLASSICPOWER/ConfigTuning.inc").read_text(encoding="utf-8")
        self.assertIn(".limitMode = ChassisPowerLimitModel", text)
        self.assertIn(".power_limit = 75.0f", text)
        with self.assertRaisesRegex(ValueError, "轮腿"):
            self.workspace.dispatch("robot.create", {"name": "BADWHEELPOWER", "setup": {
                "board": "dm_mc02_h7", "template": "wheelleg", "mechanisms": ["wheelleg"],
                "radio": "dji_sbus", "power": {"limitMode": "Model"}}})

    def test_c_board_setup_creates_real_board_and_receiver_choices(self):
        for name, radio, port_role, port in (
                ("CDBUS", "dji_sbus", "rc_sbus", "usart3"),
                ("CELRS", "elrs_crsf", "elrs_crsf", "usart1")):
            with self.subTest(radio=radio):
                result = self.workspace.dispatch("robot.create", {"name": name, "setup": {
                    "board": "dji_c_f407", "template": "c_classic_single",
                    "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"], "radio": radio,
                    "motors": {"gimbal": ["MOTOR_MODEL_6623", ""]},
                    "power": {"limit_mode": "Model"}}})
                self.assertTrue(result["validation"]["ok"])
                self.assertEqual(result["based_on"], "BASE-C")
                config = result["detail"]["config"]
                self.assertEqual(config["board"], "dji_c_f407")
                self.assertEqual(config["controllers"]["gimbal"]["type"], "single")
                assignment = config["ports"][port_role]
                self.assertEqual(assignment if isinstance(assignment, str) else assignment["port"], port)
                self.assertNotIn("InfantryDiag.c", config["build"]["sources"])
                self.assertNotIn("SDLOG", config["services"])
                path = self.root / "Robotconfig" / name
                self.assertIn("MOTOR_MODEL_6623", (path / "ConfigHardware.inc").read_text(encoding="utf-8"))
                self.assertIn(".target_motor = (uint8_t)MotorCount", (path / "ConfigOperation.inc").read_text(encoding="utf-8"))
                self.assertIn("ChassisPowerLimitModel", (path / "ConfigTuning.inc").read_text(encoding="utf-8"))
                source = "ELRS" if radio == "elrs_crsf" else "DBUS"
                self.assertIn(".active_source = MANUAL_INPUT_SRC_" + source,
                              (path / "ConfigInput.inc").read_text(encoding="utf-8"))

    def test_c_board_setup_rejects_foreign_template_without_directory(self):
        with self.assertRaisesRegex(ValueError, "不匹配"):
            self.workspace.dispatch("robot.create", {"name": "CBOT", "setup": {
                "board": "dji_c_f407", "template": "classic_single",
                "mechanisms": ["classic_chassis"], "radio": "dji_sbus"}})
        self.assertFalse((self.root / "Robotconfig/CBOT").exists())


if __name__ == "__main__":
    unittest.main()
