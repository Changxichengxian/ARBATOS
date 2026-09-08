import importlib.util
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("configedit_workspace", REPO / "client/backend/workspace.py")
WORKSPACE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKSPACE)


class ConfigEditTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "repo"
        shutil.copytree(REPO / "Robotconfig", self.root / "Robotconfig")
        shutil.copytree(REPO / "shared", self.root / "shared")
        self.workspace = WORKSPACE.Workspace(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def get(self, target="HERO-M"):
        return self.workspace.dispatch("config.get", {"target": target})

    @staticmethod
    def fields(detail):
        return {field["id"]: field for group in detail["groups"] for field in group["fields"]}

    def test_contract_covers_real_targets_and_requested_sections(self):
        for target in ("HERO-M", "SENTINEL-M", "MINIWHEELEG-M"):
            detail = self.get(target)
            self.assertEqual(detail["target"], target)
            self.assertRegex(detail["revision"], r"^[0-9a-f]{64}$")
            self.assertEqual(detail["models"]["MOTOR_MODEL_3510"]["ratio"], 19.0)
            self.assertEqual(detail["models"]["MOTOR_MODEL_N6014B"]["transport"], "RS485")
            categories = {group["category"] for group in detail["groups"]}
            self.assertTrue({"tuning", "motors", "input", "servo", "other"}.issubset(categories))
            fields = self.fields(detail)
            self.assertEqual([fields[f"tuning.chassis.motor_speed_pid[{i}]"]["label"] for i in range(5)],
                             list(WORKSPACE.CONFIGEDIT.PID_LABELS))
            self.assertIn("tuning.gimbal.yaw_middle_ecd", fields)
            self.assertIn("tuning.gimbal.pitch_middle_ecd", fields)
            self.assertEqual(len([key for key in fields if key.endswith(".model") and key.startswith("motor.")]), 18)
            self.assertEqual(fields["input.elrs.ElrsChMap.0"]["value"], 1)
            self.assertEqual(fields["input.elrs.ElrsChMap.0"]["options"][-1], {"value": 16, "label": "CH16"})
            self.assertIn("input.axis.INPUT_AXIS_CHASSIS_X.channel", fields)
            self.assertIn("servo.channels[3].centerUs", fields)
            for field in fields.values():
                self.assertIn(field["type"], {"number", "enum", "boolean", "text"})
                if "options" in field:
                    self.assertTrue(all(set(option) == {"value", "label"} for option in field["options"]))

    def test_tuning_update_changes_only_literal_and_preserves_bom_crlf_comment(self):
        path = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        text = path.read_text(encoding="utf-8")
        text = "// 保留页首说明\n" + text
        original = b"\xef\xbb\xbf" + text.replace("\n", "\r\n").encode("utf-8")
        path.write_bytes(original)
        detail = self.get()
        before = path.read_bytes()
        saved = self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                           "changes": {"tuning.gimbal.yaw_middle_ecd": 1777}})
        after = path.read_bytes()
        self.assertTrue(after.startswith(b"\xef\xbb\xbf"))
        self.assertNotIn(b"\n", after.replace(b"\r\n", b""))
        self.assertIn("// 保留页首说明".encode("utf-8"), after)
        self.assertEqual(after, before.replace(b".yaw_middle_ecd = 1677", b".yaw_middle_ecd = 1777", 1))
        self.assertNotEqual(saved["revision"], detail["revision"])

    def test_expression_is_read_only_and_is_never_evaluated(self):
        path = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        text = path.read_text(encoding="utf-8").replace(".yaw_middle_ecd = 1677", ".yaw_middle_ecd = 1600 + 77")
        path.write_text(text, encoding="utf-8")
        detail = self.get()
        field = self.fields(detail)["tuning.gimbal.yaw_middle_ecd"]
        self.assertEqual(field["type"], "text")
        self.assertFalse(field["editable"])
        self.assertEqual(field["value"], "1600 + 77")
        with self.assertRaisesRegex(ValueError, "未知或只读"):
            self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                       "changes": {field["id"]: 1777}})

    def test_motor_bus_resolves_only_a_direct_integer_macro(self):
        header = self.root / "Robotconfig/MINIWHEELEG-M/RobotConfig.h"
        header.write_text(header.read_text(encoding="utf-8").replace(
            "#define MINIWHEELEG_MIT_MOTOR_CAN_BUS 1u",
            "#define MINIWHEELEG_MIT_MOTOR_CAN_BUS 2u"), encoding="utf-8")
        fields = self.fields(self.get("MINIWHEELEG-M"))
        self.assertEqual(fields["motor.arm[0].bus"]["value"], "CAN2")

        header.write_text(header.read_text(encoding="utf-8").replace(
            "#define MINIWHEELEG_MIT_MOTOR_CAN_BUS 2u",
            "#define MINIWHEELEG_MIT_MOTOR_CAN_BUS (1u + 1u)"), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "无法安全解析整数表达式"):
            self.get("MINIWHEELEG-M")

    def test_rejects_invalid_values_bus_model_and_motor_conflicts(self):
        detail = self.get()
        bad_changes = (
            {"tuning.chassis.motor_speed_pid[0]": math.nan},
            {"motor.chassis[0].id": 9},
            {"motor.yaw.id": 5},
            {"motor.chassis[0].bus": "RS485_1"},
            {"motor.chassis[1].id": 1},
            {"motor.trigger.external_reduction_ratio": 2},
            {"motor.chassis[0].external_reduction_ratio": 0.5},
            {"motor.chassis[0].external_reduction_ratio": 10001},
            {"input.elrs.ElrsChMap.0": 17},
            {"servo.channels[0].minUs": 1500},
            {"servo.channels[0].stepUs": 0},
        )
        for changes in bad_changes:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                           "changes": changes})
        with self.assertRaises(ValueError):
            self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                       "changes": {"motor.chassis[0].model": "MOTOR_MODEL_N6014B"}})
        self.assertEqual(self.fields(detail)["motor.yaw.id"]["max"], 4)

        sentinel = self.get("SENTINEL-M")
        with self.assertRaisesRegex(ValueError, "反馈 ID 冲突"):
            self.workspace.dispatch("config.update", {"target": "SENTINEL-M", "revision": sentinel["revision"],
                "changes": {"motor.arm[2].id": 7, "motor.arm[2].bus": "CAN2"}})

    def test_motor_insert_uses_real_bus_and_keeps_existing_comment_and_comma(self):
        path = self.root / "Robotconfig/HERO-M/ConfigHardware.inc"
        before = path.read_text(encoding="utf-8")
        detail = self.get()
        saved = self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
            "changes": {"motor.pitch.external_reduction_ratio": 2.5, "motor.pitch.bus": "CAN2"}})
        after = path.read_text(encoding="utf-8")
        self.assertIn(".pitch = {MOTOR_MODEL_3510, 5u, .can_bus = 2u, .transport = MOTOR_TRANSPORT_CAN, "
                      ".external_reduction_ratio = 2.5f}", after)
        self.assertIn("// Included by RobotConfig.c", after)
        self.assertNotIn(", ,", after)
        fields = self.fields(saved)
        self.assertEqual(fields["motor.pitch.bus"]["value"], "CAN2")
        self.assertEqual(fields["motor.pitch.external_reduction_ratio"]["value"], 2.5)
        self.assertEqual(fields["motor.pitch.built_in_reduction_ratio"]["value"], 19.0)
        self.assertFalse(fields["motor.pitch.built_in_reduction_ratio"]["editable"])

        sentinel_path = self.root / "Robotconfig/SENTINEL-M/ConfigHardware.inc"
        sentinel = self.get("SENTINEL-M")
        self.workspace.dispatch("config.update", {"target": "SENTINEL-M", "revision": sentinel["revision"],
            "changes": {"motor.yaw.external_reduction_ratio": 2.0}})
        sentinel_text = sentinel_path.read_text(encoding="utf-8")
        self.assertIn(".transport = MOTOR_TRANSPORT_CAN,\n                    .external_reduction_ratio = 2.0f,", sentinel_text)
        self.assertNotIn(", ,", sentinel_text)

    def test_motor_model_migration_resets_protocol_specific_ids(self):
        path = self.root / "Robotconfig/SENTINEL-M/ConfigHardware.inc"
        detail = self.get("SENTINEL-M")
        before = self.fields(detail)
        self.assertEqual(before["motor.arm[0].id"]["value"], 1)
        self.assertEqual(before["motor.arm[0].feedback_id"]["value"], 0)

        saved = self.workspace.dispatch("config.update", {"target": "SENTINEL-M", "revision": detail["revision"],
            "changes": {"motor.arm[0].model": "MOTOR_MODEL_N6014B",
                        "motor.arm[0].bus": "RS485_1"}})
        row = next(line for line in path.read_text(encoding="utf-8").splitlines() if "[0] =" in line)
        self.assertIn(".model = MOTOR_MODEL_N6014B", row)
        self.assertIn(".can_id = 1u", row)
        self.assertIn(".protocol = MOTOR_PROTOCOL_INHERIT", row)
        self.assertIn(".control_mode = MOTOR_CONTROL_MODE_INHERIT", row)
        self.assertIn(".master_id = 0u", row)
        self.assertIn(".feedback_id = 0u", row)
        self.assertIn(".feedback_id_enable = 0u", row)
        fields = self.fields(saved)
        self.assertEqual(fields["motor.arm[0].id"]["value"], 1)
        self.assertEqual(fields["motor.arm[0].feedback_id"]["value"], 1)
        self.assertFalse(fields["motor.arm[0].feedback_id"]["editable"])

    def test_legacy_n6014_explicit_device_zero_is_not_shown_as_disabled(self):
        path = self.root / "Robotconfig/SENTINEL-M/ConfigHardware.inc"
        lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
        index = next(i for i, line in enumerate(lines) if "[0] =" in line)
        lines[index] = lines[index].replace("MOTOR_MODEL_DM_6215", "MOTOR_MODEL_N6014B").replace(
            "MOTOR_TRANSPORT_CAN", "MOTOR_TRANSPORT_RS485")
        path.write_text("".join(lines), encoding="utf-8")

        field = self.fields(self.get("SENTINEL-M"))["motor.arm[0].id"]
        self.assertEqual(field["value"], 0)
        self.assertNotIn("关闭", field["label"])
        self.assertIn("设备 0", field["description"])
        self.assertIn("当前确实会输出", field["description"])

    def test_n6014_feedback_override_does_not_hide_framework_disabled_can_id(self):
        path = self.root / "Robotconfig/SENTINEL-M/ConfigHardware.inc"
        lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
        index = next(i for i, line in enumerate(lines) if "[0] =" in line)
        lines[index] = lines[index].replace("MOTOR_MODEL_DM_6215", "MOTOR_MODEL_N6014B").replace(
            ".can_id = 1u", ".can_id = 0u").replace("MOTOR_TRANSPORT_CAN", "MOTOR_TRANSPORT_RS485").replace(
            ".feedback_id = 0u", ".feedback_id = 3u")
        path.write_text("".join(lines), encoding="utf-8")

        detail = self.get("SENTINEL-M")
        fields = self.fields(detail)
        self.assertEqual(fields["motor.arm[0].id"]["value"], 0)
        self.assertIn("0 表示关闭", fields["motor.arm[0].id"]["label"])
        self.assertIn("框架实例已关闭", fields["motor.arm[0].id"]["description"])
        self.assertEqual(fields["motor.arm[0].feedback_id"]["value"], 3)

        saved = self.workspace.dispatch("config.update", {"target": "SENTINEL-M", "revision": detail["revision"],
            "changes": {"motor.arm[0].id": 3}})
        row = next(line for line in path.read_text(encoding="utf-8").splitlines() if "[0] =" in line)
        self.assertIn(".can_id = 3u", row)
        self.assertIn(".feedback_id = 0u", row)
        self.assertIn(".feedback_id_enable = 0u", row)
        fields = self.fields(saved)
        self.assertEqual(fields["motor.arm[0].id"]["value"], 3)
        self.assertEqual(fields["motor.arm[0].feedback_id"]["value"], 3)

    def test_can_feedback_id_can_be_seen_changed_and_restored_to_auto(self):
        path = self.root / "Robotconfig/SENTINEL-M/ConfigHardware.inc"
        detail = self.get("SENTINEL-M")
        fields = self.fields(detail)
        self.assertEqual(fields["motor.arm[1].feedback_id"]["value"], 3)
        self.assertTrue(fields["motor.arm[1].feedback_id"]["editable"])

        automatic = self.workspace.dispatch("config.update", {"target": "SENTINEL-M",
            "revision": detail["revision"], "changes": {"motor.arm[1].feedback_id": -1}})
        row = next(line for line in path.read_text(encoding="utf-8").splitlines() if "[1] =" in line)
        self.assertIn(".master_id = 0u", row)
        self.assertIn(".feedback_id = 0u", row)
        self.assertIn(".feedback_id_enable = 0u", row)
        self.assertEqual(self.fields(automatic)["motor.arm[1].feedback_id"]["value"], 6)

        explicit = self.workspace.dispatch("config.update", {"target": "SENTINEL-M",
            "revision": automatic["revision"], "changes": {"motor.arm[1].feedback_id": 2}})
        self.assertEqual(self.fields(explicit)["motor.arm[1].feedback_id"]["value"], 2)
        row = next(line for line in path.read_text(encoding="utf-8").splitlines() if "[1] =" in line)
        self.assertIn(".feedback_id = 2u", row)
        self.assertIn(".feedback_id_enable = 1u", row)

    def test_rejects_c_literal_overflow_and_encoder_middle_out_of_range(self):
        detail = self.get()
        fields = self.fields(detail)
        self.assertEqual(fields["tuning.gimbal.yaw_middle_ecd"]["max"], 8191)
        for changes in (
            {"tuning.gimbal.yaw_middle_ecd": -1},
            {"tuning.gimbal.yaw_middle_ecd": 8192},
            {"tuning.chassis.motor_speed_pid[0]": WORKSPACE.CONFIGEDIT.FP32_MAX * 2.0},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                           "changes": changes})
        with self.assertRaises(ValueError):
            WORKSPACE.CONFIGEDIT._format_number(-1, "1u", True)
        with self.assertRaises(ValueError):
            WORKSPACE.CONFIGEDIT._format_number(10 ** 400, "1", True)

    def test_pitch_middle_and_servo_virtual_fields_create_one_config_source(self):
        tuning = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        hardware = self.root / "Robotconfig/HERO-M/ConfigHardware.inc"
        detail = self.get()
        changes = {
            "tuning.gimbal.pitch_middle_ecd": 2345,
            "servo.channels[0].enabled": True,
            "servo.channels[0].port": 2,
            "servo.channels[0].inputMode": 1,
            "servo.channels[0].inputChannel": 5,
            "servo.channels[0].centerUs": 1520,
        }
        saved = self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                                                           "changes": changes})
        tuning_text = tuning.read_text(encoding="utf-8")
        hardware_text = hardware.read_text(encoding="utf-8")
        self.assertIn(".pitch_middle_ecd = 2345u", tuning_text)
        self.assertIn(".pitch_middle_ecd_enable = 1u", tuning_text)
        self.assertEqual(hardware_text.count(".servo ="), 1)
        self.assertIn(".configured = 1u", hardware_text)
        self.assertIn(".enabled = 1u, .port = 1u, .inputMode = 1u, .inputChannel = 4u", hardware_text)
        self.assertEqual(self.fields(saved)["servo.channels[0].centerUs"]["value"], 1520)

    def test_revision_covers_every_config_file_and_path_is_rejected(self):
        detail = self.get()
        operation = self.root / "Robotconfig/HERO-M/ConfigOperation.inc"
        operation.write_bytes(operation.read_bytes() + b"// external\n")
        tuning = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        before = tuning.read_bytes()
        with self.assertRaisesRegex(ValueError, "外部修改"):
            self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                "changes": {"tuning.gimbal.yaw_middle_ecd": 1700}})
        self.assertEqual(tuning.read_bytes(), before)
        with self.assertRaises(ValueError):
            self.get("../HERO-M")

    def test_multi_file_write_rolls_back_if_second_replace_fails(self):
        detail = self.get()
        tuning = self.root / "Robotconfig/HERO-M/ConfigTuning.inc"
        inputs = self.root / "Robotconfig/HERO-M/ConfigInput.inc"
        before_tuning, before_inputs = tuning.read_bytes(), inputs.read_bytes()
        real_replace = os.replace
        calls = 0

        def fail_second(source, destination):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError("injected replace failure")
            return real_replace(source, destination)

        with mock.patch.object(WORKSPACE.CONFIGEDIT.os, "replace", side_effect=fail_second):
            with self.assertRaisesRegex(ValueError, "injected replace failure"):
                self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
                    "changes": {"tuning.gimbal.yaw_middle_ecd": 1800, "input.elrs.ElrsChMap.0": 16}})
        self.assertEqual(tuning.read_bytes(), before_tuning)
        self.assertEqual(inputs.read_bytes(), before_inputs)

    def test_two_file_update_saves_once_and_returns_new_revision(self):
        detail = self.get()
        saved = self.workspace.dispatch("config.update", {"target": "HERO-M", "revision": detail["revision"],
            "changes": {"tuning.gimbal.yaw_middle_ecd": 1800, "input.elrs.ElrsChMap.0": 16}})
        self.assertNotEqual(saved["revision"], detail["revision"])
        self.assertEqual(self.fields(saved)["tuning.gimbal.yaw_middle_ecd"]["value"], 1800)
        self.assertEqual(self.fields(saved)["input.elrs.ElrsChMap.0"]["value"], 16)


if __name__ == "__main__":
    unittest.main()
