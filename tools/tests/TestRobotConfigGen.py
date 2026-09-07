import importlib.util
import tempfile
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("robot_config_gen", REPO / "tools/config/RobotConfigGen.py")
GEN = importlib.util.module_from_spec(spec)
spec.loader.exec_module(GEN)

def row(symbol, ident, kind="Service", deps="", header="Task.h", entry="Task", sm=512, so=256, priority="Normal", source="@target/RobotConfig.c"):
    return (f'ROBOT_TASK({symbol}, {ident}, "module.{symbol.lower()}", "task.{symbol.lower()}", '
            f'RobotModuleKind{kind}, 0u, 0u, {sm}u, {so}u, RobotModulePriority{priority}, '
            f'ROBOT_MODULE_FLAG_HAS_TASK, Generic, "{header}", {entry}, {ident}u, "{deps}", "{source}")')

class RobotConfigGenTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "repo"
        self.catalog = self.root / "shared/application/robot/RobotTaskCatalog.def"
        self.catalog.parent.mkdir(parents=True)
        self.write_catalog()
        self.target("BASE")

    def tearDown(self):
        self.tmp.cleanup()

    def write_catalog(self, extra=""):
        rows = [
            row("RC", 1, "Input", header="Rc.h", entry="RcTask", priority="AboveNormal"),
            row("CAN_COMMAND_TX", 2, "Comm", header="Can.h", entry="CanTxTask", priority="AboveNormal"),
            row("CAN_FEEDBACK_RX", 3, "Comm", header="Can.h", entry="CanRxTask", priority="High"),
            row("CLASSIC_CHASSIS", 4, "Control", "CAN_COMMAND_TX|CAN_FEEDBACK_RX", "Classic.h", "ClassicTask"),
            row("CONTROL_CHASSIS", 5, "Control", "CAN_COMMAND_TX|CAN_FEEDBACK_RX", "Control.h", "ControlChassisTask"),
            row("CONTROL_GIMBAL", 6, "Control", "CAN_COMMAND_TX|CAN_FEEDBACK_RX", "Control.h", "ControlGimbalTask"),
            row("IMU", 7, "Device", header="Imu.h", entry="ImuTask", priority="Realtime"),
            row("WHEELLEG_SERVO", 8, "Control", header="", entry="None"),
            row("LOGGER", 9, header="Log.h", entry="LogTask", sm=768, so=384)]
        self.catalog.write_text("\n".join(rows) + "\n" + extra, encoding="utf-8")

    def target(self, name, content=None):
        directory = self.root / "Robotconfig" / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "RobotConfig.c").write_text("int config;\n", encoding="utf-8")
        (directory / "ConfigOperation.inc").write_text(
            '.mode = ROBOT_RUN_MODE_NORMAL,\n.target_motor = 0u,\n.variant = ROBOT_RUN_VARIANT_FAST,\n', encoding="utf-8")
        (directory / "RobotConfig.toml").write_text(content or '''schema = 1
board = "dm_mc02_h7"
profile = "custom"
services = ["RC", "LOGGER"]
[build]
sources = ["RobotConfig.c"]
''', encoding="utf-8")

    def resolve(self, name="BASE"):
        return GEN.resolve(self.root, name)

    def bad(self, text, name="BAD"):
        with self.assertRaisesRegex(GEN.ConfigError, text):
            self.resolve(name)

    def plugin(self, requires="", outputs=2):
        d = self.root / "shared/controllers/demo"; d.mkdir(parents=True)
        (d / "Demo.h").write_text("", encoding="utf-8"); (d / "Demo.c").write_text("", encoding="utf-8")
        (d / "Controller.toml").write_text(f'''schema = 1
name = "demo"
domain = "chassis"
header = "Demo.h"
symbol = "DemoController"
sources = ["Demo.c"]
requires = [{requires}]
outputs = {outputs}
[parameters.gain]
min = 0.0
max = 2.0
''', encoding="utf-8")

    def test_real_old_targets_keep_tasks_and_key_properties(self):
        expected = {"HERO-M": {"RC_SBUS","HEALTH_MONITOR","SDLOG","BATTERY_MONITOR","REFEREE_RX","CLASSIC_CHASSIS","SINGLE_GIMBAL","CAN_COMMAND_TX","CAN_FEEDBACK_RX","IMU"}, "SENTINEL-M": {"RC_SBUS","HEALTH_MONITOR","SDLOG","HOST_LINK","CLASSIC_CHASSIS","DUAL_YAW_GIMBAL","CAN_COMMAND_TX","CAN_FEEDBACK_RX","IMU"}, "MINIWHEELEG-M": {"RC_SBUS","HEALTH_MONITOR","SDLOG","BATTERY_MONITOR","REFEREE_RX","WHEELLEG_MIT","CAN_COMMAND_TX","CAN_FEEDBACK_RX","IMU"}}
        for name, tasks in expected.items():
            rows = {x["symbol"]: x for x in GEN.resolve(REPO, name)["tasks"]}
            self.assertEqual(set(rows), tasks)
            self.assertEqual((rows["CAN_COMMAND_TX"]["stack_words"], rows["CAN_COMMAND_TX"]["priority"]), (512, "AboveNormal"))

    def test_new_target_discovery_and_identity(self):
        self.target("NEW")
        self.assertIn("NEW", {x["name"] for x in GEN.targets(self.root)})
        self.assertEqual(self.resolve("NEW")["name"], "NEW")

    def test_controller_manifest_rejects_ambiguous_symbols_and_invalid_outputs(self):
        self.plugin()
        manifest = self.root / "shared/controllers/demo/Controller.toml"
        original = manifest.read_text(encoding="utf-8")
        manifest.write_text(original.replace("outputs = 2", "outputs = 0"), encoding="utf-8")
        with self.assertRaisesRegex(GEN.ConfigError, "outputs"):
            self.resolve()
        manifest.write_text(original.replace('name = "demo"', 'name = "none"'), encoding="utf-8")
        with self.assertRaisesRegex(GEN.ConfigError, "保留名字"):
            self.resolve()
        manifest.write_text(original, encoding="utf-8")
        second = self.root / "shared/controllers/another"
        second.mkdir()
        (second / "Controller.toml").write_text(original.replace('name = "demo"', 'name = "another"'), encoding="utf-8")
        (second / "Demo.c").write_text("", encoding="utf-8")
        (second / "Demo.h").write_text("", encoding="utf-8")
        with self.assertRaisesRegex(GEN.ConfigError, "重复 C 导出符号"):
            self.resolve()

    def test_profile_header_runtime_same_selection(self):
        out = self.root / "out"; GEN.generate(self.root, self.resolve(), out)
        for token, file in [("ROBOT_TASK_MODULE_RC", "RobotTargetProfile.inc"), ("ROBOT_TASK_BUILD_RC 1", "RobotTargetConfig.h"), ("ROBOT_RUNTIME_TASK(RC,", "RobotTargetTasks.inc")]:
            self.assertIn(token, (out / file).read_text(encoding="utf-8"))

    def test_unknown_and_duplicate_service(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["NOPE"]\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("NOPE")
        self.target("DUP", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["RC", "RC"]\n[build]\nsources = ["RobotConfig.c"]\n')
        with self.assertRaisesRegex(GEN.ConfigError, "重复"): self.resolve("DUP")

    def test_unknown_field_and_path_escape(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nextra = 1\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("未知字段")
        self.target("ESC", 'schema = 1\nboard = "dm_mc02_h7"\n[build]\nsources = ["../x.c"]\n')
        with self.assertRaisesRegex(GEN.ConfigError, "路径超出"): self.resolve("ESC")

    def test_unknown_board_and_missing_config_source(self):
        self.target("BAD", 'schema = 1\nboard = "missing"\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("未支持")
        (self.root / "Robotconfig/BAD/RobotConfig.toml").unlink()
        self.target("SOURCE", 'schema = 1\nboard = "dm_mc02_h7"\n[build]\nsources = []\n')
        with self.assertRaisesRegex(GEN.ConfigError, "必须包含"): self.resolve("SOURCE")

    def test_unknown_controller_and_invalid_feature(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\n[controllers.chassis]\ntype = "missing"\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("找不到控制器")
        self.target("FEATURE", 'schema = 1\nboard = "dm_mc02_h7"\n[features]\nsubboard_music = "yes"\n[build]\nsources = ["RobotConfig.c"]\n')
        with self.assertRaisesRegex(GEN.ConfigError, "只能为 true/false"): self.resolve("FEATURE")

    def test_shoot_requires_its_current_execution_task(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\n[controllers.shoot]\ntype = "rm"\n[build]\nsources = ["RobotConfig.c"]\n')
        self.bad("没有执行入口")

    def test_catalog_duplicate_id_and_cycle(self):
        self.write_catalog(row("DUP", 1) + "\n")
        with self.assertRaisesRegex(GEN.ConfigError, "编号无效"): self.resolve()
        self.write_catalog(row("A",20,deps="B")+"\n"+row("B",21,deps="A")+"\n")
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["A"]\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("依赖循环")

    def test_unimplemented_wheelleg_servo_rejected(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["WHEELLEG_SERVO"]\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("WHEELLEG_SERVO")

    def test_selected_task_without_source_is_rejected_before_generation(self):
        self.write_catalog(row("UNMIGRATED", 31, header="Missing.h", entry="MissingTask", source="") + "\n")
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["UNMIGRATED"]\n[build]\nsources = ["RobotConfig.c"]\n')
        self.bad("尚未迁移实现源")

    def test_selected_service_source_is_automatically_added_once(self):
        source = self.root / "shared/AutoTask.c"; source.parent.mkdir(exist_ok=True); source.write_text("", encoding="utf-8")
        self.write_catalog(row("AUTO", 30, header="Auto.h", entry="AutoTask", source="shared/AutoTask.c") + "\n")
        self.target("AUTO", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["AUTO"]\n[build]\nsources = ["RobotConfig.c"]\nshared_sources = ["shared/AutoTask.c"]\n')
        self.assertEqual(self.resolve("AUTO")["sources"].count("shared/AutoTask.c"), 1)

    def test_plugin_missing_parameter_range_and_output(self):
        self.plugin()
        base = '''schema = 1
board = "dm_mc02_h7"
[controllers.chassis]
type = "demo"
motors = [%s]
%s
[build]
sources = ["RobotConfig.c"]
'''
        self.target("BAD", base % ('"a", "b"', "")); self.bad("缺少参数")
        self.target("RANGE", base % ('"a", "b"', "[controllers.chassis.parameters]\ngain = 3.0"))
        with self.assertRaisesRegex(GEN.ConfigError, "必须在"): self.resolve("RANGE")
        self.target("COUNT", base % ('"a"', "[controllers.chassis.parameters]\ngain = 1.0"))
        with self.assertRaisesRegex(GEN.ConfigError, "需要 2 个电机"): self.resolve("COUNT")

    def test_plugin_dependency_and_generic_period(self):
        self.plugin('"IMU"')
        self.target("PLUGIN", '''schema = 1
board = "dm_mc02_h7"
[controllers.chassis]
type = "demo"
motors = ["a", "b"]
period_ms = 7
[controllers.chassis.parameters]
gain = 1.0
[build]
sources = ["RobotConfig.c"]
''')
        out = self.root / "plugin-out"; GEN.generate(self.root, self.resolve("PLUGIN"), out)
        self.assertIn("IMU", {x["symbol"] for x in self.resolve("PLUGIN")["tasks"]})
        self.assertTrue(self.resolve("PLUGIN")["controllers"][0]["requires_imu"])
        self.assertIn("ROBOT_CONTROL_CHASSIS_PERIOD_MS 7u", (out / "RobotTargetConfig.h").read_text(encoding="utf-8"))

    def test_stack_priority_override_and_stable_rewrite(self):
        self.target("OVERRIDE", '''schema = 1
board = "dm_mc02_h7"
services = ["RC"]
[tasks.RC]
stack_words = 333
priority = "High"
[build]
sources = ["RobotConfig.c"]
''')
        out = self.root / "override-out"; GEN.generate(self.root, self.resolve("OVERRIDE"), out)
        header = (out / "RobotTargetConfig.h").read_text(encoding="utf-8"); runtime = (out / "RobotTargetTasks.inc").read_text(encoding="utf-8")
        self.assertIn("ROBOT_TASK_STACK_RC 333u", header); self.assertIn("RobotModulePriorityHigh", header); self.assertIn("osPriorityHigh, 333u", runtime)
        stamp = (out / "RobotTargetConfig.h").stat().st_mtime_ns; time.sleep(.02); GEN.generate(self.root, self.resolve("OVERRIDE"), out)
        self.assertEqual(stamp, (out / "RobotTargetConfig.h").stat().st_mtime_ns)

    def test_create_robot_copies_new_identity_and_safe_motor_mode(self):
        result = GEN.create_robot(self.root, "NEW", "BASE")
        self.assertEqual((result["name"], result["based_on"]), ("NEW", "BASE"))
        resolved = self.resolve("NEW")
        self.assertEqual(resolved["name"], "NEW")
        self.assertTrue(all(not source.startswith("Robotconfig/BASE/") for source in resolved["sources"]))
        operation = (self.root / "Robotconfig/NEW/ConfigOperation.inc").read_text(encoding="utf-8")
        self.assertIn("ROBOT_RUN_MODE_SINGLE_MOTOR", operation)
        self.assertIn("(uint8_t)MotorCount", operation)

    def test_create_robot_never_overwrites_existing_name(self):
        GEN.create_robot(self.root, "NEW", "BASE")
        marker = self.root / "Robotconfig/NEW/marker.txt"; marker.write_text("keep", encoding="utf-8")
        with self.assertRaisesRegex(GEN.ConfigError, "不覆盖"):
            GEN.create_robot(self.root, "NEW", "BASE")
        self.assertEqual(marker.read_text(encoding="utf-8"), "keep")

    def test_create_robot_rejects_template_without_safe_motor_selector(self):
        (self.root / "Robotconfig/BASE/ConfigOperation.inc").write_text(
            '.mode = ROBOT_RUN_MODE_FULL,\n', encoding="utf-8")
        with self.assertRaisesRegex(GEN.ConfigError, "不能保证新车默认停机"):
            GEN.create_robot(self.root, "NEW", "BASE")
        self.assertFalse((self.root / "Robotconfig/NEW").exists())

    def test_update_presets_preserves_user_data_and_adds_new_target(self):
        GEN.create_robot(self.root, "NEW", "BASE")
        preset_path = self.root / "projects/CMakeUserPresets.json"; preset_path.parent.mkdir()
        preset_path.write_text('{"version": 3, "configurePresets": [{"name": "base-local", "inherits": "user-base", "hidden": true, "cacheVariables": {"KEEP": "yes"}}], "buildPresets": [{"name": "base-local", "configurePreset": "base-local", "jobs": 7, "vendor": {"keep": true}}]}', encoding="utf-8")
        first = GEN.update_presets(self.root, "all")
        data = __import__("json").loads(preset_path.read_text(encoding="utf-8"))
        kept = next(p for p in data["configurePresets"] if p["name"] == "base-local")
        self.assertEqual(kept, {"name": "base-local", "inherits": "user-base", "hidden": True, "cacheVariables": {"KEEP": "yes"}})
        self.assertIn("new-local", first["added"])
        stamp = preset_path.stat().st_mtime_ns; time.sleep(.02); second = GEN.update_presets(self.root, "all")
        self.assertEqual(second["added"], [])
        self.assertEqual(stamp, preset_path.stat().st_mtime_ns)

    def test_stack_override_bounds_and_unknown_priority(self):
        self.target("BAD", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["RC"]\n[tasks.RC]\nstack_words = 12\n[build]\nsources = ["RobotConfig.c"]\n'); self.bad("必须在")
        self.target("PRIORITY", 'schema = 1\nboard = "dm_mc02_h7"\nservices = ["RC"]\n[tasks.RC]\npriority = "Urgent"\n[build]\nsources = ["RobotConfig.c"]\n')
        with self.assertRaisesRegex(GEN.ConfigError, "未知优先级"): self.resolve("PRIORITY")

if __name__ == "__main__":
    unittest.main()
