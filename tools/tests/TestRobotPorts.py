import importlib.util
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("robot_config_ports", REPO / "tools/config/RobotConfigGen.py")
GEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)


class RobotPortsTest(unittest.TestCase):
    @staticmethod
    def hero_text(services, ports):
        text = (REPO / "Robotconfig/HERO-M/RobotConfig.toml").read_text(encoding="utf-8-sig")
        start = text.index("services = ")
        end = text.index("\n", start)
        text = text[:start] + "services = [" + ", ".join(f'\"{item}\"' for item in services) + "]" + text[end:]
        return text + "\n[ports]\n" + ports + "\n"

    def test_catalog_exposes_real_ports_and_hi14_uart_imu(self):
        catalog = GEN.port_catalog(REPO, "dm_mc02_h7")
        ports = {port["id"]: port for port in catalog["ports"]}
        roles = {role["id"]: role for role in catalog["roles"]}
        self.assertEqual(ports["uart5"]["pins"], ["PD2"])
        self.assertNotIn("tx", ports["uart5"]["capabilities"])
        self.assertEqual(roles["elrs_crsf"]["default_baud"], 420000)
        self.assertTrue(roles["external_imu"]["supported"])
        self.assertEqual(roles["external_imu"]["protocols"], ["hipnuc_hi14"])

    def test_generate_binds_elrs_to_separate_uart_and_writes_overlay(self):
        services = ["RC_SBUS", "ELRS_LINK", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
elrs_crsf = { port = "usart10", baud = 420000, protocol = "crsf" }
referee = "uart7"
rs485_0 = "usart2"
rs485_1 = "usart3"
"""
        target = GEN.resolve(REPO, "HERO-M", self.hero_text(services, ports))
        self.assertIn("shared/application/input/ElrsTask.c", target["sources"])
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            GEN.generate(REPO, target, out)
            header = (out / "RobotTargetConfig.h").read_text(encoding="utf-8")
            overlay = (out / "RobotTargetPorts.overlay").read_text(encoding="utf-8")
            cmake = (out / "RobotTarget.cmake").read_text(encoding="utf-8")
        self.assertIn("#define ARB_UART_ELRS_NODE DT_NODELABEL(usart10)", header)
        self.assertIn("#define ARB_UART_ELRS_BAUD 420000u", header)
        self.assertIn("&usart10", overlay)
        self.assertIn("current-speed = <420000>;", overlay)
        self.assertIn("RobotTargetPorts.overlay", cmake)

    def test_invalid_duplicate_draft_keeps_options_but_strict_resolve_rejects(self):
        services = ["RC_SBUS", "ELRS_LINK", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
elrs_crsf = "uart5"
referee = "uart7"
"""
        text = self.hero_text(services, ports)
        view = GEN.port_options(REPO, "HERO-M", text)
        elrs = next(role for role in view["roles"] if role["id"] == "elrs_crsf")
        selected = next(option for option in elrs["options"] if option["port"] == "uart5")
        self.assertFalse(selected["available"])
        self.assertIn("已由 rc_sbus 占用", selected["reasons"])
        with self.assertRaisesRegex(GEN.ConfigError, "重复占用"):
            GEN.resolve(REPO, "HERO-M", text)

    def test_inactive_sbus_assignment_does_not_occupy_elrs_port(self):
        services = ["ELRS_LINK", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
elrs_crsf = "uart5"
referee = "uart7"
"""
        target = GEN.resolve(REPO, "HERO-M", self.hero_text(services, ports))
        self.assertNotIn("rc_sbus", target["ports"]["active_roles"])
        self.assertIn("elrs_crsf", target["ports"]["active_roles"])

    def test_external_imu_generates_hi14_binding_and_source(self):
        services = ["RC_SBUS", "IMU", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
external_imu = { port = "usart10", baud = 115200 }
referee = "uart7"
"""
        target = GEN.resolve(REPO, "HERO-M", self.hero_text(services, ports))
        self.assertIn("shared/zephyr/port/sensors/Hi14Parser.c", target["port_sources"])
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            GEN.generate(REPO, target, out)
            header = (out / "RobotTargetConfig.h").read_text(encoding="utf-8")
        self.assertIn("#define ROBOT_EXTERNAL_IMU_HI14 1", header)
        self.assertIn("#define ARB_UART_IMU_NODE DT_NODELABEL(usart10)", header)
        self.assertIn("#define ARB_UART_IMU_BAUD 115200u", header)

    def test_external_imu_selection_requires_imu_service(self):
        services = ["RC_SBUS", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
external_imu = "usart10"
referee = "uart7"
"""
        text = self.hero_text(services, ports)
        data = GEN.read_toml_text(text, REPO / "Robotconfig/HERO-M/RobotConfig.toml")
        with self.assertRaisesRegex(GEN.ConfigError, "必须启用 IMU 服务"):
            GEN.resolve_port_config(REPO, {"board": "dm_mc02_h7"}, data, set(services))

    def test_tuning_aux_uses_real_host_link_and_removes_stub(self):
        services = ["RC_SBUS", "HOST_LINK", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        ports = """rc_sbus = "uart5"
tuning_aux = { port = "usart10", baud = 230400, protocol = "arbatos_aux" }
referee = "uart7"
"""
        target = GEN.resolve(REPO, "HERO-M", self.hero_text(services, ports))
        self.assertIn("shared/application/comm/host/HostLinkTask.c", target["sources"])
        self.assertIn("shared/application/comm/host/AuxTune.c", target["sources"])
        self.assertIn("shared/application/input/ImageRemoteLink.c", target["sources"])
        self.assertNotIn("shared/application/input/ElrsTask.c", target["sources"])
        self.assertNotIn("shared/application/comm/host/HostLinkTaskStub.c", target["sources"])

    def test_default_targets_keep_configured_host_link_sources(self):
        expected = {
            "HERO-M": "shared/application/comm/host/HostLinkTaskStub.c",
            "SENTINEL-M": "Robotconfig/SENTINEL-M/UsbHostLinkTask.c",
            "MINIWHEELEG-M": "shared/application/comm/host/HostLinkTaskStub.c",
        }
        for name, source in expected.items():
            with self.subTest(name=name):
                target = GEN.resolve(REPO, name)
                self.assertIn(source, target["sources"])
                self.assertNotIn("shared/application/comm/host/HostLinkTask.c", target["sources"])

    def test_sentinel_tuning_aux_keeps_subboard_bringup(self):
        text = (REPO / "Robotconfig/SENTINEL-M/RobotConfig.toml").read_text(encoding="utf-8-sig")
        text += "\n[ports]\nrc_sbus = \"uart5\"\ntuning_aux = \"usart10\"\n"
        target = GEN.resolve(REPO, "SENTINEL-M", text)
        self.assertIn("shared/application/comm/host/HostLinkTask.c", target["sources"])
        self.assertIn("shared/zephyr/port/subboard/SubBoardBringupZephyr.c", target["port_sources"])
        self.assertNotIn("Robotconfig/SENTINEL-M/UsbHostLinkTask.c", target["sources"])
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            GEN.generate(REPO, target, out)
            header = (out / "RobotTargetConfig.h").read_text(encoding="utf-8")
        self.assertIn("#define ROBOT_SUBBOARD_RTC_SERVICE 1", header)

    def test_servo_service_has_m_board_source(self):
        services = ["RC_SBUS", "SERVO", "HEALTH_MONITOR", "SDLOG", "BATTERY_MONITOR", "REFEREE_RX"]
        target = GEN.resolve(REPO, "HERO-M", self.hero_text(services, "rc_sbus = \"uart5\"\nreferee = \"uart7\""))
        self.assertIn("shared/application/services/servo/ServoControlTask.c", target["sources"])


if __name__ == "__main__":
    unittest.main()
