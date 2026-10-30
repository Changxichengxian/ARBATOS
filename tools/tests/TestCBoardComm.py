import importlib.util
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("c_board_ports", REPO / "tools/config/RobotConfigGen.py")
GEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)


class CBoardCommTest(unittest.TestCase):
    def resolve(self, ports, services):
        return GEN.resolve_port_config(REPO, {"board": "dji_c_f407"}, {"ports": ports}, set(services))

    def test_tuning_aux_can_use_both_full_duplex_ports(self):
        for port in ("usart1", "usart6"):
            with self.subTest(port=port):
                config = self.resolve({"tuning_aux": port}, ["HOST_LINK"])
                self.assertIn("tuning_aux", config["active_roles"])
                self.assertEqual(config["assignments"]["tuning_aux"]["baud"], 230400)

    def test_tuning_aux_and_referee_cannot_share_a_port(self):
        with self.assertRaisesRegex(GEN.ConfigError, "重复占用"):
            self.resolve({"tuning_aux": "usart6", "referee": "usart6"}, ["HOST_LINK", "REFEREE_RX"])

    def test_tuning_aux_does_not_take_dbus_port(self):
        with self.assertRaisesRegex(GEN.ConfigError, "缺少能力"):
            self.resolve({"tuning_aux": "usart3"}, ["HOST_LINK"])

    def test_dbus_aux_and_referee_can_coexist(self):
        config = self.resolve({"rc_sbus": "usart3", "tuning_aux": "usart1", "referee": "usart6"},
                              ["RC_SBUS", "HOST_LINK", "REFEREE_RX"])
        self.assertEqual(config["active_roles"], ["rc_sbus", "referee", "tuning_aux"])


if __name__ == "__main__":
    unittest.main()
