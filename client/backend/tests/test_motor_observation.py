from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motor_observation import MotorObservation


class MotorObservationTest(unittest.TestCase):
    def test_keeps_internal_route_id_separate_from_observed_feedback(self):
        clock = [1000.0]
        observation = MotorObservation(lambda: clock[0])
        observation.feed(
            "MOTOR motor.chassis2 id=2 bus=1 cfg_fb=0x204 online=1 last_fb=0x204 "
            "count=12 age=3 rpm=-145 fb_cur=22 cmd=0 mode=0 cmd_cur=0", 999.9)
        snapshot = observation.snapshot()
        self.assertTrue(snapshot["available"])
        self.assertFalse(snapshot["encoderAvailable"])
        self.assertEqual(snapshot["motionEvidence"], "rpm")
        motor = snapshot["motors"][0]
        self.assertEqual(motor["configuredRouteId"], 2)
        self.assertEqual(motor["configuredBus"], 1)
        self.assertEqual(motor["configuredFeedbackId"], 0x204)
        self.assertEqual(motor["observedFeedbackId"], 0x204)
        self.assertEqual(motor["rpm"], -145)
        self.assertEqual(motor["ageMs"], 100)
        self.assertTrue(motor["fresh"])

    def test_freshness_requires_new_uart_row_online_and_recent_firmware_feedback(self):
        clock = [10.0]
        observation = MotorObservation(lambda: clock[0])
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=201 rpm=1 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 10.0)
        self.assertFalse(observation.snapshot()["motors"][0]["fresh"])
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=2 age=1 rpm=1 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 10.0)
        self.assertTrue(observation.snapshot()["motors"][0]["fresh"])
        clock[0] = 13.1
        stale = observation.snapshot()["motors"][0]
        self.assertEqual(stale["ageMs"], 3100)
        self.assertFalse(stale["fresh"])

    def test_lifecycle_is_exposed_as_separate_freshness_bounded_safety_evidence(self):
        clock = [1.0]
        observation = MotorObservation(lambda: clock[0])
        observation.feed("INFANTRY-TIAN-A uptime=44 run_mode=1 state=ACTIVE reason=1 output=0 task=1 variant=0",
                         1.0)
        safety = observation.snapshot()["safety"]
        self.assertEqual(safety, {"outputAllowed": False, "hardwareUptimeMs": 44,
                                  "receivedAt": 1.0, "ageMs": 0, "fresh": True})
        clock[0] = 4.1
        self.assertFalse(observation.snapshot()["safety"]["fresh"])

    def test_unknown_or_enabled_output_fails_closed_for_hand_turn_candidates(self):
        clock = [1.0]
        observation = MotorObservation(lambda: clock[0])
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=1 rpm=5 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 1.0)
        self.assertFalse(observation.snapshot()["handTurnCandidateReady"])
        observation.feed("INFANTRY-TIAN-A uptime=44 run_mode=1 state=ACTIVE reason=1 output=1 task=1 variant=0",
                         1.0)
        self.assertFalse(observation.snapshot()["handTurnCandidateReady"])
        observation.feed("INFANTRY-TIAN-A uptime=45 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        self.assertTrue(observation.snapshot()["handTurnCandidateReady"])
        clock[0] = 4.1
        self.assertFalse(observation.snapshot()["handTurnCandidateReady"])

    def test_hardware_uptime_rollback_clears_old_observation_and_advances_generation(self):
        observation = MotorObservation(lambda: 1.0)
        observation.feed("INFANTRY-TIAN-A uptime=100 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=1 rpm=5 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 1.0)
        previous = observation.snapshot()["generation"]
        observation.feed("INFANTRY-TIAN-A uptime=1 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        snapshot = observation.snapshot()
        self.assertGreater(snapshot["generation"], previous)
        self.assertEqual(snapshot["motors"], [])

    def test_unsafe_transition_invalidates_generation_even_if_polling_misses_it(self):
        observation = MotorObservation(lambda: 1.0)
        observation.feed("INFANTRY-TIAN-A uptime=100 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=1 rpm=5 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 1.0)
        generation = observation.snapshot()["generation"]
        observation.feed("INFANTRY-TIAN-A uptime=101 run_mode=1 state=ACTIVE reason=1 output=1 task=1 variant=0",
                         1.0)
        observation.feed("INFANTRY-TIAN-A uptime=102 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        snapshot = observation.snapshot()
        self.assertGreater(snapshot["generation"], generation)
        self.assertEqual(snapshot["motors"], [])
        self.assertFalse(snapshot["handTurnCandidateReady"])

    def test_unavailable_motor_or_invalid_header_immediately_closes_old_ready_state(self):
        observation = MotorObservation(lambda: 1.0)
        observation.feed("INFANTRY-TIAN-A uptime=100 run_mode=1 state=SAFE reason=1 output=0 task=1 variant=0",
                         1.0)
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=1 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=1 rpm=5 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 1.0)
        self.assertTrue(observation.snapshot()["handTurnCandidateReady"])
        generation = observation.snapshot()["generation"]
        observation.feed("MOTOR snapshot=unavailable", 1.0)
        unavailable = observation.snapshot()
        self.assertGreater(unavailable["generation"], generation)
        self.assertFalse(unavailable["handTurnCandidateReady"])
        self.assertIsNone(unavailable["safety"])
        observation.feed("INFANTRY-TIAN-A uptime=101 state=SAFE", 1.0)
        invalid = observation.snapshot()
        self.assertGreater(invalid["generation"], unavailable["generation"])
        self.assertFalse(invalid["handTurnCandidateReady"])
        self.assertIsNone(invalid["safety"])

    def test_rejects_incomplete_or_invalid_motor_lines_without_fabricating_data(self):
        observation = MotorObservation(lambda: 1.0)
        observation.feed("MOTOR motor.chassis0 id=0 bus=1 online=1", 1.0)
        observation.feed(
            "MOTOR motor.chassis0 id=0 bus=0 cfg_fb=0x201 online=1 last_fb=0x201 "
            "count=1 age=0 rpm=0 fb_cur=0 cmd=0 mode=0 cmd_cur=0", 1.0)
        self.assertEqual(observation.snapshot()["motors"], [])


if __name__ == "__main__":
    unittest.main()
