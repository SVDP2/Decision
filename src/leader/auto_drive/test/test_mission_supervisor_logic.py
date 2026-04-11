import unittest

from auto_drive.mission_supervisor_logic import MissionState
from auto_drive.mission_supervisor_logic import MissionSupervisorCore
from auto_drive.mission_supervisor_logic import SafetyStatus


class MissionSupervisorCoreTest(unittest.TestCase):
    def test_highway_planning_throttle_passes_through(self):
        core = MissionSupervisorCore()

        core.set_planning_throttle(0.62, now_sec=1.0)
        snapshot = core.update(now_sec=1.0)

        self.assertEqual(snapshot.mission_state, MissionState.HIGHWAY)
        self.assertAlmostEqual(snapshot.output_throttle, 0.62)
        self.assertFalse(snapshot.safety_active)

    def test_city_context_applies_lower_speed_policy(self):
        core = MissionSupervisorCore()

        core.set_drive_context('city intersection zone')
        core.set_planning_throttle(0.8, now_sec=1.0)
        snapshot = core.update(now_sec=1.0)

        self.assertEqual(snapshot.mission_state, MissionState.CITY)
        self.assertAlmostEqual(snapshot.output_throttle, 0.5)

    def test_traffic_stop_preempts_and_releases_after_hysteresis(self):
        core = MissionSupervisorCore(
            release_hysteresis_sec=0.5, command_timeout_sec=1.0
        )

        core.set_planning_throttle(0.6, now_sec=1.0)
        core.set_traffic_stop(True)
        core.set_intersection(True)
        blocked = core.update(now_sec=1.0)

        self.assertTrue(blocked.safety_active)
        self.assertEqual(blocked.safety_status, SafetyStatus.STOP_TRAFFIC)
        self.assertAlmostEqual(blocked.output_throttle, 0.0)

        core.set_traffic_stop(False)
        core.set_intersection(False)
        held = core.update(now_sec=1.2)
        released = core.update(now_sec=1.7)

        self.assertTrue(held.safety_active)
        self.assertFalse(released.safety_active)
        self.assertEqual(released.safety_status, SafetyStatus.SAFE_OK)
        self.assertAlmostEqual(released.output_throttle, 0.6)

    def test_generic_safety_stop_keeps_backward_compatibility(self):
        core = MissionSupervisorCore()

        core.set_planning_throttle(0.55, now_sec=2.0)
        core.set_generic_safety_stop(True)
        snapshot = core.update(now_sec=2.0)

        self.assertTrue(snapshot.safety_active)
        self.assertEqual(snapshot.safety_status, SafetyStatus.STOP_GENERIC)
        self.assertAlmostEqual(snapshot.output_throttle, 0.0)

    def test_planning_timeout_forces_zero_output(self):
        core = MissionSupervisorCore(command_timeout_sec=0.5)

        core.set_planning_throttle(0.65, now_sec=1.0)
        snapshot = core.update(now_sec=1.6)

        self.assertTrue(snapshot.planning_timed_out)
        self.assertAlmostEqual(snapshot.output_throttle, 0.0)


if __name__ == '__main__':
    unittest.main()
