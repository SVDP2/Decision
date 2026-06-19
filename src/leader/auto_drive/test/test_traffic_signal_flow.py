import unittest

from auto_drive.mission_supervisor_logic import MissionSupervisorCore
from auto_drive.mission_supervisor_logic import SafetyStatus
from auto_drive.mission_zone_core import has_active_zone
from auto_drive.mission_zone_core import load_csv_path
from auto_drive.mission_zone_core import MissionZone
from auto_drive.mission_zone_core import MissionZoneTracker
from auto_drive.mission_zone_core import resolve_zones
from auto_drive.traffic_signal_gate_logic import TrafficSignalGateCore


class TrafficSignalFlowTest(unittest.TestCase):
    def set_signal(self, gate, present, red, green, now_sec):
        gate.set_signal_present(present, now_sec)
        gate.set_signal_red(red, now_sec)
        gate.set_signal_green(green, now_sec)

    def test_red_stops_and_green_restarts_inside_traffic_zone(self):
        traffic_zone = MissionZone(
            name='traffic_zone',
            mode='utm',
            x=10.0,
            y=10.0,
            radius=2.0,
            context='',
            once=False,
        )
        tracker = MissionZoneTracker(
            resolve_zones([traffic_zone], load_csv_path(''))
        )
        zone_evaluation = tracker.evaluate((10.0, 10.0))

        supervisor = MissionSupervisorCore(
            release_hysteresis_sec=0.5,
            command_timeout_sec=2.0,
        )
        supervisor.set_drive_context('city')
        supervisor.set_intersection(
            has_active_zone(zone_evaluation.active_zones, ['traffic_zone'])
        )
        supervisor.set_planning_throttle(0.28, now_sec=1.0)

        gate = TrafficSignalGateCore(signal_timeout_sec=2.0)
        self.set_signal(
            gate, present=True, red=True, green=False, now_sec=1.0
        )
        red_decision = gate.evaluate(now_sec=1.0)
        supervisor.set_traffic_stop(red_decision.stop_required)
        stopped = supervisor.update(now_sec=1.0)

        self.assertEqual(stopped.safety_status, SafetyStatus.STOP_TRAFFIC)
        self.assertEqual(stopped.output_throttle, 0.0)

        self.set_signal(
            gate, present=True, red=False, green=True, now_sec=1.1
        )
        green_decision = gate.evaluate(now_sec=1.1)
        supervisor.set_traffic_stop(green_decision.stop_required)
        held = supervisor.update(now_sec=1.1)
        restarted = supervisor.update(now_sec=1.61)

        self.assertTrue(held.safety_active)
        self.assertFalse(restarted.safety_active)
        self.assertEqual(restarted.safety_status, SafetyStatus.SAFE_OK)
        self.assertAlmostEqual(restarted.output_throttle, 0.28)


if __name__ == '__main__':
    unittest.main()
