import unittest

from auto_drive.command_mux_logic import CommandMuxCore
from auto_drive.command_mux_logic import PlannerCommand
from auto_drive.mission_supervisor_logic import MissionState
from auto_drive.mission_supervisor_logic import MissionSupervisorCore


class MissionSpeedFlowTest(unittest.TestCase):
    def make_command(self, steer, throttle, stamp_sec=1.0):
        return PlannerCommand(
            steer=steer,
            throttle=throttle,
            steer_stamp_sec=stamp_sec,
            throttle_stamp_sec=stamp_sec,
        )

    def apply_mission(self, mux, supervisor, mission, commands, now_sec):
        highway, city, complex_cmd = commands
        mux_result = mux.select(
            mission,
            highway,
            city,
            complex_cmd,
            now_sec=now_sec,
        )
        supervisor.set_drive_context(mission)
        supervisor.set_planning_throttle(mux_result.throttle, now_sec)
        return mux_result, supervisor.update(now_sec)

    def test_highway_city_and_complex_use_distinct_sources_and_speeds(self):
        mux = CommandMuxCore(command_timeout_sec=1.0)
        supervisor = MissionSupervisorCore(command_timeout_sec=1.0)
        commands = (
            self.make_command(steer=1.0, throttle=0.38),
            self.make_command(steer=1.0, throttle=0.28),
            self.make_command(steer=-2.0, throttle=0.25),
        )

        highway_mux, highway = self.apply_mission(
            mux, supervisor, 'highway', commands, now_sec=1.0
        )
        city_mux, city = self.apply_mission(
            mux, supervisor, 'city', commands, now_sec=1.0
        )
        complex_mux, complex_snapshot = self.apply_mission(
            mux, supervisor, 'complex', commands, now_sec=1.0
        )

        self.assertEqual(highway_mux.source, 'highway')
        self.assertEqual(highway.mission_state, MissionState.HIGHWAY)
        self.assertAlmostEqual(highway.output_throttle, 0.38)

        self.assertEqual(city_mux.source, 'city')
        self.assertEqual(city.mission_state, MissionState.CITY)
        self.assertAlmostEqual(city.output_throttle, 0.28)

        self.assertEqual(complex_mux.source, 'complex')
        self.assertEqual(complex_snapshot.mission_state, MissionState.COMPLEX)
        self.assertAlmostEqual(complex_snapshot.output_throttle, 0.15)


if __name__ == '__main__':
    unittest.main()
