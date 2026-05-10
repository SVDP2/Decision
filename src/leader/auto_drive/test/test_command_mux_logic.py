import unittest

from auto_drive.command_mux_logic import CommandMuxCore
from auto_drive.command_mux_logic import PlannerCommand
from auto_drive.command_mux_logic import parse_mission_state


class CommandMuxCoreTest(unittest.TestCase):
    def test_complex_mission_selects_complex_command(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = PlannerCommand(steer=1.0, throttle=0.3,
                                 steer_stamp_sec=1.0, throttle_stamp_sec=1.0)
        complex_cmd = PlannerCommand(steer=-4.0, throttle=0.25,
                                     steer_stamp_sec=1.0,
                                     throttle_stamp_sec=1.0)

        result = core.select('COMPLEX', highway, complex_cmd, now_sec=1.1)

        self.assertTrue(result.valid)
        self.assertEqual(result.source, 'complex')
        self.assertAlmostEqual(result.steer, -4.0)
        self.assertAlmostEqual(result.throttle, 0.25)

    def test_city_uses_highway_gps_tracking_command(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = PlannerCommand(steer=2.0, throttle=0.35,
                                 steer_stamp_sec=2.0, throttle_stamp_sec=2.0)
        complex_cmd = PlannerCommand(steer=-8.0, throttle=0.2,
                                     steer_stamp_sec=2.0,
                                     throttle_stamp_sec=2.0)

        result = core.select('CITY', highway, complex_cmd, now_sec=2.1)

        self.assertTrue(result.valid)
        self.assertEqual(result.source, 'highway')
        self.assertAlmostEqual(result.steer, 2.0)

    def test_stale_selected_source_outputs_zero(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = PlannerCommand(steer=2.0, throttle=0.35,
                                 steer_stamp_sec=1.0, throttle_stamp_sec=1.0)

        result = core.select('HIGHWAY', highway, None, now_sec=1.6)

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, 'highway_stale')
        self.assertAlmostEqual(result.steer, 0.0)
        self.assertAlmostEqual(result.throttle, 0.0)

    def test_context_tokens_parse_to_complex(self):
        self.assertEqual(parse_mission_state('cone rrt zone'), 'complex')


if __name__ == '__main__':
    unittest.main()
