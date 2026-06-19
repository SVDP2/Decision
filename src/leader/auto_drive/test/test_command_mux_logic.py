import unittest

from auto_drive.command_mux_logic import CommandMuxCore
from auto_drive.command_mux_logic import PlannerCommand
from auto_drive.command_mux_logic import parse_mission_state


class CommandMuxCoreTest(unittest.TestCase):
    def make_command(self, steer, throttle, stamp_sec):
        return PlannerCommand(
            steer=steer,
            throttle=throttle,
            steer_stamp_sec=stamp_sec,
            throttle_stamp_sec=stamp_sec,
        )

    def test_complex_mission_selects_complex_command(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = self.make_command(steer=1.0, throttle=0.3, stamp_sec=1.0)
        city = self.make_command(steer=2.0, throttle=0.28, stamp_sec=1.0)
        complex_cmd = self.make_command(
            steer=-4.0, throttle=0.25, stamp_sec=1.0
        )

        result = core.select(
            'COMPLEX', highway, city, complex_cmd, now_sec=1.1
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.source, 'complex')
        self.assertAlmostEqual(result.steer, -4.0)
        self.assertAlmostEqual(result.throttle, 0.25)

    def test_city_selects_city_gps_tracking_command(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = self.make_command(steer=2.0, throttle=0.38, stamp_sec=2.0)
        city = self.make_command(steer=1.5, throttle=0.28, stamp_sec=2.0)
        complex_cmd = self.make_command(
            steer=-8.0, throttle=0.2, stamp_sec=2.0
        )

        result = core.select('CITY', highway, city, complex_cmd, now_sec=2.1)

        self.assertTrue(result.valid)
        self.assertEqual(result.source, 'city')
        self.assertAlmostEqual(result.steer, 1.5)
        self.assertAlmostEqual(result.throttle, 0.28)

    def test_stale_selected_source_outputs_zero(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = self.make_command(steer=2.0, throttle=0.35, stamp_sec=1.0)

        result = core.select('HIGHWAY', highway, None, None, now_sec=1.6)

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, 'highway_stale')
        self.assertAlmostEqual(result.steer, 0.0)
        self.assertAlmostEqual(result.throttle, 0.0)

    def test_missing_city_command_does_not_fall_back_to_highway(self):
        core = CommandMuxCore(command_timeout_sec=0.5)
        highway = self.make_command(steer=2.0, throttle=0.38, stamp_sec=1.0)

        result = core.select('CITY', highway, None, None, now_sec=1.1)

        self.assertFalse(result.valid)
        self.assertEqual(result.source, 'city')
        self.assertEqual(result.reason, 'city_missing')
        self.assertAlmostEqual(result.throttle, 0.0)

    def test_context_tokens_parse_to_complex(self):
        self.assertEqual(parse_mission_state('cone rrt zone'), 'complex')


if __name__ == '__main__':
    unittest.main()
