import unittest

from auto_drive.traffic_signal_gate_logic import TrafficSignalGateCore


class TrafficSignalGateCoreTest(unittest.TestCase):
    def set_signal(self, core, present, red, green, now_sec):
        core.set_signal_present(present, now_sec)
        core.set_signal_red(red, now_sec)
        core.set_signal_green(green, now_sec)

    def test_missing_signal_defaults_to_go(self):
        decision = TrafficSignalGateCore().evaluate(now_sec=1.0)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.state, 'STALE_OR_MISSING')
        self.assertFalse(decision.fresh)

    def test_red_requires_stop(self):
        core = TrafficSignalGateCore()
        self.set_signal(core, present=True, red=True, green=False, now_sec=1.0)

        decision = core.evaluate(now_sec=1.1)

        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.state, 'RED')

    def test_confirmed_green_releases_stop(self):
        core = TrafficSignalGateCore()
        self.set_signal(core, present=True, red=False, green=True, now_sec=1.0)

        decision = core.evaluate(now_sec=1.1)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.state, 'GREEN')

    def test_not_detected_does_not_stop(self):
        core = TrafficSignalGateCore()
        self.set_signal(
            core, present=False, red=False, green=False, now_sec=1.0
        )

        decision = core.evaluate(now_sec=1.1)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.state, 'NOT_DETECTED')

    def test_present_without_color_does_not_stop(self):
        core = TrafficSignalGateCore()
        self.set_signal(
            core, present=True, red=False, green=False, now_sec=1.0
        )

        decision = core.evaluate(now_sec=1.1)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.state, 'UNKNOWN')

    def test_conflicting_red_and_green_stops(self):
        core = TrafficSignalGateCore()
        self.set_signal(core, present=True, red=True, green=True, now_sec=1.0)

        decision = core.evaluate(now_sec=1.1)

        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.state, 'CONFLICT')

    def test_stale_signal_does_not_stop(self):
        core = TrafficSignalGateCore(signal_timeout_sec=0.5)
        self.set_signal(core, present=True, red=False, green=True, now_sec=1.0)

        decision = core.evaluate(now_sec=1.6)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.state, 'STALE_OR_MISSING')
        self.assertFalse(decision.fresh)

    def test_stale_red_releases_stop(self):
        core = TrafficSignalGateCore(signal_timeout_sec=0.5)
        self.set_signal(core, present=True, red=True, green=False, now_sec=1.0)

        fresh_red = core.evaluate(now_sec=1.1)
        stale_red = core.evaluate(now_sec=1.6)

        self.assertTrue(fresh_red.stop_required)
        self.assertFalse(stale_red.stop_required)
        self.assertEqual(stale_red.state, 'STALE_OR_MISSING')


if __name__ == '__main__':
    unittest.main()
