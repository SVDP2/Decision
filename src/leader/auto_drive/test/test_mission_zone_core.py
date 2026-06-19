import tempfile
import unittest
from pathlib import Path

from auto_drive.mission_zone_core import load_csv_path
from auto_drive.mission_zone_core import has_active_zone
from auto_drive.mission_zone_core import MissionZone
from auto_drive.mission_zone_core import MissionZoneTracker
from auto_drive.mission_zone_core import resolve_zones


class MissionZoneCoreTest(unittest.TestCase):
    def write_csv(self, rows):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.csv', delete=False
        )
        with handle:
            for x, y in rows:
                handle.write(f'{x},{y}\n')
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_csv_index_zone_triggers_context_once(self):
        csv_path = load_csv_path(
            self.write_csv([(100.0, 200.0), (102.0, 200.0)])
        )
        zones = [
            MissionZone(
                name='complex_start',
                mode='csv_index',
                csv_index=1,
                radius=1.0,
                context='complex',
                once=True,
            )
        ]
        tracker = MissionZoneTracker(resolve_zones(zones, csv_path))

        before = tracker.evaluate((100.0, 200.0))
        trigger = tracker.evaluate((102.5, 200.0))
        repeated = tracker.evaluate((102.2, 200.0))

        self.assertFalse(before.triggered)
        self.assertTrue(trigger.triggered)
        self.assertEqual(trigger.context, 'complex')
        self.assertFalse(repeated.triggered)

    def test_csv_range_zone_uses_polyline_distance(self):
        csv_path = load_csv_path(
            self.write_csv([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)])
        )
        zones = [
            MissionZone(
                name='complex_section',
                mode='csv_range',
                start_index=0,
                end_index=2,
                radius=1.5,
                context='complex',
                once=True,
            )
        ]
        tracker = MissionZoneTracker(resolve_zones(zones, csv_path))

        outside = tracker.evaluate((5.0, 2.0))
        inside = tracker.evaluate((5.0, 1.0))

        self.assertFalse(outside.triggered)
        self.assertTrue(inside.triggered)
        self.assertEqual(inside.closest_zone_name, 'complex_section')

    def test_utm_zone_can_publish_exit_context(self):
        zones = [
            MissionZone(
                name='temporary_complex',
                mode='utm',
                x=10.0,
                y=10.0,
                radius=1.0,
                exit_radius=1.5,
                context='complex',
                once=False,
                publish_on_exit_context='highway',
            )
        ]
        tracker = MissionZoneTracker(resolve_zones(zones, load_csv_path('')))

        enter = tracker.evaluate((10.0, 10.0))
        still_active = tracker.evaluate((11.2, 10.0))
        exit_zone = tracker.evaluate((12.0, 10.0))

        self.assertEqual(enter.context, 'complex')
        self.assertFalse(still_active.triggered)
        self.assertEqual(exit_zone.context, 'highway')

    def test_active_traffic_zone_can_drive_intersection_state(self):
        zones = [
            MissionZone(
                name='traffic_zone',
                mode='utm',
                x=10.0,
                y=10.0,
                radius=1.0,
                exit_radius=1.5,
                context='',
                once=False,
            )
        ]
        tracker = MissionZoneTracker(resolve_zones(zones, load_csv_path('')))

        outside = tracker.evaluate((8.0, 10.0))
        inside = tracker.evaluate((10.0, 10.0))
        hysteresis = tracker.evaluate((11.2, 10.0))
        exited = tracker.evaluate((12.0, 10.0))

        self.assertFalse(
            has_active_zone(outside.active_zones, ['traffic_zone'])
        )
        self.assertTrue(has_active_zone(inside.active_zones, ['traffic_zone']))
        self.assertTrue(
            has_active_zone(hysteresis.active_zones, ['traffic_zone'])
        )
        self.assertFalse(
            has_active_zone(exited.active_zones, ['traffic_zone'])
        )

    def test_city_then_complex_transition_sequence(self):
        zones = [
            MissionZone(
                name='city_zone',
                mode='utm',
                x=10.0,
                y=0.0,
                radius=1.0,
                context='city',
                once=True,
            ),
            MissionZone(
                name='complex_start',
                mode='utm',
                x=20.0,
                y=0.0,
                radius=1.0,
                context='complex',
                once=True,
            ),
        ]
        tracker = MissionZoneTracker(resolve_zones(zones, load_csv_path('')))

        highway = tracker.evaluate((0.0, 0.0))
        city = tracker.evaluate((10.0, 0.0))
        between = tracker.evaluate((15.0, 0.0))
        complex_zone = tracker.evaluate((20.0, 0.0))

        self.assertFalse(highway.triggered)
        self.assertEqual(city.context, 'city')
        self.assertFalse(between.triggered)
        self.assertEqual(complex_zone.context, 'complex')


if __name__ == '__main__':
    unittest.main()
