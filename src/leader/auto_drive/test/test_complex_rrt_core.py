import math
import random
import unittest

from auto_drive.complex_rrt_core import RrtPlanner
from auto_drive.complex_rrt_core import RrtPlannerConfig
from auto_drive.complex_rrt_core import detection_object_ids_for_labels
from auto_drive.complex_rrt_core import normalize_detection_label
from auto_drive.complex_rrt_core import parse_label_names


class ComplexRrtCoreTest(unittest.TestCase):
    def test_direct_path_when_target_is_clear(self):
        planner = RrtPlanner(
            RrtPlannerConfig(require_cones=False, allow_direct_path=True)
        )

        result = planner.plan([], (4.0, 0.0), rng=random.Random(1))

        self.assertEqual(result.reason, 'direct_clear')
        self.assertGreater(len(result.path_points), 2)
        self.assertAlmostEqual(result.path_points[0][0], 0.0)
        self.assertAlmostEqual(result.path_points[-1][0], 4.0)

    def test_rrt_routes_around_cone_obstacle(self):
        config = RrtPlannerConfig(
            require_cones=False,
            allow_direct_path=False,
            max_iterations=2000,
            cone_radius=0.8,
        )
        planner = RrtPlanner(config)

        result = planner.plan(
            [(3.0, 0.0)], (5.0, 0.0), rng=random.Random(4)
        )

        self.assertEqual(result.reason, 'reached_target')
        self.assertGreater(len(result.path_points), 4)
        for x, y in result.path_points:
            self.assertGreater(math.hypot(x - 3.0, y), config.cone_radius)

    def test_corridor_mode_builds_midpoints_between_cones(self):
        config = RrtPlannerConfig(
            require_cones=True,
            allow_direct_path=False,
            corridor_mode=True,
            plan_distance=4.0,
            expand_distance=0.45,
            max_iterations=1000,
            cone_radius=0.3,
            target_sample_probability=0.75,
            target_sample_radius=2.0,
            random_sample_x_max=5.0,
            random_sample_y_abs=3.0,
            path_resolution=0.25,
        )
        planner = RrtPlanner(config)
        cones = [
            (1.5, -1.0),
            (2.5, -1.0),
            (3.5, -1.0),
            (1.5, 1.0),
            (2.5, 1.0),
            (3.5, 1.0),
        ]

        result = planner.plan(cones, (5.0, 0.0), rng=random.Random(1))

        self.assertEqual(result.reason, 'corridor_midpoints')
        self.assertGreaterEqual(len(result.corridor_waypoints), 3)
        self.assertGreater(
            result.branch_score, config.corridor_min_branch_score
        )
        self.assertIn((2.5, 0.0), result.corridor_waypoints)
        for _, y in result.corridor_waypoints:
            self.assertAlmostEqual(y, 0.0)
        for _, y in result.path_points:
            self.assertAlmostEqual(y, 0.0)

    def test_requires_forward_target(self):
        planner = RrtPlanner(RrtPlannerConfig(require_cones=False))

        result = planner.plan([], (0.2, 0.0), rng=random.Random(1))

        self.assertEqual(result.reason, 'target_not_forward')
        self.assertEqual(result.path_points, [])

    def test_detected_object_label_matching_extracts_cones(self):
        label_names = parse_label_names('cone,traffic_cone')

        object_ids = detection_object_ids_for_labels(
            [
                (10000, 'Cone\n(0.74)'),
                (10001, 'Car\n(0.92)'),
                (10002, 'traffic_cone\n(0.65)'),
            ],
            10000,
            label_names,
        )

        self.assertEqual(object_ids, {0, 2})

    def test_normalize_detection_label_uses_first_line(self):
        self.assertEqual(normalize_detection_label('Cone\n(0.74)'), 'cone')
        self.assertEqual(normalize_detection_label('Cone (0.74)'), 'cone')


if __name__ == '__main__':
    unittest.main()
