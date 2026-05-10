import math
import random
import unittest

from auto_drive.complex_rrt_core import RrtPlanner
from auto_drive.complex_rrt_core import RrtPlannerConfig


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

    def test_requires_forward_target(self):
        planner = RrtPlanner(RrtPlannerConfig(require_cones=False))

        result = planner.plan([], (0.2, 0.0), rng=random.Random(1))

        self.assertEqual(result.reason, 'target_not_forward')
        self.assertEqual(result.path_points, [])


if __name__ == '__main__':
    unittest.main()
