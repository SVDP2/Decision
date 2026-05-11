from __future__ import annotations

import math
import random
from dataclasses import dataclass
from dataclasses import field


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_detection_label(text):
    lines = str(text or '').strip().splitlines()
    if not lines:
        return ''
    label = lines[0].strip().lower()
    return label.split('(', 1)[0].strip()


def parse_label_names(text):
    labels = set()
    for item in str(text or '').split(','):
        normalized = item.strip().lower()
        if normalized:
            labels.add(normalized)
    return labels or {'cone'}


def detection_object_ids_for_labels(label_items, label_id_offset, label_names):
    accepted_labels = set(label_names or {'cone'})
    return {
        int(marker_id) - int(label_id_offset)
        for marker_id, marker_text in label_items
        if normalize_detection_label(marker_text) in accepted_labels
    }


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class RrtPlannerConfig:
    plan_distance: float = 5.0
    expand_distance: float = 0.7
    turn_angle_deg: float = 20.0
    max_iterations: int = 700
    goal_tolerance: float = 0.8
    target_sample_probability: float = 0.75
    target_sample_radius: float = 2.5
    random_sample_x_min: float = 0.0
    random_sample_x_max: float = 7.0
    random_sample_y_abs: float = 4.0
    cone_radius: float = 0.8
    predicted_obstacle_radius: float = 0.6
    min_obstacle_x: float = -0.5
    max_obstacle_distance: float = 12.0
    max_obstacle_abs_y: float = 6.0
    min_target_x: float = 0.8
    path_resolution: float = 0.25
    allow_direct_path: bool = True
    require_cones: bool = True


@dataclass(frozen=True)
class RrtPlanResult:
    path_points: list[tuple[float, float]]
    tree_edges: list[tuple[tuple[float, float], tuple[float, float]]] = field(
        default_factory=list
    )
    obstacles: list[Obstacle] = field(default_factory=list)
    reason: str = ''


@dataclass
class _Node:
    x: float
    y: float
    yaw: float
    cost: float
    parent: int


class RrtPlanner:
    def __init__(self, config=None):
        self.config = config or RrtPlannerConfig()

    def plan(self, cones, target, predicted_obstacles=None, rng=None):
        cfg = self.config
        rng = rng or random
        predicted_obstacles = predicted_obstacles or []

        if target is None:
            return RrtPlanResult([], reason='missing_target')

        target_x, target_y = float(target[0]), float(target[1])
        if target_x < cfg.min_target_x:
            return RrtPlanResult([], reason='target_not_forward')

        if cfg.require_cones and not cones:
            return RrtPlanResult([], reason='missing_cones')

        obstacles = self._build_obstacles(cones, predicted_obstacles)
        target_dist = math.hypot(target_x, target_y)
        bounded_target = (target_x, target_y)
        if target_dist > cfg.plan_distance:
            scale = cfg.plan_distance / max(target_dist, 1e-6)
            bounded_target = (target_x * scale, target_y * scale)

        if cfg.allow_direct_path and self._segment_is_clear(
            (0.0, 0.0), bounded_target, obstacles
        ):
            return RrtPlanResult(
                self._resample_path([(0.0, 0.0), bounded_target]),
                obstacles=obstacles,
                reason='direct_clear',
            )

        nodes = [_Node(0.0, 0.0, 0.0, 0.0, -1)]
        tree_edges = []
        best_index = 0
        best_score = self._score_node(nodes[0], bounded_target)
        reached_index = None

        for _ in range(max(cfg.max_iterations, 1)):
            sample = self._sample_point(bounded_target, rng)
            nearest_index = self._nearest_node_index(nodes, sample)
            nearest = nodes[nearest_index]
            if nearest.cost >= cfg.plan_distance:
                continue

            candidate = self._steer(nearest, nearest_index, sample)
            if candidate.cost > cfg.plan_distance + 1e-6:
                continue

            if not self._segment_is_clear(
                (nearest.x, nearest.y), (candidate.x, candidate.y), obstacles
            ):
                continue

            nodes.append(candidate)
            candidate_index = len(nodes) - 1
            tree_edges.append(
                ((nearest.x, nearest.y), (candidate.x, candidate.y))
            )

            score = self._score_node(candidate, bounded_target)
            if score > best_score:
                best_score = score
                best_index = candidate_index

            target_gap = math.hypot(
                candidate.x - bounded_target[0],
                candidate.y - bounded_target[1],
            )
            can_connect_target = (
                candidate.cost + target_gap
                <= cfg.plan_distance + cfg.goal_tolerance
                and self._segment_is_clear(
                    (candidate.x, candidate.y), bounded_target, obstacles
                )
            )
            if target_gap <= cfg.goal_tolerance or can_connect_target:
                reached_index = candidate_index
                break

        final_index = reached_index if reached_index is not None else best_index
        if final_index == 0:
            return RrtPlanResult(
                [], tree_edges=tree_edges, obstacles=obstacles,
                reason='no_valid_branch'
            )

        points = self._extract_path(nodes, final_index)
        if reached_index is not None and self._segment_is_clear(
            points[-1], bounded_target, obstacles
        ):
            points.append(bounded_target)

        return RrtPlanResult(
            self._resample_path(points),
            tree_edges=tree_edges,
            obstacles=obstacles,
            reason='reached_target' if reached_index is not None else 'best_branch',
        )

    def _build_obstacles(self, cones, predicted_obstacles):
        cfg = self.config
        obstacles = []

        for x, y in cones:
            x = float(x)
            y = float(y)
            if x < cfg.min_obstacle_x:
                continue
            if abs(y) > cfg.max_obstacle_abs_y:
                continue
            if math.hypot(x, y) > cfg.max_obstacle_distance:
                continue
            obstacles.append(Obstacle(x, y, cfg.cone_radius))

        for item in predicted_obstacles:
            if len(item) == 2:
                x, y = item
                radius = cfg.predicted_obstacle_radius
            else:
                x, y, radius = item
            x = float(x)
            y = float(y)
            radius = float(radius)
            if x < cfg.min_obstacle_x:
                continue
            if abs(y) > cfg.max_obstacle_abs_y:
                continue
            if math.hypot(x, y) > cfg.max_obstacle_distance:
                continue
            obstacles.append(Obstacle(x, y, radius))

        return obstacles

    def _sample_point(self, target, rng):
        cfg = self.config
        if rng.random() < clamp(cfg.target_sample_probability, 0.0, 1.0):
            radius = cfg.target_sample_radius * math.sqrt(rng.random())
            theta = rng.uniform(-math.pi, math.pi)
            return (
                target[0] + radius * math.cos(theta),
                target[1] + radius * math.sin(theta),
            )

        return (
            rng.uniform(cfg.random_sample_x_min, cfg.random_sample_x_max),
            rng.uniform(-cfg.random_sample_y_abs, cfg.random_sample_y_abs),
        )

    def _nearest_node_index(self, nodes, sample):
        best_index = 0
        best_dist_sq = float('inf')
        sx, sy = sample
        for index, node in enumerate(nodes):
            dist_sq = (node.x - sx) ** 2 + (node.y - sy) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = index
        return best_index

    def _steer(self, nearest, nearest_index, sample):
        cfg = self.config
        desired_yaw = math.atan2(sample[1] - nearest.y, sample[0] - nearest.x)
        delta_yaw = wrap_to_pi(desired_yaw - nearest.yaw)
        max_delta = math.radians(cfg.turn_angle_deg)
        next_yaw = wrap_to_pi(nearest.yaw + clamp(delta_yaw, -max_delta, max_delta))

        step = cfg.expand_distance
        return _Node(
            nearest.x + step * math.cos(next_yaw),
            nearest.y + step * math.sin(next_yaw),
            next_yaw,
            nearest.cost + step,
            nearest_index,
        )

    def _score_node(self, node, target):
        target_dist = math.hypot(target[0], target[1])
        if target_dist < 1e-6:
            return -math.hypot(node.x, node.y)

        ux = target[0] / target_dist
        uy = target[1] / target_dist
        progress = node.x * ux + node.y * uy
        lateral_error = abs(node.x * uy - node.y * ux)
        dist_to_target = math.hypot(node.x - target[0], node.y - target[1])
        forward_bonus = max(node.x, 0.0) * 0.15
        return progress - 0.8 * dist_to_target - 0.25 * lateral_error + forward_bonus

    def _extract_path(self, nodes, index):
        points = []
        while index >= 0:
            node = nodes[index]
            points.append((node.x, node.y))
            index = node.parent
        points.reverse()
        return points

    def _segment_is_clear(self, start, end, obstacles):
        if not obstacles:
            return True

        cfg = self.config
        sx, sy = start
        ex, ey = end
        length = math.hypot(ex - sx, ey - sy)
        steps = max(int(length / max(cfg.path_resolution, 0.05)), 1)

        for step in range(steps + 1):
            ratio = step / steps
            x = sx + ratio * (ex - sx)
            y = sy + ratio * (ey - sy)
            for obstacle in obstacles:
                if math.hypot(x - obstacle.x, y - obstacle.y) <= obstacle.radius:
                    return False
        return True

    def _resample_path(self, points):
        if len(points) < 2:
            return points

        resolution = max(self.config.path_resolution, 0.05)
        output = [points[0]]
        for start, end in zip(points, points[1:]):
            sx, sy = start
            ex, ey = end
            length = math.hypot(ex - sx, ey - sy)
            steps = max(int(length / resolution), 1)
            for step in range(1, steps + 1):
                ratio = step / steps
                output.append((sx + ratio * (ex - sx), sy + ratio * (ey - sy)))
        return output
