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
    corridor_mode: bool = False
    corridor_cone_distance_limit: float = 4.0
    corridor_both_sides_reward: float = 3.0
    corridor_one_side_penalty: float = 3.0
    corridor_min_branch_score: float = 60.0
    corridor_edge_max_length: float = 7.0
    corridor_edge_max_parts_ratio: float = 3.0
    corridor_min_waypoints: int = 1
    corridor_append_branch_endpoint: bool = False


@dataclass(frozen=True)
class RrtPlanResult:
    path_points: list[tuple[float, float]]
    tree_edges: list[tuple[tuple[float, float], tuple[float, float]]] = field(
        default_factory=list
    )
    obstacles: list[Obstacle] = field(default_factory=list)
    best_branch_points: list[tuple[float, float]] = field(default_factory=list)
    cone_edges: list[
        tuple[tuple[float, float], tuple[float, float]]
    ] = field(default_factory=list)
    corridor_waypoints: list[tuple[float, float]] = field(default_factory=list)
    branch_score: float = 0.0
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

        corridor_cones = self._filter_cones(cones)
        if cfg.require_cones and not corridor_cones:
            return RrtPlanResult([], reason='missing_cones')

        obstacles = self._build_obstacles(corridor_cones, predicted_obstacles)
        target_dist = math.hypot(target_x, target_y)
        bounded_target = (target_x, target_y)
        if target_dist > cfg.plan_distance:
            scale = cfg.plan_distance / max(target_dist, 1e-6)
            bounded_target = (target_x * scale, target_y * scale)

        if (
            cfg.allow_direct_path
            and not cfg.corridor_mode
            and self._segment_is_clear(
                (0.0, 0.0), bounded_target, obstacles
            )
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
        leaf_indices = []

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

            if candidate.cost >= cfg.plan_distance - cfg.expand_distance:
                leaf_indices.append(candidate_index)

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
                if reached_index is None:
                    reached_index = candidate_index
                if candidate_index not in leaf_indices:
                    leaf_indices.append(candidate_index)
                if not cfg.corridor_mode:
                    break

        if cfg.corridor_mode:
            corridor_candidates = list(leaf_indices)
            if reached_index is not None:
                corridor_candidates.append(reached_index)
            if best_index != 0:
                corridor_candidates.append(best_index)

            corridor_result = self._build_corridor_result(
                nodes,
                corridor_candidates,
                tree_edges,
                obstacles,
                corridor_cones,
            )
            if corridor_result is not None:
                return corridor_result

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

    def _filter_cones(self, cones):
        cfg = self.config
        filtered = []
        for x, y in cones:
            x = float(x)
            y = float(y)
            if x < cfg.min_obstacle_x:
                continue
            if abs(y) > cfg.max_obstacle_abs_y:
                continue
            if math.hypot(x, y) > cfg.max_obstacle_distance:
                continue
            filtered.append((x, y))
        return filtered

    def _build_obstacles(self, cones, predicted_obstacles):
        cfg = self.config
        obstacles = []

        for x, y in cones:
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

    def _build_corridor_result(
        self,
        nodes,
        candidate_indices,
        tree_edges,
        obstacles,
        cones,
    ):
        cfg = self.config
        if len(cones) < 2:
            return None

        branch = self._select_corridor_branch(nodes, candidate_indices, cones)
        if branch is None:
            return None

        branch_index, branch_score = branch
        best_branch_points = self._extract_path(nodes, branch_index)
        cone_edges = self._build_cone_edges(cones)
        corridor_waypoints = self._waypoints_from_branch_edges(
            best_branch_points, cone_edges
        )

        if len(corridor_waypoints) >= max(int(cfg.corridor_min_waypoints), 1):
            points = [(0.0, 0.0)] + corridor_waypoints
            branch_end = best_branch_points[-1]
            if (
                cfg.corridor_append_branch_endpoint
                and math.hypot(
                    points[-1][0] - branch_end[0],
                    points[-1][1] - branch_end[1],
                )
                > cfg.path_resolution
            ):
                points.append(branch_end)
            reason = 'corridor_midpoints'
        else:
            points = best_branch_points
            reason = 'corridor_branch'

        return RrtPlanResult(
            self._resample_path(points),
            tree_edges=tree_edges,
            obstacles=obstacles,
            best_branch_points=best_branch_points,
            cone_edges=cone_edges,
            corridor_waypoints=corridor_waypoints,
            branch_score=branch_score,
            reason=reason,
        )

    def _select_corridor_branch(self, nodes, candidate_indices, cones):
        cfg = self.config
        unique_indices = []
        seen = set()
        for index in candidate_indices:
            if index is None or index <= 0 or index >= len(nodes):
                continue
            if index in seen:
                continue
            unique_indices.append(index)
            seen.add(index)

        best_index = None
        best_score = -float('inf')
        for index in unique_indices:
            score = self._score_corridor_branch(nodes, index, cones)
            if score > best_score:
                best_score = score
                best_index = index

        if best_index is None:
            return None
        if best_score < cfg.corridor_min_branch_score:
            return None
        return best_index, best_score

    def _score_corridor_branch(self, nodes, index, cones):
        cfg = self.config
        limit = max(cfg.corridor_cone_distance_limit, 1e-6)
        reward = max(cfg.corridor_both_sides_reward, 0.0)
        one_side_penalty = max(cfg.corridor_one_side_penalty, 1e-6)
        cost_span = max(cfg.plan_distance - cfg.expand_distance, 1e-6)

        branch_score = 0.0
        while index >= 0:
            node = nodes[index]
            if node.parent < 0:
                break

            parent = nodes[node.parent]
            start = (parent.x, parent.y)
            end = (node.x, node.y)
            left_score = 0.0
            right_score = 0.0

            for cone in cones:
                dist = self._point_to_segment_distance(cone, start, end)
                if dist > limit:
                    continue
                contribution = limit - dist
                cross = (
                    (end[0] - start[0]) * (cone[1] - start[1])
                    - (end[1] - start[1]) * (cone[0] - start[0])
                )
                if cross >= 0.0:
                    left_score += contribution
                else:
                    right_score += contribution

            node_score = left_score + right_score
            if node_score > 0.0:
                if left_score > 0.0 and right_score > 0.0:
                    node_score *= reward
                else:
                    node_score /= one_side_penalty
                node_factor = (
                    (node.cost - cfg.expand_distance) / cost_span
                ) + 1.0
                branch_score += node_score * max(node_factor, 0.0)

            index = node.parent

        return branch_score

    def _build_cone_edges(self, cones):
        max_length = max(self.config.corridor_edge_max_length, 0.0)
        edges = []
        for first_index, first in enumerate(cones):
            for second in cones[first_index + 1:]:
                if (
                    math.hypot(first[0] - second[0], first[1] - second[1])
                    <= max_length
                ):
                    edges.append((first, second))
        return edges

    def _waypoints_from_branch_edges(self, branch_points, cone_edges):
        if len(branch_points) < 2 or not cone_edges:
            return []

        cfg = self.config
        waypoint_items_by_edge = {}
        accumulated = 0.0
        for start, end in zip(branch_points, branch_points[1:]):
            segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
            if segment_length <= 1e-6:
                continue

            for edge_start, edge_end in cone_edges:
                if not self._segments_intersect(
                    start, end, edge_start, edge_end
                ):
                    continue

                intersection = self._line_intersection(
                    start, end, edge_start, edge_end
                )
                if intersection is None:
                    continue

                first_part = math.hypot(
                    intersection[0] - edge_start[0],
                    intersection[1] - edge_start[1],
                )
                second_part = math.hypot(
                    intersection[0] - edge_end[0],
                    intersection[1] - edge_end[1],
                )
                shorter = min(first_part, second_part)
                if shorter <= 1e-6:
                    continue
                if (
                    max(first_part, second_part) / shorter
                    > cfg.corridor_edge_max_parts_ratio
                ):
                    continue

                midpoint = (
                    (edge_start[0] + edge_end[0]) * 0.5,
                    (edge_start[1] + edge_end[1]) * 0.5,
                )
                along = accumulated + math.hypot(
                    intersection[0] - start[0],
                    intersection[1] - start[1],
                )
                edge_key = tuple(sorted((edge_start, edge_end)))
                current = waypoint_items_by_edge.get(edge_key)
                if current is None or along < current[0]:
                    waypoint_items_by_edge[edge_key] = (along, midpoint)

            accumulated += segment_length

        waypoint_items = list(waypoint_items_by_edge.values())
        waypoint_items.sort(key=lambda item: item[0])
        waypoints = []
        seen_waypoints = set()
        for _, waypoint in waypoint_items:
            waypoint_key = (
                round(waypoint[0] / max(cfg.path_resolution, 0.05)),
                round(waypoint[1] / max(cfg.path_resolution, 0.05)),
            )
            if waypoint_key in seen_waypoints:
                continue
            if waypoints and math.hypot(
                waypoint[0] - waypoints[-1][0],
                waypoint[1] - waypoints[-1][1],
            ) < max(cfg.path_resolution, 0.05):
                continue
            seen_waypoints.add(waypoint_key)
            waypoints.append(waypoint)
        return waypoints

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

    @staticmethod
    def _point_to_segment_distance(point, start, end):
        px, py = point
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return math.hypot(px - sx, py - sy)

        ratio = clamp(((px - sx) * dx + (py - sy) * dy) / length_sq, 0.0, 1.0)
        closest_x = sx + ratio * dx
        closest_y = sy + ratio * dy
        return math.hypot(px - closest_x, py - closest_y)

    @staticmethod
    def _orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (
            b[1] - a[1]
        ) * (c[0] - a[0])

    @staticmethod
    def _point_on_segment(point, start, end):
        tolerance = 1e-9
        if abs(RrtPlanner._orientation(start, end, point)) > tolerance:
            return False
        return (
            min(start[0], end[0]) - tolerance
            <= point[0]
            <= max(start[0], end[0]) + tolerance
            and min(start[1], end[1]) - tolerance
            <= point[1]
            <= max(start[1], end[1]) + tolerance
        )

    @staticmethod
    def _segments_intersect(a, b, c, d):
        tolerance = 1e-9
        o1 = RrtPlanner._orientation(a, b, c)
        o2 = RrtPlanner._orientation(a, b, d)
        o3 = RrtPlanner._orientation(c, d, a)
        o4 = RrtPlanner._orientation(c, d, b)

        if o1 * o2 < -tolerance and o3 * o4 < -tolerance:
            return True
        if abs(o1) <= tolerance and RrtPlanner._point_on_segment(c, a, b):
            return True
        if abs(o2) <= tolerance and RrtPlanner._point_on_segment(d, a, b):
            return True
        if abs(o3) <= tolerance and RrtPlanner._point_on_segment(a, c, d):
            return True
        if abs(o4) <= tolerance and RrtPlanner._point_on_segment(b, c, d):
            return True
        return False

    @staticmethod
    def _line_intersection(a, b, c, d):
        x1, y1 = a
        x2, y2 = b
        x3, y3 = c
        x4, y4 = d
        denominator = (x1 - x2) * (y3 - y4) - (
            y1 - y2
        ) * (x3 - x4)
        if abs(denominator) <= 1e-9:
            return None

        det1 = x1 * y2 - y1 * x2
        det2 = x3 * y4 - y3 * x4
        return (
            (det1 * (x3 - x4) - (x1 - x2) * det2) / denominator,
            (det1 * (y3 - y4) - (y1 - y2) * det2) / denominator,
        )

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
