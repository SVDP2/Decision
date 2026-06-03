from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path


def clamp_index(index, point_count):
    if point_count <= 0:
        return 0
    return max(0, min(int(index), point_count - 1))


@dataclass(frozen=True)
class CsvPath:
    points: list[tuple[float, float]]

    @property
    def origin(self):
        if not self.points:
            return None
        return self.points[0]


@dataclass(frozen=True)
class MissionZone:
    name: str
    context: str
    radius: float
    mode: str = 'utm'
    x: float | None = None
    y: float | None = None
    csv_index: int | None = None
    start_index: int | None = None
    end_index: int | None = None
    once: bool = True
    enabled: bool = True
    enter_radius: float | None = None
    exit_radius: float | None = None
    publish_on_exit_context: str = ''

    def normalized_mode(self):
        return str(self.mode or 'utm').strip().lower()

    def active_radius(self):
        if self.enter_radius is not None:
            return float(self.enter_radius)
        return float(self.radius)

    def inactive_radius(self):
        if self.exit_radius is not None:
            return float(self.exit_radius)
        return float(self.radius) + 1.0


@dataclass(frozen=True)
class ResolvedMissionZone:
    source: MissionZone
    center: tuple[float, float] | None = None
    range_points: list[tuple[float, float]] = field(default_factory=list)


@dataclass(frozen=True)
class ZoneEvaluation:
    zone_name: str | None
    context: str | None
    triggered: bool
    active_zones: list[str]
    distances: dict[str, float]
    closest_zone_name: str | None
    closest_distance: float | None


def load_csv_path(csv_file_path):
    points = []
    if not csv_file_path:
        return CsvPath(points)

    path = Path(str(csv_file_path)).expanduser()
    if not path.exists():
        return CsvPath(points)

    with path.open(newline='') as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    return CsvPath(points)


def resolve_zones(zones, csv_path):
    return [resolve_zone(zone, csv_path) for zone in zones if zone.enabled]


def resolve_zone(zone, csv_path):
    mode = zone.normalized_mode()
    if mode == 'csv_index':
        if not csv_path.points:
            return ResolvedMissionZone(zone)
        index = clamp_index(zone.csv_index or 0, len(csv_path.points))
        return ResolvedMissionZone(zone, center=csv_path.points[index])

    if mode == 'csv_range':
        if not csv_path.points:
            return ResolvedMissionZone(zone)
        start = clamp_index(zone.start_index or 0, len(csv_path.points))
        end_index = zone.end_index if zone.end_index is not None else start
        end = clamp_index(end_index, len(csv_path.points))
        if end < start:
            start, end = end, start
        return ResolvedMissionZone(zone, range_points=csv_path.points[start:end + 1])

    if zone.x is None or zone.y is None:
        return ResolvedMissionZone(zone)
    return ResolvedMissionZone(zone, center=(float(zone.x), float(zone.y)))


class MissionZoneTracker:
    def __init__(self, resolved_zones):
        self.resolved_zones = list(resolved_zones)
        self.zone_active = {zone.source.name: False for zone in self.resolved_zones}
        self.zone_consumed = {zone.source.name: False for zone in self.resolved_zones}

    def evaluate(self, vehicle_point):
        distances = {}
        active_zones = []
        closest_zone_name = None
        closest_distance = None

        triggered_zone = None
        triggered_context = None

        for resolved in self.resolved_zones:
            zone = resolved.source
            distance = distance_to_resolved_zone(vehicle_point, resolved)
            if distance is None:
                continue

            distances[zone.name] = distance
            if closest_distance is None or distance < closest_distance:
                closest_zone_name = zone.name
                closest_distance = distance

            was_active = self.zone_active.get(zone.name, False)
            next_active = distance <= (
                zone.inactive_radius() if was_active else zone.active_radius()
            )
            self.zone_active[zone.name] = next_active

            if next_active:
                active_zones.append(zone.name)

            rising_edge = next_active and not was_active
            if (
                triggered_zone is None
                and rising_edge
                and not (zone.once and self.zone_consumed.get(zone.name, False))
            ):
                triggered_zone = zone.name
                triggered_context = zone.context
                self.zone_consumed[zone.name] = True

            if (
                triggered_zone is None
                and was_active
                and not next_active
                and zone.publish_on_exit_context
            ):
                triggered_zone = zone.name
                triggered_context = zone.publish_on_exit_context

        return ZoneEvaluation(
            zone_name=triggered_zone,
            context=triggered_context,
            triggered=triggered_zone is not None,
            active_zones=active_zones,
            distances=distances,
            closest_zone_name=closest_zone_name,
            closest_distance=closest_distance,
        )


def distance_to_resolved_zone(vehicle_point, resolved):
    if resolved.center is not None:
        return point_distance(vehicle_point, resolved.center)
    if resolved.range_points:
        return distance_to_polyline(vehicle_point, resolved.range_points)
    return None


def point_distance(first, second):
    return math.hypot(
        float(first[0]) - float(second[0]),
        float(first[1]) - float(second[1]),
    )


def distance_to_polyline(point, points):
    if len(points) == 1:
        return point_distance(point, points[0])

    best = float('inf')
    for start, end in zip(points, points[1:]):
        best = min(best, point_to_segment_distance(point, start, end))
    return best


def point_to_segment_distance(point, start, end):
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return point_distance(point, start)

    ratio = ((px - sx) * dx + (py - sy) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    closest = (sx + ratio * dx, sy + ratio * dy)
    return point_distance(point, closest)
