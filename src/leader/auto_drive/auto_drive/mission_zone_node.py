import math

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from auto_drive.mission_zone_core import load_csv_path
from auto_drive.mission_zone_core import has_active_zone
from auto_drive.mission_zone_core import MissionZone
from auto_drive.mission_zone_core import MissionZoneTracker
from auto_drive.mission_zone_core import resolve_zones
from auto_drive.mission_zone_core import resolve_zone


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def make_point(x, y, z=0.0):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def optional_float(value):
    value = float(value)
    if value < 0.0 or math.isnan(value):
        return None
    return value


class MissionZoneNode(Node):
    def __init__(self):
        super().__init__('mission_zone_node')

        self.vehicle_utm_topic = self.declare_parameter(
            'vehicle_utm_topic', '/vehicle_ref_utm'
        ).value
        self.drive_context_topic = self.declare_parameter(
            'drive_context_topic', '/mission_context'
        ).value
        self.intersection_topic = self.declare_parameter(
            'intersection_topic', '/intersection'
        ).value
        self.traffic_zone_names = list(
            self.declare_parameter(
                'traffic_zone_names', ['traffic_zone']
            ).value
        )
        self.status_topic = self.declare_parameter(
            'status_topic', '/mission_zone_status'
        ).value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/mission_zones'
        ).value
        self.csv_file_path = self.declare_parameter(
            'csv_file_path', ''
        ).value
        self.csv_frame_id = self.declare_parameter(
            'csv_frame_id', 'csv'
        ).value
        self.marker_publish_rate_hz = float(
            self.declare_parameter('marker_publish_rate_hz', 2.0).value
        )
        self.context_latch_duration_sec = float(
            self.declare_parameter('context_latch_duration_sec', 2.0).value
        )

        self.csv_path = load_csv_path(self.csv_file_path)
        self.zones = self.load_zones_from_params()
        self.resolved_zones = resolve_zones(self.zones, self.csv_path)
        self.marker_zones = [
            resolve_zone(zone, self.csv_path) for zone in self.zones
        ]
        self.tracker = MissionZoneTracker(self.resolved_zones)

        self.latest_context = None
        self.latest_context_until_sec = 0.0
        self.latest_evaluation = None
        self.last_logged_context = None
        self.intersection_active = False
        self.last_logged_intersection_active = None

        self.create_subscription(
            PointStamped, self.vehicle_utm_topic, self.vehicle_callback, 10
        )
        self.drive_context_pub = self.create_publisher(
            String, self.drive_context_topic, 10
        )
        self.intersection_pub = self.create_publisher(
            Bool, self.intersection_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, TRANSIENT_QOS
        )

        self.timer = self.create_timer(
            1.0 / max(self.marker_publish_rate_hz, 1e-3), self.timer_callback
        )

        self.get_logger().info(
            'mission_zone_node ready: '
            f'enabled_zones={len(self.resolved_zones)} '
            f'marker_zones={len(self.marker_zones)} '
            f'csv_points={len(self.csv_path.points)}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def load_zones_from_params(self):
        zone_names = self.declare_parameter(
            'zone_names', ['city_zone', 'complex_start']
        ).value
        zones = []
        for name in zone_names:
            prefix = str(name)
            enabled = bool(
                self.declare_parameter(f'{prefix}.enabled', False).value
            )
            mode = self.declare_parameter(f'{prefix}.mode', 'utm').value
            context = self.declare_parameter(f'{prefix}.context', '').value
            radius = float(
                self.declare_parameter(f'{prefix}.radius', 0.0).value
            )
            x = float(self.declare_parameter(f'{prefix}.x', 0.0).value)
            y = float(self.declare_parameter(f'{prefix}.y', 0.0).value)
            csv_index = int(
                self.declare_parameter(f'{prefix}.csv_index', 0).value
            )
            start_index = int(
                self.declare_parameter(f'{prefix}.start_index', 0).value
            )
            end_index = int(
                self.declare_parameter(f'{prefix}.end_index', start_index).value
            )
            once = bool(self.declare_parameter(f'{prefix}.once', True).value)
            enter_radius = optional_float(
                self.declare_parameter(f'{prefix}.enter_radius', -1.0).value
            )
            exit_radius = optional_float(
                self.declare_parameter(f'{prefix}.exit_radius', -1.0).value
            )
            publish_on_exit_context = self.declare_parameter(
                f'{prefix}.publish_on_exit_context', ''
            ).value

            zones.append(
                MissionZone(
                    name=prefix,
                    context=context,
                    radius=radius,
                    mode=mode,
                    x=x,
                    y=y,
                    csv_index=csv_index,
                    start_index=start_index,
                    end_index=end_index,
                    once=once,
                    enabled=enabled,
                    enter_radius=enter_radius,
                    exit_radius=exit_radius,
                    publish_on_exit_context=publish_on_exit_context,
                )
            )
        return zones

    def vehicle_callback(self, msg: PointStamped):
        vehicle_point = (msg.point.x, msg.point.y)
        evaluation = self.tracker.evaluate(vehicle_point)
        self.latest_evaluation = evaluation
        self.intersection_active = has_active_zone(
            evaluation.active_zones, self.traffic_zone_names
        )
        self.publish_intersection()

        if evaluation.triggered and evaluation.context:
            self.publish_context(evaluation.context)
            self.latest_context = evaluation.context
            self.latest_context_until_sec = (
                self.now_sec() + self.context_latch_duration_sec
            )
            if evaluation.context != self.last_logged_context:
                self.get_logger().info(
                    'Mission zone trigger: '
                    f'zone={evaluation.zone_name} context={evaluation.context}'
                )
                self.last_logged_context = evaluation.context

        self.publish_status(evaluation)

    def timer_callback(self):
        if self.latest_context and self.now_sec() <= self.latest_context_until_sec:
            self.publish_context(self.latest_context)
        self.publish_intersection()
        self.publish_markers()

    def publish_context(self, context):
        msg = String()
        msg.data = str(context)
        self.drive_context_pub.publish(msg)

    def publish_intersection(self):
        msg = Bool()
        msg.data = self.intersection_active
        self.intersection_pub.publish(msg)

        if self.intersection_active != self.last_logged_intersection_active:
            state = 'active' if self.intersection_active else 'inactive'
            self.get_logger().info(f'Traffic zone -> {state}')
            self.last_logged_intersection_active = self.intersection_active

    def publish_status(self, evaluation):
        closest = 'none'
        if evaluation.closest_zone_name is not None:
            closest = (
                f'{evaluation.closest_zone_name}:'
                f'{evaluation.closest_distance:.2f}m'
            )
        msg = String()
        msg.data = (
            f'triggered={evaluation.zone_name or "none"} '
            f'context={evaluation.context or "none"} '
            f'active={",".join(evaluation.active_zones) or "none"} '
            f'closest={closest}'
        )
        self.status_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        stamp = Time().to_msg()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.csv_frame_id
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        origin = self.csv_path.origin
        if origin is None:
            self.marker_pub.publish(marker_array)
            return

        marker_id = 0
        for resolved in self.marker_zones:
            zone = resolved.source
            active = bool(
                zone.enabled and self.tracker.zone_active.get(zone.name, False)
            )
            color = self.zone_color(zone.context, active)

            if resolved.center is not None:
                marker_array.markers.append(
                    self.make_zone_cylinder(
                        marker_id, stamp, zone, resolved.center, origin, color
                    )
                )
                marker_id += 1
            elif resolved.range_points:
                marker_array.markers.append(
                    self.make_zone_line(
                        marker_id, stamp, zone, resolved.range_points, origin, color
                    )
                )
                marker_id += 1

            label_point = resolved.center
            if label_point is None and resolved.range_points:
                label_point = resolved.range_points[0]
            if label_point is not None:
                marker_array.markers.append(
                    self.make_zone_label(
                        marker_id,
                        stamp,
                        zone,
                        label_point,
                        origin,
                        active,
                    )
                )
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def make_zone_cylinder(self, marker_id, stamp, zone, center, origin, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.csv_frame_id
        marker.ns = f'{zone.name}_area'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position = make_point(
            center[0] - origin[0], center[1] - origin[1], -0.2
        )
        marker.pose.orientation.w = 1.0
        marker.scale.x = zone.radius * 2.0
        marker.scale.y = zone.radius * 2.0
        marker.scale.z = 0.12
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def make_zone_line(self, marker_id, stamp, zone, points, origin, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.csv_frame_id
        marker.ns = f'{zone.name}_range'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(zone.radius * 0.35, 0.08)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = min(color[3] + 0.25, 1.0)
        for x, y in points:
            marker.points.append(make_point(x - origin[0], y - origin[1], 0.15))
        return marker

    def make_zone_label(self, marker_id, stamp, zone, point, origin, active):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.csv_frame_id
        marker.ns = f'{zone.name}_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position = make_point(
            point[0] - origin[0], point[1] - origin[1], 0.9
        )
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.55
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'{zone.name}\\n{zone.context}'
        if not zone.enabled:
            marker.text += '\\nDISABLED'
        if active:
            marker.text += '\\nACTIVE'
        return marker

    @staticmethod
    def zone_color(context, active):
        normalized = str(context or '').strip().lower()
        alpha = 0.42 if active else 0.2
        if normalized == 'complex':
            return 1.0, 0.25, 0.9, alpha
        if normalized == 'city':
            return 1.0, 0.85, 0.05, alpha
        if normalized == 'highway':
            return 0.1, 0.8, 1.0, alpha
        return 0.9, 0.9, 0.9, alpha


def main(args=None):
    rclpy.init(args=args)
    node = MissionZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
