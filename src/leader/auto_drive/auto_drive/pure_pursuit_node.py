import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.roi_path_topic = self.declare_parameter(
            'roi_path_topic', '/roi_path'
        ).value
        self.heading_valid_topic = self.declare_parameter(
            'heading_valid_topic', '/vehicle_heading_valid'
        ).value
        self.steer_topic = self.declare_parameter(
            'steer_topic', '/auto_steer_angle'
        ).value
        self.throttle_topic = self.declare_parameter(
            'throttle_topic', '/throttle_from_planning'
        ).value
        self.base_frame = self.declare_parameter(
            'base_frame', 'vehicle_ref'
        ).value
        self.path_frame = self.declare_parameter('path_frame', 'csv').value
        self.wheelbase = float(
            self.declare_parameter('wheelbase', 0.724).value
        )
        self.min_lookahead = float(
            self.declare_parameter('min_lookahead', 1.6).value
        )
        self.max_lookahead = float(
            self.declare_parameter('max_lookahead', 2.8).value
        )
        self.beta = float(self.declare_parameter('beta', 4.0).value)
        self.curvature_window = int(
            self.declare_parameter('curvature_window', 7).value
        )
        self.curvature_ema_alpha = float(
            self.declare_parameter('curvature_ema_alpha', 0.7).value
        )
        self.ld_ema_alpha = float(
            self.declare_parameter('ld_ema_alpha', 0.5).value
        )
        self.steer_limit_deg = float(
            self.declare_parameter('steer_limit_deg', 20.0).value
        )
        self.min_throttle = float(
            self.declare_parameter('min_throttle', 0.45).value
        )
        self.max_throttle = float(
            self.declare_parameter('max_throttle', 0.7).value
        )
        self.throttle_ema_alpha = float(
            self.declare_parameter('throttle_ema_alpha', 0.5).value
        )
        self.control_rate_hz = float(
            self.declare_parameter('control_rate_hz', 30.0).value
        )
        self.steer_zero_when_heading_invalid = bool(
            self.declare_parameter(
                'steer_zero_when_heading_invalid', True
            ).value
        )
        self.throttle_zero_when_heading_invalid = bool(
            self.declare_parameter(
                'throttle_zero_when_heading_invalid', True
            ).value
        )
        self.goal_reached_distance = float(
            self.declare_parameter('goal_reached_distance', 0.8).value
        )
        self.visualize = bool(self.declare_parameter('visualize', True).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_points = []
        self.path_frame_id = self.path_frame
        self.heading_valid = False
        self.kappa_filtered = 0.0
        self.lookahead_filtered = None
        self.throttle_filtered = None
        self.last_tf_warn_ns = 0

        self.create_subscription(
            Path, self.roi_path_topic, self.path_callback, TRANSIENT_QOS
        )
        self.create_subscription(
            Bool, self.heading_valid_topic, self.heading_valid_callback, 10
        )

        self.steer_pub = self.create_publisher(Float32, self.steer_topic, 10)
        self.throttle_pub = self.create_publisher(
            Float32, self.throttle_topic, 10
        )

        if self.visualize:
            self.target_marker_pub = self.create_publisher(
                Marker, '/pp/lookahead_point', 10
            )

        self.timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1e-3), self.timer_callback
        )

    def path_callback(self, msg: Path):
        self.path_frame_id = msg.header.frame_id or self.path_frame
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]

    def heading_valid_callback(self, msg: Bool):
        self.heading_valid = msg.data

    def timer_callback(self):
        if len(self.path_points) < 2:
            self.publish_zero(reset_filters=True)
            return

        if not self.heading_valid and (
            self.steer_zero_when_heading_invalid
            or self.throttle_zero_when_heading_invalid
        ):
            self.publish_zero(reset_filters=False)
            return

        transform = self.lookup_vehicle_transform()
        if transform is None:
            self.publish_zero(reset_filters=False)
            return

        translation = transform.transform.translation
        vehicle_x = translation.x
        vehicle_y = translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        nearest_idx = self.find_nearest_index(vehicle_x, vehicle_y)
        if nearest_idx is None:
            self.publish_zero(reset_filters=False)
            return

        if self.is_goal_reached(nearest_idx, vehicle_x, vehicle_y):
            self.publish_zero(reset_filters=True)
            return

        curvature = self.local_curvature_abs_avg(nearest_idx)
        self.kappa_filtered = (
            self.curvature_ema_alpha * curvature
            + (1.0 - self.curvature_ema_alpha) * self.kappa_filtered
        )

        span = max(self.max_lookahead - self.min_lookahead, 0.0)
        raw_lookahead = self.min_lookahead + span * math.exp(
            -self.beta * self.kappa_filtered
        )
        raw_lookahead = clamp(
            raw_lookahead, self.min_lookahead, self.max_lookahead
        )

        if self.lookahead_filtered is None:
            self.lookahead_filtered = raw_lookahead
        else:
            self.lookahead_filtered = (
                self.ld_ema_alpha * raw_lookahead
                + (1.0 - self.ld_ema_alpha) * self.lookahead_filtered
            )
        lookahead = clamp(
            self.lookahead_filtered, self.min_lookahead, self.max_lookahead
        )

        target_x, target_y = self.interpolate_lookahead_from(
            nearest_idx, lookahead
        )
        steer_deg = self.compute_steer_deg(
            vehicle_x, vehicle_y, yaw, target_x, target_y, lookahead
        )
        throttle = self.compute_throttle(steer_deg)

        self.publish_command(steer_deg, throttle)
        if self.visualize:
            self.publish_target_marker(target_x, target_y)

    def lookup_vehicle_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.path_frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    'TF lookup failed '
                    f'({self.path_frame_id} <- {self.base_frame}): {exc}'
                )
                self.last_tf_warn_ns = now_ns
            return None

    def find_nearest_index(self, vehicle_x, vehicle_y):
        if not self.path_points:
            return None

        best_idx = 0
        best_distance_sq = float('inf')
        for index, (x, y) in enumerate(self.path_points):
            distance_sq = (x - vehicle_x) ** 2 + (y - vehicle_y) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_idx = index
        return best_idx

    def is_goal_reached(self, nearest_idx, vehicle_x, vehicle_y):
        if nearest_idx < len(self.path_points) - 1:
            return False

        goal_x, goal_y = self.path_points[-1]
        return (
            math.hypot(goal_x - vehicle_x, goal_y - vehicle_y)
            <= self.goal_reached_distance
        )

    def local_curvature_abs_avg(self, center_index):
        point_count = len(self.path_points)
        if point_count < 3:
            return 0.0

        half_window = max(self.curvature_window // 2, 1)
        start_idx = max(center_index - half_window, 1)
        end_idx = min(center_index + half_window, point_count - 2)

        samples = []
        for index in range(start_idx, end_idx + 1):
            p0 = self.path_points[index - 1]
            p1 = self.path_points[index]
            p2 = self.path_points[index + 1]

            a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
            denom = max(a * b * c, 1e-9)
            area2 = abs(
                (p1[0] - p0[0]) * (p2[1] - p0[1])
                - (p1[1] - p0[1]) * (p2[0] - p0[0])
            )
            samples.append((2.0 * area2) / denom)

        if not samples:
            return 0.0
        return sum(samples) / len(samples)

    def interpolate_lookahead_from(self, start_index, lookahead):
        accumulated = 0.0
        for index in range(start_index, len(self.path_points) - 1):
            x0, y0 = self.path_points[index]
            x1, y1 = self.path_points[index + 1]
            segment_length = math.hypot(x1 - x0, y1 - y0)
            if accumulated + segment_length >= lookahead:
                ratio = (lookahead - accumulated) / max(segment_length, 1e-6)
                return (
                    x0 + ratio * (x1 - x0),
                    y0 + ratio * (y1 - y0),
                )
            accumulated += segment_length
        return self.path_points[-1]

    def compute_steer_deg(
        self, vehicle_x, vehicle_y, yaw, target_x, target_y, lookahead
    ):
        dx = target_x - vehicle_x
        dy = target_y - vehicle_y

        vx_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        vy_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(vy_body, vx_body)

        steer_rad = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            max(lookahead, 1e-6),
        )
        steer_deg = math.degrees(steer_rad)
        return clamp(steer_deg, -self.steer_limit_deg, self.steer_limit_deg)

    def compute_throttle(self, steer_deg):
        steer_ratio = abs(steer_deg) / max(self.steer_limit_deg, 1e-6)
        raw_throttle = self.max_throttle - steer_ratio * (
            self.max_throttle - self.min_throttle
        )
        raw_throttle = clamp(
            raw_throttle, self.min_throttle, self.max_throttle
        )

        if self.throttle_filtered is None:
            self.throttle_filtered = raw_throttle
        else:
            self.throttle_filtered = (
                self.throttle_ema_alpha * raw_throttle
                + (1.0 - self.throttle_ema_alpha) * self.throttle_filtered
            )
        return clamp(
            self.throttle_filtered, self.min_throttle, self.max_throttle
        )

    def publish_zero(self, reset_filters):
        if reset_filters:
            self.kappa_filtered = 0.0
            self.lookahead_filtered = None
            self.throttle_filtered = None
        self.publish_command(0.0, 0.0)

    def publish_command(self, steer_deg, throttle):
        steer_msg = Float32()
        steer_msg.data = float(-steer_deg)
        self.steer_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)

    def publish_target_marker(self, x, y):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.path_frame_id
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.pose.position = Point(x=x, y=y, z=0.0)
        marker.pose.orientation.w = 1.0
        self.target_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
