import math

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
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


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_point(x, y, z=0.0):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def transform_xy(x, y, transform):
    translation = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        translation.x + cos_yaw * x - sin_yaw * y,
        translation.y + sin_yaw * x + cos_yaw * y,
    )


class ComplexTargetNode(Node):
    def __init__(self):
        super().__init__('complex_target_node')

        self.csv_path_topic = self.declare_parameter(
            'csv_path_topic', '/csv_path'
        ).value
        self.target_topic = self.declare_parameter(
            'target_topic', '/complex/rrt_target'
        ).value
        self.target_marker_topic = self.declare_parameter(
            'target_marker_topic', '/complex/rrt_target_marker'
        ).value
        self.roi_path_topic = self.declare_parameter(
            'roi_path_topic', '/complex/target_roi_path'
        ).value
        self.target_frame = self.declare_parameter(
            'target_frame', 'vehicle_ref'
        ).value
        self.csv_frame = self.declare_parameter('csv_frame', 'csv').value
        self.timer_frequency = float(
            self.declare_parameter('timer_frequency', 20.0).value
        )
        self.target_distance_m = float(
            self.declare_parameter('target_distance_m', 6.0).value
        )
        self.search_span_points = int(
            self.declare_parameter('search_span_points', 2000).value
        )
        self.min_forward_x = float(
            self.declare_parameter('min_forward_x', 0.2).value
        )
        self.max_target_abs_y = float(
            self.declare_parameter('max_target_abs_y', 6.0).value
        )
        self.line_width = float(self.declare_parameter('line_width', 0.18).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_points = []
        self.path_frame = self.csv_frame
        self.last_nearest_idx = 0
        self.last_tf_warn_ns = 0
        self.last_target_warn_ns = 0

        self.create_subscription(
            Path, self.csv_path_topic, self.path_callback, TRANSIENT_QOS
        )
        self.target_pub = self.create_publisher(
            PointStamped, self.target_topic, TRANSIENT_QOS
        )
        self.target_marker_pub = self.create_publisher(
            Marker, self.target_marker_topic, TRANSIENT_QOS
        )
        self.roi_path_pub = self.create_publisher(
            Path, self.roi_path_topic, TRANSIENT_QOS
        )
        self.timer = self.create_timer(
            1.0 / max(self.timer_frequency, 1e-3), self.timer_callback
        )

    def path_callback(self, msg: Path):
        self.path_frame = msg.header.frame_id or self.csv_frame
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        if self.path_points:
            self.last_nearest_idx = min(
                self.last_nearest_idx, len(self.path_points) - 1
            )

    def timer_callback(self):
        if len(self.path_points) < 2:
            return

        transform = self.lookup_transform()
        if transform is None:
            return

        local_points = [
            transform_xy(x, y, transform) for x, y in self.path_points
        ]
        nearest_idx = self.find_nearest_index(local_points)
        if nearest_idx is None:
            return

        roi_points = self.collect_forward_roi(local_points, nearest_idx)
        if not roi_points:
            self.warn_target_throttled('No forward GPS target in vehicle frame.')
            return

        target = roi_points[-1]
        if abs(target[1]) > self.max_target_abs_y:
            self.warn_target_throttled(
                'Rejecting RRT target with large lateral offset '
                f'y={target[1]:.2f} m.'
            )
            return

        self.publish_target(target, roi_points)
        self.last_nearest_idx = nearest_idx

    def lookup_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.path_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    'TF lookup failed '
                    f'({self.target_frame} <- {self.path_frame}): {exc}'
                )
                self.last_tf_warn_ns = now_ns
            return None

    def find_nearest_index(self, local_points):
        point_count = len(local_points)
        if point_count == 0:
            return None

        start = min(self.last_nearest_idx, point_count - 1)
        end = min(start + max(self.search_span_points, 1), point_count - 1)
        if start >= end:
            start = 0
            end = point_count - 1

        best_idx = start
        best_dist_sq = float('inf')
        for index in range(start, end + 1):
            x, y = local_points[index]
            dist_sq = x * x + y * y
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = index
        return best_idx

    def collect_forward_roi(self, local_points, nearest_idx):
        start_idx = nearest_idx
        while (
            start_idx < len(local_points)
            and local_points[start_idx][0] < self.min_forward_x
        ):
            start_idx += 1

        if start_idx >= len(local_points):
            return []

        roi_points = [local_points[start_idx]]
        accumulated = 0.0
        previous = local_points[start_idx]
        for point in local_points[start_idx + 1:]:
            if point[0] < self.min_forward_x:
                break

            segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
            accumulated += segment
            roi_points.append(point)
            previous = point
            if accumulated >= max(self.target_distance_m, 0.1):
                break

        return roi_points

    def publish_target(self, target, roi_points):
        stamp = self.get_clock().now().to_msg()

        target_msg = PointStamped()
        target_msg.header.stamp = stamp
        target_msg.header.frame_id = self.target_frame
        target_msg.point = make_point(target[0], target[1], 0.0)
        self.target_pub.publish(target_msg)

        marker = Marker()
        marker.header = target_msg.header
        marker.ns = 'complex_rrt_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        marker.color.a = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position = target_msg.point
        marker.pose.orientation.w = 1.0
        self.target_marker_pub.publish(marker)

        roi_path = Path()
        roi_path.header = target_msg.header
        for x, y in roi_points:
            pose = PoseStamped()
            pose.header = roi_path.header
            pose.pose.position = make_point(x, y, 0.0)
            pose.pose.orientation.w = 1.0
            roi_path.poses.append(pose)
        self.roi_path_pub.publish(roi_path)

    def warn_target_throttled(self, text):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_target_warn_ns > 2_000_000_000:
            self.get_logger().warn(text)
            self.last_target_warn_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = ComplexTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
