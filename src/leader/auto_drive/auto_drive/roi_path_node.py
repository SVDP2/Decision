import math

import rclpy
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


class RoiPathNode(Node):
    def __init__(self):
        super().__init__('roi_path_node')

        self.csv_path_topic = self.declare_parameter(
            'csv_path_topic', '/csv_path'
        ).value
        self.roi_path_topic = self.declare_parameter(
            'roi_path_topic', '/roi_path'
        ).value
        self.roi_marker_topic = self.declare_parameter(
            'roi_marker_topic', '/roi_path_marker'
        ).value
        self.target_frame = self.declare_parameter(
            'target_frame', 'vehicle_ref'
        ).value
        self.csv_frame = self.declare_parameter('csv_frame', 'csv').value
        self.timer_frequency = float(
            self.declare_parameter('timer_frequency', 20.0).value
        )
        self.roi_length_m = float(
            self.declare_parameter('roi_length_m', 6.0).value
        )
        self.use_points_length = bool(
            self.declare_parameter('use_points_length', False).value
        )
        self.roi_length_points = int(
            self.declare_parameter('roi_length_points', 60).value
        )
        self.search_span_points = int(
            self.declare_parameter('search_span_points', 2000).value
        )
        self.hysteresis_k = int(
            self.declare_parameter('hysteresis_k', 1).value
        )
        self.line_width = float(
            self.declare_parameter('line_width', 0.3).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_points = []
        self.path_frame = self.csv_frame
        self.last_start_idx = 0
        self.last_end_idx = 0
        self.published_once = False
        self.last_tf_warn_ns = 0

        self.create_subscription(
            Path, self.csv_path_topic, self.path_callback, TRANSIENT_QOS
        )
        self.roi_path_pub = self.create_publisher(
            Path, self.roi_path_topic, TRANSIENT_QOS
        )
        self.roi_marker_pub = self.create_publisher(
            Marker, self.roi_marker_topic, TRANSIENT_QOS
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
            limit = len(self.path_points) - 1
            self.last_start_idx = min(self.last_start_idx, limit)
            self.last_end_idx = max(
                self.last_start_idx,
                min(self.last_end_idx, limit),
            )

    def timer_callback(self):
        if len(self.path_points) < 2:
            return

        vehicle_xy = self.lookup_vehicle_position()
        if vehicle_xy is None:
            return

        start_idx, end_idx = self.compute_roi_indices(vehicle_xy)
        if start_idx is None or end_idx is None:
            return

        self.publish_roi(start_idx, end_idx)
        self.last_start_idx = start_idx
        self.last_end_idx = end_idx
        self.published_once = True

    def lookup_vehicle_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    'TF lookup failed '
                    f'({self.path_frame} <- {self.target_frame}): {exc}'
                )
                self.last_tf_warn_ns = now_ns
            return None

        translation = transform.transform.translation
        return (translation.x, translation.y)

    def compute_roi_indices(self, vehicle_xy):
        point_count = len(self.path_points)
        if point_count == 0:
            return None, None

        if self.published_once:
            cand_begin = min(self.last_start_idx, point_count - 1)
            cand_end = min(max(self.last_end_idx, cand_begin), point_count - 1)
        else:
            cand_begin = min(self.last_start_idx, point_count - 1)
            cand_end = min(
                cand_begin + max(self.search_span_points, 1), point_count - 1
            )

        vx, vy = vehicle_xy
        nearest_idx = cand_begin
        best_distance_sq = float('inf')

        for index in range(cand_begin, cand_end + 1):
            px, py = self.path_points[index]
            distance_sq = (px - vx) ** 2 + (py - vy) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                nearest_idx = index

        proposed_start = max(nearest_idx, self.last_start_idx)
        if (
            self.hysteresis_k > 0
            and proposed_start < self.last_start_idx + self.hysteresis_k
        ):
            proposed_start = self.last_start_idx

        start_idx = min(proposed_start, point_count - 1)

        if self.use_points_length:
            end_idx = min(
                start_idx + max(self.roi_length_points, 1), point_count - 1
            )
            return start_idx, end_idx

        accumulated = 0.0
        end_idx = start_idx
        while (
            end_idx + 1 < point_count
            and accumulated < max(self.roi_length_m, 0.1)
        ):
            x0, y0 = self.path_points[end_idx]
            x1, y1 = self.path_points[end_idx + 1]
            accumulated += math.hypot(x1 - x0, y1 - y0)
            end_idx += 1

        return start_idx, end_idx

    def publish_roi(self, start_idx, end_idx):
        stamp = self.get_clock().now().to_msg()

        roi_path = Path()
        roi_path.header.stamp = stamp
        roi_path.header.frame_id = self.path_frame

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.path_frame
        marker.ns = 'roi_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0

        for x, y in self.path_points[start_idx : end_idx + 1]:
            pose = PoseStamped()
            pose.header = roi_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            roi_path.poses.append(pose)

            point = pose.pose.position
            marker.points.append(point)

        self.roi_path_pub.publish(roi_path)
        self.roi_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RoiPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
