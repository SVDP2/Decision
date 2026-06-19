from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def reliable_transient_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def normalize_source(text: str) -> str:
    normalized = str(text or '').strip().lower()
    if any(token in normalized for token in ('complex', 'cone', 'rrt', 'unstructured')):
        return 'complex'
    if any(token in normalized for token in ('highway', 'main', 'freeway', 'gps_tracking')):
        return 'highway'
    return ''


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


@dataclass
class PathSample:
    path: Path
    stamp: rclpy.time.Time


class LeaderReferencePathRelayNode(Node):
    """Publish the leader's active planning path to the V2V reference-path contract."""

    def __init__(self) -> None:
        super().__init__('leader_reference_path_relay_node')

        self.declare_parameter('highway_path_topic', '/roi_path')
        self.declare_parameter('complex_path_topic', '/complex/local_path')
        self.declare_parameter('leader_odom_topic', '/v2v/leader/odom')
        self.declare_parameter('input_path_topic', '/roi_path')
        self.declare_parameter('output_path_topic', '/v2v/leader/reference_path')
        self.declare_parameter('marker_topic', '/v2v/leader/reference_path_marker')
        self.declare_parameter('status_topic', '/v2v/leader/reference_path_status')
        self.declare_parameter('command_source_topic', '/planning_command_source')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('active_algorithm_topic', '/active_algorithm')
        self.declare_parameter('expected_frame_id', 'map')
        self.declare_parameter('leader_frame_id', 'leader/base_link')
        self.declare_parameter('allow_empty_frame_id', False)
        self.declare_parameter('max_points', 80)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('source_timeout_sec', 1.0)
        self.declare_parameter('input_transient_local', True)
        self.declare_parameter('default_source', 'highway')

        self.expected_frame_id = str(self.get_parameter('expected_frame_id').value)
        self.leader_frame_id = str(self.get_parameter('leader_frame_id').value)
        self.allow_empty_frame_id = bool(self.get_parameter('allow_empty_frame_id').value)
        self.max_points = max(2, int(self.get_parameter('max_points').value))
        self.input_timeout_sec = max(0.0, float(self.get_parameter('input_timeout_sec').value))
        self.source_timeout_sec = max(0.0, float(self.get_parameter('source_timeout_sec').value))
        self.default_source = normalize_source(str(self.get_parameter('default_source').value))
        if not self.default_source:
            self.default_source = 'highway'
        publish_rate_hz = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        input_transient_local = bool(self.get_parameter('input_transient_local').value)

        self.highway_path: Optional[PathSample] = None
        self.complex_path: Optional[PathSample] = None
        self.latest_leader_odom: Optional[Odometry] = None
        self.latest_leader_odom_time: Optional[rclpy.time.Time] = None
        self.command_source = ''
        self.command_source_time: Optional[rclpy.time.Time] = None
        self.mission_source = ''
        self.mission_source_time: Optional[rclpy.time.Time] = None
        self.algorithm_source = ''
        self.algorithm_source_time: Optional[rclpy.time.Time] = None
        self.last_status = ''

        input_qos = reliable_transient_qos() if input_transient_local else best_effort_qos()
        self.create_subscription(
            Path,
            str(self.get_parameter('highway_path_topic').value),
            self._highway_path_callback,
            input_qos,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('complex_path_topic').value),
            self._complex_path_callback,
            input_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('leader_odom_topic').value),
            self._leader_odom_callback,
            best_effort_qos(),
        )
        self.create_subscription(
            String,
            str(self.get_parameter('command_source_topic').value),
            self._command_source_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_state_topic').value),
            self._mission_state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('active_algorithm_topic').value),
            self._active_algorithm_callback,
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('output_path_topic').value),
            best_effort_qos(),
        )
        self.marker_pub = self.create_publisher(
            Marker,
            str(self.get_parameter('marker_topic').value),
            best_effort_qos(),
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.get_logger().info(
            'leader_reference_path_relay_node started: '
            f'highway={self.get_parameter("highway_path_topic").value}, '
            f'complex={self.get_parameter("complex_path_topic").value}, '
            f'output={self.get_parameter("output_path_topic").value}, '
            f'expected_frame_id={self.expected_frame_id or "<any>"}'
        )

    def _highway_path_callback(self, msg: Path) -> None:
        self._store_path('highway', msg)

    def _complex_path_callback(self, msg: Path) -> None:
        self._store_path('complex', msg)

    def _leader_odom_callback(self, msg: Odometry) -> None:
        self.latest_leader_odom = msg
        self.latest_leader_odom_time = self.get_clock().now()

    def _command_source_callback(self, msg: String) -> None:
        source = normalize_source(msg.data)
        if source:
            self.command_source = source
            self.command_source_time = self.get_clock().now()

    def _mission_state_callback(self, msg: String) -> None:
        source = normalize_source(msg.data)
        if source:
            self.mission_source = source
            self.mission_source_time = self.get_clock().now()

    def _active_algorithm_callback(self, msg: String) -> None:
        source = normalize_source(msg.data)
        if source:
            self.algorithm_source = source
            self.algorithm_source_time = self.get_clock().now()

    def _store_path(self, source: str, msg: Path) -> None:
        path = self._normalize_path(source, msg)
        if path is None:
            return
        sample = PathSample(path, self.get_clock().now())
        if source == 'complex':
            self.complex_path = sample
        else:
            self.highway_path = sample

    def _normalize_path(self, source: str, msg: Path) -> Optional[Path]:
        frame_id = msg.header.frame_id or ''
        if not frame_id and not self.allow_empty_frame_id:
            self._warn_drop(source, 'empty_frame_id')
            return None
        if len(msg.poses) < 2:
            self._warn_drop(source, 'insufficient_points')
            return None
        if not self.expected_frame_id or frame_id == self.expected_frame_id:
            return msg
        if source == 'complex' and frame_id == self.leader_frame_id:
            converted = self._leader_frame_path_to_map(msg)
            if converted is None:
                self._warn_drop(source, 'missing_leader_odom_for_frame_transform')
            return converted
        if self.expected_frame_id and frame_id != self.expected_frame_id:
            self._warn_drop(source, f'frame_mismatch:{frame_id}')
            return None
        return msg

    def _leader_frame_path_to_map(self, msg: Path) -> Optional[Path]:
        leader_odom = self.latest_leader_odom
        if leader_odom is None or leader_odom.header.frame_id != self.expected_frame_id:
            return None
        leader_pose = leader_odom.pose.pose
        leader_yaw = yaw_from_quaternion(
            leader_pose.orientation.x,
            leader_pose.orientation.y,
            leader_pose.orientation.z,
            leader_pose.orientation.w,
        )
        if leader_yaw is None:
            return None

        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        output = Path()
        output.header = msg.header
        output.header.frame_id = self.expected_frame_id
        output.poses = []
        for pose in msg.poses:
            converted = PoseStamped()
            converted.header = pose.header
            converted.header.frame_id = self.expected_frame_id
            local_x = float(pose.pose.position.x)
            local_y = float(pose.pose.position.y)
            converted.pose.position.x = (
                leader_pose.position.x + local_x * cos_yaw - local_y * sin_yaw
            )
            converted.pose.position.y = (
                leader_pose.position.y + local_x * sin_yaw + local_y * cos_yaw
            )
            converted.pose.position.z = leader_pose.position.z + float(
                pose.pose.position.z
            )
            local_yaw = yaw_from_quaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            qx, qy, qz, qw = quaternion_from_yaw(
                leader_yaw + (local_yaw if local_yaw is not None else 0.0)
            )
            converted.pose.orientation.x = qx
            converted.pose.orientation.y = qy
            converted.pose.orientation.z = qz
            converted.pose.orientation.w = qw
            output.poses.append(converted)
        return output

    def _publish_loop(self) -> None:
        now = self.get_clock().now()
        source = self._select_source(now)
        sample = self.complex_path if source == 'complex' else self.highway_path
        if sample is None:
            self._publish_status(f'{source}_missing')
            return
        age_sec = (now - sample.stamp).nanoseconds * 1e-9
        if age_sec > self.input_timeout_sec:
            self._publish_status(f'{source}_stale:{age_sec:.2f}s')
            return

        output = Path()
        output.header = sample.path.header
        output.poses = list(sample.path.poses[: self.max_points])
        self.path_pub.publish(output)
        self.marker_pub.publish(self._make_path_marker(output, source))
        self._publish_status(f'{source}:ok:{len(output.poses)}')

    def _select_source(self, now: rclpy.time.Time) -> str:
        for source, stamp in (
            (self.command_source, self.command_source_time),
            (self.mission_source, self.mission_source_time),
            (self.algorithm_source, self.algorithm_source_time),
        ):
            if source and stamp is not None:
                age_sec = (now - stamp).nanoseconds * 1e-9
                if age_sec <= self.source_timeout_sec:
                    return source
        return self.default_source

    def _make_path_marker(self, path: Path, source: str) -> Marker:
        marker = Marker()
        marker.header = path.header
        marker.ns = 'leader_active_reference_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.a = 1.0
        if source == 'complex':
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.05
        else:
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 1.0
        marker.points = [pose.pose.position for pose in path.poses]
        return marker

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if status != self.last_status:
            self.get_logger().info(f'leader reference path relay -> {status}')
            self.last_status = status

    def _warn_drop(self, source: str, reason: str) -> None:
        self.get_logger().warning(
            f'dropping {source} leader reference path: {reason}',
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderReferencePathRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
