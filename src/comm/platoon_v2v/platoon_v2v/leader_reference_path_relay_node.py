from __future__ import annotations

from typing import Optional

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


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


class LeaderReferencePathRelayNode(Node):
    """Relay leader rolling path to the public V2V reference-path contract."""

    def __init__(self) -> None:
        super().__init__('leader_reference_path_relay_node')

        self.declare_parameter('input_path_topic', '/roi_path')
        self.declare_parameter('output_path_topic', '/v2v/leader/reference_path')
        self.declare_parameter('expected_frame_id', 'csv')
        self.declare_parameter('allow_empty_frame_id', False)
        self.declare_parameter('max_points', 80)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('input_transient_local', True)

        self.expected_frame_id = str(self.get_parameter('expected_frame_id').value)
        self.allow_empty_frame_id = bool(self.get_parameter('allow_empty_frame_id').value)
        self.max_points = max(2, int(self.get_parameter('max_points').value))
        self.input_timeout_sec = max(0.0, float(self.get_parameter('input_timeout_sec').value))
        publish_rate_hz = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        input_transient_local = bool(self.get_parameter('input_transient_local').value)

        self.latest_path: Optional[Path] = None
        self.last_path_time: Optional[rclpy.time.Time] = None
        self.last_drop_reason = ''

        input_qos = reliable_transient_qos() if input_transient_local else best_effort_qos()
        self.create_subscription(
            Path,
            str(self.get_parameter('input_path_topic').value),
            self._path_callback,
            input_qos,
        )
        self.path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('output_path_topic').value),
            best_effort_qos(),
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.get_logger().info(
            'leader_reference_path_relay_node started: '
            f'input={self.get_parameter("input_path_topic").value}, '
            f'output={self.get_parameter("output_path_topic").value}, '
            f'expected_frame_id={self.expected_frame_id or "<any>"}'
        )

    def _path_callback(self, msg: Path) -> None:
        frame_id = msg.header.frame_id or ''
        if not frame_id and not self.allow_empty_frame_id:
            self.last_drop_reason = 'empty_frame_id'
            self._warn_drop(self.last_drop_reason)
            return
        if self.expected_frame_id and frame_id != self.expected_frame_id:
            self.last_drop_reason = f'frame_mismatch:{frame_id}'
            self._warn_drop(self.last_drop_reason)
            return
        if len(msg.poses) < 2:
            self.last_drop_reason = 'insufficient_points'
            self._warn_drop(self.last_drop_reason)
            return
        self.latest_path = msg
        self.last_path_time = self.get_clock().now()
        self.last_drop_reason = ''

    def _publish_loop(self) -> None:
        if self.latest_path is None or self.last_path_time is None:
            return
        now = self.get_clock().now()
        age_sec = (now - self.last_path_time).nanoseconds * 1e-9
        if age_sec > self.input_timeout_sec:
            return
        output = Path()
        output.header = self.latest_path.header
        output.poses = list(self.latest_path.poses[: self.max_points])
        self.path_pub.publish(output)

    def _warn_drop(self, reason: str) -> None:
        self.get_logger().warning(
            f'dropping leader reference path: {reason}',
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
