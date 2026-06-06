from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from platoon_interfaces.msg import LeaderMotionState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class LeaderPreviewPathNode(Node):

    def __init__(self) -> None:
        super().__init__('leader_preview_path_node')

        self.declare_parameter('motion_state_topic', '/v2v/leader/motion_state')
        self.declare_parameter('path_topic', '/leader_path')
        self.declare_parameter('frame_id', 'leader/leader_rear')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('path_length_m', 3.0)
        self.declare_parameter('point_spacing_m', 0.1)
        self.declare_parameter('motion_timeout_sec', 0.4)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.path_length_m = max(0.2, float(self.get_parameter('path_length_m').value))
        self.point_spacing_m = max(0.02, float(self.get_parameter('point_spacing_m').value))
        self.motion_timeout_sec = float(self.get_parameter('motion_timeout_sec').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.latest_motion: Optional[LeaderMotionState] = None
        self.latest_motion_time: Optional[rclpy.time.Time] = None

        self.create_subscription(
            LeaderMotionState,
            str(self.get_parameter('motion_state_topic').value),
            self._motion_callback,
            best_effort_qos(),
        )
        self.path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('path_topic').value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.get_logger().info(
            'leader_preview_path_node started: '
            f'frame_id={self.frame_id}, path_length_m={self.path_length_m:.2f}, '
            'steering feedforward disabled'
        )

    def _motion_callback(self, msg: LeaderMotionState) -> None:
        self.latest_motion = msg
        self.latest_motion_time = self.get_clock().now()

    def _publish_loop(self) -> None:
        now = self.get_clock().now()
        path = Path()
        path.header.stamp = now.to_msg()
        path.header.frame_id = self.frame_id

        motion = self.latest_motion
        if (
            motion is None
            or self.latest_motion_time is None
            or (now - self.latest_motion_time).nanoseconds * 1e-9 > self.motion_timeout_sec
            or not motion.motion_valid
        ):
            self.path_pub.publish(path)
            return

        point_count = max(2, int(self.path_length_m / self.point_spacing_m) + 1)
        for index in range(point_count):
            s = min(self.path_length_m, index * self.point_spacing_m)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(s)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderPreviewPathNode()
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
