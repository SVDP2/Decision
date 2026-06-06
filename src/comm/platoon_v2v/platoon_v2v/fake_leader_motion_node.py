import math

from platoon_interfaces.msg import Heartbeat
from platoon_interfaces.msg import LeaderMotionState
import rclpy
from rclpy.node import Node


class FakeLeaderMotionNode(Node):

    def __init__(self) -> None:
        super().__init__('fake_leader_motion_node')
        self.declare_parameter('motion_state_topic', '/v2v/leader/motion_state')
        self.declare_parameter('heartbeat_topic', '/v2v/leader/heartbeat')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('speed_mps', 0.4)
        self.declare_parameter('steering_angle_rad', 0.0)
        self.declare_parameter('wheel_base_m', 0.72)

        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.steering_angle_rad = float(self.get_parameter('steering_angle_rad').value)
        self.wheel_base_m = max(1e-3, float(self.get_parameter('wheel_base_m').value))
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.seq = 0
        self.heartbeat_seq = 0

        self.motion_pub = self.create_publisher(
            LeaderMotionState,
            str(self.get_parameter('motion_state_topic').value),
            10,
        )
        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            str(self.get_parameter('heartbeat_topic').value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish)

    def _publish(self) -> None:
        now = self.get_clock().now()
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        curvature = math.tan(self.steering_angle_rad) / self.wheel_base_m

        motion = LeaderMotionState()
        motion.header.stamp = now.to_msg()
        motion.header.frame_id = 'leader/base_link'
        motion.seq = self.seq
        motion.boot_id = 1
        motion.drive_mode = LeaderMotionState.MODE_MANUAL_RC
        motion.source = LeaderMotionState.SOURCE_SIMULATED
        motion.motion_valid = True
        motion.speed_mps = self.speed_mps
        motion.steering_angle_rad = self.steering_angle_rad
        motion.curvature_1pm = curvature
        motion.yaw_rate_radps = self.speed_mps * curvature
        self.motion_pub.publish(motion)

        if self.seq % 3 == 0:
            self.heartbeat_seq = (self.heartbeat_seq + 1) & 0xFFFFFFFF
            heartbeat = Heartbeat()
            heartbeat.header.stamp = now.to_msg()
            heartbeat.role = Heartbeat.ROLE_LEADER
            heartbeat.seq = self.heartbeat_seq
            heartbeat.boot_id = 1
            heartbeat.last_motion_seq = self.seq
            heartbeat.system_ok = True
            heartbeat.status = 'OK'
            self.heartbeat_pub.publish(heartbeat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeLeaderMotionNode()
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
