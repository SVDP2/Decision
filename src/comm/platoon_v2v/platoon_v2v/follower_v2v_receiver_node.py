from typing import Optional

from platoon_interfaces.msg import Heartbeat
from platoon_interfaces.msg import LeaderMotionState
from platoon_interfaces.msg import LeaderSafetyState
from platoon_interfaces.msg import LinkStats
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


def best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class FollowerV2vReceiverNode(Node):

    def __init__(self) -> None:
        super().__init__('follower_v2v_receiver_node')

        self.declare_parameter('motion_state_topic', '/v2v/leader/motion_state')
        self.declare_parameter('safety_state_topic', '/v2v/leader/safety_state')
        self.declare_parameter('heartbeat_topic', '/v2v/leader/heartbeat')
        self.declare_parameter('leader_speed_topic', '/leader/velocity_mps')
        self.declare_parameter('leader_state_abnormal_topic', '/leader/state_abnormal')
        self.declare_parameter('leader_state_reason_topic', '/leader/state_reason')
        self.declare_parameter('link_stats_topic', '/v2v/link_stats')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('motion_timeout_sec', 0.3)
        self.declare_parameter('heartbeat_timeout_sec', 0.5)
        self.declare_parameter('latency_warn_ms', 150.0)

        self.motion_timeout_sec = float(self.get_parameter('motion_timeout_sec').value)
        self.heartbeat_timeout_sec = float(
            self.get_parameter('heartbeat_timeout_sec').value
        )
        self.latency_warn_ms = float(self.get_parameter('latency_warn_ms').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.latest_motion: Optional[LeaderMotionState] = None
        self.latest_motion_time: Optional[rclpy.time.Time] = None
        self.latest_safety: Optional[LeaderSafetyState] = None
        self.latest_safety_time: Optional[rclpy.time.Time] = None
        self.latest_heartbeat: Optional[Heartbeat] = None
        self.latest_heartbeat_time: Optional[rclpy.time.Time] = None

        self.received_count = 0
        self.dropped_count = 0
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.last_seq: Optional[int] = None
        self.last_latency_ms = 0.0
        self.mean_latency_ms = 0.0

        self.create_subscription(
            LeaderMotionState,
            str(self.get_parameter('motion_state_topic').value),
            self._motion_callback,
            best_effort_qos(),
        )
        self.create_subscription(
            LeaderSafetyState,
            str(self.get_parameter('safety_state_topic').value),
            self._safety_callback,
            10,
        )
        self.create_subscription(
            Heartbeat,
            str(self.get_parameter('heartbeat_topic').value),
            self._heartbeat_callback,
            10,
        )

        self.leader_speed_pub = self.create_publisher(
            Float32,
            str(self.get_parameter('leader_speed_topic').value),
            10,
        )
        self.leader_state_abnormal_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('leader_state_abnormal_topic').value),
            10,
        )
        self.leader_state_reason_pub = self.create_publisher(
            String,
            str(self.get_parameter('leader_state_reason_topic').value),
            10,
        )
        self.link_stats_pub = self.create_publisher(
            LinkStats,
            str(self.get_parameter('link_stats_topic').value),
            10,
        )

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.get_logger().info(
            'follower_v2v_receiver_node started: '
            f'motion_timeout_sec={self.motion_timeout_sec:.2f}, '
            f'heartbeat_timeout_sec={self.heartbeat_timeout_sec:.2f}'
        )

    def _motion_callback(self, msg: LeaderMotionState) -> None:
        now = self.get_clock().now()
        self.latest_motion = msg
        self.latest_motion_time = now
        self.received_count += 1
        self._update_sequence_counters(int(msg.seq))

        sent_time = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = max(0.0, (now - sent_time).nanoseconds * 1e-6)
        self.last_latency_ms = latency_ms
        alpha = 0.1
        if self.received_count <= 1:
            self.mean_latency_ms = latency_ms
        else:
            self.mean_latency_ms = (
                (1.0 - alpha) * self.mean_latency_ms + alpha * latency_ms
            )

    def _safety_callback(self, msg: LeaderSafetyState) -> None:
        self.latest_safety = msg
        self.latest_safety_time = self.get_clock().now()

    def _heartbeat_callback(self, msg: Heartbeat) -> None:
        self.latest_heartbeat = msg
        self.latest_heartbeat_time = self.get_clock().now()

    def _update_sequence_counters(self, seq: int) -> None:
        if self.last_seq is None:
            self.last_seq = seq
            return
        expected = (self.last_seq + 1) & 0xFFFFFFFF
        if seq == self.last_seq:
            self.duplicate_count += 1
        elif seq < self.last_seq and self.last_seq - seq < 0x7FFFFFFF:
            self.out_of_order_count += 1
        elif seq != expected:
            self.dropped_count += (seq - expected) & 0xFFFFFFFF
        self.last_seq = seq

    def _publish_loop(self) -> None:
        now = self.get_clock().now()
        motion_age_ms = self._age_ms(now, self.latest_motion_time)
        link_ok, status = self._evaluate_link(now)

        speed = 0.0
        if link_ok and self.latest_motion is not None and self.latest_motion.motion_valid:
            speed = float(self.latest_motion.speed_mps)
        self.leader_speed_pub.publish(Float32(data=float(speed)))

        abnormal = not link_ok
        abnormal_reason = status if not link_ok else 'NONE'
        if self.latest_safety is not None and self.latest_safety.stop_required:
            abnormal = True
            abnormal_reason = (
                self.latest_safety.reason
                if self.latest_safety.reason
                else 'LEADER_SAFETY_STOP_REQUIRED'
            )
        if self.latest_motion is not None and not self.latest_motion.motion_valid:
            abnormal = True
            abnormal_reason = 'MOTION_INVALID'
        self.leader_state_abnormal_pub.publish(Bool(data=bool(abnormal)))
        self.leader_state_reason_pub.publish(String(data=abnormal_reason))

        stats = LinkStats()
        stats.header.stamp = now.to_msg()
        stats.link_ok = bool(link_ok)
        stats.received_count = int(self.received_count)
        stats.dropped_count = int(self.dropped_count)
        stats.duplicate_count = int(self.duplicate_count)
        stats.out_of_order_count = int(self.out_of_order_count)
        stats.last_seq = int(self.last_seq) if self.last_seq is not None else 0
        stats.last_latency_ms = float(self.last_latency_ms)
        stats.mean_latency_ms = float(self.mean_latency_ms)
        stats.last_message_age_ms = float(motion_age_ms)
        stats.status = status
        self.link_stats_pub.publish(stats)

    def _evaluate_link(self, now: rclpy.time.Time) -> tuple[bool, str]:
        motion_age_sec = self._age_sec(now, self.latest_motion_time)
        if motion_age_sec is None or motion_age_sec > self.motion_timeout_sec:
            return False, 'MOTION_TIMEOUT'
        heartbeat_age_sec = self._age_sec(now, self.latest_heartbeat_time)
        if heartbeat_age_sec is None or heartbeat_age_sec > self.heartbeat_timeout_sec:
            return False, 'HEARTBEAT_TIMEOUT'
        if self.last_latency_ms > self.latency_warn_ms:
            return True, 'HIGH_LATENCY'
        return True, 'OK'

    @staticmethod
    def _age_sec(
        now: rclpy.time.Time,
        stamp: Optional[rclpy.time.Time],
    ) -> Optional[float]:
        if stamp is None:
            return None
        return max(0.0, (now - stamp).nanoseconds * 1e-9)

    def _age_ms(
        self,
        now: rclpy.time.Time,
        stamp: Optional[rclpy.time.Time],
    ) -> float:
        age_sec = self._age_sec(now, stamp)
        return -1.0 if age_sec is None else age_sec * 1000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerV2vReceiverNode()
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
