from __future__ import annotations

import math
from typing import Optional

from nav_msgs.msg import Odometry
from platoon_interfaces.msg import Heartbeat
from platoon_interfaces.msg import LeaderMotionState
from platoon_interfaces.msg import LeaderSafetyState
from platoon_interfaces.msg import RelativeLeaderState
import rclpy
from rclpy.node import Node


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class RelativeGpsLeaderNode(Node):
    """Build follower-frame leader state from same-map leader/follower odometry."""

    def __init__(self) -> None:
        super().__init__("relative_gps_leader_node")

        self.declare_parameter("leader_odom_topic", "/v2v/leader/odom")
        self.declare_parameter(
            "follower_odom_topic",
            "/follower/localization/global/odom",
        )
        self.declare_parameter("leader_motion_topic", "/v2v/leader/motion_state")
        self.declare_parameter("leader_safety_topic", "/v2v/leader/safety_state")
        self.declare_parameter("heartbeat_topic", "/v2v/leader/heartbeat")
        self.declare_parameter("output_topic", "/platoon/relative_leader/state")
        self.declare_parameter("base_frame", "follower/base_link")
        self.declare_parameter("expected_leader_child_frame", "leader/base_link")
        self.declare_parameter("target_frame", "leader/base_link")
        self.declare_parameter("expected_follower_child_frame", "follower/base_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("motion_timeout_sec", 0.5)
        self.declare_parameter("heartbeat_timeout_sec", 0.5)
        self.declare_parameter("safety_timeout_sec", 0.5)
        self.declare_parameter("require_safety_state", True)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.expected_leader_child_frame = str(
            self.get_parameter("expected_leader_child_frame").value
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.expected_follower_child_frame = str(
            self.get_parameter("expected_follower_child_frame").value
        )
        self.odom_timeout_sec = max(0.0, float(self.get_parameter("odom_timeout_sec").value))
        self.motion_timeout_sec = max(
            0.0, float(self.get_parameter("motion_timeout_sec").value)
        )
        self.heartbeat_timeout_sec = max(
            0.0, float(self.get_parameter("heartbeat_timeout_sec").value)
        )
        self.safety_timeout_sec = max(
            0.0, float(self.get_parameter("safety_timeout_sec").value)
        )
        self.require_safety_state = bool(self.get_parameter("require_safety_state").value)
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self.latest_leader_odom: Optional[Odometry] = None
        self.last_leader_odom_time: Optional[rclpy.time.Time] = None
        self.latest_follower_odom: Optional[Odometry] = None
        self.last_follower_odom_time: Optional[rclpy.time.Time] = None
        self.latest_motion: Optional[LeaderMotionState] = None
        self.last_motion_time: Optional[rclpy.time.Time] = None
        self.latest_safety: Optional[LeaderSafetyState] = None
        self.last_safety_time: Optional[rclpy.time.Time] = None
        self.latest_heartbeat: Optional[Heartbeat] = None
        self.last_heartbeat_time: Optional[rclpy.time.Time] = None

        self.create_subscription(
            Odometry,
            str(self.get_parameter("leader_odom_topic").value),
            self._leader_odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("follower_odom_topic").value),
            self._follower_odom_callback,
            10,
        )
        self.create_subscription(
            LeaderMotionState,
            str(self.get_parameter("leader_motion_topic").value),
            self._motion_callback,
            10,
        )
        self.create_subscription(
            LeaderSafetyState,
            str(self.get_parameter("leader_safety_topic").value),
            self._safety_callback,
            10,
        )
        self.create_subscription(
            Heartbeat,
            str(self.get_parameter("heartbeat_topic").value),
            self._heartbeat_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            RelativeLeaderState,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.get_logger().info(
            "relative_gps_leader_node started: "
            f"base_frame={self.base_frame}, odom_timeout={self.odom_timeout_sec:.2f}s"
        )

    def _leader_odom_callback(self, msg: Odometry) -> None:
        self.latest_leader_odom = msg
        self.last_leader_odom_time = self.get_clock().now()

    def _follower_odom_callback(self, msg: Odometry) -> None:
        self.latest_follower_odom = msg
        self.last_follower_odom_time = self.get_clock().now()

    def _motion_callback(self, msg: LeaderMotionState) -> None:
        self.latest_motion = msg
        self.last_motion_time = self.get_clock().now()

    def _safety_callback(self, msg: LeaderSafetyState) -> None:
        self.latest_safety = msg
        self.last_safety_time = self.get_clock().now()

    def _heartbeat_callback(self, msg: Heartbeat) -> None:
        self.latest_heartbeat = msg
        self.last_heartbeat_time = self.get_clock().now()

    def _publish_loop(self) -> None:
        now = self.get_clock().now()
        msg = RelativeLeaderState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.source = RelativeLeaderState.SOURCE_V2V_GPS
        msg.target_frame = self.target_frame

        pose_valid, pose_status = self._fill_relative_pose(msg, now)
        motion_valid, motion_status = self._fill_motion(msg, now)
        heartbeat_ok = self._fresh(
            self.last_heartbeat_time, self.heartbeat_timeout_sec, now
        )
        safety_ok = (
            not self.require_safety_state
            or self._fresh(self.last_safety_time, self.safety_timeout_sec, now)
        )

        msg.pose_valid = bool(pose_valid)
        msg.motion_valid = bool(motion_valid)
        msg.stop_required = self._stop_required()
        msg.reference_age_sec = float(self._reference_age_sec(now))

        status = "OK"
        if not pose_valid:
            status = pose_status
        elif not motion_valid:
            status = motion_status
        elif not heartbeat_ok:
            status = "HEARTBEAT_TIMEOUT"
        elif not safety_ok:
            status = "SAFETY_TIMEOUT"
        elif msg.stop_required:
            status = self._stop_reason()

        msg.valid = bool(
            pose_valid
            and motion_valid
            and heartbeat_ok
            and safety_ok
            and not msg.stop_required
        )
        msg.status = status
        self.state_pub.publish(msg)

    def _fill_relative_pose(
        self, msg: RelativeLeaderState, now: rclpy.time.Time
    ) -> tuple[bool, str]:
        leader = self.latest_leader_odom
        follower = self.latest_follower_odom
        if leader is None or self.last_leader_odom_time is None:
            return False, "LEADER_ODOM_MISSING"
        if follower is None or self.last_follower_odom_time is None:
            return False, "FOLLOWER_ODOM_MISSING"
        if not self._fresh(self.last_leader_odom_time, self.odom_timeout_sec, now):
            return False, "LEADER_ODOM_TIMEOUT"
        if not self._fresh(self.last_follower_odom_time, self.odom_timeout_sec, now):
            return False, "FOLLOWER_ODOM_TIMEOUT"
        if leader.header.frame_id != follower.header.frame_id:
            return False, "ODOM_FRAME_MISMATCH"
        if (
            self.expected_leader_child_frame
            and leader.child_frame_id != self.expected_leader_child_frame
        ):
            return False, "LEADER_CHILD_FRAME_MISMATCH"
        if (
            self.expected_follower_child_frame
            and follower.child_frame_id != self.expected_follower_child_frame
        ):
            return False, "FOLLOWER_CHILD_FRAME_MISMATCH"

        lp = leader.pose.pose.position
        fp = follower.pose.pose.position
        fq = follower.pose.pose.orientation
        lq = leader.pose.pose.orientation
        if not _finite(lp.x, lp.y, lp.z, fp.x, fp.y, fp.z):
            return False, "ODOM_POSITION_NONFINITE"
        if not _finite(fq.x, fq.y, fq.z, fq.w, lq.x, lq.y, lq.z, lq.w):
            return False, "ODOM_ORIENTATION_NONFINITE"

        follower_yaw = _yaw_from_quaternion(fq.x, fq.y, fq.z, fq.w)
        leader_yaw = _yaw_from_quaternion(lq.x, lq.y, lq.z, lq.w)
        dx_map = float(lp.x - fp.x)
        dy_map = float(lp.y - fp.y)
        cos_yaw = math.cos(follower_yaw)
        sin_yaw = math.sin(follower_yaw)
        x_base = cos_yaw * dx_map + sin_yaw * dy_map
        y_base = -sin_yaw * dx_map + cos_yaw * dy_map
        z_base = float(lp.z - fp.z)
        range_m = math.sqrt(x_base * x_base + y_base * y_base + z_base * z_base)
        if not _finite(x_base, y_base, z_base, range_m):
            return False, "RELATIVE_POSE_NONFINITE"

        rel_yaw = leader_yaw - follower_yaw
        qx, qy, qz, qw = _quaternion_from_yaw(rel_yaw)
        msg.relative_pose.pose.position.x = float(x_base)
        msg.relative_pose.pose.position.y = float(y_base)
        msg.relative_pose.pose.position.z = float(z_base)
        msg.relative_pose.pose.orientation.x = qx
        msg.relative_pose.pose.orientation.y = qy
        msg.relative_pose.pose.orientation.z = qz
        msg.relative_pose.pose.orientation.w = qw
        msg.longitudinal_distance_m = float(x_base)
        msg.lateral_offset_m = float(y_base)
        msg.range_m = float(range_m)

        for index in range(36):
            msg.relative_pose.covariance[index] = 0.0
        msg.relative_pose.covariance[0] = self._covariance_sum(leader, follower, 0)
        msg.relative_pose.covariance[7] = self._covariance_sum(leader, follower, 7)
        msg.relative_pose.covariance[14] = self._covariance_sum(leader, follower, 14)
        msg.relative_pose.covariance[35] = self._covariance_sum(leader, follower, 35)
        return True, "OK"

    def _fill_motion(
        self, msg: RelativeLeaderState, now: rclpy.time.Time
    ) -> tuple[bool, str]:
        motion = self.latest_motion
        if motion is None or self.last_motion_time is None:
            return False, "MOTION_MISSING"
        if not self._fresh(self.last_motion_time, self.motion_timeout_sec, now):
            return False, "MOTION_TIMEOUT"
        msg.leader_speed_mps = float(motion.speed_mps)
        msg.leader_acceleration_mps2 = float(motion.acceleration_mps2)
        msg.leader_curvature_1pm = float(motion.curvature_1pm)
        msg.leader_yaw_rate_radps = float(motion.yaw_rate_radps)
        if not _finite(
            msg.leader_speed_mps,
            msg.leader_acceleration_mps2,
            msg.leader_curvature_1pm,
            msg.leader_yaw_rate_radps,
        ):
            return False, "MOTION_NONFINITE"
        if not motion.motion_valid:
            return False, "MOTION_INVALID"
        return True, "OK"

    def _stop_required(self) -> bool:
        if self.latest_motion is not None and self.latest_motion.stop_required:
            return True
        if self.latest_safety is not None and self.latest_safety.stop_required:
            return True
        return False

    def _stop_reason(self) -> str:
        if self.latest_safety is not None and self.latest_safety.stop_required:
            return self.latest_safety.reason or "LEADER_SAFETY_STOP_REQUIRED"
        if self.latest_motion is not None and self.latest_motion.stop_required:
            return "LEADER_MOTION_STOP_REQUIRED"
        return "STOP_REQUIRED"

    def _reference_age_sec(self, now: rclpy.time.Time) -> float:
        ages = [
            self._age_sec(now, self.last_leader_odom_time),
            self._age_sec(now, self.last_follower_odom_time),
            self._age_sec(now, self.last_motion_time),
            self._age_sec(now, self.last_heartbeat_time),
        ]
        if self.require_safety_state:
            ages.append(self._age_sec(now, self.last_safety_time))
        finite_ages = [age for age in ages if age is not None]
        return max(finite_ages) if finite_ages else float("inf")

    @staticmethod
    def _covariance_sum(leader: Odometry, follower: Odometry, index: int) -> float:
        value = float(leader.pose.covariance[index]) + float(follower.pose.covariance[index])
        return value if math.isfinite(value) and value >= 0.0 else 0.0

    @staticmethod
    def _age_sec(
        now: rclpy.time.Time, stamp: Optional[rclpy.time.Time]
    ) -> Optional[float]:
        if stamp is None:
            return None
        return max(0.0, (now - stamp).nanoseconds * 1e-9)

    def _fresh(
        self,
        stamp: Optional[rclpy.time.Time],
        timeout_sec: float,
        now: rclpy.time.Time,
    ) -> bool:
        age = self._age_sec(now, stamp)
        return age is not None and age <= timeout_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeGpsLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
