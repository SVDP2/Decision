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
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12 or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def _rotation_matrix_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x, y, z, w = _normalize_quaternion(x, y, z, w)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def fill_relative_pose_from_odom(
    msg: RelativeLeaderState,
    odom: Odometry,
    expected_odom_frame: str,
    expected_odom_child_frame: str,
    require_expected_frames: bool,
    status_prefix: str,
    odom_pose_is_base_to_target: bool = False,
) -> tuple[bool, str]:
    if require_expected_frames:
        if odom.header.frame_id != expected_odom_frame:
            return False, f"{status_prefix}_ODOM_FRAME_MISMATCH"
        if odom.child_frame_id != expected_odom_child_frame:
            return False, f"{status_prefix}_ODOM_CHILD_FRAME_MISMATCH"

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    if not _finite(position.x, position.y, position.z):
        return False, f"{status_prefix}_POSITION_NONFINITE"
    if not _finite(orientation.x, orientation.y, orientation.z, orientation.w):
        return False, f"{status_prefix}_ORIENTATION_NONFINITE"

    qx, qy, qz, qw = _normalize_quaternion(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    tx = float(position.x)
    ty = float(position.y)
    tz = float(position.z)

    if odom_pose_is_base_to_target:
        x_base = tx
        y_base = ty
        z_base = tz
        out_qx, out_qy, out_qz, out_qw = qx, qy, qz, qw
    else:
        rotation = _rotation_matrix_from_quaternion(qx, qy, qz, qw)
        x_base = -(rotation[0][0] * tx + rotation[1][0] * ty + rotation[2][0] * tz)
        y_base = -(rotation[0][1] * tx + rotation[1][1] * ty + rotation[2][1] * tz)
        z_base = -(rotation[0][2] * tx + rotation[1][2] * ty + rotation[2][2] * tz)
        out_qx, out_qy, out_qz, out_qw = -qx, -qy, -qz, qw

    range_m = math.sqrt(x_base * x_base + y_base * y_base + z_base * z_base)
    if not _finite(x_base, y_base, z_base, range_m):
        return False, "RELATIVE_POSE_NONFINITE"

    msg.relative_pose.pose.position.x = float(x_base)
    msg.relative_pose.pose.position.y = float(y_base)
    msg.relative_pose.pose.position.z = float(z_base)
    msg.relative_pose.pose.orientation.x = out_qx
    msg.relative_pose.pose.orientation.y = out_qy
    msg.relative_pose.pose.orientation.z = out_qz
    msg.relative_pose.pose.orientation.w = out_qw
    msg.longitudinal_distance_m = float(x_base)
    msg.lateral_offset_m = float(y_base)
    msg.range_m = float(range_m)
    msg.relative_pose.covariance = list(odom.pose.covariance)
    return True, "OK"


class RelativeOdomLeaderNode(Node):
    """Build follower-frame leader state from leader_rear odometry."""

    def __init__(
        self,
        *,
        node_name: str,
        default_odom_topic: str,
        default_source: int,
        status_prefix: str,
        legacy_odom_parameter: Optional[str] = None,
        default_expected_odom_frame: str = "leader/leader_rear",
        default_expected_odom_child_frame: str = "follower/base_link",
        default_target_frame: str = "",
        default_odom_pose_is_base_to_target: bool = False,
    ) -> None:
        super().__init__(node_name)

        self.status_prefix = status_prefix
        self.declare_parameter("odom_topic", default_odom_topic)
        if legacy_odom_parameter is not None:
            self.declare_parameter(legacy_odom_parameter, default_odom_topic)
        self.declare_parameter("leader_motion_topic", "/v2v/leader/motion_state")
        self.declare_parameter("leader_safety_topic", "/v2v/leader/safety_state")
        self.declare_parameter("heartbeat_topic", "/v2v/leader/heartbeat")
        self.declare_parameter("output_topic", "/platoon/relative_leader/state")
        self.declare_parameter("base_frame", "follower/base_link")
        self.declare_parameter("expected_odom_frame", default_expected_odom_frame)
        self.declare_parameter(
            "expected_odom_child_frame",
            default_expected_odom_child_frame,
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("motion_timeout_sec", 0.5)
        self.declare_parameter("heartbeat_timeout_sec", 0.5)
        self.declare_parameter("safety_timeout_sec", 0.5)
        self.declare_parameter("require_safety_state", True)
        self.declare_parameter("require_expected_frames", True)
        self.declare_parameter("source", default_source)
        self.declare_parameter("target_frame", default_target_frame)
        self.declare_parameter(
            "odom_pose_is_base_to_target",
            default_odom_pose_is_base_to_target,
        )

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.expected_odom_frame = str(self.get_parameter("expected_odom_frame").value)
        self.expected_odom_child_frame = str(
            self.get_parameter("expected_odom_child_frame").value
        )
        self.pose_timeout_sec = max(0.0, float(self.get_parameter("pose_timeout_sec").value))
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
        self.require_expected_frames = bool(
            self.get_parameter("require_expected_frames").value
        )
        self.odom_pose_is_base_to_target = bool(
            self.get_parameter("odom_pose_is_base_to_target").value
        )
        self.source = int(self.get_parameter("source").value)
        target_frame = str(self.get_parameter("target_frame").value)
        self.target_frame = target_frame or self.expected_odom_frame
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        odom_topic = str(self.get_parameter("odom_topic").value)
        if legacy_odom_parameter is not None and odom_topic == default_odom_topic:
            odom_topic = str(self.get_parameter(legacy_odom_parameter).value)

        self.latest_odom: Optional[Odometry] = None
        self.last_odom_time: Optional[rclpy.time.Time] = None
        self.latest_motion: Optional[LeaderMotionState] = None
        self.last_motion_time: Optional[rclpy.time.Time] = None
        self.latest_safety: Optional[LeaderSafetyState] = None
        self.last_safety_time: Optional[rclpy.time.Time] = None
        self.latest_heartbeat: Optional[Heartbeat] = None
        self.last_heartbeat_time: Optional[rclpy.time.Time] = None

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(
            LeaderMotionState,
            str(self.get_parameter("leader_motion_topic").value),
            self._motion_callback,
            _best_effort_qos(),
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
            f"{node_name} started: odom={odom_topic}, "
            f"base_frame={self.base_frame}, pose_timeout={self.pose_timeout_sec:.2f}s"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.last_odom_time = self.get_clock().now()

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
        msg.source = self.source
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
        odom = self.latest_odom
        if odom is None or self.last_odom_time is None:
            return False, f"{self.status_prefix}_ODOM_MISSING"
        if not self._fresh(self.last_odom_time, self.pose_timeout_sec, now):
            return False, f"{self.status_prefix}_ODOM_TIMEOUT"
        return fill_relative_pose_from_odom(
            msg,
            odom,
            self.expected_odom_frame,
            self.expected_odom_child_frame,
            self.require_expected_frames,
            self.status_prefix,
            self.odom_pose_is_base_to_target,
        )

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
            self._age_sec(now, self.last_odom_time),
            self._age_sec(now, self.last_motion_time),
            self._age_sec(now, self.last_heartbeat_time),
        ]
        if self.require_safety_state:
            ages.append(self._age_sec(now, self.last_safety_time))
        finite_ages = [age for age in ages if age is not None]
        return max(finite_ages) if finite_ages else float("inf")

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
