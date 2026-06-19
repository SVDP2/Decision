import math
from typing import Optional

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from platoon_interfaces.msg import Heartbeat
from platoon_interfaces.msg import LeaderDriveTelemetry
from platoon_interfaces.msg import LeaderMissionState
from platoon_interfaces.msg import LeaderMotionState
from platoon_interfaces.msg import LeaderSafetyState
from platoon_v2v.speed_resolution import signed_speed_from_telemetry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String


def best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class LeaderV2vAdapterNode(Node):

    def __init__(self) -> None:
        super().__init__('leader_v2v_adapter_node')

        self.declare_parameter('drive_telemetry_topic', '/leader/drive_telemetry')
        self.declare_parameter('encoder_twist_topic', '/encoder/twist')
        self.declare_parameter('leader_odom_topic', '/leader/localization/gps/odom')
        self.declare_parameter('vehicle_ref_utm_topic', '/vehicle_ref_utm')
        self.declare_parameter('heading_topic', '/vehicle_heading_rad')
        self.declare_parameter('heading_valid_topic', '/vehicle_heading_valid')
        self.declare_parameter('fix_velocity_topic', '/f9p/fix_velocity')
        self.declare_parameter('map_origin_topic', '/leader/map_origin_utm')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('active_algorithm_topic', '/active_algorithm')
        self.declare_parameter('safety_status_topic', '/safety_status')
        self.declare_parameter('safety_active_topic', '/safety_active')
        self.declare_parameter('motion_state_topic', '/v2v/leader/motion_state')
        self.declare_parameter('v2v_odom_topic', '/v2v/leader/odom')
        self.declare_parameter('v2v_mission_state_topic', '/v2v/leader/mission_state')
        self.declare_parameter('v2v_safety_state_topic', '/v2v/leader/safety_state')
        self.declare_parameter('heartbeat_topic', '/v2v/leader/heartbeat')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('heartbeat_rate_hz', 10.0)
        self.declare_parameter('wheel_base_m', 0.72)
        self.declare_parameter('leader_odom_timeout_sec', 0.5)
        self.declare_parameter('autonomy_pose_timeout_sec', 0.5)
        self.declare_parameter('heading_timeout_sec', 0.5)
        self.declare_parameter('fix_velocity_timeout_sec', 0.5)
        self.declare_parameter('fallback_to_leader_odom', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('leader_base_frame', 'leader/base_link')
        self.declare_parameter('origin_easting_m', 330344.301121512195095)
        self.declare_parameter('origin_northing_m', 4156712.793599965050817)
        self.declare_parameter('autonomy_fixed_z_m', 0.0)
        self.declare_parameter('autonomy_position_variance_m2', 0.01)
        self.declare_parameter('autonomy_z_variance_m2', 0.01)
        self.declare_parameter('autonomy_yaw_variance_rad2', 0.01)
        self.declare_parameter('telemetry_timeout_sec', 0.3)
        self.declare_parameter('prefer_encoder_twist_speed', False)
        self.declare_parameter('use_throttle_signed_speed_fallback', True)
        self.declare_parameter('telemetry_speed_deadband_mps', 0.02)
        self.declare_parameter('throttle_speed_deadband', 0.05)
        self.declare_parameter('throttle_speed_fallback_gain_mps', 0.50)
        self.declare_parameter('rc_throttle_neutral_us', 1500)
        self.declare_parameter('rc_throttle_deadband_us', 80)
        self.declare_parameter('rc_throttle_full_scale_us', 400)

        self.wheel_base_m = max(1e-3, float(self.get_parameter('wheel_base_m').value))
        self.leader_odom_timeout_sec = float(
            self.get_parameter('leader_odom_timeout_sec').value
        )
        self.autonomy_pose_timeout_sec = float(
            self.get_parameter('autonomy_pose_timeout_sec').value
        )
        self.heading_timeout_sec = float(self.get_parameter('heading_timeout_sec').value)
        self.fix_velocity_timeout_sec = float(
            self.get_parameter('fix_velocity_timeout_sec').value
        )
        self.fallback_to_leader_odom = bool(
            self.get_parameter('fallback_to_leader_odom').value
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.leader_base_frame = str(self.get_parameter('leader_base_frame').value)
        self.origin_easting_m = float(self.get_parameter('origin_easting_m').value)
        self.origin_northing_m = float(self.get_parameter('origin_northing_m').value)
        self.autonomy_fixed_z_m = float(self.get_parameter('autonomy_fixed_z_m').value)
        self.autonomy_position_variance_m2 = max(
            0.0, float(self.get_parameter('autonomy_position_variance_m2').value)
        )
        self.autonomy_z_variance_m2 = max(
            0.0, float(self.get_parameter('autonomy_z_variance_m2').value)
        )
        self.autonomy_yaw_variance_rad2 = max(
            0.0, float(self.get_parameter('autonomy_yaw_variance_rad2').value)
        )
        self.telemetry_timeout_sec = float(
            self.get_parameter('telemetry_timeout_sec').value
        )
        self.prefer_encoder_twist_speed = bool(
            self.get_parameter('prefer_encoder_twist_speed').value
        )
        self.use_throttle_signed_speed_fallback = bool(
            self.get_parameter('use_throttle_signed_speed_fallback').value
        )
        self.telemetry_speed_deadband_mps = max(
            0.0, float(self.get_parameter('telemetry_speed_deadband_mps').value)
        )
        self.throttle_speed_deadband = max(
            0.0, float(self.get_parameter('throttle_speed_deadband').value)
        )
        self.throttle_speed_fallback_gain_mps = max(
            0.0, float(self.get_parameter('throttle_speed_fallback_gain_mps').value)
        )
        self.rc_throttle_neutral_us = int(self.get_parameter('rc_throttle_neutral_us').value)
        self.rc_throttle_deadband_us = max(
            0, int(self.get_parameter('rc_throttle_deadband_us').value)
        )
        self.rc_throttle_full_scale_us = max(
            1, int(self.get_parameter('rc_throttle_full_scale_us').value)
        )
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        heartbeat_rate_hz = max(1.0, float(self.get_parameter('heartbeat_rate_hz').value))

        self.latest_telemetry: Optional[LeaderDriveTelemetry] = None
        self.latest_telemetry_time: Optional[rclpy.time.Time] = None
        self.latest_encoder_twist: Optional[TwistWithCovarianceStamped] = None
        self.latest_encoder_twist_time: Optional[rclpy.time.Time] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_time: Optional[rclpy.time.Time] = None
        self.latest_vehicle_ref_utm: Optional[PointStamped] = None
        self.latest_vehicle_ref_utm_time: Optional[rclpy.time.Time] = None
        self.latest_heading_rad: Optional[float] = None
        self.latest_heading_time: Optional[rclpy.time.Time] = None
        self.latest_heading_valid = False
        self.latest_fix_velocity: Optional[TwistWithCovarianceStamped] = None
        self.latest_fix_velocity_time: Optional[rclpy.time.Time] = None
        self.latest_mission_state = ''
        self.latest_active_algorithm = ''
        self.latest_safety_status = 'UNKNOWN'
        self.latest_safety_active = False
        self.prev_speed_mps: Optional[float] = None
        self.prev_motion_time: Optional[rclpy.time.Time] = None
        self.heartbeat_seq = 0
        self.mission_seq = 0
        self.safety_seq = 0
        self.last_heartbeat_time = self.get_clock().now()

        self.create_subscription(
            LeaderDriveTelemetry,
            str(self.get_parameter('drive_telemetry_topic').value),
            self._telemetry_callback,
            best_effort_qos(),
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            str(self.get_parameter('encoder_twist_topic').value),
            self._encoder_twist_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('leader_odom_topic').value),
            self._odom_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('vehicle_ref_utm_topic').value),
            self._vehicle_ref_utm_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter('heading_topic').value),
            self._heading_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('heading_valid_topic').value),
            self._heading_valid_callback,
            10,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            str(self.get_parameter('fix_velocity_topic').value),
            self._fix_velocity_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('map_origin_topic').value),
            self._map_origin_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_state_topic').value),
            self._mission_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('active_algorithm_topic').value),
            self._active_algorithm_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('safety_status_topic').value),
            self._safety_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('safety_active_topic').value),
            self._safety_active_callback,
            10,
        )

        self.motion_pub = self.create_publisher(
            LeaderMotionState,
            str(self.get_parameter('motion_state_topic').value),
            best_effort_qos(),
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter('v2v_odom_topic').value),
            best_effort_qos(),
        )
        self.mission_pub = self.create_publisher(
            LeaderMissionState,
            str(self.get_parameter('v2v_mission_state_topic').value),
            10,
        )
        self.safety_pub = self.create_publisher(
            LeaderSafetyState,
            str(self.get_parameter('v2v_safety_state_topic').value),
            10,
        )
        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            str(self.get_parameter('heartbeat_topic').value),
            10,
        )

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.heartbeat_period_sec = 1.0 / heartbeat_rate_hz

        self.get_logger().info(
            'leader_v2v_adapter_node started: '
            f'wheel_base_m={self.wheel_base_m:.3f}, '
            f'autonomy_origin=({self.origin_easting_m:.3f}, '
            f'{self.origin_northing_m:.3f}), '
            f'fallback_to_leader_odom={self.fallback_to_leader_odom}, '
            f'telemetry_timeout_sec={self.telemetry_timeout_sec:.2f}'
        )

    def _telemetry_callback(self, msg: LeaderDriveTelemetry) -> None:
        self.latest_telemetry = msg
        self.latest_telemetry_time = self.get_clock().now()

    def _encoder_twist_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_encoder_twist = msg
        self.latest_encoder_twist_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_time = self.get_clock().now()

    def _vehicle_ref_utm_callback(self, msg: PointStamped) -> None:
        self.latest_vehicle_ref_utm = msg
        self.latest_vehicle_ref_utm_time = self.get_clock().now()

    def _heading_callback(self, msg: Float64) -> None:
        self.latest_heading_rad = float(msg.data)
        self.latest_heading_time = self.get_clock().now()

    def _heading_valid_callback(self, msg: Bool) -> None:
        self.latest_heading_valid = bool(msg.data)

    def _fix_velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_fix_velocity = msg
        self.latest_fix_velocity_time = self.get_clock().now()

    def _map_origin_callback(self, msg: PointStamped) -> None:
        if not self._all_finite(msg.point.x, msg.point.y):
            return
        previous = (self.origin_easting_m, self.origin_northing_m)
        self.origin_easting_m = float(msg.point.x)
        self.origin_northing_m = float(msg.point.y)
        if (
            abs(previous[0] - self.origin_easting_m) > 1e-6
            or abs(previous[1] - self.origin_northing_m) > 1e-6
        ):
            self.get_logger().info(
                'V2V map origin updated from leader origin topic: '
                f'easting={self.origin_easting_m:.6f}, '
                f'northing={self.origin_northing_m:.6f}'
            )

    def _mission_callback(self, msg: String) -> None:
        self.latest_mission_state = msg.data

    def _active_algorithm_callback(self, msg: String) -> None:
        self.latest_active_algorithm = msg.data

    def _safety_status_callback(self, msg: String) -> None:
        self.latest_safety_status = msg.data

    def _safety_active_callback(self, msg: Bool) -> None:
        self.latest_safety_active = bool(msg.data)

    def _publish_loop(self) -> None:
        now = self.get_clock().now()
        telemetry = self.latest_telemetry
        if telemetry is not None:
            self.motion_pub.publish(self._build_motion_state(now, telemetry))

        self.mission_pub.publish(self._build_mission_state(now, telemetry))
        self.safety_pub.publish(self._build_safety_state(now, telemetry))

        autonomy_odom = self._build_autonomy_odom(now)
        if autonomy_odom is not None:
            self.odom_pub.publish(autonomy_odom)
        elif (
            self.fallback_to_leader_odom
            and self.latest_odom is not None
            and self._odom_fresh(now)
        ):
            self.odom_pub.publish(self.latest_odom)

        heartbeat_age_sec = (now - self.last_heartbeat_time).nanoseconds * 1e-9
        if heartbeat_age_sec >= self.heartbeat_period_sec:
            self.heartbeat_pub.publish(self._build_heartbeat(now, telemetry))
            self.last_heartbeat_time = now

    def _build_motion_state(
        self,
        now: rclpy.time.Time,
        telemetry: LeaderDriveTelemetry,
    ) -> LeaderMotionState:
        msg = LeaderMotionState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'leader/base_link'
        msg.seq = int(telemetry.seq)
        msg.boot_id = int(telemetry.boot_id)
        msg.drive_mode = int(telemetry.drive_mode)
        msg.source = int(telemetry.source)
        msg.motion_valid = self._telemetry_fresh(now) and not telemetry.stop_required
        msg.stop_required = bool(telemetry.stop_required)
        msg.speed_mps = self._resolve_speed_mps(now)
        msg.steering_angle_rad = float(telemetry.steering_angle_rad)
        msg.curvature_1pm = math.tan(msg.steering_angle_rad) / self.wheel_base_m
        msg.yaw_rate_radps = msg.speed_mps * msg.curvature_1pm
        msg.throttle_norm = float(telemetry.throttle_norm)
        msg.source_time_ms = int(telemetry.arduino_time_ms)

        if self.prev_speed_mps is None or self.prev_motion_time is None:
            msg.acceleration_mps2 = 0.0
        else:
            dt = (now - self.prev_motion_time).nanoseconds * 1e-9
            msg.acceleration_mps2 = (
                (msg.speed_mps - self.prev_speed_mps) / dt if dt > 1e-6 else 0.0
            )
        self.prev_speed_mps = float(msg.speed_mps)
        self.prev_motion_time = now
        return msg

    def _build_mission_state(
        self,
        now: rclpy.time.Time,
        telemetry: Optional[LeaderDriveTelemetry],
    ) -> LeaderMissionState:
        self.mission_seq = (self.mission_seq + 1) & 0xFFFFFFFF
        msg = LeaderMissionState()
        msg.header.stamp = now.to_msg()
        msg.seq = self.mission_seq
        msg.boot_id = int(telemetry.boot_id) if telemetry is not None else 0
        msg.mission_state = self.latest_mission_state or self._mode_to_mission(telemetry)
        msg.active_algorithm = self.latest_active_algorithm
        msg.mission_valid = telemetry is not None and self._telemetry_fresh(now)
        return msg

    def _build_safety_state(
        self,
        now: rclpy.time.Time,
        telemetry: Optional[LeaderDriveTelemetry],
    ) -> LeaderSafetyState:
        self.safety_seq = (self.safety_seq + 1) & 0xFFFFFFFF
        stop_required = self.latest_safety_active
        if telemetry is None or not self._telemetry_fresh(now):
            stop_required = True
            reason = 'TELEMETRY_TIMEOUT'
        elif telemetry.stop_required:
            stop_required = True
            reason = 'LEADER_STOP_REQUIRED'
        else:
            reason = self.latest_safety_status or 'NONE'

        msg = LeaderSafetyState()
        msg.header.stamp = now.to_msg()
        msg.seq = self.safety_seq
        msg.boot_id = int(telemetry.boot_id) if telemetry is not None else 0
        msg.stop_required = bool(stop_required)
        msg.safety_active = bool(stop_required)
        msg.reason = reason
        return msg

    def _build_heartbeat(
        self,
        now: rclpy.time.Time,
        telemetry: Optional[LeaderDriveTelemetry],
    ) -> Heartbeat:
        self.heartbeat_seq = (self.heartbeat_seq + 1) & 0xFFFFFFFF
        msg = Heartbeat()
        msg.header.stamp = now.to_msg()
        msg.role = Heartbeat.ROLE_LEADER
        msg.seq = self.heartbeat_seq
        msg.boot_id = int(telemetry.boot_id) if telemetry is not None else 0
        msg.last_motion_seq = int(telemetry.seq) if telemetry is not None else 0
        msg.system_ok = telemetry is not None and self._telemetry_fresh(now)
        msg.status = 'OK' if msg.system_ok else 'TELEMETRY_TIMEOUT'
        return msg

    def _build_autonomy_odom(self, now: rclpy.time.Time) -> Optional[Odometry]:
        if (
            self.latest_vehicle_ref_utm is None
            or self.latest_heading_rad is None
            or not self._vehicle_ref_utm_fresh(now)
            or not self._heading_fresh(now)
        ):
            return None

        point = self.latest_vehicle_ref_utm.point
        heading_rad = float(self.latest_heading_rad)
        if not self._all_finite(point.x, point.y, heading_rad):
            return None

        msg = Odometry()
        msg.header.stamp = self.latest_vehicle_ref_utm.header.stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.leader_base_frame
        msg.pose.pose.position.x = float(point.x) - self.origin_easting_m
        msg.pose.pose.position.y = float(point.y) - self.origin_northing_m
        msg.pose.pose.position.z = self.autonomy_fixed_z_m
        msg.pose.pose.orientation = self._quaternion_from_yaw(heading_rad)
        self._fill_autonomy_pose_covariance(msg)

        if self.latest_fix_velocity is not None and self._fix_velocity_fresh(now):
            msg.twist = self.latest_fix_velocity.twist
        return msg

    def _fill_autonomy_pose_covariance(self, msg: Odometry) -> None:
        msg.pose.covariance[0] = self.autonomy_position_variance_m2
        msg.pose.covariance[7] = self.autonomy_position_variance_m2
        msg.pose.covariance[14] = self.autonomy_z_variance_m2
        msg.pose.covariance[35] = self.autonomy_yaw_variance_rad2

    def _resolve_speed_mps(self, now: rclpy.time.Time) -> float:
        if (
            self.prefer_encoder_twist_speed
            and self.latest_encoder_twist is not None
            and self.latest_encoder_twist_time is not None
            and self._encoder_twist_fresh(now)
        ):
            speed_mps = float(self.latest_encoder_twist.twist.twist.linear.x)
            if math.isfinite(speed_mps):
                return speed_mps
        if self.latest_telemetry is None or not self._telemetry_fresh(now):
            return 0.0
        speed_mps = float(self.latest_telemetry.speed_estimate_mps)
        if not math.isfinite(speed_mps):
            speed_mps = 0.0
        if (
            self.use_throttle_signed_speed_fallback
            and self.latest_telemetry.drive_mode == LeaderDriveTelemetry.MODE_MANUAL_RC
        ):
            return signed_speed_from_telemetry(
                speed_mps,
                float(self.latest_telemetry.throttle_norm),
                int(self.latest_telemetry.rc_throttle_us),
                speed_deadband_mps=self.telemetry_speed_deadband_mps,
                throttle_deadband=self.throttle_speed_deadband,
                throttle_speed_gain_mps=self.throttle_speed_fallback_gain_mps,
                rc_throttle_neutral_us=self.rc_throttle_neutral_us,
                rc_throttle_deadband_us=self.rc_throttle_deadband_us,
                rc_throttle_full_scale_us=self.rc_throttle_full_scale_us,
            )
        return speed_mps

    def _telemetry_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_telemetry_time is None:
            return False
        age_sec = (now - self.latest_telemetry_time).nanoseconds * 1e-9
        return age_sec <= self.telemetry_timeout_sec

    def _encoder_twist_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_encoder_twist_time is None:
            return False
        age_sec = (now - self.latest_encoder_twist_time).nanoseconds * 1e-9
        return age_sec <= self.telemetry_timeout_sec

    def _odom_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_odom_time is None:
            return False
        age_sec = (now - self.latest_odom_time).nanoseconds * 1e-9
        return age_sec <= self.leader_odom_timeout_sec

    def _vehicle_ref_utm_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_vehicle_ref_utm_time is None:
            return False
        age_sec = (now - self.latest_vehicle_ref_utm_time).nanoseconds * 1e-9
        return age_sec <= self.autonomy_pose_timeout_sec

    def _heading_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_heading_time is None or not self.latest_heading_valid:
            return False
        age_sec = (now - self.latest_heading_time).nanoseconds * 1e-9
        return age_sec <= self.heading_timeout_sec

    def _fix_velocity_fresh(self, now: rclpy.time.Time) -> bool:
        if self.latest_fix_velocity_time is None:
            return False
        age_sec = (now - self.latest_fix_velocity_time).nanoseconds * 1e-9
        return age_sec <= self.fix_velocity_timeout_sec

    @staticmethod
    def _quaternion_from_yaw(yaw_rad: float):
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        half_yaw = 0.5 * yaw_rad
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        return quat

    @staticmethod
    def _all_finite(*values: float) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def _mode_to_mission(telemetry: Optional[LeaderDriveTelemetry]) -> str:
        if telemetry is None:
            return 'UNKNOWN'
        if telemetry.drive_mode == LeaderDriveTelemetry.MODE_MANUAL_RC:
            return 'MANUAL_SYNC'
        if telemetry.drive_mode == LeaderDriveTelemetry.MODE_AUTONOMOUS:
            return 'AUTONOMOUS'
        if telemetry.drive_mode == LeaderDriveTelemetry.MODE_ESTOP:
            return 'ESTOP'
        return 'UNKNOWN'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderV2vAdapterNode()
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
