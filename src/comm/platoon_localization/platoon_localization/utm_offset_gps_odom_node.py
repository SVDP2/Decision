from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


WGS84_A = 6378137.0
WGS84_E = 0.081819190842622


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _to_utm(latitude_deg: float, longitude_deg: float) -> tuple[float, float, int]:
    if latitude_deg < -80.0 or latitude_deg > 84.0:
        raise ValueError("latitude outside UTM range")

    zone = int((longitude_deg + 180.0) / 6.0) + 1
    lon_rad = math.radians(longitude_deg)
    lat_rad = math.radians(latitude_deg)
    zone_lon_rad = math.radians(zone * 6.0 - 183.0)

    e2 = WGS84_E * WGS84_E
    e4 = e2 * e2
    e6 = e4 * e2
    n_radius = WGS84_A / math.sqrt(1.0 - e2 * math.sin(lat_rad) ** 2)
    tan_sq = math.tan(lat_rad) ** 2
    c_term = e2 / (1.0 - e2) * math.cos(lat_rad) ** 2
    a_term = (lon_rad - zone_lon_rad) * math.cos(lat_rad)
    meridian = WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat_rad
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
        * math.sin(2.0 * lat_rad)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * math.sin(4.0 * lat_rad)
        - (35.0 * e6 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    easting = (
        0.9996
        * n_radius
        * (
            a_term
            + (1.0 - tan_sq + c_term) * a_term**3 / 6.0
            + (
                5.0
                - 18.0 * tan_sq
                + tan_sq * tan_sq
                + 72.0 * c_term
                - 58.0 * e2
            )
            * a_term**5
            / 120.0
        )
        + 500000.0
    )
    northing = 0.9996 * (
        meridian
        + n_radius
        * math.tan(lat_rad)
        * (
            a_term * a_term / 2.0
            + (5.0 - tan_sq + 9.0 * c_term + 4.0 * c_term * c_term)
            * a_term**4
            / 24.0
            + (
                61.0
                - 58.0 * tan_sq
                + tan_sq * tan_sq
                + 600.0 * c_term
                - 330.0 * e2
            )
            * a_term**6
            / 720.0
        )
    )
    if latitude_deg < 0.0:
        northing += 10000000.0
    return easting, northing, zone


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class UtmOffsetGpsOdomNode(Node):
    """Convert WGS84 GPS fixes to odometry in a shared local UTM-offset map frame."""

    def __init__(self) -> None:
        super().__init__("utm_offset_gps_odom_node")

        self.declare_parameter("fix_topic", "/f9p/fix")
        self.declare_parameter("fix_velocity_topic", "/f9p/fix_velocity")
        self.declare_parameter("heading_topic", "")
        self.declare_parameter("heading_valid_topic", "")
        self.declare_parameter("gps_odom_topic", "")
        self.declare_parameter("base_odom_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_frame", "gps")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("utm_zone", 52)
        self.declare_parameter("origin_easting_m", 0.0)
        self.declare_parameter("origin_northing_m", 0.0)
        self.declare_parameter("origin_altitude_m", 0.0)
        self.declare_parameter("origin_source_csv", "")
        self.declare_parameter("gps_to_base_x_m", 0.0)
        self.declare_parameter("gps_to_base_y_m", 0.0)
        self.declare_parameter("gps_to_base_z_m", 0.0)
        self.declare_parameter("use_fixed_base_z", False)
        self.declare_parameter("fixed_base_z_m", 0.0)
        self.declare_parameter("min_position_variance_m2", 1.0e-4)
        self.declare_parameter("min_heading_speed_mps", 0.5)
        self.declare_parameter("initial_heading_rad", 0.0)
        self.declare_parameter("initial_heading_valid", False)
        self.declare_parameter("publish_base_without_heading", True)
        self.declare_parameter("heading_yaw_variance_rad2", 0.0100)
        self.declare_parameter("unknown_heading_yaw_variance_rad2", math.pi * math.pi)
        self.declare_parameter("min_fix_status", 2)
        self.declare_parameter("publish_base_tf", False)

        self.fix_topic = str(self.get_parameter("fix_topic").value)
        velocity_topic = str(self.get_parameter("fix_velocity_topic").value)
        heading_topic = str(self.get_parameter("heading_topic").value)
        heading_valid_topic = str(self.get_parameter("heading_valid_topic").value)
        self.gps_odom_topic = str(self.get_parameter("gps_odom_topic").value)
        self.base_odom_topic = str(self.get_parameter("base_odom_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_frame = str(self.get_parameter("gps_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.utm_zone = int(self.get_parameter("utm_zone").value)
        self.origin_easting_m = float(self.get_parameter("origin_easting_m").value)
        self.origin_northing_m = float(self.get_parameter("origin_northing_m").value)
        self.origin_altitude_m = float(self.get_parameter("origin_altitude_m").value)
        self.gps_to_base_x_m = float(self.get_parameter("gps_to_base_x_m").value)
        self.gps_to_base_y_m = float(self.get_parameter("gps_to_base_y_m").value)
        self.gps_to_base_z_m = float(self.get_parameter("gps_to_base_z_m").value)
        self.use_fixed_base_z = bool(self.get_parameter("use_fixed_base_z").value)
        self.fixed_base_z_m = float(self.get_parameter("fixed_base_z_m").value)
        self.min_position_variance_m2 = max(
            1.0e-12,
            float(self.get_parameter("min_position_variance_m2").value),
        )
        self.min_heading_speed_mps = max(
            0.0,
            float(self.get_parameter("min_heading_speed_mps").value),
        )
        self.min_fix_status = int(self.get_parameter("min_fix_status").value)
        self.publish_base_tf = bool(self.get_parameter("publish_base_tf").value)
        self.publish_base_without_heading = bool(
            self.get_parameter("publish_base_without_heading").value
        )
        self.heading_yaw_variance_rad2 = max(
            1.0e-9,
            float(self.get_parameter("heading_yaw_variance_rad2").value),
        )
        self.unknown_heading_yaw_variance_rad2 = max(
            self.heading_yaw_variance_rad2,
            float(self.get_parameter("unknown_heading_yaw_variance_rad2").value),
        )

        self.latest_velocity: Optional[TwistWithCovarianceStamped] = None
        self.latest_heading: Optional[float] = None
        self.fallback_heading = float(self.get_parameter("initial_heading_rad").value)
        self.heading_valid = bool(self.get_parameter("initial_heading_valid").value)
        if self.heading_valid:
            self.latest_heading = self.fallback_heading

        self.create_subscription(
            NavSatFix,
            self.fix_topic,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        if velocity_topic:
            self.create_subscription(
                TwistWithCovarianceStamped,
                velocity_topic,
                self._velocity_callback,
                qos_profile_sensor_data,
            )
        if heading_topic:
            self.create_subscription(Float64, heading_topic, self._heading_callback, 10)
        if heading_valid_topic:
            self.create_subscription(
                Bool,
                heading_valid_topic,
                self._heading_valid_callback,
                10,
            )

        self.gps_pub = (
            self.create_publisher(Odometry, self.gps_odom_topic, 10)
            if self.gps_odom_topic
            else None
        )
        self.base_pub = (
            self.create_publisher(Odometry, self.base_odom_topic, 10)
            if self.base_odom_topic
            else None
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_base_tf else None

        self.get_logger().info(
            "utm_offset_gps_odom_node started: "
            f"fix={self.fix_topic}, map={self.map_frame}, zone={self.utm_zone}, "
            f"gps_frame={self.gps_frame}, base_frame={self.base_frame}, "
            f"fixed_base_z={self.fixed_base_z_m if self.use_fixed_base_z else 'disabled'}, "
            f"min_fix_status={self.min_fix_status}"
        )

    def _velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_velocity = msg
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        if _finite(vx, vy) and math.hypot(vx, vy) >= self.min_heading_speed_mps:
            self.latest_heading = math.atan2(vy, vx)
            self.fallback_heading = self.latest_heading
            self.heading_valid = True

    def _heading_callback(self, msg: Float64) -> None:
        if math.isfinite(float(msg.data)):
            self.latest_heading = float(msg.data)
            self.fallback_heading = self.latest_heading
            self.heading_valid = True

    def _heading_valid_callback(self, msg: Bool) -> None:
        self.heading_valid = bool(msg.data)

    def _fix_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < self.min_fix_status:
            self.get_logger().warning(
                "dropping GPS fix: "
                f"status={msg.status.status} below min_fix_status={self.min_fix_status}",
                throttle_duration_sec=1.0,
            )
            return

        if msg.status.status < 0:
            self.get_logger().warning("dropping GPS fix: status reports no fix")
            return
        if not _finite(msg.latitude, msg.longitude, msg.altitude):
            self.get_logger().warning("dropping GPS fix: non-finite LLA")
            return

        try:
            easting, northing, zone = _to_utm(msg.latitude, msg.longitude)
        except ValueError as exc:
            self.get_logger().warning(f"dropping GPS fix: {exc}")
            return
        if zone != self.utm_zone:
            self.get_logger().error(
                f"dropping GPS fix: UTM zone mismatch fix={zone} expected={self.utm_zone}",
                throttle_duration_sec=1.0,
            )
            return

        x_map = easting - self.origin_easting_m
        y_map = northing - self.origin_northing_m
        z_map = float(msg.altitude) - self.origin_altitude_m
        if not _finite(x_map, y_map, z_map):
            self.get_logger().warning("dropping GPS fix: non-finite map position")
            return

        if self.gps_pub:
            self.gps_pub.publish(
                self._make_odom(
                    msg,
                    self.gps_frame,
                    x_map,
                    y_map,
                    z_map,
                    0.0,
                    yaw_valid=False,
                )
            )

        if self.base_pub:
            heading_valid = self.latest_heading is not None and self.heading_valid
            if not heading_valid and not self.publish_base_without_heading:
                self.get_logger().warning(
                    "base odom waiting for valid heading",
                    throttle_duration_sec=1.0,
                )
                return
            base_heading = self.latest_heading if heading_valid else self.fallback_heading
            if not heading_valid:
                self.get_logger().warning(
                    "base odom publishing GPS position with unknown heading covariance",
                    throttle_duration_sec=1.0,
                )
            base_x, base_y, base_z = self._base_position_from_gps(
                x_map,
                y_map,
                z_map,
                base_heading,
            )
            base_odom = self._make_odom(
                msg,
                self.base_frame,
                base_x,
                base_y,
                base_z,
                base_heading,
                yaw_valid=heading_valid,
            )
            self.base_pub.publish(base_odom)
            if self.tf_broadcaster and heading_valid:
                self.tf_broadcaster.sendTransform(self._make_base_tf(base_odom))

    def _base_position_from_gps(
        self,
        gps_x: float,
        gps_y: float,
        gps_z: float,
        yaw: float,
    ) -> tuple[float, float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        offset_x = self.gps_to_base_x_m * cos_yaw - self.gps_to_base_y_m * sin_yaw
        offset_y = self.gps_to_base_x_m * sin_yaw + self.gps_to_base_y_m * cos_yaw
        base_z = self.fixed_base_z_m if self.use_fixed_base_z else gps_z - self.gps_to_base_z_m
        return gps_x - offset_x, gps_y - offset_y, base_z

    def _make_odom(
        self,
        fix: NavSatFix,
        child_frame: str,
        x_map: float,
        y_map: float,
        z_map: float,
        yaw: float,
        *,
        yaw_valid: bool,
    ) -> Odometry:
        odom = Odometry()
        odom.header.stamp = fix.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = float(x_map)
        odom.pose.pose.position.y = float(y_map)
        odom.pose.pose.position.z = float(z_map)
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self._fill_pose_covariance(odom, fix, yaw_valid)
        if self.latest_velocity is not None:
            odom.twist = self.latest_velocity.twist
        return odom

    def _fill_pose_covariance(self, odom: Odometry, fix: NavSatFix, yaw_valid: bool) -> None:
        for index in range(36):
            odom.pose.covariance[index] = 0.0
        source = list(fix.position_covariance)
        xx = self._safe_variance(source[0] if len(source) > 0 else 0.0)
        yy = self._safe_variance(source[4] if len(source) > 4 else 0.0)
        zz = self._safe_variance(source[8] if len(source) > 8 else 0.0)
        odom.pose.covariance[0] = xx
        odom.pose.covariance[7] = yy
        odom.pose.covariance[14] = zz
        odom.pose.covariance[35] = (
            self.heading_yaw_variance_rad2
            if yaw_valid
            else self.unknown_heading_yaw_variance_rad2
        )

    def _safe_variance(self, value: float) -> float:
        if not math.isfinite(float(value)) or value <= 0.0:
            return self.min_position_variance_m2
        return max(float(value), self.min_position_variance_m2)

    def _make_base_tf(self, odom: Odometry) -> TransformStamped:
        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        return tf_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UtmOffsetGpsOdomNode()
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
