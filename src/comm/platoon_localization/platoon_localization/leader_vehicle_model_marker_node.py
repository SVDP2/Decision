from __future__ import annotations

import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rclpy
from rclpy.node import Node


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


def _quaternion_from_roll(roll_rad: float) -> Quaternion:
    quat = Quaternion()
    quat.x = math.sin(0.5 * roll_rad)
    quat.y = 0.0
    quat.z = 0.0
    quat.w = math.cos(0.5 * roll_rad)
    return quat


def _identity_quaternion() -> Quaternion:
    quat = Quaternion()
    quat.w = 1.0
    return quat


class LeaderVehicleModelMarkerNode(Node):
    """Publish a simple vehicle geometry model for RViz checks."""

    def __init__(self) -> None:
        super().__init__("leader_vehicle_model_marker_node")

        self.declare_parameter("base_frame", "leader/base_link")
        self.declare_parameter("marker_topic", "/leader/vehicle_model/markers")
        self.declare_parameter("marker_namespace", "leader_vehicle_model")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("show_rear_reference", True)

        self.declare_parameter("wheelbase_m", 0.720)
        self.declare_parameter("track_width_m", 0.700)
        self.declare_parameter("wheel_diameter_m", 0.265)
        self.declare_parameter("wheel_width_m", 0.110)
        self.declare_parameter("gps_x_m", 0.270)
        self.declare_parameter("gps_y_m", 0.0)
        self.declare_parameter("gps_z_m", 1.4675)
        self.declare_parameter("leader_rear_x_m", -0.275)
        self.declare_parameter("leader_rear_y_m", 0.0)
        self.declare_parameter("leader_rear_z_m", 0.0525)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.marker_namespace = str(self.get_parameter("marker_namespace").value)
        self.wheelbase_m = max(0.0, float(self.get_parameter("wheelbase_m").value))
        self.track_width_m = max(0.0, float(self.get_parameter("track_width_m").value))
        self.wheel_diameter_m = max(0.0, float(self.get_parameter("wheel_diameter_m").value))
        self.wheel_width_m = max(0.0, float(self.get_parameter("wheel_width_m").value))
        self.gps_x_m = float(self.get_parameter("gps_x_m").value)
        self.gps_y_m = float(self.get_parameter("gps_y_m").value)
        self.gps_z_m = float(self.get_parameter("gps_z_m").value)
        self.leader_rear_x_m = float(self.get_parameter("leader_rear_x_m").value)
        self.leader_rear_y_m = float(self.get_parameter("leader_rear_y_m").value)
        self.leader_rear_z_m = float(self.get_parameter("leader_rear_z_m").value)
        self.show_rear_reference = bool(self.get_parameter("show_rear_reference").value)

        publish_rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_markers)

        self.get_logger().info(
            "vehicle marker model: "
            f"frame={self.base_frame}, wheelbase={self.wheelbase_m:.3f}m, "
            f"track={self.track_width_m:.3f}m, wheel_diameter={self.wheel_diameter_m:.3f}m"
        )

    def _wheel_centers(self) -> list[tuple[str, float, float, float]]:
        half_track = 0.5 * self.track_width_m
        return [
            ("rear_left", 0.0, half_track, 0.0),
            ("rear_right", 0.0, -half_track, 0.0),
            ("front_left", self.wheelbase_m, half_track, 0.0),
            ("front_right", self.wheelbase_m, -half_track, 0.0),
        ]

    def _base_marker(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        # RViz resolves stamp 0 against the latest available TF, which is more
        # robust for static/debug geometry than requiring an exact odom timestamp.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.header.frame_id = self.base_frame
        marker.ns = self.marker_namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation = _identity_quaternion()
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _wheel_marker(
        self,
        marker_id: int,
        center: tuple[str, float, float, float],
    ) -> Marker:
        name, x, y, z = center
        marker = self._base_marker(marker_id, Marker.CYLINDER)
        marker.ns = f"{self.marker_namespace}/wheels"
        marker.pose.position = _point(x, y, z)
        marker.pose.orientation = _quaternion_from_roll(-0.5 * math.pi)
        marker.scale.x = self.wheel_diameter_m
        marker.scale.y = self.wheel_diameter_m
        marker.scale.z = self.wheel_width_m
        marker.color = (
            _color(0.12, 0.12, 0.12, 0.88)
            if "left" in name
            else _color(0.20, 0.20, 0.20, 0.88)
        )
        return marker

    def _center_marker(
        self,
        marker_id: int,
        center: tuple[str, float, float, float],
    ) -> Marker:
        _, x, y, z = center
        marker = self._base_marker(marker_id, Marker.SPHERE)
        marker.ns = f"{self.marker_namespace}/wheel_centers"
        marker.pose.position = _point(x, y, z)
        diameter = max(0.035, 0.18 * self.wheel_diameter_m)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = _color(0.05, 0.65, 1.0, 1.0)
        return marker

    def _line_marker(self, marker_id: int) -> Marker:
        centers = {name: _point(x, y, z) for name, x, y, z in self._wheel_centers()}
        marker = self._base_marker(marker_id, Marker.LINE_LIST)
        marker.ns = f"{self.marker_namespace}/geometry_lines"
        marker.scale.x = 0.018
        marker.color = _color(0.92, 0.78, 0.18, 1.0)
        marker.points = [
            centers["rear_left"],
            centers["rear_right"],
            centers["front_left"],
            centers["front_right"],
            centers["rear_left"],
            centers["front_left"],
            centers["rear_right"],
            centers["front_right"],
            _point(0.0, 0.0, 0.0),
            _point(self.wheelbase_m, 0.0, 0.0),
        ]
        return marker

    def _base_axis_markers(self, start_id: int) -> list[Marker]:
        axis_len = max(0.25, 0.35 * self.wheelbase_m)
        x_axis = self._base_marker(start_id, Marker.ARROW)
        x_axis.ns = f"{self.marker_namespace}/base_axes"
        x_axis.points = [_point(0.0, 0.0, 0.0), _point(axis_len, 0.0, 0.0)]
        x_axis.scale.x = 0.025
        x_axis.scale.y = 0.055
        x_axis.scale.z = 0.055
        x_axis.color = _color(0.95, 0.12, 0.12, 1.0)

        y_axis = self._base_marker(start_id + 1, Marker.ARROW)
        y_axis.ns = f"{self.marker_namespace}/base_axes"
        y_axis.points = [_point(0.0, 0.0, 0.0), _point(0.0, axis_len, 0.0)]
        y_axis.scale.x = 0.025
        y_axis.scale.y = 0.055
        y_axis.scale.z = 0.055
        y_axis.color = _color(0.12, 0.95, 0.12, 1.0)

        z_axis = self._base_marker(start_id + 2, Marker.ARROW)
        z_axis.ns = f"{self.marker_namespace}/base_axes"
        z_axis.points = [_point(0.0, 0.0, 0.0), _point(0.0, 0.0, axis_len)]
        z_axis.scale.x = 0.025
        z_axis.scale.y = 0.055
        z_axis.scale.z = 0.055
        z_axis.color = _color(0.12, 0.32, 0.95, 1.0)

        return [x_axis, y_axis, z_axis]

    def _gps_markers(self, start_id: int) -> list[Marker]:
        mast = self._base_marker(start_id, Marker.LINE_LIST)
        mast.ns = f"{self.marker_namespace}/gps"
        mast.scale.x = 0.025
        mast.color = _color(0.70, 0.35, 1.0, 1.0)
        mast.points = [
            _point(self.gps_x_m, self.gps_y_m, 0.0),
            _point(self.gps_x_m, self.gps_y_m, self.gps_z_m),
        ]

        antenna = self._base_marker(start_id + 1, Marker.SPHERE)
        antenna.ns = f"{self.marker_namespace}/gps"
        antenna.pose.position = _point(self.gps_x_m, self.gps_y_m, self.gps_z_m)
        antenna.scale.x = 0.11
        antenna.scale.y = 0.11
        antenna.scale.z = 0.11
        antenna.color = _color(0.70, 0.35, 1.0, 1.0)

        return [mast, antenna]

    def _leader_rear_markers(self, start_id: int) -> list[Marker]:
        center = self._base_marker(start_id, Marker.SPHERE)
        center.ns = f"{self.marker_namespace}/leader_rear"
        center.pose.position = _point(
            self.leader_rear_x_m,
            self.leader_rear_y_m,
            self.leader_rear_z_m,
        )
        center.scale.x = 0.08
        center.scale.y = 0.08
        center.scale.z = 0.08
        center.color = _color(1.0, 0.48, 0.12, 1.0)

        stem = self._base_marker(start_id + 1, Marker.LINE_LIST)
        stem.ns = f"{self.marker_namespace}/leader_rear"
        stem.scale.x = 0.018
        stem.color = _color(1.0, 0.48, 0.12, 1.0)
        stem.points = [
            _point(self.leader_rear_x_m, self.leader_rear_y_m, 0.0),
            _point(self.leader_rear_x_m, self.leader_rear_y_m, self.leader_rear_z_m),
        ]

        return [center, stem]

    def _publish_markers(self) -> None:
        array = MarkerArray()
        centers = self._wheel_centers()
        marker_id = 0

        for center in centers:
            array.markers.append(self._wheel_marker(marker_id, center))
            marker_id += 1
        for center in centers:
            array.markers.append(self._center_marker(marker_id, center))
            marker_id += 1

        array.markers.append(self._line_marker(marker_id))
        marker_id += 1
        array.markers.extend(self._base_axis_markers(marker_id))
        marker_id += 3
        array.markers.extend(self._gps_markers(marker_id))
        marker_id += 2
        if self.show_rear_reference:
            array.markers.extend(self._leader_rear_markers(marker_id))

        self.marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderVehicleModelMarkerNode()
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
