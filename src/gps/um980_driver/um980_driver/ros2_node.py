"""ROS2 node wrapper for the UM980 serial driver."""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from um980_msgs.msg import Status, StatusVerbose

from .ntrip_client import EmbeddedNtripClient
from .ntrip_config import load_private_yaml
from .parsers import BestNav, RtcmStatus, parse_line
from .profiles import get_profile
from .protocol import RTCM3StreamParser
from .serial_client import UM980SerialClient, select_default_port
from .state import GNSSState


class UM980DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("um980_driver")
        self._declare_parameters()

        port = self.get_parameter("port").value or select_default_port()
        if not port:
            raise RuntimeError("no UM980 serial device found")
        requested_baud = int(self.get_parameter("baud").value)
        self.serial_port = str(port)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.covariance_timeout_s = float(self.get_parameter("covariance_timeout_s").value)
        self.covariance_max_std_m = float(self.get_parameter("covariance_max_std_m").value)
        self.unknown_velocity_variance = float(self.get_parameter("unknown_velocity_variance").value)
        self.ntrip_enabled = bool(self.get_parameter("ntrip_enabled").value)
        self.ntrip_client: Optional[EmbeddedNtripClient] = None
        self.base_station_name = str(self.get_parameter("base_station_name").value)
        self.ntrip_host = ""
        self.ntrip_mountpoint = ""
        self.configured_profile = ""

        self.state = GNSSState()
        self.rtcm_parser = RTCM3StreamParser()
        self.client = UM980SerialClient(self.serial_port, requested_baud, timeout=0.2)
        self.client.open()
        self.serial_baud = self.client.baud
        self.get_logger().info(f"Opened UM980 serial port {port} at {self.serial_baud}")

        if bool(self.get_parameter("configure_on_start").value):
            mode = str(self.get_parameter("mode").value)
            save_config = bool(self.get_parameter("save_config").value)
            profile = get_profile(mode)
            self.get_logger().warn(f"Applying UM980 profile on start: {profile.description}")
            self.configured_profile = profile.name
            for result in self.client.apply_commands(
                profile.commands_with_save(save_config),
                optional_commands=profile.optional_commands,
                fallbacks=dict(profile.fallbacks),
                reconnect_after_commands=("CONFIG SIGNALGROUP 8",) if profile.may_reboot else (),
            ):
                if result.ok:
                    self.get_logger().info(f"UM980 command OK: {result.command}")
                else:
                    self.get_logger().error(f"UM980 command failed: {result.command}: {result.error}")

        self.fix_pub = self.create_publisher(NavSatFix, "/f9p/fix", 10)
        self.fix_velocity_pub = self.create_publisher(TwistWithCovarianceStamped, "/f9p/fix_velocity", 10)
        self.status_pub = self.create_publisher(Status, "/f9p/status", 10)
        self.status_verbose_pub = self.create_publisher(StatusVerbose, "/f9p/status_verbose", 10)

        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()
        self._start_embedded_ntrip_if_enabled()
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._pub_timer = self.create_timer(self.publish_period_s, self._publish_latest)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._status_verbose_timer = self.create_timer(
            float(self.get_parameter("status_verbose_period_s").value),
            self._publish_status_verbose,
        )

    def destroy_node(self) -> bool:
        self._running = False
        if self.ntrip_client:
            self.ntrip_client.stop()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self.client.close()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 0)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("configure_on_start", False)
        self.declare_parameter("save_config", False)
        self.declare_parameter("mode", "signalgroup1_20hz")
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("covariance_timeout_s", 2.0)
        self.declare_parameter("covariance_max_std_m", 100.0)
        self.declare_parameter("unknown_velocity_variance", 100.0)
        self.declare_parameter("ntrip_enabled", True)
        self.declare_parameter("ntrip_private_yaml", "")
        self.declare_parameter("ntrip_gga_interval_s", 5.0)
        self.declare_parameter("status_verbose_period_s", 1.0)
        self.declare_parameter("base_station_name", "")

    def _start_embedded_ntrip_if_enabled(self) -> None:
        if not self.ntrip_enabled:
            return
        private_yaml = str(self.get_parameter("ntrip_private_yaml").value)
        if not private_yaml:
            self._mark_ntrip_unavailable("ntrip_private_yaml is empty")
            return
        if not Path(private_yaml).exists():
            self._mark_ntrip_unavailable(f"NTRIP private YAML not found: {private_yaml}")
            return
        try:
            config = load_private_yaml(private_yaml)
        except Exception as exc:
            self._mark_ntrip_unavailable(f"NTRIP private YAML invalid: {exc}")
            return
        self.ntrip_host = config.host
        self.ntrip_mountpoint = config.mountpoint
        interval_s = float(self.get_parameter("ntrip_gga_interval_s").value)
        self.ntrip_client = EmbeddedNtripClient(
            config=config,
            get_gga=lambda: self.state.latest_raw_gga,
            on_rtcm=self._on_embedded_rtcm,
            gga_interval_s=interval_s,
        )
        self.ntrip_client.start()
        self.get_logger().info(f"Started embedded NTRIP client for {config.host}:{config.port}/{config.mountpoint}")

    def _mark_ntrip_unavailable(self, reason: str) -> None:
        self.state.update_ntrip(connected=False, last_error=reason, gga_age_s=None)
        self.get_logger().warn(f"NTRIP unavailable; continuing GNSS-only: {reason}")

    def _read_loop(self) -> None:
        while self._running and self.client.is_open:
            try:
                line = self.client.read_line()
                if not line:
                    continue
                self.state.observe_raw_line(line)
                try:
                    parsed = parse_line(line)
                except ValueError as exc:
                    self.state.observe_parse_error(str(exc), line)
                    continue
            except Exception as exc:
                self.state.last_error = str(exc)
                continue
            if parsed is None:
                self.state.observe_unknown_line(line)
                continue
            self.state.update(parsed)

    def _on_embedded_rtcm(self, data: bytes) -> None:
        try:
            for message in self.rtcm_parser.parse(data):
                count = self.state.rtcm_message_counts.get(message.message_id, 0) + 1
                parsed = RtcmStatus(
                    raw=f"RTCM3:{message.raw.hex()}",
                    header_fields=["RTCM3"],
                    message_id=message.message_id,
                    message_count=count,
                    base_station_id=message.station_id,
                    satellite_count=None,
                    l1_observation_count=None,
                    l2_observation_count=None,
                    l3_observation_count=None,
                    l4_observation_count=None,
                    l5_observation_count=None,
                    l6_observation_count=None,
                )
                self.state.observe_rtcm_message(parsed)
            self.client.write_rtcm(data)
            self.state.observe_rtcm(len(data))
        except Exception as exc:
            self.state.last_error = f"RTCM write failed: {exc}"
            self.get_logger().error(self.state.last_error)

    def _publish_latest(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        if self.state.latest_gga or self.state.latest_bestnav:
            self.fix_pub.publish(self._fix_msg())
        if self.state.latest_bestnav:
            velocity = self._fix_velocity_msg(self.state.latest_bestnav)
            if velocity:
                self.fix_velocity_pub.publish(velocity)

    def _publish_status(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        if self.ntrip_client:
            status_snapshot = self.ntrip_client.status
            self.state.update_ntrip(
                connected=status_snapshot.connected,
                last_error=status_snapshot.last_error,
                gga_age_s=status_snapshot.gga_age_s(),
            )
        summary = self.state.summary(
            covariance_timeout_s=self.covariance_timeout_s,
            covariance_max_std_m=self.covariance_max_std_m,
        )
        self.status_pub.publish(self._status_msg(summary))

    def _publish_status_verbose(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        self.status_verbose_pub.publish(self._status_verbose_msg())

    def _fix_msg(self) -> NavSatFix:
        msg = NavSatFix()
        msg.header = self._header()
        position = self.state.position_solution()
        covariance = self.state.covariance_solution(
            timeout_s=self.covariance_timeout_s,
            max_std_m=self.covariance_max_std_m,
        )
        msg.status.status = self._navsat_status()
        msg.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        msg.latitude = _nan_if_none(position["lat"])
        msg.longitude = _nan_if_none(position["lon"])
        msg.altitude = _nan_if_none(position["altitude_ellipsoid_m"])
        east_std = covariance["east_std_m"]
        north_std = covariance["north_std_m"]
        up_std = covariance["up_std_m"]
        if east_std is not None and north_std is not None and up_std is not None:
            msg.position_covariance[0] = float(east_std) ** 2
            msg.position_covariance[4] = float(north_std) ** 2
            msg.position_covariance[8] = float(up_std) ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def _fix_velocity_msg(self, data: BestNav) -> Optional[TwistWithCovarianceStamped]:
        if data.east_velocity_mps is None or data.north_velocity_mps is None or data.up_velocity_mps is None:
            return None
        msg = TwistWithCovarianceStamped()
        msg.header = self._header()
        msg.twist.twist.linear.x = data.east_velocity_mps
        msg.twist.twist.linear.y = data.north_velocity_mps
        msg.twist.twist.linear.z = data.up_velocity_mps
        msg.twist.covariance[0] = self.unknown_velocity_variance
        msg.twist.covariance[7] = self.unknown_velocity_variance
        msg.twist.covariance[14] = self.unknown_velocity_variance
        if data.horizontal_speed_std_mps is not None:
            horizontal_variance = data.horizontal_speed_std_mps ** 2
            msg.twist.covariance[0] = horizontal_variance
            msg.twist.covariance[7] = horizontal_variance
        if data.vertical_speed_std_mps is not None:
            msg.twist.covariance[14] = data.vertical_speed_std_mps ** 2
        return msg

    def _header(self):
        header = self.get_clock().now().to_msg()
        from std_msgs.msg import Header

        return Header(stamp=header, frame_id=self.frame_id)

    def _navsat_status(self) -> int:
        gga = self.state.latest_gga
        if not gga or gga.fix_quality == 0:
            return NavSatStatus.STATUS_NO_FIX
        if self.state.has_fixed_solution():
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _status_msg(self, summary: dict[str, object]) -> Status:
        msg = Status()
        msg.header = self._header()
        msg.fix_mode = self._fix_mode(summary)
        msg.solution_status = str(summary["solution_status"])
        msg.health_level, msg.health_message = self._health(summary, msg.fix_mode)

        msg.position_source = str(summary["position_source"])
        msg.latitude = _nan_if_none(summary["lat"])
        msg.longitude = _nan_if_none(summary["lon"])
        msg.altitude_msl_m = _nan_if_none(summary["altitude_msl_m"])
        msg.altitude_ellipsoid_m = _nan_if_none(summary["altitude_ellipsoid_m"])
        msg.differential_age_s = _nan_if_none(summary["differential_age_s"])

        east_std = summary["east_std_m"]
        north_std = summary["north_std_m"]
        up_std = summary["up_std_m"]
        msg.accuracy_source = str(summary["covariance_source"])
        msg.accuracy_age_s = _nan_if_none(summary["covariance_age_s"])
        msg.horizontal_accuracy_m = (
            float("nan") if east_std is None or north_std is None else math.hypot(float(east_std), float(north_std))
        )
        msg.vertical_accuracy_m = _nan_if_none(up_std)
        msg.hdop = _nan_if_none(summary["hdop"])

        msg.satellites_visible = self._satellites_visible()
        msg.satellites_used = self._satellites_used(summary)

        velocity = self._velocity_summary()
        msg.speed_mps = _nan_if_none(velocity["speed_mps"])
        msg.course_deg = _nan_if_none(velocity["course_deg"])
        msg.east_velocity_mps = _nan_if_none(velocity["east_velocity_mps"])
        msg.north_velocity_mps = _nan_if_none(velocity["north_velocity_mps"])
        msg.up_velocity_mps = _nan_if_none(velocity["up_velocity_mps"])

        msg.base_station_name = self.base_station_name
        msg.ntrip_host = self.ntrip_host
        msg.ntrip_mountpoint = self.ntrip_mountpoint
        msg.ntrip_connected = bool(summary["ntrip_connected"])
        msg.ntrip_last_error = "" if summary["ntrip_last_error"] is None else str(summary["ntrip_last_error"])
        msg.ntrip_gga_age_s = _nan_if_none(summary["ntrip_gga_age_s"])
        msg.rtcm_hz = float(summary["rtcm_hz"] or 0.0)
        msg.rtcm_bytes_per_sec = float(summary["rtcm_bytes_per_sec"] or 0.0)
        msg.rtcm_age_s = _nan_if_none(summary["rtcm_age_s"])
        msg.rtcm_base_station_id = int(summary["rtcm_status_base_station_id"] or 0)
        msg.rtcm_last_message_id = int(summary["rtcm_status_message_id"] or 0)
        msg.rtcm_message_ids, msg.rtcm_message_counts = self._rtcm_counts()

        msg.serial_port = self.serial_port
        msg.serial_baud = int(self.serial_baud)
        msg.configured_profile = self.configured_profile
        msg.solution_rate_hz = float(summary["solution_time_rate_hz"] or 0.0)
        msg.publish_period_s = self.publish_period_s

        msg.checksum_error_count = int(summary["checksum_error_count"] or 0)
        msg.parse_drop_count = int(summary["parse_drop_count"] or 0)
        msg.unknown_line_count = int(summary["unknown_line_count"] or 0)
        msg.serial_line_count = int(summary["serial_line_count"] or 0)
        msg.last_error = "" if summary["last_error"] is None else str(summary["last_error"])
        msg.last_unknown_sentence_type = (
            "" if summary["last_unknown_sentence_type"] is None else str(summary["last_unknown_sentence_type"])
        )
        return msg

    def _status_verbose_msg(self) -> StatusVerbose:
        msg = StatusVerbose()
        msg.header = self._header()

        bestnav = self.state.latest_bestnav
        gsv = self.state.latest_gsv
        rates = self.state.rates.rates()
        msg.native_position_type = self.state.current_position_type()
        msg.native_velocity_type = bestnav.velocity_type if bestnav and bestnav.velocity_type else ""
        msg.velocity_solution_status = bestnav.velocity_solution_status if bestnav and bestnav.velocity_solution_status else ""
        msg.velocity_latency_s = _nan_if_none(bestnav.velocity_latency_s if bestnav else None)
        msg.velocity_age_s = _nan_if_none(bestnav.velocity_age_s if bestnav else None)

        msg.latest_raw_gga = self._raw("latest_gga")
        msg.latest_raw_rmc = self._raw("latest_rmc")
        msg.latest_raw_gsa = self._raw("latest_gsa")
        msg.latest_raw_gsv = self._raw("latest_gsv")
        msg.latest_raw_gst = self._raw("latest_gst")
        msg.latest_raw_bestnav = self._raw("latest_bestnav")
        msg.latest_raw_pvtsln = self._raw("latest_pvtsln")
        msg.latest_raw_rtkstatus = self._raw("latest_rtk_status")
        msg.latest_raw_rtcmstatus = self._raw("latest_rtcm_status")
        msg.latest_rtcm_frame_hex = self._latest_rtcm_frame_hex()

        msg.recent_raw_lines = list(self.state.recent_raw_lines)
        msg.last_error_line = "" if self.state.last_error_line is None else self.state.last_error_line
        msg.last_unknown_line = "" if self.state.last_unknown_line is None else self.state.last_unknown_line

        msg.sentence_rate_names = list(rates.keys())
        msg.sentence_rate_hz = [float(value) for value in rates.values()]

        msg.satellites_tracked = self._satellites_tracked()
        msg.bestnav_satellites_used = bestnav.satellites_used if bestnav and bestnav.satellites_used else 0
        msg.pvtsln_satellites_used = (
            self.state.latest_pvtsln.best_satellites_used
            if self.state.latest_pvtsln and self.state.latest_pvtsln.best_satellites_used
            else 0
        )
        msg.gsv_satellites_in_view = gsv.satellites_in_view if gsv and gsv.satellites_in_view else 0
        gsv_snrs = [sv.snr_db for sv in gsv.satellites if sv.snr_db is not None] if gsv else []
        msg.gsv_max_snr_db = max(gsv_snrs) if gsv_snrs else 0
        return msg

    def _fix_mode(self, summary: dict[str, object]) -> str:
        rtk_state = str(summary["rtk_state"])
        fix_quality = int(summary["fix_quality"] or 0)
        if rtk_state == "FIXED":
            return "RTK_FIXED"
        if rtk_state == "FLOAT":
            return "RTK_FLOAT"
        if fix_quality == 2:
            return "DGNSS"
        if fix_quality > 0:
            return "GNSS"
        if str(summary["fix"]) in ("UNKNOWN", "NO_FIX"):
            return "NO_GNSS"
        return "UNKNOWN"

    def _health(self, summary: dict[str, object], fix_mode: str) -> tuple[str, str]:
        if summary["last_error"]:
            return "ERROR", str(summary["last_error"])
        if fix_mode in ("NO_GNSS", "UNKNOWN"):
            return "WARN", "No GNSS fix"
        if self.ntrip_enabled and not summary["ntrip_connected"]:
            return "WARN", f"{fix_mode}; NTRIP disconnected"
        if summary["rtcm_age_s"] is None:
            rtcm_state = "RTCM unavailable"
        else:
            rtcm_state = f"RTCM {float(summary['rtcm_hz'] or 0.0):.2f} Hz"
        return "OK", f"{fix_mode}; {rtcm_state}; {self._satellites_used(summary)} satellites used"

    def _satellites_visible(self) -> int:
        if self.state.latest_gsv and self.state.latest_gsv.satellites_in_view is not None:
            return self.state.latest_gsv.satellites_in_view
        candidates: list[int] = []
        if self.state.latest_bestnav and self.state.latest_bestnav.satellites_tracked is not None:
            candidates.append(self.state.latest_bestnav.satellites_tracked)
        if self.state.latest_pvtsln and self.state.latest_pvtsln.best_satellites_tracked is not None:
            candidates.append(self.state.latest_pvtsln.best_satellites_tracked)
        return max(candidates) if candidates else 0

    def _satellites_tracked(self) -> int:
        candidates: list[int] = []
        if self.state.latest_bestnav and self.state.latest_bestnav.satellites_tracked is not None:
            candidates.append(self.state.latest_bestnav.satellites_tracked)
        if self.state.latest_pvtsln and self.state.latest_pvtsln.best_satellites_tracked is not None:
            candidates.append(self.state.latest_pvtsln.best_satellites_tracked)
        return max(candidates) if candidates else 0

    def _satellites_used(self, summary: dict[str, object]) -> int:
        candidates = [
            self.state.latest_bestnav.satellites_used if self.state.latest_bestnav else None,
            self.state.latest_pvtsln.best_satellites_used if self.state.latest_pvtsln else None,
            summary["satellites"],
        ]
        return max(int(value) for value in candidates if value is not None) if any(value is not None for value in candidates) else 0

    def _velocity_summary(self) -> dict[str, Optional[float]]:
        bestnav = self.state.latest_bestnav
        rmc = self.state.latest_rmc
        if bestnav:
            return {
                "speed_mps": bestnav.horizontal_speed_mps,
                "course_deg": bestnav.track_ground_deg,
                "east_velocity_mps": bestnav.east_velocity_mps,
                "north_velocity_mps": bestnav.north_velocity_mps,
                "up_velocity_mps": bestnav.up_velocity_mps,
            }
        return {
            "speed_mps": rmc.speed_mps if rmc else None,
            "course_deg": rmc.course_deg if rmc else None,
            "east_velocity_mps": None,
            "north_velocity_mps": None,
            "up_velocity_mps": None,
        }

    def _rtcm_counts(self) -> tuple[list[int], list[int]]:
        items = sorted(self.state.rtcm_message_counts.items())
        return [int(key) for key, _ in items], [int(value) for _, value in items]

    def _raw(self, attribute: str) -> str:
        value = getattr(self.state, attribute)
        return "" if value is None else value.raw

    def _latest_rtcm_frame_hex(self) -> str:
        raw = self._raw("latest_rtcm_status")
        return raw.removeprefix("RTCM3:") if raw.startswith("RTCM3:") else ""


def _nan_if_none(value) -> float:
    return float("nan") if value is None else float(value)


def main() -> None:
    rclpy.init()
    node = UM980DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
