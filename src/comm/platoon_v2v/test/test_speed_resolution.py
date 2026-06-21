import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from platoon_interfaces.msg import LeaderDriveTelemetry
from platoon_v2v.leader_reference_path_relay_node import LeaderReferencePathRelayNode
from platoon_v2v.leader_v2v_adapter_node import LeaderV2vAdapterNode
from platoon_v2v.speed_resolution import signed_speed_from_telemetry
import pytest
from rclpy.time import Time


def test_throttle_sign_restores_reverse_encoder_speed():
    assert signed_speed_from_telemetry(0.31, -0.4) == -0.31


def test_throttle_fallback_provides_signed_low_speed_command():
    assert signed_speed_from_telemetry(0.0, -0.2) == -0.1


def test_neutral_throttle_keeps_measured_speed():
    assert signed_speed_from_telemetry(0.12, 0.0) == 0.12


def test_rc_pwm_fallback_provides_reverse_when_normalized_throttle_is_zero():
    assert signed_speed_from_telemetry(0.0, 0.0, 1300) == -0.25


def test_rc_pwm_deadband_keeps_trimmed_neutral_at_zero():
    assert signed_speed_from_telemetry(0.0, 0.0, 1460) == 0.0


def _telemetry(drive_mode, speed_estimate_mps=0.0, throttle_norm=0.2):
    msg = LeaderDriveTelemetry()
    msg.drive_mode = drive_mode
    msg.speed_estimate_mps = speed_estimate_mps
    msg.throttle_norm = throttle_norm
    msg.rc_throttle_us = 1500
    return msg


def _adapter_for_speed(telemetry):
    node = LeaderV2vAdapterNode.__new__(LeaderV2vAdapterNode)
    node.prefer_encoder_twist_speed = False
    node.latest_encoder_twist = None
    node.latest_encoder_twist_time = None
    node.latest_telemetry = telemetry
    node.latest_telemetry_time = Time(nanoseconds=0)
    node.telemetry_timeout_sec = 0.3
    node.use_throttle_signed_speed_fallback = True
    node.telemetry_speed_deadband_mps = 0.02
    node.throttle_speed_deadband = 0.05
    node.throttle_speed_fallback_gain_mps = 0.50
    node.rc_throttle_neutral_us = 1500
    node.rc_throttle_deadband_us = 80
    node.rc_throttle_full_scale_us = 400
    return node


def test_manual_rc_allows_throttle_signed_speed_fallback():
    node = _adapter_for_speed(_telemetry(LeaderDriveTelemetry.MODE_MANUAL_RC))

    assert node._resolve_speed_mps(Time(nanoseconds=100_000_000)) == pytest.approx(
        0.1
    )


def test_autonomous_ignores_throttle_signed_speed_fallback():
    node = _adapter_for_speed(_telemetry(LeaderDriveTelemetry.MODE_AUTONOMOUS))

    assert node._resolve_speed_mps(Time(nanoseconds=100_000_000)) == 0.0


def test_odom_fresh_rejects_odom_after_timeout():
    node = LeaderV2vAdapterNode.__new__(LeaderV2vAdapterNode)
    node.latest_odom_time = Time(nanoseconds=0)
    node.leader_odom_timeout_sec = 0.5

    assert node._odom_fresh(Time(nanoseconds=500_000_000))
    assert not node._odom_fresh(Time(nanoseconds=600_000_000))


class _Clock:
    def __init__(self, now):
        self._now = now

    def now(self):
        return self._now


def _relay_for_path_transform(now_ns=0):
    node = LeaderReferencePathRelayNode.__new__(LeaderReferencePathRelayNode)
    node.expected_frame_id = 'map'
    node.leader_frame_id = 'leader/base_link'
    node.allow_empty_frame_id = False
    node.leader_odom_timeout_sec = 0.5
    node.max_points = 80
    node.enable_breadcrumb = True
    node.breadcrumb_min_point_spacing_m = 0.1
    node.breadcrumb_max_path_length_m = 15.0
    node.breadcrumb_max_position_jump_m = 0.75
    node.leader_breadcrumb = []
    node.latest_leader_odom = None
    node.latest_leader_odom_time = None
    node.get_clock = lambda: _Clock(Time(nanoseconds=now_ns))
    return node


def _path_in_leader_frame():
    path = Path()
    path.header.frame_id = 'leader/base_link'
    for x in (1.0, 2.0):
        pose = PoseStamped()
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _leader_odom(*, frame_id='map', x=10.0, y=2.0, yaw=math.pi / 2.0):
    odom = Odometry()
    odom.header.frame_id = frame_id
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
    odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
    return odom


def _map_path(*xs):
    path = Path()
    path.header.frame_id = 'map'
    for x in xs:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def test_reference_path_relay_transforms_fresh_complex_path_to_map():
    node = _relay_for_path_transform(now_ns=200_000_000)
    node.latest_leader_odom = _leader_odom()
    node.latest_leader_odom_time = Time(nanoseconds=0)
    drops = []
    node._warn_drop = lambda source, reason: drops.append((source, reason))

    converted = node._normalize_path('complex', _path_in_leader_frame())

    assert converted is not None
    assert converted.header.frame_id == 'map'
    assert converted.poses[0].pose.position.x == pytest.approx(10.0)
    assert converted.poses[0].pose.position.y == pytest.approx(3.0)
    assert converted.poses[1].pose.position.x == pytest.approx(10.0)
    assert converted.poses[1].pose.position.y == pytest.approx(4.0)
    assert drops == []


def test_reference_path_relay_prepends_leader_breadcrumb_to_forward_path():
    node = _relay_for_path_transform()
    node._append_breadcrumb_from_odom(_leader_odom(x=0.0, y=0.0, yaw=0.0))
    node._append_breadcrumb_from_odom(_leader_odom(x=0.2, y=0.0, yaw=0.0))

    output = node._combined_reference_path(_map_path(1.0, 2.0))

    assert output.header.frame_id == 'map'
    assert [pose.pose.position.x for pose in output.poses] == pytest.approx(
        [0.0, 0.2, 1.0, 2.0]
    )


def test_reference_path_relay_can_publish_breadcrumb_only_path():
    node = _relay_for_path_transform()
    node._append_breadcrumb_from_odom(_leader_odom(x=0.0, y=0.0, yaw=0.0))
    node._append_breadcrumb_from_odom(_leader_odom(x=0.2, y=0.0, yaw=0.0))

    output = node._combined_reference_path(None)

    assert output.header.frame_id == 'map'
    assert [pose.pose.position.x for pose in output.poses] == pytest.approx([0.0, 0.2])


def test_reference_path_relay_resets_breadcrumb_on_position_jump():
    node = _relay_for_path_transform()
    node._append_breadcrumb_from_odom(_leader_odom(x=0.0, y=0.0, yaw=0.0))
    node._append_breadcrumb_from_odom(_leader_odom(x=2.0, y=0.0, yaw=0.0))

    assert [pose.pose.position.x for pose in node.leader_breadcrumb] == pytest.approx([2.0])


def test_reference_path_relay_rejects_stale_odom_for_complex_transform():
    node = _relay_for_path_transform(now_ns=600_000_000)
    node.latest_leader_odom = _leader_odom()
    node.latest_leader_odom_time = Time(nanoseconds=0)
    drops = []
    node._warn_drop = lambda source, reason: drops.append((source, reason))

    converted = node._normalize_path('complex', _path_in_leader_frame())

    assert converted is None
    assert drops == [
        ('complex', 'stale_leader_odom_for_frame_transform:0.60s')
    ]
