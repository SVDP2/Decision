from platoon_interfaces.msg import LeaderDriveTelemetry
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
