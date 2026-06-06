from platoon_v2v.telemetry_protocol import FLAG_RC_VALID
from platoon_v2v.telemetry_protocol import FLAG_STOP_REQUIRED
from platoon_v2v.telemetry_protocol import LeaderTelemetryFrame
from platoon_v2v.telemetry_protocol import LeaderTelemetryStreamParser
from platoon_v2v.telemetry_protocol import MODE_MANUAL_RC
from platoon_v2v.telemetry_protocol import pack_leader_telemetry
from platoon_v2v.telemetry_protocol import SOURCE_RC
from platoon_v2v.telemetry_protocol import unpack_leader_telemetry


def make_frame(seq=1):
    return LeaderTelemetryFrame(
        seq=seq,
        boot_id=7,
        drive_mode=MODE_MANUAL_RC,
        source=SOURCE_RC,
        flags=FLAG_RC_VALID,
        rc_throttle_us=1520,
        rc_steer_us=1480,
        rc_mode_us=1100,
        throttle_norm=0.25,
        throttle_cmd_pwm=50.0,
        throttle_output_pwm=42.0,
        steering_target_adc=590.0,
        steering_current_adc=585.0,
        steering_angle_rad=0.05,
        speed_estimate_mps=0.31,
        auto_speed_cmd_mps=0.0,
        auto_steering_cmd_rad=0.0,
        arduino_time_ms=12345,
        bridge_rx_ok_count=9,
        bridge_crc_fail_count=2,
    )


def test_pack_unpack_round_trip():
    frame = make_frame()
    packet = pack_leader_telemetry(frame)
    parsed = unpack_leader_telemetry(packet)

    assert parsed.seq == 1
    assert parsed.boot_id == 7
    assert parsed.rc_valid
    assert not parsed.stop_required
    assert parsed.rc_throttle_us == 1520
    assert abs(parsed.speed_estimate_mps - 0.31) < 1e-6


def test_stream_parser_resynchronizes_after_garbage():
    parser = LeaderTelemetryStreamParser()
    packet_a = pack_leader_telemetry(make_frame(seq=10))
    packet_b = pack_leader_telemetry(make_frame(seq=11))

    frames = parser.feed(b'noise' + packet_a[:8])
    assert frames == []
    frames = parser.feed(packet_a[8:] + packet_b)

    assert [frame.seq for frame in frames] == [10, 11]
    assert parser.sync_drop_count == 5


def test_stream_parser_recovers_after_crc_error():
    parser = LeaderTelemetryStreamParser()
    bad = bytearray(pack_leader_telemetry(make_frame(seq=20)))
    bad[-1] ^= 0xFF
    good = pack_leader_telemetry(make_frame(seq=21))

    frames = parser.feed(bytes(bad) + good)

    assert [frame.seq for frame in frames] == [21]
    assert parser.crc_fail_count == 1


def test_stop_flag_property():
    frame = make_frame()
    packet = pack_leader_telemetry(
        LeaderTelemetryFrame(**{**frame.__dict__, 'flags': FLAG_STOP_REQUIRED})
    )
    parsed = unpack_leader_telemetry(packet)

    assert parsed.stop_required
    assert not parsed.rc_valid
