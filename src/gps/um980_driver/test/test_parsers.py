import pytest

from um980_driver.parsers import BestNav, GGA, GST, PvtSln, RMC, RtcmStatus, RtkStatus, parse_line
from um980_driver.protocol import append_checksum, unicore_crc32_hex


def test_parse_no_fix_gga():
    parsed = parse_line(append_checksum("GNGGA,,,,,,0,,,,,,,,"))

    assert isinstance(parsed, GGA)
    assert parsed.fix_quality == 0
    assert parsed.fix_name == "NO_FIX"
    assert parsed.lat is None
    assert parsed.lon is None


def test_parse_rtk_gga():
    parsed = parse_line(
        append_checksum("GNGGA,123519.05,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")
    )

    assert isinstance(parsed, GGA)
    assert parsed.fix_name == "RTK_FIXED"
    assert parsed.num_satellites == 24
    assert parsed.hdop == 0.62
    assert round(parsed.lat, 8) == 37.38745833
    assert round(parsed.lon, 8) == 127.00205667


def test_parse_rmc_speed_course():
    parsed = parse_line(
        append_checksum("GNRMC,123519.05,A,3723.2475,N,12700.1234,E,10.0,182.5,290426,,,A")
    )

    assert isinstance(parsed, RMC)
    assert parsed.valid
    assert round(parsed.speed_mps, 5) == 5.14444
    assert parsed.course_deg == 182.5
    assert parsed.date == "290426"


def _append_unicore_crc(content: str) -> str:
    return f"#{content}*{unicore_crc32_hex(content)}"


def test_parse_gst_covariance_fields():
    parsed = parse_line(append_checksum("GNGST,123519.00,0.1,0.2,0.3,45.0,0.011,0.012,0.025"))

    assert isinstance(parsed, GST)
    assert parsed.lat_std_m == 0.011
    assert parsed.lon_std_m == 0.012
    assert parsed.alt_std_m == 0.025


def test_parse_bestnav_core_solution_fields():
    body = ",".join([
        "SOL_COMPUTED", "NARROW_FLOAT", "37.1", "127.2", "48.3", "23.0", "WGS84",
        "0.012", "0.013", "0.020", "7", "0.8", "0.0", "30", "25", "0", "0",
        "0", "0", "0", "0", "SOL_COMPUTED", "NARROW_FLOAT", "0.1", "0.2", "2.0",
        "30.0", "-0.3", "0.04", "0.05",
    ])
    parsed = parse_line(_append_unicore_crc(f"BESTNAVA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;{body}"))

    assert isinstance(parsed, BestNav)
    assert parsed.position_type == "NARROW_FLOAT"
    assert parsed.lat == 37.1
    assert parsed.lon_std_m == 0.013
    assert parsed.satellites_used == 25
    assert parsed.velocity_latency_s == 0.1
    assert parsed.velocity_age_s == 0.2
    assert parsed.horizontal_speed_mps == 2.0
    assert parsed.track_ground_deg == 30.0
    assert parsed.vertical_speed_mps == -0.3
    assert parsed.vertical_speed_std_mps == 0.04
    assert parsed.horizontal_speed_std_mps == 0.05
    assert parsed.north_velocity_mps == pytest.approx(1.7320508)
    assert parsed.east_velocity_mps == pytest.approx(1.0)
    assert parsed.up_velocity_mps == -0.3


def test_parse_pvtsln_core_solution_fields():
    fields = [
        "NARROW_INT", "48.3", "37.1", "127.2", "0.020", "0.012", "0.013", "0.8",
        "SINGLE", "48.4", "37.1001", "127.2001", "23.0", "30", "25", "28", "20",
        "0.1", "0.2", "0.22", "NONE", "0.0", "0.0", "0.0", "10", "8", "7",
        "6", "2.1", "1.8", "1.5",
    ]
    parsed = parse_line(_append_unicore_crc(f"PVTSLNA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;{','.join(fields)}"))

    assert isinstance(parsed, PvtSln)
    assert parsed.best_position_type == "NARROW_INT"
    assert parsed.best_lat_std_m == 0.012
    assert parsed.best_satellites_tracked == 30
    assert parsed.best_satellites_used == 25
    assert parsed.psr_satellites_tracked == 28
    assert parsed.psr_satellites_used == 20
    assert parsed.north_velocity_mps == 0.1
    assert parsed.east_velocity_mps == 0.2
    assert parsed.ground_speed_mps == 0.22
    assert parsed.heading_type == "NONE"
    assert parsed.baseline_m == 0.0
    assert parsed.heading_satellites_tracked == 10
    assert parsed.heading_satellites_used == 8
    assert parsed.heading_l1_satellites == 7
    assert parsed.heading_l1l2_satellites == 6
    assert parsed.gdop == 2.1
    assert parsed.pdop == 1.8
    assert parsed.hdop == 1.5


def test_parse_rtk_and_rtcm_status():
    rtk = parse_line(_append_unicore_crc("RTKSTATUSA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;1,0,3,4,0,6,0,7,8,9,0,NARROW_INT,OK,1,0,12,0"))
    rtcm = parse_line(_append_unicore_crc("RTCMSTATUSA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;1074,3,7,25,10,5,6,4,0,1"))

    assert isinstance(rtk, RtkStatus)
    assert rtk.position_type == "NARROW_INT"
    assert rtk.bds_source1 == 3
    assert rtk.glonass_source == 6
    assert rtk.galileo_source1 == 7
    assert rtk.calculate_status == "OK"
    assert rtk.ion_detected is True
    assert isinstance(rtcm, RtcmStatus)
    assert rtcm.message_id == 1074
    assert rtcm.base_station_id == 7
    assert rtcm.satellite_count == 25
    assert rtcm.l1_observation_count == 10
    assert rtcm.l6_observation_count == 1
