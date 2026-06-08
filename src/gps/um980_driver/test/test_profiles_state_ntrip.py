import pytest

from um980_driver.ntrip_config import validate_ntrip_config
from um980_driver.parsers import parse_line
from um980_driver.profiles import get_profile
from um980_driver.protocol import append_checksum, unicore_crc32_hex
from um980_driver import serial_client
from um980_driver.serial_client import CommandResult, UM980SerialClient, detect_baud
from um980_driver.state import ByteRateTracker, GNSSState


def test_profile_does_not_save_by_default():
    profile = get_profile("signalgroup1_20hz")

    assert "SAVECONFIG" not in profile.commands_with_save(False)
    assert profile.commands_with_save(True)[-1] == "SAVECONFIG"


def test_survey_profile_uses_onchanged_rtcmstatus():
    profile = get_profile("survey_20hz")

    assert "MODE ROVER" in profile.commands
    assert "RTCMSTATUSA ONCHANGED" in profile.commands
    assert "RTCMSTATUSA 1" not in profile.commands
    assert profile.is_optional("RTCMSTATUSA ONCHANGED")


class FakeCommandClient(UM980SerialClient):
    def __init__(self, responses):
        self.responses = responses
        self.reconnect_count = 0

    def send_command(self, command: str, timeout: float = 2.0, expect_response: bool = True) -> CommandResult:
        ok = self.responses.get(command, True)
        lines = [f"$command,{command},response: {'OK' if ok else 'PARSING FAILED'}"]
        return CommandResult(command=command, lines=lines, ok=ok)

    def reconnect(self, delay_s: float = 1.0) -> None:
        self.reconnect_count += 1


def test_apply_commands_continues_after_optional_failure():
    client = FakeCommandClient({"GPGSV 1": False})

    results = client.apply_commands(["GPGSV 1", "GPGGA 0.05"], optional_commands=("GPGSV 1",))

    assert [result.command for result in results] == ["GPGSV 1", "GPGGA 0.05"]


def test_apply_commands_uses_fallback_before_stopping():
    client = FakeCommandClient({"GPGGA 0.05": False, "GNGGA 0.05": True})

    results = client.apply_commands(["GPGGA 0.05", "GPRMC 0.05"], fallbacks={"GPGGA 0.05": "GNGGA 0.05"})

    assert [result.command for result in results] == ["GPGGA 0.05", "GNGGA 0.05", "GPRMC 0.05"]


def test_apply_commands_reconnects_after_signalgroup8():
    client = FakeCommandClient({})

    results = client.apply_commands(
        ["CONFIG SIGNALGROUP 8", "MODE ROVER", "GPGGA 0.02"],
        reconnect_after_commands=("CONFIG SIGNALGROUP 8",),
        reconnect_delay_s=0.0,
    )

    assert [result.command for result in results] == ["CONFIG SIGNALGROUP 8", "MODE ROVER", "GPGGA 0.02"]
    assert client.reconnect_count == 1


def test_detect_baud_finds_passive_receiver_output(monkeypatch):
    class FakeSerial:
        def __init__(self, port, baud, timeout=0.2, write_timeout=0.2):
            self.baud = baud
            self.lines = [b"$GNGGA,,,,,,0,,,,,,,,*78\r\n"] if baud == 115200 else []

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def readline(self):
            return self.lines.pop(0) if self.lines else b""

        def write(self, data):
            return len(data)

        def flush(self):
            return None

    monkeypatch.setattr(serial_client.serial, "Serial", FakeSerial)

    result = detect_baud("/dev/fake", candidates=[460800, 115200], passive_window_s=0.01, query_timeout_s=0.01)

    assert result.baud == 115200


def test_state_tracks_line_and_solution_rates():
    state = GNSSState()
    line1 = append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")
    line2 = append_checksum("GNGGA,123519.05,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")

    state.update(parse_line(line1), now=1.0)
    state.update(parse_line(line2), now=1.05)

    summary = state.summary(now=1.05)
    assert summary["fix"] == "RTK_FIXED"
    assert summary["rates"]["GGA"] == pytest.approx(20.0)
    assert summary["solution_time_rate_hz"] == pytest.approx(20.0)


def test_repeated_solution_time_is_separated_from_line_rate():
    state = GNSSState()
    line = append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")

    state.update(parse_line(line), now=1.0)
    state.update(parse_line(line), now=1.02)

    summary = state.summary(now=1.02)
    assert summary["rates"]["GGA"] == pytest.approx(50.0)
    assert summary["solution_time_rate_hz"] == 0.0


def test_byte_rate_tracker():
    tracker = ByteRateTracker()
    tracker.observe(100, now=1.0)
    tracker.observe(150, now=2.0)

    assert tracker.rate_hz(now=2.0) == pytest.approx(1.0)
    assert tracker.bytes_per_sec(now=2.0) == pytest.approx(250.0)
    assert tracker.age_s(now=2.0) == pytest.approx(0.0)


def test_private_yaml_validation_accepts_v1_shape():
    config = validate_ntrip_config({
        "host": "www.gnssdata.or.kr",
        "port": 2101,
        "mountpoint": "SWGS-RTCM31",
        "authenticate": True,
        "username": "user",
        "password": "pass",
        "reconnect_wait_s": 5.0,
        "rtcm_timeout_s": 4.0,
    })

    assert config.port == 2101
    assert config.reconnect_wait_s == 5.0
    assert config.rtcm_timeout_s == 4.0


def test_private_yaml_validation_rejects_bad_timeout():
    with pytest.raises(ValueError):
        validate_ntrip_config({
            "host": "host",
            "port": 2101,
            "mountpoint": "mount",
            "authenticate": True,
            "username": "user",
            "password": "pass",
            "rtcm_timeout_s": 0,
        })


def test_state_tracks_ntrip_status():
    state = GNSSState()

    state.update_ntrip(connected=True, last_error=None, gga_age_s=1.5)
    summary = state.summary()

    assert summary["ntrip_connected"] is True
    assert summary["ntrip_last_error"] is None
    assert summary["ntrip_gga_age_s"] == 1.5


def test_state_records_parse_error_line():
    state = GNSSState()

    state.observe_parse_error("invalid NMEA checksum", "$GPGST,bad*00")
    summary = state.summary()

    assert summary["parse_drop_count"] == 1
    assert summary["checksum_error_count"] == 1
    assert summary["last_error_line"] == "$GPGST,bad*00"


def test_state_records_unknown_line_type():
    state = GNSSState()

    state.observe_unknown_line("#BESTPOSA,COM1,0,0;1,2,3*00000000")
    summary = state.summary()

    assert summary["unknown_line_count"] == 1
    assert summary["last_unknown_sentence_type"] == "BESTPOSA"


def test_state_exposes_gsa_gsv_diagnostics():
    state = GNSSState()
    gsa = append_checksum("GNGSA,A,1,,,,,,,,,,,,,9999.0,9999.0,9999.0,1")
    gsv = append_checksum("GPGSV,2,1,08,01,45,120,38,02,30,250,35,03,20,180,29,04,10,070,")

    state.update(parse_line(gsa))
    state.update(parse_line(gsv))
    summary = state.summary()

    assert summary["gsa_fix_type"] == 1
    assert summary["gsa_satellites_used"] == 0
    assert summary["gsv_satellites_in_view"] == 8
    assert summary["gsv_satellites_reported"] == 4
    assert summary["gsv_max_snr_db"] == 38


def _append_unicore_crc(content: str) -> str:
    return f"#{content}*{unicore_crc32_hex(content)}"


def test_state_tracks_latest_valid_gga_for_ntrip():
    state = GNSSState()
    valid = append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,1,12,0.9,54.2,M,19.5,M,,")
    invalid = append_checksum("GNGGA,,,,,,0,,,,,,,,")

    state.update(parse_line(valid), now=1.0)
    state.update(parse_line(invalid), now=2.0)

    assert state.latest_valid_gga_for_ntrip is not None
    assert state.latest_valid_gga_for_ntrip.raw == valid
    assert state.latest_raw_gga == valid


def test_state_distinguishes_float_from_fixed():
    state = GNSSState()
    state.update(parse_line(append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,5,24,0.62,54.2,M,19.5,M,0.4,0000")))

    assert state.rtk_state() == "FLOAT"
    assert state.summary()["fix"] == "RTK_FLOAT"
    assert not state.has_fixed_solution()

    state.update(parse_line(_append_unicore_crc("RTKSTATUSA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;1,2,3,4,5,6,0,7,8,9,10,NARROW_INT,2,1,0,0,12")))

    assert state.rtk_state() == "FIXED"
    assert state.summary()["fix"] == "RTK_FIXED"
    assert state.has_fixed_solution()


def test_state_reports_native_float_over_dgps_gga_label():
    state = GNSSState()
    state.update(parse_line(append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,2,24,0.62,54.2,M,19.5,M,1.4,0000")))
    state.update(parse_line(_append_unicore_crc("RTKSTATUSA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;1,2,3,4,5,6,0,7,8,9,10,NARROW_FLOAT,2,1,0,0,12")))

    summary = state.summary()

    assert summary["rtk_state"] == "FLOAT"
    assert summary["fix"] == "RTK_FLOAT"
    assert summary["fix_quality"] == 2


def test_state_covariance_prefers_bestnav_over_gst():
    state = GNSSState()
    gst = append_checksum("GNGST,123519.00,0.1,0.2,0.3,45.0,0.5,0.6,0.7")
    body = ",".join([
        "SOL_COMPUTED", "NARROW_FLOAT", "37.1", "127.2", "48.3", "23.0", "WGS84",
        "0.012", "0.013", "0.020", "7", "0.8", "0.0", "30", "25", "0", "0",
        "0", "0", "0", "0", "SOL_COMPUTED", "NARROW_FLOAT", "0.1", "0.2", "2.0",
        "45.0", "0.0", "0.04", "0.05",
    ])

    state.update(parse_line(gst))
    state.update(parse_line(_append_unicore_crc(f"BESTNAVA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;{body}")))

    covariance = state.covariance_solution()
    assert covariance["source"] == "BESTNAV"
    assert covariance["east_std_m"] == 0.013
    assert covariance["north_std_m"] == 0.012
    assert covariance["age_s"] is not None


def test_state_ignores_bestnav_covariance_without_solution():
    state = GNSSState()
    body = ",".join([
        "INSUFFICIENT_OBS", "NONE", "0.0", "0.0", "-17.0", "17.0", "WGS84",
        "0.0", "0.0", "0.0", "0", "0.0", "280.0", "0", "0", "0", "0",
        "0", "0", "0", "0", "INSUFFICIENT_OBS", "NONE", "0.0", "0.0", "0.0",
        "0.0", "0.0", "0.0", "0.0",
    ])

    state.update(parse_line(_append_unicore_crc(f"BESTNAVA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;{body}")))

    covariance = state.covariance_solution()
    assert covariance["source"] == "UNKNOWN"


def test_state_reports_ellipsoid_altitude_from_gga():
    state = GNSSState()
    line = append_checksum("GNGGA,123519.00,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")

    state.update(parse_line(line), now=1.0)

    summary = state.summary(now=1.0)
    assert summary["altitude_msl_m"] == 54.2
    assert summary["altitude_ellipsoid_m"] == pytest.approx(73.7)


def test_state_rejects_stale_covariance():
    state = GNSSState()
    gst = append_checksum("GNGST,123519.00,0.1,0.2,0.3,45.0,0.5,0.6,0.7")

    state.update(parse_line(gst), now=1.0)

    covariance = state.covariance_solution(now=4.0, timeout_s=2.0)
    assert covariance["source"] == "UNKNOWN"
