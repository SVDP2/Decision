"""In-memory GNSS state and runtime rate tracking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from time import monotonic
from typing import Optional

from .parsers import BestNav, GGA, GSA, GST, GSV, PvtSln, RMC, RtkStatus, RtcmStatus, ParsedGNSS


FIXED_POSITION_TYPES = {"L1_INT", "WIDE_INT", "NARROW_INT", "INS_RTKFIXED"}
FLOAT_POSITION_TYPES = {"L1_FLOAT", "NARROW_FLOAT", "INS_RTKFLOAT"}
VALID_POSITION_TYPES = FIXED_POSITION_TYPES | FLOAT_POSITION_TYPES | {
    "INS",
    "INS_PSRDIFF",
    "INS_PSRSP",
    "IONOFREE_FLOAT",
    "PPP_CONVERGING",
    "SINGLE",
    "PSRDIFF",
    "WAAS",
    "SBAS",
    "PPP",
}


class RateTracker:
    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self._events: dict[str, deque[float]] = {}

    def observe(self, key: str, now: Optional[float] = None) -> None:
        now = monotonic() if now is None else now
        events = self._events.setdefault(key, deque())
        events.append(now)
        self._trim(events, now)

    def rate(self, key: str, now: Optional[float] = None) -> float:
        now = monotonic() if now is None else now
        events = self._events.get(key)
        if not events:
            return 0.0
        self._trim(events, now)
        if len(events) < 2:
            return 0.0
        elapsed = events[-1] - events[0]
        return 0.0 if elapsed <= 0.0 else (len(events) - 1) / elapsed

    def rates(self, now: Optional[float] = None) -> dict[str, float]:
        now = monotonic() if now is None else now
        return {key: self.rate(key, now) for key in sorted(self._events)}

    def _trim(self, events: deque[float], now: float) -> None:
        cutoff = now - self.window_s
        while events and events[0] < cutoff:
            events.popleft()


class ByteRateTracker:
    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self._events: deque[tuple[float, int]] = deque()

    def observe(self, byte_count: int, now: Optional[float] = None) -> None:
        now = monotonic() if now is None else now
        self._events.append((now, byte_count))
        self._trim(now)

    def rate_hz(self, now: Optional[float] = None) -> float:
        now = monotonic() if now is None else now
        self._trim(now)
        if len(self._events) < 2:
            return 0.0
        elapsed = self._events[-1][0] - self._events[0][0]
        return 0.0 if elapsed <= 0.0 else (len(self._events) - 1) / elapsed

    def bytes_per_sec(self, now: Optional[float] = None) -> float:
        now = monotonic() if now is None else now
        self._trim(now)
        if len(self._events) < 2:
            return 0.0
        elapsed = self._events[-1][0] - self._events[0][0]
        if elapsed <= 0.0:
            return 0.0
        return sum(count for _, count in self._events) / elapsed

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        if not self._events:
            return None
        now = monotonic() if now is None else now
        return now - self._events[-1][0]

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._events and self._events[0][0] < cutoff:
            self._events.popleft()


@dataclass
class GNSSState:
    latest_gga: Optional[GGA] = None
    latest_valid_gga_for_ntrip: Optional[GGA] = None
    latest_rmc: Optional[RMC] = None
    latest_gsa: Optional[GSA] = None
    latest_gsv: Optional[GSV] = None
    latest_gst: Optional[GST] = None
    latest_bestnav: Optional[BestNav] = None
    latest_pvtsln: Optional[PvtSln] = None
    latest_gst_at: Optional[float] = None
    latest_bestnav_at: Optional[float] = None
    latest_pvtsln_at: Optional[float] = None
    latest_rtk_status: Optional[RtkStatus] = None
    latest_rtcm_status: Optional[RtcmStatus] = None
    latest_raw_gga: Optional[str] = None
    recent_raw_lines: deque[str] = field(default_factory=lambda: deque(maxlen=25))
    last_error: Optional[str] = None
    last_error_line: Optional[str] = None
    checksum_error_count: int = 0
    parse_drop_count: int = 0
    unknown_line_count: int = 0
    last_unknown_sentence_type: Optional[str] = None
    last_unknown_line: Optional[str] = None
    serial_line_count: int = 0
    rates: RateTracker = field(default_factory=RateTracker)
    solution_rates: RateTracker = field(default_factory=RateTracker)
    rtcm_rates: ByteRateTracker = field(default_factory=ByteRateTracker)
    rtcm_message_counts: dict[int, int] = field(default_factory=dict)
    ntrip_connected: bool = False
    ntrip_last_error: Optional[str] = None
    ntrip_gga_age_s: Optional[float] = None
    _last_gga_utc: Optional[str] = None

    def observe_raw_line(self, line: str) -> None:
        self.serial_line_count += 1
        self.recent_raw_lines.append(line)

    def update(self, parsed: ParsedGNSS, now: Optional[float] = None) -> None:
        now = monotonic() if now is None else now
        if isinstance(parsed, GGA):
            self.latest_gga = parsed
            if parsed.fix_quality > 0 and parsed.lat is not None and parsed.lon is not None:
                self.latest_valid_gga_for_ntrip = parsed
                self.latest_raw_gga = parsed.raw
            self.rates.observe("GGA", now)
            if parsed.utc and parsed.utc != self._last_gga_utc:
                self.solution_rates.observe("GGA_UTC", now)
                self._last_gga_utc = parsed.utc
        elif isinstance(parsed, RMC):
            self.latest_rmc = parsed
            self.rates.observe("RMC", now)
        elif isinstance(parsed, GSA):
            self.latest_gsa = parsed
            self.rates.observe("GSA", now)
        elif isinstance(parsed, GSV):
            self.latest_gsv = parsed
            self.rates.observe("GSV", now)
        elif isinstance(parsed, GST):
            self.latest_gst = parsed
            self.latest_gst_at = now
            self.rates.observe("GST", now)
        elif isinstance(parsed, BestNav):
            self.latest_bestnav = parsed
            self.latest_bestnav_at = now
            self.rates.observe("BESTNAV", now)
            self.solution_rates.observe("BESTNAV", now)
        elif isinstance(parsed, PvtSln):
            self.latest_pvtsln = parsed
            self.latest_pvtsln_at = now
            self.rates.observe("PVTSLN", now)
        elif isinstance(parsed, RtkStatus):
            self.latest_rtk_status = parsed
            self.rates.observe("RTKSTATUS", now)
        elif isinstance(parsed, RtcmStatus):
            self.latest_rtcm_status = parsed
            self.rates.observe("RTCMSTATUS", now)

    def observe_parse_error(self, error: str, line: Optional[str] = None) -> None:
        self.parse_drop_count += 1
        self.last_error = error
        self.last_error_line = line
        if "checksum" in error.lower():
            self.checksum_error_count += 1

    def observe_unknown_line(self, line: str) -> None:
        self.unknown_line_count += 1
        self.last_unknown_sentence_type = unknown_sentence_type(line)
        self.last_unknown_line = line

    def observe_rtcm(self, byte_count: int, now: Optional[float] = None) -> None:
        self.rtcm_rates.observe(byte_count, now)

    def observe_rtcm_message(self, parsed: RtcmStatus, now: Optional[float] = None) -> None:
        self.latest_rtcm_status = parsed
        self.rates.observe("RTCM3", now)
        if parsed.message_id is not None:
            self.rtcm_message_counts[parsed.message_id] = self.rtcm_message_counts.get(parsed.message_id, 0) + 1

    def update_ntrip(self, connected: bool, last_error: Optional[str], gga_age_s: Optional[float]) -> None:
        self.ntrip_connected = connected
        self.ntrip_last_error = last_error
        self.ntrip_gga_age_s = gga_age_s

    def position_solution(self) -> dict[str, object]:
        bestnav = self.latest_bestnav
        if bestnav and bestnav.solution_status == "SOL_COMPUTED" and bestnav.lat is not None and bestnav.lon is not None:
            return {
                "source": "BESTNAV",
                "lat": bestnav.lat,
                "lon": bestnav.lon,
                "altitude_msl_m": bestnav.altitude_msl_m,
                "altitude_ellipsoid_m": ellipsoid_altitude(bestnav.altitude_msl_m, bestnav.undulation_m),
                "position_type": bestnav.position_type,
                "solution_status": bestnav.solution_status,
                "differential_age_s": bestnav.differential_age_s,
            }
        gga = self.latest_gga
        return {
            "source": "GGA" if gga else "UNKNOWN",
            "lat": gga.lat if gga else None,
            "lon": gga.lon if gga else None,
            "altitude_msl_m": gga.altitude_msl_m if gga else None,
            "altitude_ellipsoid_m": ellipsoid_altitude(gga.altitude_msl_m, gga.geoid_sep_m) if gga else None,
            "position_type": gga.fix_name if gga else "UNKNOWN",
            "solution_status": "SOL_COMPUTED" if gga and gga.fix_quality > 0 else "NO_FIX",
            "differential_age_s": gga.differential_age_s if gga else None,
        }

    def covariance_solution(
        self,
        now: Optional[float] = None,
        timeout_s: float = 2.0,
        max_std_m: float = 100.0,
    ) -> dict[str, object]:
        now = monotonic() if now is None else now
        bestnav = self.latest_bestnav
        if (
            bestnav
            and bestnav.solution_status == "SOL_COMPUTED"
            and bestnav.position_type in VALID_POSITION_TYPES
            and self._fresh(self.latest_bestnav_at, now, timeout_s)
            and sane_std_triplet(bestnav.lat_std_m, bestnav.lon_std_m, bestnav.altitude_std_m, max_std_m)
        ):
            return {
                "source": "BESTNAV",
                "east_std_m": bestnav.lon_std_m,
                "north_std_m": bestnav.lat_std_m,
                "up_std_m": bestnav.altitude_std_m,
                "age_s": age_s(self.latest_bestnav_at, now),
            }
        pvtsln = self.latest_pvtsln
        if (
            pvtsln
            and pvtsln.best_position_type in VALID_POSITION_TYPES
            and self._fresh(self.latest_pvtsln_at, now, timeout_s)
            and sane_std_triplet(
                pvtsln.best_lat_std_m,
                pvtsln.best_lon_std_m,
                pvtsln.best_altitude_std_m,
                max_std_m,
            )
        ):
            return {
                "source": "PVTSLN",
                "east_std_m": pvtsln.best_lon_std_m,
                "north_std_m": pvtsln.best_lat_std_m,
                "up_std_m": pvtsln.best_altitude_std_m,
                "age_s": age_s(self.latest_pvtsln_at, now),
            }
        gst = self.latest_gst
        if (
            gst
            and self._fresh(self.latest_gst_at, now, timeout_s)
            and sane_std_triplet(gst.lat_std_m, gst.lon_std_m, gst.alt_std_m, max_std_m)
        ):
            return {
                "source": "GST",
                "east_std_m": gst.lon_std_m,
                "north_std_m": gst.lat_std_m,
                "up_std_m": gst.alt_std_m,
                "age_s": age_s(self.latest_gst_at, now),
            }
        return {"source": "UNKNOWN", "east_std_m": None, "north_std_m": None, "up_std_m": None, "age_s": None}

    def _fresh(self, observed_at: Optional[float], now: float, timeout_s: float) -> bool:
        if observed_at is None:
            return False
        return now - observed_at <= timeout_s

    def rtk_state(self) -> str:
        position_type = self.current_position_type()
        if position_type in FIXED_POSITION_TYPES:
            return "FIXED"
        if position_type in FLOAT_POSITION_TYPES:
            return "FLOAT"
        gga = self.latest_gga
        if gga and gga.fix_quality == 4:
            return "FIXED"
        if gga and gga.fix_quality == 5:
            return "FLOAT"
        if gga and gga.fix_quality > 0:
            return "GNSS"
        return "NO_FIX"

    def fix_name(self) -> str:
        rtk_state = self.rtk_state()
        if rtk_state == "FIXED":
            return "RTK_FIXED"
        if rtk_state == "FLOAT":
            return "RTK_FLOAT"
        gga = self.latest_gga
        return gga.fix_name if gga else "UNKNOWN"

    def current_position_type(self) -> str:
        if self.latest_rtk_status and self.latest_rtk_status.position_type:
            return self.latest_rtk_status.position_type
        if self.latest_bestnav and self.latest_bestnav.position_type:
            return self.latest_bestnav.position_type
        if self.latest_pvtsln and self.latest_pvtsln.best_position_type:
            return self.latest_pvtsln.best_position_type
        if self.latest_gga:
            return self.latest_gga.fix_name
        return "UNKNOWN"

    def has_fixed_solution(self) -> bool:
        return self.rtk_state() == "FIXED"

    def summary(
        self,
        now: Optional[float] = None,
        covariance_timeout_s: float = 2.0,
        covariance_max_std_m: float = 100.0,
    ) -> dict[str, object]:
        now = monotonic() if now is None else now
        gga = self.latest_gga
        rmc = self.latest_rmc
        gsa = self.latest_gsa
        gsv = self.latest_gsv
        position = self.position_solution()
        covariance = self.covariance_solution(
            now,
            timeout_s=covariance_timeout_s,
            max_std_m=covariance_max_std_m,
        )
        gsv_snrs = [sv.snr_db for sv in gsv.satellites if sv.snr_db is not None] if gsv else []
        return {
            "fix": self.fix_name(),
            "fix_quality": gga.fix_quality if gga else None,
            "rtk_state": self.rtk_state(),
            "position_type": self.current_position_type(),
            "solution_status": position["solution_status"],
            "position_source": position["source"],
            "covariance_source": covariance["source"],
            "covariance_age_s": covariance["age_s"],
            "east_std_m": covariance["east_std_m"],
            "north_std_m": covariance["north_std_m"],
            "up_std_m": covariance["up_std_m"],
            "differential_age_s": position["differential_age_s"],
            "satellites": gga.num_satellites if gga else None,
            "hdop": gga.hdop if gga else None,
            "gsa_fix_type": gsa.fix_type if gsa else None,
            "gsa_satellites_used": len(gsa.satellites) if gsa else None,
            "gsv_satellites_in_view": gsv.satellites_in_view if gsv else None,
            "gsv_satellites_reported": len(gsv.satellites) if gsv else None,
            "gsv_max_snr_db": max(gsv_snrs) if gsv_snrs else None,
            "lat": position["lat"],
            "lon": position["lon"],
            "altitude_msl_m": position["altitude_msl_m"],
            "altitude_ellipsoid_m": position["altitude_ellipsoid_m"],
            "speed_mps": rmc.speed_mps if rmc else None,
            "course_deg": rmc.course_deg if rmc else None,
            "rates": self.rates.rates(now),
            "solution_time_rate_hz": self.solution_rates.rate("GGA_UTC", now),
            "rtcm_hz": self.rtcm_rates.rate_hz(now),
            "rtcm_bytes_per_sec": self.rtcm_rates.bytes_per_sec(now),
            "rtcm_age_s": self.rtcm_rates.age_s(now),
            "ntrip_connected": self.ntrip_connected,
            "ntrip_last_error": self.ntrip_last_error,
            "ntrip_gga_age_s": self.ntrip_gga_age_s,
            "rtcm_status_message_id": self.latest_rtcm_status.message_id if self.latest_rtcm_status else None,
            "rtcm_status_message_count": self.latest_rtcm_status.message_count if self.latest_rtcm_status else None,
            "rtcm_status_base_station_id": self.latest_rtcm_status.base_station_id if self.latest_rtcm_status else None,
            "rtcm_message_counts": dict(sorted(self.rtcm_message_counts.items())),
            "checksum_error_count": self.checksum_error_count,
            "parse_drop_count": self.parse_drop_count,
            "unknown_line_count": self.unknown_line_count,
            "serial_line_count": self.serial_line_count,
            "last_error": self.last_error,
            "last_error_line": self.last_error_line,
            "last_unknown_sentence_type": self.last_unknown_sentence_type,
        }


def ellipsoid_altitude(msl_m: Optional[float], undulation_m: Optional[float]) -> Optional[float]:
    if msl_m is None:
        return None
    if undulation_m is None:
        return msl_m
    return msl_m + undulation_m


def age_s(observed_at: Optional[float], now: float) -> Optional[float]:
    if observed_at is None:
        return None
    return now - observed_at


def sane_std_triplet(
    first: Optional[float],
    second: Optional[float],
    third: Optional[float],
    max_std_m: float,
) -> bool:
    for value in (first, second, third):
        if value is None or not math.isfinite(value) or value < 0.0 or value > max_std_m:
            return False
    return True


def unknown_sentence_type(line: str) -> str:
    line = line.strip()
    if not line:
        return ""
    if line.startswith("#"):
        header = line[1:].split(";", 1)[0]
        return header.split(",", 1)[0].upper()
    if line.startswith("$"):
        payload = line[1:].split("*", 1)[0]
        return payload.split(",", 1)[0].upper()
    return "BINARY_OR_TEXT"
