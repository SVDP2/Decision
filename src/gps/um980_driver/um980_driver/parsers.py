"""Typed parsers for UM980 NMEA sentences."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Union

from .protocol import (
    NMEASentence,
    UnicoreSentence,
    coordinate_to_decimal,
    parse_float,
    parse_int,
    parse_int_auto,
    parse_sentence,
    parse_unicore_sentence,
)


FIX_QUALITY_NAMES = {
    0: "NO_FIX",
    1: "SINGLE",
    2: "DGPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
}


@dataclass(frozen=True)
class GGA:
    raw: str
    utc: Optional[str]
    lat: Optional[float]
    lon: Optional[float]
    fix_quality: int
    fix_name: str
    num_satellites: Optional[int]
    hdop: Optional[float]
    altitude_msl_m: Optional[float]
    geoid_sep_m: Optional[float]
    differential_age_s: Optional[float]
    station_id: Optional[str]


@dataclass(frozen=True)
class RMC:
    raw: str
    utc: Optional[str]
    valid: bool
    lat: Optional[float]
    lon: Optional[float]
    speed_mps: Optional[float]
    course_deg: Optional[float]
    date: Optional[str]


@dataclass(frozen=True)
class GSA:
    raw: str
    mode: Optional[str]
    fix_type: Optional[int]
    satellites: list[str]
    pdop: Optional[float]
    hdop: Optional[float]
    vdop: Optional[float]


@dataclass(frozen=True)
class GSVSatellite:
    prn: str
    elevation_deg: Optional[int]
    azimuth_deg: Optional[int]
    snr_db: Optional[int]


@dataclass(frozen=True)
class GSV:
    raw: str
    total_messages: Optional[int]
    message_number: Optional[int]
    satellites_in_view: Optional[int]
    satellites: list[GSVSatellite]


@dataclass(frozen=True)
class GST:
    raw: str
    utc: Optional[str]
    rms_m: Optional[float]
    semi_major_std_m: Optional[float]
    semi_minor_std_m: Optional[float]
    orientation_deg: Optional[float]
    lat_std_m: Optional[float]
    lon_std_m: Optional[float]
    alt_std_m: Optional[float]


@dataclass(frozen=True)
class BestNav:
    raw: str
    header_fields: list[str]
    solution_status: str
    position_type: str
    lat: Optional[float]
    lon: Optional[float]
    altitude_msl_m: Optional[float]
    undulation_m: Optional[float]
    datum: Optional[str]
    lat_std_m: Optional[float]
    lon_std_m: Optional[float]
    altitude_std_m: Optional[float]
    base_station_id: Optional[str]
    differential_age_s: Optional[float]
    solution_age_s: Optional[float]
    satellites_tracked: Optional[int]
    satellites_used: Optional[int]
    velocity_solution_status: Optional[str]
    velocity_type: Optional[str]
    velocity_latency_s: Optional[float]
    velocity_age_s: Optional[float]
    horizontal_speed_mps: Optional[float]
    track_ground_deg: Optional[float]
    vertical_speed_mps: Optional[float]
    horizontal_speed_std_mps: Optional[float]
    vertical_speed_std_mps: Optional[float]
    north_velocity_mps: Optional[float]
    east_velocity_mps: Optional[float]
    up_velocity_mps: Optional[float]


@dataclass(frozen=True)
class PvtSln:
    raw: str
    header_fields: list[str]
    best_position_type: str
    best_altitude_msl_m: Optional[float]
    best_lat: Optional[float]
    best_lon: Optional[float]
    best_altitude_std_m: Optional[float]
    best_lat_std_m: Optional[float]
    best_lon_std_m: Optional[float]
    best_differential_age_s: Optional[float]
    psr_position_type: Optional[str]
    psr_altitude_msl_m: Optional[float]
    psr_lat: Optional[float]
    psr_lon: Optional[float]
    undulation_m: Optional[float]
    best_satellites_tracked: Optional[int]
    best_satellites_used: Optional[int]
    psr_satellites_tracked: Optional[int]
    psr_satellites_used: Optional[int]
    north_velocity_mps: Optional[float]
    east_velocity_mps: Optional[float]
    ground_speed_mps: Optional[float]
    heading_type: Optional[str]
    baseline_m: Optional[float]
    heading_deg: Optional[float]
    pitch_deg: Optional[float]
    heading_satellites_tracked: Optional[int]
    heading_satellites_used: Optional[int]
    heading_l1_satellites: Optional[int]
    heading_l1l2_satellites: Optional[int]
    gdop: Optional[float]
    pdop: Optional[float]
    hdop: Optional[float]


@dataclass(frozen=True)
class RtkStatus:
    raw: str
    header_fields: list[str]
    gps_source: Optional[int]
    bds_source1: Optional[int]
    bds_source2: Optional[int]
    glonass_source: Optional[int]
    galileo_source1: Optional[int]
    galileo_source2: Optional[int]
    qzss_source: Optional[int]
    position_type: Optional[str]
    calculate_status: Optional[str]
    ion_detected: Optional[bool]
    dual_rtk_flag: Optional[bool]
    adr_number: Optional[int]


@dataclass(frozen=True)
class RtcmStatus:
    raw: str
    header_fields: list[str]
    message_id: Optional[int]
    message_count: Optional[int]
    base_station_id: Optional[int]
    satellite_count: Optional[int]
    l1_observation_count: Optional[int]
    l2_observation_count: Optional[int]
    l3_observation_count: Optional[int]
    l4_observation_count: Optional[int]
    l5_observation_count: Optional[int]
    l6_observation_count: Optional[int]


@dataclass(frozen=True)
class UniLogList:
    raw: str
    header_fields: list[str]
    fields: list[str]


ParsedNMEA = Union[GGA, RMC, GSA, GSV, GST]
ParsedUnicore = Union[BestNav, PvtSln, RtkStatus, RtcmStatus, UniLogList]
ParsedGNSS = Union[ParsedNMEA, ParsedUnicore]


def parse_line(line: str) -> Optional[ParsedGNSS]:
    if line.strip().startswith("#"):
        return parse_unicore(parse_unicore_sentence(line))
    sentence = parse_sentence(line)
    if sentence.sentence_type == "GGA":
        return parse_gga(sentence)
    if sentence.sentence_type == "RMC":
        return parse_rmc(sentence)
    if sentence.sentence_type == "GSA":
        return parse_gsa(sentence)
    if sentence.sentence_type == "GSV":
        return parse_gsv(sentence)
    if sentence.sentence_type == "GST":
        return parse_gst(sentence)
    return None


def _field(fields: list[str], index: int) -> str:
    return fields[index] if index < len(fields) else ""


def parse_gga(sentence: NMEASentence) -> GGA:
    fields = sentence.fields
    fix_quality = parse_int(_field(fields, 5)) or 0
    return GGA(
        raw=sentence.raw,
        utc=_field(fields, 0) or None,
        lat=coordinate_to_decimal(_field(fields, 1), _field(fields, 2)),
        lon=coordinate_to_decimal(_field(fields, 3), _field(fields, 4)),
        fix_quality=fix_quality,
        fix_name=FIX_QUALITY_NAMES.get(fix_quality, f"FIX_{fix_quality}"),
        num_satellites=parse_int(_field(fields, 6)),
        hdop=parse_float(_field(fields, 7)),
        altitude_msl_m=parse_float(_field(fields, 8)),
        geoid_sep_m=parse_float(_field(fields, 10)),
        differential_age_s=parse_float(_field(fields, 12)),
        station_id=_field(fields, 13) or None,
    )


def parse_rmc(sentence: NMEASentence) -> RMC:
    fields = sentence.fields
    speed_knots = parse_float(_field(fields, 6))
    speed_mps = None if speed_knots is None else speed_knots * 0.514444
    return RMC(
        raw=sentence.raw,
        utc=_field(fields, 0) or None,
        valid=_field(fields, 1).upper() == "A",
        lat=coordinate_to_decimal(_field(fields, 2), _field(fields, 3)),
        lon=coordinate_to_decimal(_field(fields, 4), _field(fields, 5)),
        speed_mps=speed_mps,
        course_deg=parse_float(_field(fields, 7)),
        date=_field(fields, 8) or None,
    )


def parse_gsa(sentence: NMEASentence) -> GSA:
    fields = sentence.fields
    satellites = [sv for sv in fields[2:14] if sv]
    return GSA(
        raw=sentence.raw,
        mode=_field(fields, 0) or None,
        fix_type=parse_int(_field(fields, 1)),
        satellites=satellites,
        pdop=parse_float(_field(fields, 14)),
        hdop=parse_float(_field(fields, 15)),
        vdop=parse_float(_field(fields, 16)),
    )


def parse_gsv(sentence: NMEASentence) -> GSV:
    fields = sentence.fields
    satellites: list[GSVSatellite] = []
    for index in range(3, len(fields), 4):
        prn = _field(fields, index)
        if not prn:
            continue
        satellites.append(
            GSVSatellite(
                prn=prn,
                elevation_deg=parse_int(_field(fields, index + 1)),
                azimuth_deg=parse_int(_field(fields, index + 2)),
                snr_db=parse_int(_field(fields, index + 3)),
            )
        )
    return GSV(
        raw=sentence.raw,
        total_messages=parse_int(_field(fields, 0)),
        message_number=parse_int(_field(fields, 1)),
        satellites_in_view=parse_int(_field(fields, 2)),
        satellites=satellites,
    )


def parse_gst(sentence: NMEASentence) -> GST:
    fields = sentence.fields
    return GST(
        raw=sentence.raw,
        utc=_field(fields, 0) or None,
        rms_m=parse_float(_field(fields, 1)),
        semi_major_std_m=parse_float(_field(fields, 2)),
        semi_minor_std_m=parse_float(_field(fields, 3)),
        orientation_deg=parse_float(_field(fields, 4)),
        lat_std_m=parse_float(_field(fields, 5)),
        lon_std_m=parse_float(_field(fields, 6)),
        alt_std_m=parse_float(_field(fields, 7)),
    )


def parse_unicore(sentence: UnicoreSentence) -> Optional[ParsedUnicore]:
    log_type = sentence.log_type
    if log_type == "BESTNAVA":
        return parse_bestnav(sentence)
    if log_type == "PVTSLNA":
        return parse_pvtsln(sentence)
    if log_type == "RTKSTATUSA":
        return parse_rtkstatus(sentence)
    if log_type == "RTCMSTATUSA":
        return parse_rtcmstatus(sentence)
    if log_type == "UNILOGLISTA":
        return UniLogList(raw=sentence.raw, header_fields=sentence.header_fields, fields=sentence.body_fields)
    return None


def parse_bestnav(sentence: UnicoreSentence) -> BestNav:
    fields = sentence.body_fields
    horizontal_speed_mps = parse_float(_field(fields, 25))
    track_ground_deg = parse_float(_field(fields, 26))
    vertical_speed_mps = parse_float(_field(fields, 27))
    north_velocity_mps, east_velocity_mps = _track_speed_to_ne(horizontal_speed_mps, track_ground_deg)
    return BestNav(
        raw=sentence.raw,
        header_fields=sentence.header_fields,
        solution_status=_field(fields, 0),
        position_type=_field(fields, 1),
        lat=parse_float(_field(fields, 2)),
        lon=parse_float(_field(fields, 3)),
        altitude_msl_m=parse_float(_field(fields, 4)),
        undulation_m=parse_float(_field(fields, 5)),
        datum=_field(fields, 6) or None,
        lat_std_m=parse_float(_field(fields, 7)),
        lon_std_m=parse_float(_field(fields, 8)),
        altitude_std_m=parse_float(_field(fields, 9)),
        base_station_id=_field(fields, 10) or None,
        differential_age_s=parse_float(_field(fields, 11)),
        solution_age_s=parse_float(_field(fields, 12)),
        satellites_tracked=parse_int(_field(fields, 13)),
        satellites_used=parse_int(_field(fields, 14)),
        velocity_solution_status=_field(fields, 21) or None,
        velocity_type=_field(fields, 22) or None,
        velocity_latency_s=parse_float(_field(fields, 23)),
        velocity_age_s=parse_float(_field(fields, 24)),
        horizontal_speed_mps=horizontal_speed_mps,
        track_ground_deg=track_ground_deg,
        vertical_speed_mps=vertical_speed_mps,
        vertical_speed_std_mps=parse_float(_field(fields, 28)),
        horizontal_speed_std_mps=parse_float(_field(fields, 29)),
        north_velocity_mps=north_velocity_mps,
        east_velocity_mps=east_velocity_mps,
        up_velocity_mps=vertical_speed_mps,
    )


def parse_pvtsln(sentence: UnicoreSentence) -> PvtSln:
    fields = sentence.body_fields
    return PvtSln(
        raw=sentence.raw,
        header_fields=sentence.header_fields,
        best_position_type=_field(fields, 0),
        best_altitude_msl_m=parse_float(_field(fields, 1)),
        best_lat=parse_float(_field(fields, 2)),
        best_lon=parse_float(_field(fields, 3)),
        best_altitude_std_m=parse_float(_field(fields, 4)),
        best_lat_std_m=parse_float(_field(fields, 5)),
        best_lon_std_m=parse_float(_field(fields, 6)),
        best_differential_age_s=parse_float(_field(fields, 7)),
        psr_position_type=_field(fields, 8) or None,
        psr_altitude_msl_m=parse_float(_field(fields, 9)),
        psr_lat=parse_float(_field(fields, 10)),
        psr_lon=parse_float(_field(fields, 11)),
        undulation_m=parse_float(_field(fields, 12)),
        best_satellites_tracked=parse_int(_field(fields, 13)),
        best_satellites_used=parse_int(_field(fields, 14)),
        psr_satellites_tracked=parse_int(_field(fields, 15)),
        psr_satellites_used=parse_int(_field(fields, 16)),
        north_velocity_mps=parse_float(_field(fields, 17)),
        east_velocity_mps=parse_float(_field(fields, 18)),
        ground_speed_mps=parse_float(_field(fields, 19)),
        heading_type=_field(fields, 20) or None,
        baseline_m=parse_float(_field(fields, 21)),
        heading_deg=parse_float(_field(fields, 22)),
        pitch_deg=parse_float(_field(fields, 23)),
        heading_satellites_tracked=parse_int(_field(fields, 24)),
        heading_satellites_used=parse_int(_field(fields, 25)),
        heading_l1_satellites=parse_int(_field(fields, 26)),
        heading_l1l2_satellites=parse_int(_field(fields, 27)),
        gdop=parse_float(_field(fields, 28)),
        pdop=parse_float(_field(fields, 29)),
        hdop=parse_float(_field(fields, 30)),
    )


def _track_speed_to_ne(
    horizontal_speed_mps: Optional[float],
    track_ground_deg: Optional[float],
) -> tuple[Optional[float], Optional[float]]:
    if horizontal_speed_mps is None or track_ground_deg is None:
        return None, None
    track = math.radians(track_ground_deg)
    return horizontal_speed_mps * math.cos(track), horizontal_speed_mps * math.sin(track)


def parse_rtkstatus(sentence: UnicoreSentence) -> RtkStatus:
    fields = sentence.body_fields
    return RtkStatus(
        raw=sentence.raw,
        header_fields=sentence.header_fields,
        gps_source=parse_int_auto(_field(fields, 0)),
        bds_source1=parse_int_auto(_field(fields, 2)),
        bds_source2=parse_int_auto(_field(fields, 3)),
        glonass_source=parse_int_auto(_field(fields, 5)),
        galileo_source1=parse_int_auto(_field(fields, 7)),
        galileo_source2=parse_int_auto(_field(fields, 8)),
        qzss_source=parse_int_auto(_field(fields, 9)),
        position_type=_field(fields, 11) or None,
        calculate_status=_field(fields, 12) or None,
        ion_detected=parse_bool_int(_field(fields, 13)),
        dual_rtk_flag=parse_bool_int(_field(fields, 14)),
        adr_number=parse_int_auto(_field(fields, 15)),
    )


def parse_rtcmstatus(sentence: UnicoreSentence) -> RtcmStatus:
    fields = sentence.body_fields
    return RtcmStatus(
        raw=sentence.raw,
        header_fields=sentence.header_fields,
        message_id=parse_int_auto(_field(fields, 0)),
        message_count=parse_int_auto(_field(fields, 1)),
        base_station_id=parse_int_auto(_field(fields, 2)),
        satellite_count=parse_int_auto(_field(fields, 3)),
        l1_observation_count=parse_int_auto(_field(fields, 4)),
        l2_observation_count=parse_int_auto(_field(fields, 5)),
        l3_observation_count=parse_int_auto(_field(fields, 6)),
        l4_observation_count=parse_int_auto(_field(fields, 7)),
        l5_observation_count=parse_int_auto(_field(fields, 8)),
        l6_observation_count=parse_int_auto(_field(fields, 9)),
    )


def parse_bool_int(value: str) -> Optional[bool]:
    parsed = parse_int_auto(value)
    if parsed is None:
        return None
    return parsed != 0
