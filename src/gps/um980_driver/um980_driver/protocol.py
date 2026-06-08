"""NMEA protocol helpers for UM980 receiver output."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class NMEASentence:
    raw: str
    talker: str
    sentence_type: str
    fields: list[str]


@dataclass(frozen=True)
class UnicoreSentence:
    raw: str
    log_type: str
    header_fields: list[str]
    body_fields: list[str]
    checksum: str


@dataclass(frozen=True)
class RTCM3Message:
    raw: bytes
    message_id: int
    payload_length: int
    station_id: Optional[int]


class RTCM3StreamParser:
    def __init__(self, max_buffer_size: int = 10240) -> None:
        self.max_buffer_size = max_buffer_size
        self._buffer = b""

    def parse(self, data: bytes) -> list[RTCM3Message]:
        self._buffer += data
        messages: list[RTCM3Message] = []
        while True:
            start = self._buffer.find(b"\xd3")
            if start < 0:
                self._trim_buffer()
                return messages
            if start:
                self._buffer = self._buffer[start:]
            if len(self._buffer) < 6:
                return messages
            length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            packet_length = length + 6
            if len(self._buffer) < packet_length:
                self._trim_buffer()
                return messages
            packet = self._buffer[:packet_length]
            self._buffer = self._buffer[packet_length:]
            expected_crc = (packet[-3] << 16) | (packet[-2] << 8) | packet[-1]
            if rtcm3_crc24q(packet[:-3]) != expected_crc:
                continue
            message_id = rtcm3_message_id(packet)
            if message_id is None:
                continue
            messages.append(
                RTCM3Message(
                    raw=packet,
                    message_id=message_id,
                    payload_length=length,
                    station_id=rtcm3_station_id(packet),
                )
            )

    def _trim_buffer(self) -> None:
        if len(self._buffer) > self.max_buffer_size:
            self._buffer = self._buffer[-self.max_buffer_size :]


def normalize_line(line: str) -> str:
    return line.strip().replace("\x00", "")


def checksum_payload(payload: str) -> int:
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return checksum


def checksum_hex(payload: str) -> str:
    return f"{checksum_payload(payload):02X}"


def append_checksum(payload: str) -> str:
    payload = payload[1:] if payload.startswith("$") else payload
    payload = payload.split("*", 1)[0]
    return f"${payload}*{checksum_hex(payload)}"


def split_payload(line: str) -> tuple[str, str]:
    line = normalize_line(line)
    if not line.startswith("$") or "*" not in line:
        raise ValueError("not a checksummed NMEA sentence")
    payload, checksum = line[1:].split("*", 1)
    return payload, checksum[:2]


def validate_checksum(line: str) -> bool:
    try:
        payload, expected = split_payload(line)
    except ValueError:
        return False
    return checksum_hex(payload) == expected.upper()


def parse_sentence(line: str, require_checksum: bool = True) -> NMEASentence:
    line = normalize_line(line)
    if require_checksum and not validate_checksum(line):
        raise ValueError("invalid NMEA checksum")
    payload = line[1:].split("*", 1)[0] if line.startswith("$") else line
    parts = payload.split(",")
    if not parts or len(parts[0]) < 5:
        raise ValueError("invalid NMEA sentence id")
    talker_type = parts[0]
    return NMEASentence(
        raw=line,
        talker=talker_type[:2],
        sentence_type=talker_type[-3:],
        fields=parts[1:],
    )


def split_unicore_payload(line: str) -> tuple[str, str]:
    line = normalize_line(line)
    if not line.startswith("#") or ";" not in line or "*" not in line:
        raise ValueError("not a checksummed Unicore sentence")
    content, checksum = line[1:].split("*", 1)
    return content, checksum[:8]


def unicore_crc32_hex(content: str) -> str:
    return f"{unicore_crc32_payload(content):08x}"


def unicore_crc32_payload(content: str) -> int:
    crc = 0
    for byte in content.encode("ascii"):
        temp1 = (crc >> 8) & 0x00FFFFFF
        temp2 = _crc32_value((crc ^ byte) & 0xFF)
        crc = temp1 ^ temp2
    return crc & 0xFFFFFFFF


def _crc32_value(value: int) -> int:
    crc = value
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xEDB88320
        else:
            crc >>= 1
    return crc & 0xFFFFFFFF


def rtcm3_crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc


def rtcm3_message_id(packet: bytes) -> Optional[int]:
    if len(packet) < 5 or packet[0] != 0xD3:
        return None
    return (packet[3] << 4) | (packet[4] >> 4)


def rtcm3_station_id(packet: bytes) -> Optional[int]:
    if len(packet) < 6 or packet[0] != 0xD3:
        return None
    return ((packet[4] & 0x0F) << 8) | packet[5]


def validate_unicore_checksum(line: str) -> bool:
    try:
        content, expected = split_unicore_payload(line)
    except ValueError:
        return False
    return unicore_crc32_hex(content) == expected.lower()


def parse_unicore_sentence(line: str, require_checksum: bool = True) -> UnicoreSentence:
    line = normalize_line(line)
    if require_checksum and not validate_unicore_checksum(line):
        raise ValueError("invalid Unicore checksum")
    content, checksum = split_unicore_payload(line)
    header, body = content.split(";", 1)
    header_fields = header.split(",")
    if not header_fields or not header_fields[0]:
        raise ValueError("invalid Unicore log header")
    return UnicoreSentence(
        raw=line,
        log_type=header_fields[0].upper(),
        header_fields=header_fields,
        body_fields=_split_csv_fields(body),
        checksum=checksum.lower(),
    )


def _split_csv_fields(payload: str) -> list[str]:
    fields: list[str] = []
    current: list[str] = []
    in_quote = False
    for char in payload:
        if char == '"':
            in_quote = not in_quote
            continue
        if char == "," and not in_quote:
            fields.append("".join(current))
            current = []
            continue
        current.append(char)
    fields.append("".join(current))
    return fields


def parse_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_int(value: str) -> Optional[int]:
    if value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def parse_int_auto(value: str) -> Optional[int]:
    if value == "":
        return None
    try:
        return int(value, 0)
    except ValueError:
        try:
            return int(value, 16)
        except ValueError:
            return None


def coordinate_to_decimal(value: str, hemisphere: str) -> Optional[float]:
    if not value or not hemisphere:
        return None
    try:
        dot_index = value.index(".")
        degree_digits = dot_index - 2
        degrees = int(value[:degree_digits])
        minutes = float(value[degree_digits:])
    except (ValueError, IndexError):
        return None

    decimal = degrees + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        decimal *= -1.0
    return decimal


def is_gga_sentence(line: str) -> bool:
    try:
        sentence = parse_sentence(line)
    except ValueError:
        return False
    return sentence.sentence_type == "GGA"
