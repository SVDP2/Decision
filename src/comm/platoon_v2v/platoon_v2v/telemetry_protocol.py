from dataclasses import dataclass
import struct
from typing import Iterable


HEADER = b'\xAA\x56'
PACKET_TYPE_LEADER_TELEMETRY = 0x01
VERSION = 1

MODE_UNKNOWN = 0
MODE_MANUAL_RC = 1
MODE_AUTONOMOUS = 2
MODE_ESTOP = 3

SOURCE_UNKNOWN = 0
SOURCE_RC = 1
SOURCE_AUTONOMOUS_CMD = 2
SOURCE_OUTPUT_ESTIMATE = 3
SOURCE_SIMULATED = 4

FLAG_RC_VALID = 1 << 0
FLAG_RC_FAILSAFE = 1 << 1
FLAG_AUTO_VALID = 1 << 2
FLAG_AUTO_FAILSAFE = 1 << 3
FLAG_STOP_REQUIRED = 1 << 4

# Little-endian, fixed payload for Arduino Mega and Python parser.
PAYLOAD_STRUCT = struct.Struct('<IBBBBHHHfffffffffIII')
PACKET_OVERHEAD_SIZE = 2 + 1 + 1 + 1 + 1
PACKET_SIZE = PACKET_OVERHEAD_SIZE + PAYLOAD_STRUCT.size


@dataclass(frozen=True)
class LeaderTelemetryFrame:
    seq: int
    boot_id: int
    drive_mode: int
    source: int
    flags: int
    rc_throttle_us: int
    rc_steer_us: int
    rc_mode_us: int
    throttle_norm: float
    throttle_cmd_pwm: float
    throttle_output_pwm: float
    steering_target_adc: float
    steering_current_adc: float
    steering_angle_rad: float
    speed_estimate_mps: float
    auto_speed_cmd_mps: float
    auto_steering_cmd_rad: float
    arduino_time_ms: int
    bridge_rx_ok_count: int
    bridge_crc_fail_count: int

    @property
    def rc_valid(self) -> bool:
        return bool(self.flags & FLAG_RC_VALID)

    @property
    def rc_failsafe(self) -> bool:
        return bool(self.flags & FLAG_RC_FAILSAFE)

    @property
    def auto_valid(self) -> bool:
        return bool(self.flags & FLAG_AUTO_VALID)

    @property
    def auto_failsafe(self) -> bool:
        return bool(self.flags & FLAG_AUTO_FAILSAFE)

    @property
    def stop_required(self) -> bool:
        return bool(self.flags & FLAG_STOP_REQUIRED)


def xor_crc(data: Iterable[int]) -> int:
    crc = 0
    for byte in data:
        crc ^= int(byte) & 0xFF
    return crc


def pack_leader_telemetry(frame: LeaderTelemetryFrame) -> bytes:
    payload = PAYLOAD_STRUCT.pack(
        int(frame.seq) & 0xFFFFFFFF,
        int(frame.boot_id) & 0xFF,
        int(frame.drive_mode) & 0xFF,
        int(frame.source) & 0xFF,
        int(frame.flags) & 0xFF,
        int(frame.rc_throttle_us) & 0xFFFF,
        int(frame.rc_steer_us) & 0xFFFF,
        int(frame.rc_mode_us) & 0xFFFF,
        float(frame.throttle_norm),
        float(frame.throttle_cmd_pwm),
        float(frame.throttle_output_pwm),
        float(frame.steering_target_adc),
        float(frame.steering_current_adc),
        float(frame.steering_angle_rad),
        float(frame.speed_estimate_mps),
        float(frame.auto_speed_cmd_mps),
        float(frame.auto_steering_cmd_rad),
        int(frame.arduino_time_ms) & 0xFFFFFFFF,
        int(frame.bridge_rx_ok_count) & 0xFFFFFFFF,
        int(frame.bridge_crc_fail_count) & 0xFFFFFFFF,
    )
    prefix = bytes([
        *HEADER,
        PACKET_TYPE_LEADER_TELEMETRY,
        VERSION,
        len(payload),
    ])
    crc_input = prefix[2:] + payload
    return prefix + payload + bytes([xor_crc(crc_input)])


def unpack_leader_telemetry(packet: bytes) -> LeaderTelemetryFrame:
    if len(packet) != PACKET_SIZE:
        raise ValueError(f'expected {PACKET_SIZE} bytes, got {len(packet)}')
    if packet[:2] != HEADER:
        raise ValueError('invalid telemetry header')
    if packet[2] != PACKET_TYPE_LEADER_TELEMETRY:
        raise ValueError(f'unsupported telemetry packet type {packet[2]}')
    if packet[3] != VERSION:
        raise ValueError(f'unsupported telemetry version {packet[3]}')
    if packet[4] != PAYLOAD_STRUCT.size:
        raise ValueError(f'invalid telemetry payload length {packet[4]}')
    expected_crc = xor_crc(packet[2:-1])
    if packet[-1] != expected_crc:
        raise ValueError('telemetry crc mismatch')

    values = PAYLOAD_STRUCT.unpack(packet[5:-1])
    return LeaderTelemetryFrame(*values)


class LeaderTelemetryStreamParser:

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.sync_drop_count = 0
        self.crc_fail_count = 0
        self.unsupported_count = 0

    def feed(self, data: bytes) -> list[LeaderTelemetryFrame]:
        self.buffer.extend(data)
        frames: list[LeaderTelemetryFrame] = []
        while len(self.buffer) >= PACKET_SIZE:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                self.sync_drop_count += len(self.buffer)
                self.buffer.clear()
                break
            if header_index > 0:
                self.sync_drop_count += header_index
                del self.buffer[:header_index]
            if len(self.buffer) < PACKET_SIZE:
                break
            candidate = bytes(self.buffer[:PACKET_SIZE])
            try:
                frames.append(unpack_leader_telemetry(candidate))
                del self.buffer[:PACKET_SIZE]
            except ValueError as exc:
                message = str(exc)
                if 'crc' in message:
                    self.crc_fail_count += 1
                else:
                    self.unsupported_count += 1
                del self.buffer[:1]
        return frames
