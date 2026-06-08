"""Serial transport and UM980 command helpers."""

from __future__ import annotations

import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import serial
from serial.tools import list_ports

BAUD_CANDIDATES = (460800, 115200, 921600, 230400, 57600, 38400, 9600)


@dataclass(frozen=True)
class SerialDevice:
    path: str
    description: str
    hwid: str


@dataclass(frozen=True)
class CommandResult:
    command: str
    lines: list[str]
    ok: bool
    timed_out: bool = False

    @property
    def error(self) -> Optional[str]:
        for line in self.lines:
            if "PARSING FAILED" in line or "ERROR" in line.upper():
                return line
        if self.timed_out:
            return "timed out waiting for command response"
        return None


@dataclass(frozen=True)
class BaudDetectionResult:
    baud: int
    evidence: list[str]


def scan_serial_devices() -> list[SerialDevice]:
    seen: set[str] = set()
    devices: list[SerialDevice] = []
    for port in list_ports.comports():
        seen.add(port.device)
        devices.append(SerialDevice(port.device, port.description or "", port.hwid or ""))
    for pattern in ("/dev/serial/by-id/*", "/dev/rtk*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in glob.glob(pattern):
            if path not in seen:
                seen.add(path)
                devices.append(SerialDevice(path, "serial candidate", ""))
    return sorted(devices, key=lambda item: item.path)


def select_default_port() -> Optional[str]:
    devices = scan_serial_devices()
    if not devices:
        return None
    preferred_prefixes = ("/dev/serial/by-id", "/dev/rtk", "/dev/ttyUSB", "/dev/ttyACM")
    for prefix in preferred_prefixes:
        for device in devices:
            if device.path.startswith(prefix):
                return device.path
    return devices[0].path


def detect_baud(
    port: str,
    candidates: Iterable[int] = BAUD_CANDIDATES,
    passive_window_s: float = 2.0,
    query_timeout_s: float = 1.0,
) -> BaudDetectionResult:
    for baud in candidates:
        evidence = _probe_baud(port, int(baud), passive_window_s, query_timeout_s)
        if evidence:
            return BaudDetectionResult(baud=int(baud), evidence=evidence)
    candidate_list = ", ".join(str(item) for item in candidates)
    raise RuntimeError(f"could not detect UM980 baud on {port}; tried {candidate_list}")


def _probe_baud(port: str, baud: int, passive_window_s: float, query_timeout_s: float) -> list[str]:
    evidence: list[str] = []
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2) as ser:
            deadline = time.monotonic() + passive_window_s
            while time.monotonic() < deadline:
                line = _read_ascii_line(ser)
                if not line:
                    continue
                evidence.append(line)
                if _looks_like_um980_output(line):
                    return evidence
            for command in ("VERSION", "CONFIG"):
                ser.write(command.encode("ascii") + b"\r\n")
                ser.flush()
                deadline = time.monotonic() + query_timeout_s
                while time.monotonic() < deadline:
                    line = _read_ascii_line(ser)
                    if not line:
                        continue
                    evidence.append(line)
                    if _looks_like_um980_output(line):
                        return evidence
    except (OSError, serial.SerialException):
        return []
    return []


def _read_ascii_line(ser: serial.Serial) -> Optional[str]:
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("ascii", errors="replace").strip()


def _looks_like_um980_output(line: str) -> bool:
    if line.startswith("#"):
        return ";" in line
    lower = line.lower()
    if lower.startswith("$command") and "response:" in lower:
        return True
    if line.startswith("$") and len(line) >= 6:
        sentence_id = line[1:].split(",", 1)[0].split("*", 1)[0]
        return len(sentence_id) >= 5
    return False


class UM980SerialClient:
    def __init__(self, port: str, baud: int = 460800, timeout: float = 1.0) -> None:
        self.port = port
        self.baud = baud
        self.auto_baud = baud == 0
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        if self.auto_baud:
            self.baud = detect_baud(self.port).baud
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout, write_timeout=self.timeout)

    def close(self) -> None:
        if self.serial:
            self.serial.close()
        self.serial = None

    def reconnect(self, delay_s: float = 1.0) -> None:
        self.close()
        time.sleep(delay_s)
        self.open()

    def write_line(self, line: str) -> None:
        ser = self._require_serial()
        payload = line.strip().encode("ascii") + b"\r\n"
        ser.write(payload)
        ser.flush()

    def write_rtcm(self, data: bytes) -> None:
        ser = self._require_serial()
        ser.write(data)
        ser.flush()

    def read_line(self) -> Optional[str]:
        ser = self._require_serial()
        raw = ser.readline()
        if not raw:
            return None
        return raw.decode("ascii", errors="replace").strip()

    def read_lines(self) -> Iterable[str]:
        while self.is_open:
            line = self.read_line()
            if line:
                yield line

    def send_command(self, command: str, timeout: float = 2.0, expect_response: bool = True) -> CommandResult:
        self.write_line(command)
        if not expect_response:
            return CommandResult(command=command, lines=[], ok=True)

        deadline = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self.read_line()
            if not line:
                continue
            lines.append(line)
            lower = line.lower()
            if lower.startswith("$command") and "response:" in lower:
                ok = "response: ok" in lower
                return CommandResult(command=command, lines=lines, ok=ok)
        return CommandResult(command=command, lines=lines, ok=False, timed_out=True)

    def apply_commands(
        self,
        commands: Iterable[str],
        timeout: float = 2.0,
        optional_commands: Iterable[str] = (),
        fallbacks: dict[str, str] | None = None,
        reconnect_after_commands: Iterable[str] = (),
        reconnect_delay_s: float = 2.0,
    ) -> list[CommandResult]:
        optional = {command.upper() for command in optional_commands}
        fallback_map = {command.upper(): fallback for command, fallback in (fallbacks or {}).items()}
        reconnect_after = {command.upper() for command in reconnect_after_commands}
        results: list[CommandResult] = []
        for command in commands:
            result = self.send_command(command, timeout=timeout)
            results.append(result)
            command_key = command.upper()
            if command_key in reconnect_after:
                self.reconnect(delay_s=reconnect_delay_s)
                continue
            fallback = fallback_map.get(command_key)
            if not result.ok and fallback:
                fallback_result = self.send_command(fallback, timeout=timeout)
                results.append(fallback_result)
                if fallback_result.ok:
                    continue
            if not result.ok and command_key not in optional and command_key != "CONFIG SIGNALGROUP 8":
                break
        return results

    def capture_snapshot(self, snapshot_dir: str | Path, label: str) -> list[Path]:
        snapshot_dir = Path(snapshot_dir)
        snapshot_dir.mkdir(parents=True, exist_ok=True)
        created: list[Path] = []
        for command in ("VERSION", "CONFIG", "MODE", "UNILOGLIST"):
            result = self.send_command(command, timeout=3.0)
            path = snapshot_dir / f"{label}_{command.lower()}.txt"
            path.write_text("\n".join(result.lines) + "\n", encoding="utf-8")
            created.append(path)
        return created

    def _require_serial(self) -> serial.Serial:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("serial port is not open")
        return self.serial
