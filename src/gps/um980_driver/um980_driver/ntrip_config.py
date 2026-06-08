"""Private NTRIP YAML validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class NtripConfig:
    host: str
    port: int
    mountpoint: str
    authenticate: bool
    username: str
    password: str
    ntrip_version: str | None = None
    ssl: bool = False
    reconnect_wait_s: float = 5.0
    rtcm_timeout_s: float = 4.0


def load_private_yaml(path: str | Path) -> NtripConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    return validate_ntrip_config(raw)


def validate_ntrip_config(raw: dict[str, Any]) -> NtripConfig:
    required = ("host", "port", "mountpoint", "authenticate", "username", "password")
    missing = [key for key in required if key not in raw or raw[key] in (None, "")]
    if missing:
        raise ValueError(f"missing NTRIP private YAML fields: {', '.join(missing)}")
    try:
        port = int(raw["port"])
    except (TypeError, ValueError) as exc:
        raise ValueError("NTRIP port must be an integer") from exc
    if port <= 0 or port > 65535:
        raise ValueError("NTRIP port must be between 1 and 65535")

    reconnect_wait_s = float(raw.get("reconnect_wait_s", 5.0))
    rtcm_timeout_s = float(raw.get("rtcm_timeout_s", 4.0))
    if reconnect_wait_s <= 0.0:
        raise ValueError("NTRIP reconnect_wait_s must be positive")
    if rtcm_timeout_s <= 0.0:
        raise ValueError("NTRIP rtcm_timeout_s must be positive")

    return NtripConfig(
        host=str(raw["host"]),
        port=port,
        mountpoint=str(raw["mountpoint"]),
        authenticate=bool(raw["authenticate"]),
        username=str(raw["username"]),
        password=str(raw["password"]),
        ntrip_version=None if str(raw.get("ntrip_version", "None")) == "None" else str(raw.get("ntrip_version")),
        ssl=bool(raw.get("ssl", False)),
        reconnect_wait_s=reconnect_wait_s,
        rtcm_timeout_s=rtcm_timeout_s,
    )
