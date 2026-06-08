"""UM980 receiver configuration profiles."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class UM980Profile:
    name: str
    description: str
    signalgroup: int
    commands: tuple[str, ...]
    optional_commands: tuple[str, ...] = ()
    fallbacks: tuple[tuple[str, str], ...] = ()
    may_reboot: bool = False

    def commands_with_save(self, save: bool = False) -> list[str]:
        commands = list(self.commands)
        if save:
            commands.append("SAVECONFIG")
        return commands

    def is_optional(self, command: str) -> bool:
        return command.upper() in {item.upper() for item in self.optional_commands}

    def fallback_for(self, command: str) -> str | None:
        fallbacks = {source.upper(): fallback for source, fallback in self.fallbacks}
        return fallbacks.get(command.upper())


PROFILES = {
    "minimal_20hz": UM980Profile(
        name="minimal_20hz",
        description="Stable mode: SIGNALGROUP 1, GGA/RMC 20 Hz, GSA/GSV 1 Hz",
        signalgroup=1,
        commands=(
            "CONFIG SIGNALGROUP 1",
            "MODE ROVER",
            "UNLOG",
            "GPGGA 0.05",
            "GPRMC 0.05",
            "GPGSA 1",
            "GPGSV 1",
        ),
        optional_commands=("GPGSA 1", "GPGSV 1"),
        fallbacks=(
            ("GPGGA 0.05", "GNGGA 0.05"),
            ("GPRMC 0.05", "GNRMC 0.05"),
            ("GPGSA 1", "GNGSA 1"),
            ("GPGSV 1", "GNGSV 1"),
        ),
    ),
    "survey_20hz": UM980Profile(
        name="survey_20hz",
        description="RTK survey mode: SIGNALGROUP 1, GGA/RMC/BESTNAV 20 Hz, diagnostics 1 Hz",
        signalgroup=1,
        commands=(
            "CONFIG SIGNALGROUP 1",
            "MODE ROVER",
            "UNLOG",
            "GPGGA 0.05",
            "GPRMC 0.05",
            "GPGST 1",
            "GPGSA 1",
            "GPGSV 1",
            "BESTNAVA 0.05",
            "PVTSLNA 1",
            "RTKSTATUSA 1",
            "RTCMSTATUSA ONCHANGED",
        ),
        optional_commands=(
            "GPGST 1",
            "GPGSA 1",
            "GPGSV 1",
            "BESTNAVA 0.05",
            "PVTSLNA 1",
            "RTKSTATUSA 1",
            "RTCMSTATUSA ONCHANGED",
        ),
        fallbacks=(
            ("GPGGA 0.05", "GNGGA 0.05"),
            ("GPRMC 0.05", "GNRMC 0.05"),
            ("GPGST 1", "GNGST 1"),
            ("GPGSA 1", "GNGSA 1"),
            ("GPGSV 1", "GNGSV 1"),
        ),
    ),
    "survey_50hz": UM980Profile(
        name="survey_50hz",
        description="Experimental RTK survey mode: SIGNALGROUP 8, GGA/RMC/BESTNAV 50 Hz, diagnostics 1 Hz",
        signalgroup=8,
        commands=(
            "CONFIG SIGNALGROUP 8",
            "MODE ROVER",
            "UNLOG",
            "GPGGA 0.02",
            "GPRMC 0.02",
            "GPGST 1",
            "GPGSA 1",
            "GPGSV 1",
            "BESTNAVA 0.02",
            "PVTSLNA 1",
            "RTKSTATUSA 1",
            "RTCMSTATUSA ONCHANGED",
        ),
        optional_commands=(
            "GPGST 1",
            "GPGSA 1",
            "GPGSV 1",
            "BESTNAVA 0.02",
            "PVTSLNA 1",
            "RTKSTATUSA 1",
            "RTCMSTATUSA ONCHANGED",
        ),
        fallbacks=(
            ("GPGGA 0.02", "GNGGA 0.02"),
            ("GPRMC 0.02", "GNRMC 0.02"),
            ("GPGST 1", "GNGST 1"),
            ("GPGSA 1", "GNGSA 1"),
            ("GPGSV 1", "GNGSV 1"),
        ),
        may_reboot=True,
    ),
}

PROFILES["signalgroup1_20hz"] = PROFILES["minimal_20hz"]
PROFILES["signalgroup8_50hz"] = PROFILES["survey_50hz"]


def get_profile(name: str) -> UM980Profile:
    try:
        return PROFILES[name]
    except KeyError as exc:
        valid = ", ".join(sorted(PROFILES))
        raise ValueError(f"unknown UM980 profile {name!r}; valid profiles: {valid}") from exc
