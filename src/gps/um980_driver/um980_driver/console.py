"""Console bring-up tool for UM980 receivers."""

from __future__ import annotations

import argparse
from time import monotonic

from .parsers import parse_line
from .profiles import get_profile
from .serial_client import UM980SerialClient, select_default_port
from .state import GNSSState


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UM980 serial console")
    parser.add_argument("--port", default=None, help="Serial port, defaults to first detected UM980 candidate")
    parser.add_argument("--baud", type=int, default=0, help="Serial baud rate, or 0 for auto-detect")
    parser.add_argument("--mode", default="signalgroup1_20hz")
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("--save", action="store_true", help="Append explicit SAVECONFIG after profile commands")
    parser.add_argument("--raw", action="store_true")
    parser.add_argument("--snapshot-dir", default=None)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    port = args.port or select_default_port()
    if not port:
        raise SystemExit("no serial device found")

    client = UM980SerialClient(port, args.baud)
    client.open()
    state = GNSSState()
    print(f"[UM980] port={port} baud={client.baud} mode={args.mode}")

    if args.snapshot_dir:
        created = client.capture_snapshot(args.snapshot_dir, "before")
        print("[UM980] before snapshot:", ", ".join(str(path) for path in created))

    if args.configure:
        profile = get_profile(args.mode)
        commands = profile.commands_with_save(args.save)
        print("[UM980] command preview:")
        for command in commands:
            print(f"  {command}")
        results = client.apply_commands(
            commands,
            optional_commands=profile.optional_commands,
            fallbacks=dict(profile.fallbacks),
            reconnect_after_commands=("CONFIG SIGNALGROUP 8",) if profile.may_reboot else (),
        )
        for result in results:
            status = "OK" if result.ok else "FAIL"
            print(f"[UM980] {status}: {result.command}")
            if result.error:
                print(f"        {result.error}")

    if args.snapshot_dir:
        created = client.capture_snapshot(args.snapshot_dir, "after")
        print("[UM980] after snapshot:", ", ".join(str(path) for path in created))

    last_print = 0.0
    try:
        for line in client.read_lines():
            try:
                parsed = parse_line(line)
            except ValueError as exc:
                state.observe_parse_error(str(exc), line)
                continue
            if parsed is None:
                continue
            state.update(parsed)
            if args.raw:
                print(line)
            now = monotonic()
            if now - last_print >= 1.0:
                last_print = now
                summary = state.summary(now)
                print(
                    "[GNSS] "
                    f"fix={summary['fix']} rtk={summary['rtk_state']} pos_type={summary['position_type']} "
                    f"cov={summary['covariance_source']} sats={summary['satellites']} hdop={summary['hdop']} "
                    f"lat={summary['lat']} lon={summary['lon']} "
                    f"rates={summary['rates']} rtcm={summary['rtcm_bytes_per_sec']:.1f}B/s"
                )
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
