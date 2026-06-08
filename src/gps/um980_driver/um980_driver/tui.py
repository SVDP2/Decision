"""Textual operations console for UM980 bring-up."""

from __future__ import annotations

from pathlib import Path
from time import monotonic

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, DataTable, Footer, Header, Input, RichLog, Select, Static

from .parsers import parse_line
from .profiles import PROFILES, get_profile
from .serial_client import UM980SerialClient, scan_serial_devices
from .state import GNSSState


class UM980TUI(App):
    CSS = """
    Screen {
        layout: vertical;
    }
    #top {
        height: auto;
        padding: 1;
    }
    #main {
        height: 1fr;
    }
    DataTable {
        height: 11;
    }
    RichLog {
        height: 1fr;
        border: solid $accent;
    }
    Button {
        margin-right: 1;
    }
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("s", "scan", "Scan"),
        ("a", "apply_profile", "Apply"),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.client: UM980SerialClient | None = None
        self.state = GNSSState()
        self.last_status = 0.0

    def compose(self) -> ComposeResult:
        yield Header()
        with Vertical(id="top"):
            with Horizontal():
                yield Select([], id="device_select", prompt="Device")
                yield Input(value="0", id="baud_input", placeholder="Baud")
                yield Select([(profile.description, name) for name, profile in PROFILES.items()], id="profile_select")
            with Horizontal():
                yield Button("Scan", id="scan_button")
                yield Button("Connect", id="connect_button")
                yield Button("Preview", id="preview_button")
                yield Button("Apply", id="apply_button", variant="primary")
                yield Button("Snapshot", id="snapshot_button")
            yield Static("Disconnected", id="status")
        with Vertical(id="main"):
            yield DataTable(id="preview_table")
            yield RichLog(id="log", wrap=True, highlight=True)
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#preview_table", DataTable)
        table.add_columns("Command", "Notes")
        self.action_scan()
        self.set_interval(0.1, self.poll_serial)

    def action_scan(self) -> None:
        devices = scan_serial_devices()
        select = self.query_one("#device_select", Select)
        options = [(f"{device.path} {device.description}", device.path) for device in devices]
        select.set_options(options)
        if options:
            select.value = options[0][1]
        self.write_log(f"Scanned {len(options)} serial candidates")

    def action_apply_profile(self) -> None:
        self.apply_profile()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        if button_id == "scan_button":
            self.action_scan()
        elif button_id == "connect_button":
            self.connect_serial()
        elif button_id == "preview_button":
            self.preview_profile()
        elif button_id == "apply_button":
            self.apply_profile()
        elif button_id == "snapshot_button":
            self.capture_snapshot()

    def connect_serial(self) -> None:
        device = self.query_one("#device_select", Select).value
        if device in (None, Select.BLANK):
            self.write_log("No serial device selected")
            return
        baud_text = self.query_one("#baud_input", Input).value
        try:
            baud = int(baud_text)
        except ValueError:
            self.write_log(f"Invalid baud: {baud_text}")
            return
        if self.client:
            self.client.close()
        self.client = UM980SerialClient(str(device), baud, timeout=0.05)
        try:
            self.client.open()
        except Exception as exc:
            self.write_log(f"Connect failed: {exc}")
            return
        self.query_one("#status", Static).update(f"Connected: {device} @ {self.client.baud}")
        self.write_log(f"Connected {device} @ {self.client.baud}")

    def preview_profile(self) -> list[str]:
        profile_name = self.query_one("#profile_select", Select).value
        if profile_name in (None, Select.BLANK):
            self.write_log("No profile selected")
            return []
        profile = get_profile(str(profile_name))
        commands = profile.commands_with_save(False)
        table = self.query_one("#preview_table", DataTable)
        table.clear()
        for command in commands:
            note = "may reset serial" if command == "CONFIG SIGNALGROUP 8" else ""
            table.add_row(command, note)
        self.write_log(f"Previewed {profile.name}: SAVECONFIG not included")
        return commands

    def apply_profile(self) -> None:
        if not self.client or not self.client.is_open:
            self.write_log("Connect before applying a profile")
            return
        commands = self.preview_profile()
        if not commands:
            return
        profile_name = self.query_one("#profile_select", Select).value
        profile = get_profile(str(profile_name))
        for result in self.client.apply_commands(
            commands,
            optional_commands=profile.optional_commands,
            fallbacks=dict(profile.fallbacks),
            reconnect_after_commands=("CONFIG SIGNALGROUP 8",) if profile.may_reboot else (),
        ):
            status = "OK" if result.ok else "FAIL"
            self.write_log(f"{status}: {result.command}")
            if result.error:
                self.write_log(f"  {result.error}")

    def capture_snapshot(self) -> None:
        if not self.client or not self.client.is_open:
            self.write_log("Connect before capturing snapshots")
            return
        snapshot_dir = Path("um980_driver_snapshots")
        try:
            paths = self.client.capture_snapshot(snapshot_dir, "tui")
        except Exception as exc:
            self.write_log(f"Snapshot failed: {exc}")
            return
        self.write_log("Snapshot files: " + ", ".join(str(path) for path in paths))

    def poll_serial(self) -> None:
        if not self.client or not self.client.is_open:
            return
        try:
            line = self.client.read_line()
        except Exception as exc:
            self.write_log(f"Serial read failed: {exc}")
            return
        if not line:
            return
        try:
            parsed = parse_line(line)
        except ValueError as exc:
            self.state.observe_parse_error(str(exc), line)
            return
        if parsed is not None:
            self.state.update(parsed)
        now = monotonic()
        if now - self.last_status >= 1.0:
            self.last_status = now
            summary = self.state.summary(now)
            self.query_one("#status", Static).update(
                f"fix={summary['fix']} rtk={summary['rtk_state']} cov={summary['covariance_source']} sats={summary['satellites']} "
                f"GGA={summary['rates'].get('GGA', 0.0):.1f}Hz "
                f"RTCM={summary['rtcm_bytes_per_sec']:.1f}B/s"
            )

    def write_log(self, message: str) -> None:
        self.query_one("#log", RichLog).write(message)


def main() -> None:
    UM980TUI().run()


if __name__ == "__main__":
    main()
