"""Embedded NTRIP rover client for direct RTCM injection into UM980."""

from __future__ import annotations

import base64
import select
import socket
import ssl
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .ntrip_config import NtripConfig


SUCCESS_RESPONSES = ("ICY 200 OK", "HTTP/1.0 200 OK", "HTTP/1.1 200 OK")


@dataclass
class NtripStatus:
    connected: bool = False
    last_error: Optional[str] = None
    last_gga_sent_at: Optional[float] = None
    last_rtcm_received_at: Optional[float] = None
    reconnect_count: int = 0

    def gga_age_s(self, now: Optional[float] = None) -> Optional[float]:
        if self.last_gga_sent_at is None:
            return None
        now = time.monotonic() if now is None else now
        return now - self.last_gga_sent_at


class EmbeddedNtripClient:
    def __init__(
        self,
        config: NtripConfig,
        get_gga: Callable[[], Optional[str]],
        on_rtcm: Callable[[bytes], None],
        gga_interval_s: float = 5.0,
        recv_chunk_size: int = 4096,
    ) -> None:
        self.config = config
        self.get_gga = get_gga
        self.on_rtcm = on_rtcm
        self.gga_interval_s = gga_interval_s
        self.recv_chunk_size = recv_chunk_size
        self.status = NtripStatus()
        self._shutdown = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None
        self._pending_data = b""

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._shutdown.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._shutdown.set()
        self._close_socket()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                self._connect()
                self._serve_connected()
            except Exception as exc:
                self.status.connected = False
                self.status.last_error = str(exc)
                self.status.reconnect_count += 1
                self._close_socket()
                self._shutdown.wait(self.config.reconnect_wait_s)

    def _connect(self) -> None:
        raw_socket = socket.create_connection((self.config.host, self.config.port), timeout=5.0)
        raw_socket.settimeout(1.0)
        if self.config.ssl:
            context = ssl.create_default_context()
            self._socket = context.wrap_socket(raw_socket, server_hostname=self.config.host)
        else:
            self._socket = raw_socket

        self._socket.sendall(self._request())
        response, self._pending_data = self._read_response()
        if not any(token in response for token in SUCCESS_RESPONSES):
            raise RuntimeError(f"NTRIP caster rejected request: {response.strip()}")
        self.status.connected = True
        self.status.last_error = None
        self.status.last_gga_sent_at = None
        self.status.last_rtcm_received_at = None

    def _serve_connected(self) -> None:
        last_gga_send = 0.0
        while not self._shutdown.is_set():
            now = time.monotonic()
            if now - last_gga_send >= self.gga_interval_s:
                if self._send_latest_gga(now):
                    last_gga_send = now

            if self._pending_data:
                data = self._pending_data
                self._pending_data = b""
                self.status.last_rtcm_received_at = time.monotonic()
                self.on_rtcm(data)
                continue

            sock = self._require_socket()
            read_ready, _, _ = select.select([sock], [], [], 0.2)
            if not read_ready:
                if self._rtcm_timed_out(now):
                    raise RuntimeError("NTRIP RTCM timeout")
                continue
            data = sock.recv(self.recv_chunk_size)
            if not data:
                raise RuntimeError("NTRIP caster closed connection")
            self.status.last_rtcm_received_at = time.monotonic()
            self.on_rtcm(data)

    def _send_latest_gga(self, now: float) -> bool:
        gga = self.get_gga()
        if not gga:
            return False
        if gga.endswith("\\r\\n"):
            payload = gga[:-4] + "\r\n"
        elif gga.endswith("\r\n"):
            payload = gga
        else:
            payload = gga.rstrip() + "\r\n"
        self._require_socket().sendall(payload.encode("ascii"))
        self.status.last_gga_sent_at = now
        return True

    def _rtcm_timed_out(self, now: float) -> bool:
        if self.status.last_rtcm_received_at is None:
            return False
        return now - self.status.last_rtcm_received_at > self.config.rtcm_timeout_s

    def _request(self) -> bytes:
        request = f"GET /{self.config.mountpoint} HTTP/1.0\r\nUser-Agent: NTRIP um980_driver\r\n"
        request += f"Host: {self.config.host}:{self.config.port}\r\n"
        if self.config.ntrip_version:
            request += f"Ntrip-Version: {self.config.ntrip_version}\r\n"
        if self.config.authenticate:
            credentials = f"{self.config.username}:{self.config.password}".encode("utf-8")
            token = base64.b64encode(credentials).decode("ascii")
            request += f"Authorization: Basic {token}\r\n"
        request += "\r\n"
        return request.encode("ascii")

    def _read_response(self) -> tuple[str, bytes]:
        response = b""
        while b"\r\n\r\n" not in response and len(response) < 8192:
            chunk = self._require_socket().recv(self.recv_chunk_size)
            if not chunk:
                raise RuntimeError("NTRIP caster closed connection during handshake")
            response += chunk

        if b"\r\n\r\n" not in response:
            return response.decode("utf-8", errors="replace"), b""

        header, pending = response.split(b"\r\n\r\n", 1)
        return header.decode("utf-8", errors="replace"), pending

    def _close_socket(self) -> None:
        if not self._socket:
            return
        try:
            self._socket.close()
        finally:
            self._socket = None
            self.status.connected = False

    def _require_socket(self) -> socket.socket:
        if not self._socket:
            raise RuntimeError("NTRIP socket is not connected")
        return self._socket
