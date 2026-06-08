from um980_driver.ntrip_client import EmbeddedNtripClient
from um980_driver.ntrip_config import NtripConfig


class FakeSocket:
    def __init__(self, recv_chunks=None):
        self.sent = b""
        self.recv_chunks = list(recv_chunks or [])

    def sendall(self, data):
        self.sent += data

    def recv(self, _size):
        return self.recv_chunks.pop(0)


def test_ntrip_request_uses_basic_auth_and_mountpoint():
    client = EmbeddedNtripClient(
        config=NtripConfig(
            host="caster.example",
            port=2101,
            mountpoint="SWGS-RTCM31",
            authenticate=True,
            username="user",
            password="pass",
            ntrip_version="Ntrip/2.0",
        ),
        get_gga=lambda: None,
        on_rtcm=lambda data: None,
    )

    request = client._request().decode("ascii")

    assert request.startswith("GET /SWGS-RTCM31 HTTP/1.0\r\n")
    assert "Host: caster.example:2101\r\n" in request
    assert "Ntrip-Version: Ntrip/2.0\r\n" in request
    assert "Authorization: Basic dXNlcjpwYXNz\r\n" in request
    assert request.endswith("\r\n\r\n")


def test_ntrip_gga_send_adds_crlf():
    fake_socket = FakeSocket()
    client = EmbeddedNtripClient(
        config=NtripConfig(
            host="caster.example",
            port=2101,
            mountpoint="mount",
            authenticate=False,
            username="",
            password="",
        ),
        get_gga=lambda: "$GNGGA,1*00",
        on_rtcm=lambda data: None,
    )
    client._socket = fake_socket

    client._send_latest_gga(now=10.0)

    assert fake_socket.sent == b"$GNGGA,1*00\r\n"
    assert client.status.last_gga_sent_at == 10.0


def test_ntrip_gga_send_reports_missing_gga_without_advancing_status():
    fake_socket = FakeSocket()
    client = EmbeddedNtripClient(
        config=NtripConfig(
            host="caster.example",
            port=2101,
            mountpoint="mount",
            authenticate=False,
            username="",
            password="",
        ),
        get_gga=lambda: None,
        on_rtcm=lambda data: None,
    )
    client._socket = fake_socket

    assert client._send_latest_gga(now=10.0) is False
    assert fake_socket.sent == b""
    assert client.status.last_gga_sent_at is None


def test_ntrip_response_preserves_rtcm_bytes_after_header():
    fake_socket = FakeSocket([b"ICY 200 OK\r\n\r\n\xd3\x00\x00"])
    client = EmbeddedNtripClient(
        config=NtripConfig(
            host="caster.example",
            port=2101,
            mountpoint="mount",
            authenticate=False,
            username="",
            password="",
        ),
        get_gga=lambda: None,
        on_rtcm=lambda data: None,
    )
    client._socket = fake_socket

    response, pending = client._read_response()

    assert response == "ICY 200 OK"
    assert pending == b"\xd3\x00\x00"
