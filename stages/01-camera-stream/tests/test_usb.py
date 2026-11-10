from dataclasses import replace
import plistlib
import socket
import struct
import threading

import pytest

from drone_camera import usb
from drone_camera.config import defaults, load_config, save_config
from drone_camera.gui import MainWindow


URL = "http://192.168.1.12:8080/stream.mjpg?token=" + "a" * 32


@pytest.mark.parametrize("value", ["http://host/video", "rtsp://host:8080/stream.mjpg?token=" + "a"*32,
    URL + "#fragment", URL + "&token=" + "b"*32, URL.replace(":8080", ":invalid"),
    URL.replace("192.168.1.12", "user:password@host")])
def test_reject_invalid_companion_urls_without_exposing_tokens(value):
    with pytest.raises(usb.UsbError) as error:
        usb.companion_url(value)
    assert "password" not in str(error.value) and "a"*32 not in str(error.value)


def test_usb_settings_survive_roundtrip_and_recreate_relay(qapp, tmp_path):
    configs = defaults()
    configs[0] = replace(configs[0], input_type="mjpeg", source=URL, usb_platform="ios", usb_serial="example-udid")
    path = tmp_path / "config.json"
    save_config(path, configs)
    assert load_config(path) == configs
    window = MainWindow(load_config(path), path)
    try:
        runtime = window.sources[0].config.source
        assert runtime.startswith("http://127.0.0.1:")
        assert window.tunnels[0].remote_port == 8080
        assert window.configs[0].source == URL  # Never persist an ephemeral PC port.
        assert usb.companion_url(runtime).query == usb.companion_url(URL).query
    finally:
        window.close()
    assert not window.tunnels[0]._thread.is_alive()


def test_real_loopback_relay_and_close(monkeypatch):
    left, right = socket.socketpair()
    monkeypatch.setattr(usb.UsbTunnel, "_connect", lambda _: left)
    tunnel = usb.UsbTunnel("ios", 8080)
    try:
        assert tunnel._server.server_address[0] == "127.0.0.1"
        with socket.create_connection(("127.0.0.1", tunnel.port), timeout=2) as client:
            right.settimeout(2)
            client.sendall(b"request")
            assert right.recv(7) == b"request"
            right.sendall(b"jpeg-response")
            assert client.recv(13) == b"jpeg-response"
    finally:
        tunnel.close(); tunnel.wait_closed(); right.close()
    assert not tunnel._thread.is_alive()


def test_android_selects_usb_and_removes_only_its_forward(monkeypatch):
    calls = []
    def adb(self, *args):
        calls.append(args)
        return "USB_SERIAL" if args == ("-d", "get-serialno") else "12345"
    monkeypatch.setattr(usb.UsbTunnel, "_adb", adb)
    tunnel = usb.UsbTunnel("android", 8080)
    left, right = socket.socketpair()
    monkeypatch.setattr(usb.socket, "create_connection", lambda *args, **kwargs: left)
    try:
        assert tunnel._connect() is left
        assert calls[:2] == [("-d", "get-serialno"), ("-s", "USB_SERIAL", "forward", "tcp:0", "tcp:8080")]
    finally:
        left.close(); right.close(); tunnel.close(); tunnel.wait_closed()
    assert calls[-1] == ("-s", "USB_SERIAL", "forward", "--remove", "tcp:12345")


def test_mux_frames_split_reads_and_connect_port_byte_order(monkeypatch):
    peers = []
    requests = []
    def channel():
        client, peer = socket.socketpair(); peers.append(peer)
        def respond():
            header = usb._read_exact(peer, 16)
            size, _, _, _ = struct.unpack("<IIII", header)
            message = plistlib.loads(usb._read_exact(peer, size - 16)); requests.append(message)
            result = {"DeviceList": [{"DeviceID": 9, "Properties": {"SerialNumber": "iphone", "ConnectionType": "USB"}}]} if message["MessageType"] == "ListDevices" else {"Number": 0}
            payload = plistlib.dumps(result)
            packet = struct.pack("<IIII", 16+len(payload), 1, 8, 1) + payload
            peer.sendall(packet[:7]); peer.sendall(packet[7:])
        threading.Thread(target=respond, daemon=True).start()
        return client
    monkeypatch.setattr(usb, "_mux_socket", channel)
    stream = usb._ios_connect(8080, "iphone")
    try:
        assert requests[1]["DeviceID"] == 9
        assert requests[1]["PortNumber"] == socket.htons(8080)
    finally:
        stream.close()
        for peer in peers:
            peer.close()


def test_ios_never_selects_wifi_device(monkeypatch):
    left, right = socket.socketpair()
    monkeypatch.setattr(usb, "_mux_socket", lambda: left)
    monkeypatch.setattr(usb, "_mux_request", lambda *a: {"DeviceList": [{"DeviceID": 1, "Properties": {"ConnectionType": "Network"}}]})
    try:
        with pytest.raises(usb.UsbError):
            usb._ios_connect(8080, "")
    finally:
        right.close()
