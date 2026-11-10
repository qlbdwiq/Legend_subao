"""USB TCP relay for the companion HTTP server. No phone-network route required.

Android uses adb. iOS uses the standard usbmux plist protocol provided by macOS
or Apple's Mobile Device service on Windows. Phone trust/debugging approval is
handled by the OS, never bypassed here. Only loopback listeners are created.
"""
from dataclasses import dataclass
import os
import plistlib
import select
import socket
import socketserver
import struct
import subprocess
import threading
from urllib.parse import urlsplit, urlunsplit, parse_qs

from .runtime import adb_executable


class UsbError(RuntimeError):
    pass


def companion_url(value: str):
    try:
        url = urlsplit(value.strip())
        tokens = parse_qs(url.query)
        token = tokens.get("token", [""])[0]
        if (url.scheme != "http" or not url.hostname or url.username or url.password
                or url.path != "/stream.mjpg" or not url.port or len(token) != 32
                or any(c not in "0123456789abcdef" for c in token)
                or tokens != {"token": [token]} or url.fragment):
            raise ValueError
        return url
    except ValueError:
        raise UsbError("휴대폰 앱에서 복사한 전체 연결 주소를 입력하세요.") from None


def _read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        data = stream.recv(size - len(result))
        if not data:
            raise UsbError("Apple USB 서비스가 연결을 종료했습니다.")
        result.extend(data)
    return bytes(result)


def _mux_socket():
    try:
        if os.name == "nt":
            return socket.create_connection(("127.0.0.1", 27015), timeout=3)
        stream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        stream.settimeout(3)
        stream.connect("/var/run/usbmuxd")
        return stream
    except OSError:
        raise UsbError("Apple USB 서비스를 찾을 수 없습니다. iPhone 신뢰 승인과 Apple Devices/iTunes 드라이버를 확인하세요.") from None


def _mux_request(stream, message):
    payload = plistlib.dumps({"ClientVersionString": "DroneCamera-0.4.0", "ProgName": "DroneCamera", "kLibUSBMuxVersion": 3, **message})
    stream.sendall(struct.pack("<IIII", len(payload) + 16, 1, 8, 1) + payload)
    length, version, kind, tag = struct.unpack("<IIII", _read_exact(stream, 16))
    if not 16 <= length <= 1024 * 1024 or version != 1 or kind != 8 or tag != 1:
        raise UsbError("Apple USB 응답 형식이 올바르지 않습니다.")
    try:
        return plistlib.loads(_read_exact(stream, length - 16))
    except (ValueError, plistlib.InvalidFileException):
        raise UsbError("Apple USB 응답을 읽을 수 없습니다.") from None


def _ios_connect(port, serial):
    with _mux_socket() as discovery:
        devices = _mux_request(discovery, {"MessageType": "ListDevices"}).get("DeviceList", [])
    devices = [d for d in devices if d.get("Properties", {}).get("ConnectionType", "USB") == "USB"]
    if serial:
        devices = [d for d in devices if d.get("Properties", {}).get("SerialNumber") == serial]
    if len(devices) != 1:
        raise UsbError("USB iPhone을 한 대 연결하거나, 여러 대라면 UDID를 지정하세요. 잠금 해제·신뢰 승인이 필요합니다.")
    stream = _mux_socket()
    try:
        result = _mux_request(stream, {"MessageType": "Connect", "DeviceID": devices[0]["DeviceID"],
                                       "PortNumber": socket.htons(port)})
        if result.get("Number") != 0:
            raise UsbError("iPhone 앱의 송출 시작, USB 케이블, 잠금 해제·신뢰 승인을 확인하세요.")
        return stream
    except BaseException:
        stream.close()
        raise


@dataclass
class UsbTunnel:
    platform: str
    remote_port: int
    serial: str = ""

    def __post_init__(self):
        if self.platform not in ("android", "ios") or not 1 <= self.remote_port <= 65535:
            raise UsbError("USB 장치 종류와 포트를 확인하세요.")
        self.error = ""
        self._closed = False
        self._sockets = set()
        self._lock = threading.Lock()
        self._slots = threading.BoundedSemaphore(2)
        self._android_port = None
        self._android_serial = None
        self._init_lock = threading.Lock()
        tunnel = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                if not tunnel._slots.acquire(blocking=False):
                    return
                upstream = None
                try:
                    with tunnel._lock:
                        if tunnel._closed:
                            return
                        tunnel._sockets.add(self.request)
                    upstream = tunnel._connect()
                    upstream.setblocking(False)
                    self.request.setblocking(False)
                    with tunnel._lock:
                        if tunnel._closed:
                            return
                        tunnel._sockets.add(upstream)
                    # Fixed bounded per-direction pending buffers, with backpressure.
                    pending = {self.request: bytearray(), upstream: bytearray()}
                    opposite = {self.request: upstream, upstream: self.request}
                    tunnel.error = ""
                    while not tunnel._closed:
                        reads = [s for s in pending if len(pending[opposite[s]]) < 65536]
                        writes = [s for s in pending if pending[s]]
                        ready, writable, _ = select.select(reads, writes, [], 0.2)
                        for stream in ready:
                            try:
                                data = stream.recv(min(32768, 65536 - len(pending[opposite[stream]])))
                            except BlockingIOError:
                                continue
                            if not data:
                                return
                            pending[opposite[stream]].extend(data)
                        for stream in writable:
                            try:
                                sent = stream.send(pending[stream])
                            except BlockingIOError:
                                continue
                            if sent == 0:
                                return
                            del pending[stream][:sent]
                except (OSError, UsbError, ValueError) as error:
                    tunnel.error = str(error) if isinstance(error, UsbError) else "USB 연결이 끊겼습니다. 케이블과 휴대폰 송출 상태를 확인하세요."
                finally:
                    with tunnel._lock:
                        tunnel._sockets.discard(self.request)
                        if upstream is not None:
                            tunnel._sockets.discard(upstream)
                    if upstream is not None:
                        upstream.close()
                    tunnel._slots.release()

        class Relay(socketserver.ThreadingTCPServer):
            daemon_threads = True
            allow_reuse_address = True
        self._server = Relay(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        self._thread.start()

    @property
    def port(self):
        return self._server.server_address[1]

    def rewrite(self, url):
        parsed = companion_url(url)
        if parsed.port != self.remote_port:
            raise UsbError("휴대폰 앱의 송출 포트가 변경됐습니다. USB 연결을 다시 설정하세요.")
        return urlunsplit(("http", f"127.0.0.1:{self.port}", parsed.path, parsed.query, ""))

    def _adb(self, *args):
        try:
            adb = adb_executable()
        except (OSError, ValueError, KeyError):
            raise UsbError("내장 Android USB 도구를 준비할 수 없습니다. 앱을 다시 내려받고 사용자 폴더 권한을 확인하세요.") from None
        if not adb:
            raise UsbError("Android SDK Platform-Tools의 adb를 설치하고 PATH에 추가하세요.")
        try:
            flags = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
            result = subprocess.run([adb, *args], capture_output=True, text=True, timeout=8, **flags)
        except (OSError, subprocess.TimeoutExpired):
            raise UsbError("adb 실행 실패: Platform-Tools와 USB 디버깅 승인을 확인하세요.") from None
        if result.returncode:
            raise UsbError("adb USB 연결 실패: 휴대폰에서 USB 디버깅을 승인하고 USB 장치 선택을 확인하세요.")
        return result.stdout.strip()

    def _connect(self):
        if self.platform == "ios":
            return _ios_connect(self.remote_port, self.serial)
        with self._init_lock:
            if self._android_port is None:
                # Ignore Wi-Fi adb transports; this helper promises a physical USB path.
                if self.serial:
                    entries = [line.split() for line in self._adb("devices", "-l").splitlines()[1:] if line.strip()]
                    devices = [e[0] for e in entries if len(e) >= 2 and e[1] == "device" and e[0] == self.serial
                               and ":" not in e[0] and "_tcp" not in e[0] and "_adb-tls" not in e[0]]
                    if len(devices) != 1:
                        raise UsbError("지정한 USB Android를 찾을 수 없습니다. USB 디버깅 승인을 확인하세요.")
                    selected = devices[0]
                else:
                    selected = self._adb("-d", "get-serialno")  # -d explicitly selects physical USB.
                if self._closed:
                    raise UsbError("USB 연결 도우미가 닫혔습니다.")
                self._android_serial = selected
                value = self._adb("-s", selected, "forward", "tcp:0", f"tcp:{self.remote_port}")
                try:
                    self._android_port = int(value)
                    if not 1 <= self._android_port <= 65535:
                        raise ValueError
                except ValueError:
                    raise UsbError("adb가 유효한 전달 포트를 반환하지 않았습니다.") from None
        try:
            return socket.create_connection(("127.0.0.1", self._android_port), timeout=3)
        except OSError:
            # adb discards forwards when a USB device is unplugged. Recreate on
            # the next retry instead of caching a dead local port indefinitely.
            with self._init_lock:
                try:
                    self._adb("-s", self._android_serial, "forward", "--remove", f"tcp:{self._android_port}")
                except UsbError:
                    pass
                self._android_port = None
            raise UsbError("Android USB 전달이 끊겼습니다. 케이블·디버깅 승인을 확인하면 다음 시도에서 다시 연결합니다.") from None

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for stream in self._sockets:
                try:
                    stream.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                stream.close()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=1)
        # An adb operation may have been in progress; wait for it off the GUI thread.
        def cleanup():
            with self._init_lock:
                if self._android_port is not None:
                    try:
                        self._adb("-s", self._android_serial, "forward", "--remove", f"tcp:{self._android_port}")
                    except UsbError:
                        pass
                    self._android_port = None
        self._cleanup = threading.Thread(target=cleanup, name="usb-cleanup", daemon=True)
        self._cleanup.start()

    def wait_closed(self, timeout=9):
        cleanup = getattr(self, "_cleanup", None)
        if cleanup is not None:
            cleanup.join(timeout=timeout)
