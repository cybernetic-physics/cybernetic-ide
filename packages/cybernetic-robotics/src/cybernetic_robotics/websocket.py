from __future__ import annotations

import base64
import hashlib
import json
import os
import socket
import struct
from typing import Any

from .config import RobotEndpoints
from .errors import ProtocolError, SimulatorUnavailable


class TinyWebSocket:
    """Minimal RFC 6455 client for the local simulator protocol.

    The package avoids a WebSocket dependency so fresh robotics examples work
    in a plain Python install. This client is intentionally small and aimed at
    local trusted simulator traffic, not arbitrary Internet WebSockets.
    """

    def __init__(self, host: str, port: int, path: str = "/", timeout: float = 5.0):
        self.host = host
        self.port = int(port)
        self.path = path or "/"
        self.timeout = float(timeout)
        self.sock: socket.socket | None = None

    @classmethod
    def from_env(cls, timeout: float = 5.0) -> "TinyWebSocket":
        endpoints = RobotEndpoints.from_env()
        return cls(endpoints.ws_host, endpoints.ws_port, endpoints.ws_path, timeout=timeout)

    def __enter__(self) -> "TinyWebSocket":
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response = self._recv_until(sock, b"\r\n\r\n")
        if b" 101 " not in response.split(b"\r\n", 1)[0]:
            raise SimulatorUnavailable(
                f"WebSocket upgrade failed:\n{response.decode(errors='replace')}"
            )

        expected_accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        )
        if expected_accept not in response:
            raise ProtocolError("WebSocket accept key did not match")

        self.sock = sock
        return self

    def __exit__(self, *_exc: object) -> None:
        if self.sock is not None:
            try:
                self.sock.sendall(b"\x88\x80" + os.urandom(4))
            finally:
                self.sock.close()
                self.sock = None

    def request_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.send_text(json.dumps(payload, separators=(",", ":")))
        while True:
            opcode, data = self.read_frame()
            if opcode == 0x1:
                value = json.loads(data.decode("utf-8"))
                if not isinstance(value, dict):
                    raise ProtocolError("WebSocket JSON response was not an object")
                return value
            if opcode == 0x8:
                raise ProtocolError("WebSocket closed before response")
            if opcode == 0x9:
                self._send_control_frame(0xA, data)

    def subscribe_once(self, topic: str) -> tuple[int | None, bytes]:
        self.send_text(f"subscribe:{topic}")
        while True:
            opcode, data = self.read_frame()
            if opcode == 0x2:
                message_type = struct.unpack("!I", data[:4])[0] if len(data) >= 4 else None
                return message_type, data
            if opcode == 0x8:
                raise ProtocolError("WebSocket closed before binary frame")
            if opcode == 0x9:
                self._send_control_frame(0xA, data)

    def send_text(self, text: str) -> None:
        self._send_frame(0x1, text.encode("utf-8"))

    def read_frame(self) -> tuple[int, bytes]:
        sock = self._socket()
        header = self._recv_exact(sock, 2)
        opcode = header[0] & 0x0F
        masked = bool(header[1] & 0x80)
        length = header[1] & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._recv_exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._recv_exact(sock, 8))[0]
        mask = self._recv_exact(sock, 4) if masked else None
        payload = self._recv_exact(sock, length)
        if mask is not None:
            payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        return opcode, payload

    def _send_control_frame(self, opcode: int, payload: bytes) -> None:
        self._send_frame(opcode, payload)

    def _send_frame(self, opcode: int, payload: bytes) -> None:
        sock = self._socket()
        mask = os.urandom(4)
        length = len(payload)
        if length < 126:
            header = bytes([0x80 | opcode, 0x80 | length])
        elif length < 65536:
            header = bytes([0x80 | opcode, 0x80 | 126]) + struct.pack("!H", length)
        else:
            header = bytes([0x80 | opcode, 0x80 | 127]) + struct.pack("!Q", length)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        sock.sendall(header + mask + masked)

    def _socket(self) -> socket.socket:
        if self.sock is None:
            raise ProtocolError("WebSocket is not connected")
        return self.sock

    @staticmethod
    def _recv_exact(sock: socket.socket, length: int) -> bytes:
        chunks: list[bytes] = []
        remaining = length
        while remaining:
            chunk = sock.recv(remaining)
            if not chunk:
                raise ProtocolError("socket closed")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    @staticmethod
    def _recv_until(sock: socket.socket, marker: bytes) -> bytes:
        data = bytearray()
        while marker not in data:
            chunk = sock.recv(4096)
            if not chunk:
                raise ProtocolError("socket closed during handshake")
            data.extend(chunk)
        return bytes(data)
