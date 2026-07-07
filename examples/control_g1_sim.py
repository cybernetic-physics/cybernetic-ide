#!/usr/bin/env python3
"""
Drive the local Unitree G1 MuJoCo harness through the reversed viewer protocol.

This intentionally uses only Python's standard library so it works in a fresh
checkout. The current harness accepts simulator lifecycle commands over the
Booster-like WebSocket channel; proper SDK2 locomotion is the next layer.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import time
from urllib.request import urlopen


HTTP_URL = os.environ.get("CYBER_G1_GAME_CONTROL_URL", "http://127.0.0.1:38383")
WS_HOST = os.environ.get("CYBER_G1_WS_HOST", "127.0.0.1")
WS_PORT = int(os.environ.get("CYBER_G1_WS_PORT", "8788"))


class TinyWebSocket:
    def __init__(self, host: str, port: int, path: str = "/"):
        self.host = host
        self.port = port
        self.path = path
        self.sock: socket.socket | None = None

    def __enter__(self) -> "TinyWebSocket":
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
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
            raise RuntimeError(f"WebSocket upgrade failed:\n{response.decode(errors='replace')}")

        expected_accept = base64.b64encode(
            hashlib.sha1(
                (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
            ).digest()
        )
        if expected_accept not in response:
            raise RuntimeError("WebSocket accept key did not match")

        self.sock = sock
        return self

    def __exit__(self, *_exc):
        if self.sock is not None:
            try:
                self.sock.sendall(b"\x88\x80" + os.urandom(4))
            finally:
                self.sock.close()

    def request_json(self, payload: dict) -> dict:
        self.send_text(json.dumps(payload, separators=(",", ":")))
        while True:
            opcode, data = self.read_frame()
            if opcode == 0x1:
                return json.loads(data.decode("utf-8"))
            if opcode == 0x8:
                raise RuntimeError("WebSocket closed before response")
            if opcode == 0x9:
                self._send_control_frame(0xA, data)

    def send_text(self, text: str):
        data = text.encode("utf-8")
        self._send_frame(0x1, data)

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

    def _send_control_frame(self, opcode: int, payload: bytes):
        self._send_frame(opcode, payload)

    def _send_frame(self, opcode: int, payload: bytes):
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
            raise RuntimeError("WebSocket is not connected")
        return self.sock

    @staticmethod
    def _recv_exact(sock: socket.socket, length: int) -> bytes:
        chunks: list[bytes] = []
        remaining = length
        while remaining:
            chunk = sock.recv(remaining)
            if not chunk:
                raise RuntimeError("socket closed")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    @staticmethod
    def _recv_until(sock: socket.socket, marker: bytes) -> bytes:
        data = bytearray()
        while marker not in data:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("socket closed during handshake")
            data.extend(chunk)
        return bytes(data)


def get_json(path: str) -> dict:
    with urlopen(f"{HTTP_URL}{path}", timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))


def save_camera_frame(path: Path):
    with urlopen(f"{HTTP_URL}/camera_frame_0.jpg", timeout=5.0) as response:
        path.write_bytes(response.read())


def print_status(label: str):
    status = get_json("/status")["simulation"]
    frame = get_json("/visual_frame")
    mode = status["robot_modes"].get("g1", "unknown")
    print(
        f"{label}: paused={status['paused']} speed={status['actual_speed_factor']:.2f}x "
        f"mode={mode} frame={frame['frame_id']} time={frame['time']:.3f}s"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--run-seconds", type=float, default=1.2)
    parser.add_argument(
        "--pose",
        choices=["raise_right_hand", "neutral"],
        help="Apply a named direct-MuJoCo pose and stop unless --run-after-pose is set.",
    )
    parser.add_argument("--run-after-pose", action="store_true")
    args = parser.parse_args()

    out_dir = Path(".runtime/g1-control-demo")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Connecting to G1 sim: ws://{WS_HOST}:{WS_PORT} and {HTTP_URL}")
    print_status("before")
    save_camera_frame(out_dir / "before.jpg")

    with TinyWebSocket(WS_HOST, WS_PORT) as ws:
        commands = [
            ("reset", {"type": "command", "command": "reset"}),
            ("reset camera", {"type": "camera", "action": "reset"}),
        ]
        for label, payload in commands:
            print(f"{label} -> {ws.request_json(payload)}")

        if args.pose:
            print(
                f"pose {args.pose} -> "
                f"{ws.request_json({'type': 'command', 'command': 'pose', 'pose': args.pose})}"
            )
            print_status(f"after pose {args.pose}")
            save_camera_frame(out_dir / f"{args.pose}.jpg")
            if not args.run_after_pose:
                print(f"Saved frames under {out_dir}")
                return

        for _ in range(args.steps):
            ws.request_json({"type": "command", "command": "step"})
        print_status(f"after {args.steps} manual steps")

        print(f"resume -> {ws.request_json({'type': 'command', 'command': 'resume'})}")
        time.sleep(args.run_seconds)
        print_status(f"after {args.run_seconds:.1f}s autorun")

        print(f"pause -> {ws.request_json({'type': 'command', 'command': 'pause'})}")
        print(f"orbit camera -> {ws.request_json({'type': 'camera', 'action': 'orbit', 'dx': 90, 'dy': -12})}")

    print_status("after")
    save_camera_frame(out_dir / "after.jpg")
    print(f"Saved frames under {out_dir}")


if __name__ == "__main__":
    main()
