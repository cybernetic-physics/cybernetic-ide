from __future__ import annotations

import json
import zlib
from dataclasses import asdict, is_dataclass
from typing import Any


class CRC:
    """Deterministic CRC helper with the same public shape as Unitree SDK2.

    The official package delegates to architecture-specific native libraries.
    Cybernetic IDE only needs a stable checksum for simulator messages, so this
    implementation hashes a JSON view of the dataclass payload.
    """

    def Crc(self, msg: Any) -> int:  # noqa: N802 - match Unitree SDK2 API.
        payload = asdict(msg) if is_dataclass(msg) else vars(msg)
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return zlib.crc32(encoded) & 0xFFFFFFFF

