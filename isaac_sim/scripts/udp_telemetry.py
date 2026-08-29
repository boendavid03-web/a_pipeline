#!/usr/bin/env python3
"""Bounded localhost UDP framing for Isaac JSON telemetry.

Small legacy messages remain plain JSON. Larger messages are zlib-compressed;
if one compressed datagram would still exceed the UDP payload ceiling, it is
split into numbered fragments and delivered only after complete reassembly.
"""

from __future__ import annotations

import json
import struct
import time
import zlib


UDP_MAX_PAYLOAD = 65_507
COMPRESSED_MAGIC = b"APZ1"
FRAGMENT_MAGIC = b"APF1"
FRAGMENT_HEADER = struct.Struct("!4sIHH")
FRAGMENT_PAYLOAD_BYTES = 60_000
MAX_FRAGMENT_COUNT = 128
MAX_DECOMPRESSED_BYTES = 4 * 1024 * 1024
FRAGMENT_TIMEOUT_SEC = 2.0


def _decode_json(encoded: bytes) -> dict:
    if len(encoded) > MAX_DECOMPRESSED_BYTES:
        raise ValueError(
            f"telemetry JSON exceeds {MAX_DECOMPRESSED_BYTES} bytes"
        )
    payload = json.loads(encoded.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("telemetry root must be an object")
    return payload


def _decompress(encoded: bytes) -> bytes:
    try:
        decoded = zlib.decompress(encoded)
    except zlib.error as exc:
        raise ValueError(f"invalid compressed telemetry: {exc}") from exc
    if len(decoded) > MAX_DECOMPRESSED_BYTES:
        raise ValueError(
            f"decompressed telemetry exceeds {MAX_DECOMPRESSED_BYTES} bytes"
        )
    return decoded


class TelemetryEncoder:
    def __init__(self) -> None:
        self._message_id = 0

    def encode(self, payload: dict[str, object]) -> list[bytes]:
        raw = json.dumps(
            payload, ensure_ascii=True, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
        if len(raw) <= UDP_MAX_PAYLOAD:
            return [raw]

        compressed = zlib.compress(raw, level=1)
        if len(COMPRESSED_MAGIC) + len(compressed) <= UDP_MAX_PAYLOAD:
            return [COMPRESSED_MAGIC + compressed]

        fragment_count = (
            len(compressed) + FRAGMENT_PAYLOAD_BYTES - 1
        ) // FRAGMENT_PAYLOAD_BYTES
        if not 1 < fragment_count <= MAX_FRAGMENT_COUNT:
            raise ValueError(
                f"compressed telemetry requires invalid fragment count {fragment_count}"
            )
        self._message_id = (self._message_id + 1) & 0xFFFFFFFF
        return [
            FRAGMENT_HEADER.pack(
                FRAGMENT_MAGIC, self._message_id, index, fragment_count
            )
            + compressed[
                index * FRAGMENT_PAYLOAD_BYTES : (index + 1)
                * FRAGMENT_PAYLOAD_BYTES
            ]
            for index in range(fragment_count)
        ]


class TelemetryDecoder:
    def __init__(self) -> None:
        self._fragments: dict[int, dict[str, object]] = {}

    def _expire(self, now: float) -> None:
        expired = [
            message_id
            for message_id, entry in self._fragments.items()
            if now - float(entry["created"]) > FRAGMENT_TIMEOUT_SEC
        ]
        for message_id in expired:
            del self._fragments[message_id]

    def feed(self, packet: bytes) -> dict | None:
        if packet.startswith(COMPRESSED_MAGIC):
            return _decode_json(_decompress(packet[len(COMPRESSED_MAGIC) :]))

        if packet.startswith(FRAGMENT_MAGIC):
            if len(packet) < FRAGMENT_HEADER.size:
                raise ValueError("truncated telemetry fragment header")
            magic, message_id, fragment_index, fragment_count = (
                FRAGMENT_HEADER.unpack_from(packet)
            )
            if magic != FRAGMENT_MAGIC:
                raise ValueError("invalid telemetry fragment magic")
            if not 1 < fragment_count <= MAX_FRAGMENT_COUNT:
                raise ValueError("invalid telemetry fragment count")
            if fragment_index >= fragment_count:
                raise ValueError("invalid telemetry fragment index")
            now = time.monotonic()
            self._expire(now)
            entry = self._fragments.get(message_id)
            if entry is None:
                entry = {
                    "created": now,
                    "count": fragment_count,
                    "parts": {},
                }
                self._fragments[message_id] = entry
            elif int(entry["count"]) != fragment_count:
                del self._fragments[message_id]
                raise ValueError("inconsistent telemetry fragment count")
            parts = entry["parts"]
            assert isinstance(parts, dict)
            parts[fragment_index] = packet[FRAGMENT_HEADER.size :]
            if len(parts) != fragment_count:
                return None
            compressed = b"".join(parts[index] for index in range(fragment_count))
            del self._fragments[message_id]
            return _decode_json(_decompress(compressed))

        return _decode_json(packet)
