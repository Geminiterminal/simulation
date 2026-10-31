"""Pure-Python reader/writer for WPILib ``.wpilog`` (DataLog) files.

No numpy, no WPILib install, no vendor libraries - only the standard library, so
it runs anywhere Python 3.8+ runs (CI, a laptop without a RoboRIO toolchain, or
inside this repository's ``python3 -m unittest discover -s tests`` suite).

The binary layout implemented here is the one written by
``edu.wpi.first.util.datalog.DataLog`` (and read back by
``edu.wpi.first.util.datalog.DataLogReader``) in WPILib 2024-2026:

    header
        "WPILOG"                       6 bytes
        uint16 version (1.0 = 0x0100)  little endian
        uint32 extra-header length     little endian
        extra-header bytes             UTF-8 JSON (usually "{}")

    record (repeated until EOF)
        uint8   header byte
                bits 0-1  entry length      field  (entryLen   = value + 1)
                bits 2-3  size length       field  (sizeLen    = value + 1)
                bits 4-6  timestamp length  field  (timeLen    = value + 1)
        varint  entry id                   little endian, entryLen bytes
        varint  record data size           little endian, sizeLen bytes
        varint  timestamp in microseconds  little endian, timeLen bytes
        bytes   record data

    control record data (entry id 0)
        0x00 start      uint32 id, string name, string type, string metadata
        0x01 finish     uint32 id
        0x02 metadata   uint32 id, string metadata
        (every string is a uint32 byte length followed by UTF-8 bytes)

    data record data (entry id != 0)
        payload encoded according to the type declared by the start record:
        boolean, int64, float, double, string, boolean[]/int64[]/float[]/
        double[]/string[], raw

Truncated logs (a roboRIO losing power / a USB stick pulled mid-match) are
common in the wild, so the reader stops cleanly at the last complete record and
reports the truncation instead of raising.

Typical use::

    from wpilog import read_wpilog

    log = read_wpilog("FRC_20260418_152525.wpilog")
    for entry in log.entries.values():
        print(entry.name, entry.type, entry.metadata)
    for timestamp_us, value in log["/Shooter/Front RPM"]:
        ...
"""

from __future__ import annotations

import gzip
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple, Union

__all__ = [
    "WpiLogError",
    "WpiLogEntry",
    "WpiLog",
    "WpiLogWriter",
    "read_wpilog",
    "parse_wpilog",
]

MAGIC = b"WPILOG"
CONTROL_START = 0
CONTROL_FINISH = 1
CONTROL_SET_METADATA = 2

#: Types whose data records decode to a single scalar.
SCALAR_TYPES = {
    "boolean",
    "int64",
    "float",
    "double",
    "string",
}

ARRAY_DECODERS = {
    "boolean[]": "?",
    "int64[]": "<q",
    "float[]": "<f",
    "double[]": "<d",
}


class WpiLogError(Exception):
    """Raised when a file is not a readable WPILib data log."""


@dataclass(frozen=True)
class WpiLogEntry:
    """One entry (channel) in a data log."""

    entry_id: int
    name: str
    type: str
    metadata: str = ""

    @property
    def unit(self) -> str:
        """Best-effort physical unit string for numeric entries.

        WPILib stores the unit of a ``DoubleLogEntry`` in the metadata field
        (``{}</dev/null...`` is unused in practice), so for SysId data this is
        things like ``Volts``, ``MetersPerSecond`` or ``RotationsPerSecond``.
        Struct/schema metadata (JSON objects) is filtered out.
        """
        text = self.metadata.strip()
        if not text or text[0] in "{[":
            return ""
        return text


@dataclass
class WpiLog:
    """A parsed data log: entries plus every data record, in log order."""

    version: int = 0x0100
    extra_header: str = ""
    entries: Dict[int, WpiLogEntry] = field(default_factory=dict)
    #: entry id -> [(timestamp_us, decoded value)]; timestamps are monotonic.
    samples: Dict[int, List[Tuple[int, object]]] = field(default_factory=dict)
    truncated: bool = False
    source: str = "<memory>"

    # -- lookups ---------------------------------------------------------
    def entry(self, name: str) -> Optional[WpiLogEntry]:
        for candidate in self.entries.values():
            if candidate.name == name:
                return candidate
        return None

    def names(self) -> List[str]:
        return sorted(entry.name for entry in self.entries.values())

    def find(self, needle: str) -> List[WpiLogEntry]:
        """All entries whose name contains ``needle`` (case-insensitive)."""
        lowered = needle.lower()
        return [e for e in self.entries.values() if lowered in e.name.lower()]

    def values(self, entry: Union[str, int, WpiLogEntry]) -> List[Tuple[int, object]]:
        entry_id = entry.entry_id if isinstance(entry, WpiLogEntry) else entry
        if isinstance(entry_id, str):
            found = self.entry(entry_id)
            if found is None:
                return []
            entry_id = found.entry_id
        return self.samples.get(entry_id, [])

    def __getitem__(self, entry: Union[str, int]) -> List[Tuple[int, object]]:
        return self.values(entry)

    def __contains__(self, name: object) -> bool:
        return isinstance(name, str) and self.entry(name) is not None

    def __iter__(self) -> Iterator[WpiLogEntry]:
        return iter(sorted(self.entries.values(), key=lambda e: e.entry_id))


def _decode_scalar(raw: bytes, type_name: str) -> object:
    if type_name == "double":
        return struct.unpack("<d", raw[:8])[0]
    if type_name == "float":
        return struct.unpack("<f", raw[:4])[0]
    if type_name == "int64":
        return struct.unpack("<q", raw[:8])[0]
    if type_name == "boolean":
        return bool(raw[0]) if raw else False
    if type_name == "string":
        return raw.decode("utf-8", errors="replace")
    if type_name == "raw":
        return raw
    if type_name in ARRAY_DECODERS:
        code = ARRAY_DECODERS[type_name]
        width = struct.calcsize(code)
        count = len(raw) // width
        return list(struct.unpack("<" + str(count) + code[1:], raw[: count * width]))
    if type_name == "string[]":
        out = []
        offset = 0
        while offset + 4 <= len(raw):
            (length,) = struct.unpack_from("<I", raw, offset)
            offset += 4
            out.append(raw[offset : offset + length].decode("utf-8", errors="replace"))
            offset += length
        return out
    return raw


def _check_not_lfs_pointer(head: bytes, source: str) -> None:
    if head.startswith(b"version https://git-lfs"):
        raise WpiLogError(
            f"{source} is a Git LFS pointer, not log data. "
            "Fetch the real file with `git lfs pull` (or download it from GitHub)."
        )


def _read_inner_string(buf: bytes, offset: int) -> Tuple[str, int]:
    (length,) = struct.unpack_from("<I", buf, offset)
    offset += 4
    return buf[offset : offset + length].decode("utf-8", errors="replace"), offset + length


def parse_wpilog(data: bytes, source: str = "<memory>") -> WpiLog:
    """Parse in-memory ``.wpilog`` bytes. Raises :class:`WpiLogError` if invalid."""
    _check_not_lfs_pointer(data[:64], source)
    if len(data) == 0:
        raise WpiLogError(f"{source} is empty")
    if len(data) < 12 or data[:6] != MAGIC:
        raise WpiLogError(
            f"{source} does not start with the WPILOG magic bytes; "
            "this is not a WPILib .wpilog data log"
        )

    (version,) = struct.unpack_from("<H", data, 6)
    (extra_len,) = struct.unpack_from("<I", data, 8)
    offset = 12
    if offset + extra_len > len(data):
        raise WpiLogError(f"{source} has a corrupt header (extra header length {extra_len})")
    extra = data[offset : offset + extra_len].decode("utf-8", errors="replace")
    offset += extra_len

    log = WpiLog(version=version, extra_header=extra, source=source)
    size = len(data)

    while offset < size:
        header_byte = data[offset]
        entry_len = (header_byte & 0x3) + 1
        size_len = ((header_byte >> 2) & 0x3) + 1
        time_len = ((header_byte >> 4) & 0x7) + 1
        header_len = 1 + entry_len + size_len + time_len
        if offset + header_len > size:
            log.truncated = True
            break

        entry_id = int.from_bytes(data[offset + 1 : offset + 1 + entry_len], "little")
        payload_len = int.from_bytes(
            data[offset + 1 + entry_len : offset + 1 + entry_len + size_len], "little"
        )
        timestamp = int.from_bytes(
            data[
                offset + 1 + entry_len + size_len : offset + header_len
            ],
            "little",
        )
        payload_start = offset + header_len
        payload_end = payload_start + payload_len
        if payload_end > size:
            # Cut mid-record: keep everything already decoded.
            log.truncated = True
            break
        payload = data[payload_start:payload_end]
        offset = payload_end

        if entry_id == 0:
            if not payload:
                continue
            control = payload[0]
            if control == CONTROL_START:
                (new_id,) = struct.unpack_from("<I", payload, 1)
                name, cursor = _read_inner_string(payload, 5)
                type_name, cursor = _read_inner_string(payload, cursor)
                metadata, _ = _read_inner_string(payload, cursor)
                log.entries[new_id] = WpiLogEntry(new_id, name, type_name, metadata)
            elif control == CONTROL_FINISH:
                if len(payload) >= 5:
                    (finished_id,) = struct.unpack_from("<I", payload, 1)
                    log.entries.pop(finished_id, None)
            elif control == CONTROL_SET_METADATA:
                if len(payload) >= 5:
                    (target_id,) = struct.unpack_from("<I", payload, 1)
                    metadata, _ = _read_inner_string(payload, 5)
                    existing = log.entries.get(target_id)
                    if existing is not None:
                        log.entries[target_id] = WpiLogEntry(
                            existing.entry_id, existing.name, existing.type, metadata
                        )
            continue

        entry = log.entries.get(entry_id)
        if entry is None:
            # Data before a start record: ignore rather than crash.
            continue
        try:
            value = _decode_scalar(payload, entry.type)
        except (struct.error, IndexError):
            continue
        log.samples.setdefault(entry_id, []).append((timestamp, value))

    for samples in log.samples.values():
        samples.sort(key=lambda sample: sample[0])
    return log


def read_wpilog(path: Union[str, Path]) -> WpiLog:
    """Read and parse a ``.wpilog`` file (plain or gzip-compressed)."""
    path = Path(path)
    raw = path.read_bytes()
    _check_not_lfs_pointer(raw[:64], str(path))
    if raw[:2] == b"\x1f\x8b":
        try:
            raw = gzip.decompress(raw)
        except OSError as exc:  # pragma: no cover - corrupt gzip
            raise WpiLogError(f"{path} looks gzip-compressed but failed to decompress: {exc}")
    return parse_wpilog(raw, source=str(path))


class WpiLogWriter:
    """Minimal writer for the same format, used for fixtures and demo logs.

    The writer produces a spec-compliant v1.0 log; WPILib's ``DataLogReader``
    and AdvantageScope both open the result.
    """

    def __init__(self, extra_header: str = "{}") -> None:
        extra = extra_header.encode("utf-8")
        self._buf = bytearray(MAGIC)
        self._buf += struct.pack("<H", 0x0100)
        self._buf += struct.pack("<I", len(extra))
        self._buf += extra
        self._next_id = 1

    # -- entries ---------------------------------------------------------
    def start_entry(self, name: str, type_name: str, metadata: str = "") -> int:
        entry_id = self._next_id
        self._next_id += 1
        payload = bytearray([CONTROL_START])
        payload += struct.pack("<I", entry_id)
        for text in (name, type_name, metadata):
            encoded = text.encode("utf-8")
            payload += struct.pack("<I", len(encoded)) + encoded
        self._write_record(0, 0, payload)
        return entry_id

    def finish_entry(self, entry_id: int) -> None:
        self._write_record(0, 0, bytes([CONTROL_FINISH]) + struct.pack("<I", entry_id))

    # -- data ------------------------------------------------------------
    def append_double(self, entry_id: int, timestamp_us: int, value: float) -> None:
        self._write_record(entry_id, timestamp_us, struct.pack("<d", float(value)))

    def append_float(self, entry_id: int, timestamp_us: int, value: float) -> None:
        self._write_record(entry_id, timestamp_us, struct.pack("<f", float(value)))

    def append_int(self, entry_id: int, timestamp_us: int, value: int) -> None:
        self._write_record(entry_id, timestamp_us, struct.pack("<q", int(value)))

    def append_bool(self, entry_id: int, timestamp_us: int, value: bool) -> None:
        self._write_record(entry_id, timestamp_us, b"\x01" if value else b"\x00")

    def append_string(self, entry_id: int, timestamp_us: int, value: str) -> None:
        self._write_record(entry_id, timestamp_us, value.encode("utf-8"))

    def to_bytes(self) -> bytes:
        return bytes(self._buf)

    def write(self, path: Union[str, Path]) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(self.to_bytes())
        return path

    # -- internals -------------------------------------------------------
    def _write_record(self, entry_id: int, timestamp_us: int, payload: bytes) -> None:
        entry_bytes = _varint(entry_id)
        size_bytes = _varint(len(payload))
        time_bytes = _varint(timestamp_us)
        header = bytes(
            [
                (len(entry_bytes) - 1)
                | ((len(size_bytes) - 1) << 2)
                | ((len(time_bytes) - 1) << 4)
            ]
        )
        self._buf += header + entry_bytes + size_bytes + time_bytes + payload


def _varint(value: int) -> bytes:
    """Twos-complement little-endian integer, minimal length."""
    if value < 0:
        raise ValueError("timestamps and ids are unsigned")
    length = max(1, (value.bit_length() + 7) // 8)
    return value.to_bytes(length, "little")
