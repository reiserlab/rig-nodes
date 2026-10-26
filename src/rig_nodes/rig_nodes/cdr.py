"""cdr.py — decode a ROS 2 message's CDR bytes from the `ros2msg` schema text stored in a bag.

Pure Python, no ROS process: a converter or a sizing tool can read a bag's typed records on a
machine that has only the `mcap` library. XCDR1 (what rmw_zenoh / Fast-CDR write for our
messages), both byte orders; the alignment origin is right after the 4-byte encapsulation header.
Only the shapes our `.msg` files use are supported — primitives, `string`, primitive and string
sequences — and the decoder says so when it meets anything else.

Moved from webDisplayTools' `rig/nodes/mcap_convert.py` on 2026-10-26 (the gateway's converter
imports it from here; the sizing study `bench/sizing/columns.py` always did).
"""

from __future__ import annotations

import struct

_PRIM = {
    "bool": ("?", 1), "byte": ("B", 1), "char": ("b", 1), "int8": ("b", 1), "uint8": ("B", 1),
    "int16": ("<h", 2), "uint16": ("<H", 2), "int32": ("<i", 4), "uint32": ("<I", 4),
    "int64": ("<q", 8), "uint64": ("<Q", 8), "float32": ("<f", 4), "float64": ("<d", 8),
}

STRING_TYPE = "std_msgs/msg/String"


class Cdr:
    def __init__(self, data: bytes) -> None:
        if len(data) < 4:
            raise ValueError("CDR payload shorter than its header")
        self.d = data
        self.p = 4  # the encapsulation header is not part of the aligned stream
        self.le = data[1] == 1

    def _align(self, n: int) -> None:
        rel = self.p - 4
        pad = (-rel) % n
        self.p += pad

    def prim(self, t: str):
        fmt, size = _PRIM[t]
        self._align(size)
        if not self.le and len(fmt) == 2:
            fmt = ">" + fmt[1]
        v = struct.unpack_from(fmt, self.d, self.p)[0]
        self.p += size
        return v

    def string(self) -> str:
        n = self.prim("uint32")
        raw = self.d[self.p : self.p + n]
        self.p += n
        return raw.rstrip(b"\x00").decode("utf-8", errors="replace")

    def seq_prim(self, t: str) -> list:
        n = self.prim("uint32")
        return [self.prim(t) for _ in range(n)]


def cdr_string(data: bytes) -> str:
    """std_msgs/String in CDR: 4-byte encapsulation header, uint32 length (incl. NUL), bytes."""
    return Cdr(data).string()


def parse_msgdef(text: str) -> list[tuple[str, str]]:
    """ros2msg schema text → [(type, name)] for the TOP message (the first block; nested
    definitions after a '====' separator are not needed by our types)."""
    fields = []
    for line in text.splitlines():
        if line.startswith("===="):
            break
        s = line.split("#", 1)[0].strip()
        if not s or "=" in s:
            continue
        typ, name = s.split()[:2]
        fields.append((typ, name))
    return fields


def decode_cdr(schema_name: str | None, schema_text: str | None, data: bytes):
    """→ dict of fields (or {'data': str} for std_msgs/String)."""
    if schema_name in (None, STRING_TYPE):
        c = Cdr(data)
        return {"data": c.string()}
    if not schema_text:
        raise ValueError(f"no schema text for {schema_name}; cannot decode without ROS")
    c = Cdr(data)
    out = {}
    for typ, name in parse_msgdef(schema_text):
        if typ.endswith("[]"):
            base = typ[:-2]
            out[name] = c.seq_prim(base) if base in _PRIM else [c.string() for _ in range(c.prim("uint32"))]
        elif typ == "string":
            out[name] = c.string()
        elif typ in _PRIM:
            out[name] = c.prim(typ)
        else:
            raise ValueError(f"{schema_name}: field {name} of type {typ} is not supported by this decoder")
    return out
