#!/usr/bin/env python3
"""foxglove_json.py — a String-topic arena bag → a Foxglove-plottable MCAP (JSON messages + JSON Schema).

    python foxglove_json.py <bag dir or .mcap> <out.mcap> [--ball-mm 9]

The default (String) environment records every topic as std_msgs/String holding JSON text, so
Foxglove shows each message as one string and cannot plot a field. This rewrites the same
messages, same topics, same log/publish times, as MCAP `json` messages with a `jsonschema`
schema per topic, so `/arena/frame.idx`, `/bias/status.framesPerSec`, … plot directly and
`docs/foxglove-arena-layout.json` works unchanged. Per topic:

- every topic: the JSON object as recorded (a non-object payload becomes {"value": …});
- /arena/frame: plus `bias_deg` (= `bias`, 0 when absent) and `turning_dps` / `forward_mms`,
  derived exactly as bridge_node.FrameDeriver does for the typed ArenaFrame (two-point
  differences on the FicTrac camera clock `ft`, else the bridge clock `t`; null where the
  typed message has NaN). These three are the only fields the original does not carry;
- /fictrac/raw: `cols`, FicTrac's 25 output columns by name, parsed from `raw`;
- /arena/browser: `msg`, the browser message parsed from `raw` (when it is JSON).

Slim by default: where `raw` was parsed into `cols` / `msg`, the `raw` text is dropped (it is the
same data twice); `--keep-raw` keeps it. The original bag stays the record of truth.

The source is read record by record, so a crash-torn bag (no metadata.yaml, no index) converts
too; the output is chunked, zstd-compressed and indexed. A metadata record `foxglove_json`
names the source file and its sha256.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path

FICTRAC_COLS = [
    "frame",                                    # 1  frame counter
    "dr_cam_x", "dr_cam_y", "dr_cam_z",         # 2-4  delta rotation vector, camera frame (rad)
    "dr_err",                                   # 5  delta rotation error score
    "dr_lab_x", "dr_lab_y", "dr_lab_z",         # 6-8  delta rotation vector, lab frame (rad)
    "r_cam_x", "r_cam_y", "r_cam_z",            # 9-11 absolute rotation vector, camera frame
    "r_lab_x", "r_lab_y", "r_lab_z",            # 12-14 absolute rotation vector, lab frame
    "x", "y",                                   # 15-16 integrated x/y position, lab (rad)
    "heading",                                  # 17 integrated animal heading, lab (rad)
    "direction",                                # 18 animal movement direction, lab (rad)
    "speed",                                    # 19 animal movement speed (rad/frame)
    "forward", "side",                          # 20-21 integrated forward / side motion (rad)
    "timestamp",                                # 22 frame timestamp (ms)
    "seq",                                      # 23 sequence counter
    "dt",                                       # 24 delta timestamp (ms)
    "alt_timestamp",                            # 25 alternative timestamp (ms)
]


def wrap_to_pi(r: float) -> float:
    return (r + math.pi) % (2 * math.pi) - math.pi


class FrameDeriver:
    """bridge_node.FrameDeriver, verbatim math (rig/nodes/bridge_node.py)."""

    def __init__(self, ball_mm: float) -> None:
        self.radius_mm = ball_mm / 2.0
        self._prev = None

    def step(self, msg: dict):
        try:
            src, clock = ("ft", float(msg["ft"])) if msg.get("ft") is not None else ("t", float(msg["t"]))
            cur = (int(msg["fc"]), src, clock, float(msg["x"]), float(msg["y"]), float(msg["hd"]))
        except (KeyError, TypeError, ValueError):
            self._prev = None
            return None, None
        prev, self._prev = self._prev, cur
        if prev is None or prev[1] != src or cur[0] < prev[0]:
            return None, None
        dt_s = (clock - prev[2]) / 1000.0
        if not dt_s > 0:
            return None, None
        d_hd = wrap_to_pi(cur[5] - prev[5])
        h_mid = prev[5] + d_hd / 2.0
        dx, dy = cur[3] - prev[3], cur[4] - prev[4]
        return math.degrees(d_hd) / dt_s, (dx * math.cos(h_mid) + dy * math.sin(h_mid)) / dt_s * self.radius_mm


def cdr_string(data: bytes) -> str:
    """std_msgs/String in CDR: 4-byte encapsulation header, uint32 length (incl. NUL), bytes."""
    little = data[1] == 1
    (n,) = struct.unpack_from("<I" if little else ">I", data, 4)
    return data[8:8 + n].rstrip(b"\x00").decode("utf-8")


def finite(v):
    """JSON has no NaN/Infinity — Foxglove's parser would reject the message."""
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: finite(x) for k, x in v.items()}
    if isinstance(v, list):
        return [finite(x) for x in v]
    return v


def json_type(v) -> str:
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "boolean"
    if isinstance(v, (int, float)):
        return "number"
    if isinstance(v, str):
        return "string"
    if isinstance(v, list):
        return "array"
    return "object"


def merge_schema(s: dict, v, depth: int = 0) -> dict:
    t = json_type(v)
    types = set(s.get("type", []) if isinstance(s.get("type"), list) else ([s["type"]] if "type" in s else []))
    types.add(t)
    s["type"] = sorted(types)
    if depth < 6 and t == "object":
        props = s.setdefault("properties", {})
        for k, x in v.items():
            props[k] = merge_schema(props.get(k, {}), x, depth + 1)
    elif depth < 6 and t == "array":
        items = s.get("items", {})
        for x in v[:64]:
            items = merge_schema(items, x, depth + 1)
        if items:
            s["items"] = items
    return s


TYPE_PRIORITY = ("object", "array", "string", "number", "boolean")


def finish_schema(s: dict) -> dict | None:
    """One `type` per node. Foxglove rejects type unions ("Unsupported type ["boolean","number"]",
    seen on /arena/browser msg.value and /bias/reply steps.value), so a field seen with several
    types gets the widest one (object > array > string > number > boolean; null dropped). A field
    that was only ever null is left out of the schema (nothing to plot; raw views still show it)."""
    t = s.get("type")
    types = [x for x in (t if isinstance(t, list) else [t]) if x and x != "null"]
    if not types:
        return None
    s["type"] = next(p for p in TYPE_PRIORITY if p in types)
    if s["type"] != "object":
        s.pop("properties", None)
    if s["type"] != "array":
        s.pop("items", None)
    for k in list(s.get("properties", {})):
        if finish_schema(s["properties"][k]) is None:
            del s["properties"][k]
    if "items" in s and finish_schema(s["items"]) is None:
        del s["items"]
    if s["type"] == "array" and "items" not in s:
        # an array that was always empty (e.g. /arena/browser msg.flagged_trials): Foxglove needs
        # an item type ("Cannot read properties of undefined (reading 'type')" without one)
        s["items"] = {"type": "string"}
    return s


def read_messages(path: Path):
    """(topic, schema_name, log_time, publish_time, sequence, data) for every message, torn tail tolerated."""
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader

    schemas, channels, out, torn = {}, {}, [], None
    with open(path, "rb") as f:
        try:
            for rec in StreamReader(f, skip_magic=False, emit_chunks=False, validate_crcs=False).records:
                if isinstance(rec, Schema):
                    schemas[rec.id] = rec
                elif isinstance(rec, Channel):
                    channels[rec.id] = rec
                elif isinstance(rec, Message):
                    ch = channels[rec.channel_id]
                    sc = schemas.get(ch.schema_id)
                    out.append((ch.topic, sc.name if sc else None, rec.log_time, rec.publish_time, rec.sequence, rec.data))
        except Exception as e:  # a SIGKILLed recorder leaves a partial record and no footer
            torn = f"{type(e).__name__}: {e}"
    return out, torn


def convert(src: Path, dst: Path, ball_mm: float, keep_raw: bool = False) -> dict:
    from mcap.writer import CompressionType, Writer

    if src.is_dir():
        src = next(p for p in sorted(src.glob("*.mcap")))
    msgs, torn = read_messages(src)
    deriver = FrameDeriver(ball_mm)
    rows, schemas, skipped = [], {}, 0
    for topic, sname, log_t, pub_t, seq, data in msgs:
        if sname != "std_msgs/msg/String":
            skipped += 1
            continue
        try:
            obj = json.loads(cdr_string(data))
        except (ValueError, UnicodeDecodeError, struct.error):
            skipped += 1
            continue
        if not isinstance(obj, dict):
            obj = {"value": obj}
        if topic == "/arena/frame" and obj.get("type", "frame") == "frame":
            obj["bias_deg"] = float(obj.get("bias", 0.0) or 0.0)
            obj["turning_dps"], obj["forward_mms"] = deriver.step(obj)
        elif topic == "/fictrac/raw" and isinstance(obj.get("raw"), str):
            parts = [p.strip() for p in obj["raw"].split(",")]
            if parts and parts[0] == "FT":
                parts = parts[1:]
            try:
                obj["cols"] = {k: float(v) for k, v in zip(FICTRAC_COLS, parts)}
                if not keep_raw:
                    del obj["raw"]
            except ValueError:
                pass
        elif topic == "/arena/browser" and isinstance(obj.get("raw"), str):
            try:
                obj["msg"] = json.loads(obj["raw"])
                if not keep_raw:
                    del obj["raw"]
            except ValueError:
                pass
        obj = finite(obj)
        schemas[topic] = merge_schema(schemas.get(topic, {}), obj)
        rows.append((topic, log_t, pub_t, seq, json.dumps(obj, separators=(",", ":")).encode()))

    sha = hashlib.sha256(src.read_bytes()).hexdigest()
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "wb") as f:
        w = Writer(f, compression=CompressionType.ZSTD)
        w.start(profile="", library="rig foxglove_json.py")
        chan = {}
        for topic in sorted(schemas):
            sc = finish_schema(schemas[topic]) or {"type": "object"}
            sc = {"$schema": "https://json-schema.org/draft/2020-12/schema", "title": topic, **sc}
            sid = w.register_schema(name=topic.strip("/").replace("/", "_"), encoding="jsonschema",
                                    data=json.dumps(sc).encode())
            chan[topic] = w.register_channel(topic=topic, message_encoding="json", schema_id=sid)
        for topic, log_t, pub_t, seq, data in sorted(rows, key=lambda r: r[1]):
            w.add_message(channel_id=chan[topic], log_time=log_t, publish_time=pub_t, sequence=seq, data=data)
        w.add_metadata("foxglove_json", {"source": src.name, "source_sha256": sha, "ball_mm": f"{ball_mm:g}",
                                         "raw_kept": "yes" if keep_raw else "no", "torn_source": torn or "no"})
        w.finish()
    counts = {}
    for r in rows:
        counts[r[0]] = counts.get(r[0], 0) + 1
    return {"source": src.name, "messages": len(rows), "skipped": skipped, "torn": torn, "topics": counts,
            "bytes": dst.stat().st_size}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--ball-mm", type=float, default=9.0, help="ball diameter for forward_mms (default 9, as the bridge)")
    ap.add_argument("--keep-raw", action="store_true", help="keep the `raw` text beside the parsed `cols` / `msg`")
    a = ap.parse_args(argv)
    print(json.dumps(convert(a.src, a.dst, a.ball_mm, a.keep_raw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
