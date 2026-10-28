#!/usr/bin/env python3
"""foxglove_json.py — a String-topic bag → a Foxglove-plottable MCAP (JSON messages + JSON Schema).

    python -m rig_nodes.foxglove_json <bag dir or .mcap> <out.mcap>

A rig's default (String) environment records every topic as std_msgs/String holding JSON text, so
Foxglove shows each message as one string and cannot plot a field. This rewrites the same
messages, same topics, same log/publish times, as MCAP `json` messages with a `jsonschema`
schema per topic (inferred from the messages), so any field plots directly. Per topic the JSON
object is written as recorded (a non-object payload becomes {"value": …}); a payload that carries
the adapter envelope (CONTRACT.md) with a JSON `raw` gets `msg` = the parsed `raw`.

This module is the GENERIC rewriter. An application adds its own derivations through two hooks of
`convert()`: `enrich(topic, obj) -> obj | None` runs per message (return None to drop it) — the
arena gateway's wrapper in webDisplayTools (`rig/nodes/foxglove_json.py`) derives the closed-loop
channels on `/arena/frame` and names FicTrac's columns on `/fictrac/raw` that way; `derive(records)`
runs once over the whole (enriched, time-ordered) record stream and yields NEW messages — topics
that were never recorded, rebuilt from the ones that were (the arena's expected-display images and
health from the browser stream). A derived message may name a well-known Foxglove schema
(`WELL_KNOWN_SCHEMAS`, e.g. `foxglove.CompressedImage` for an image) instead of an inferred one.

The source is read record by record, so a crash-torn bag (no metadata.yaml, no index) converts
too; the output is chunked, zstd-compressed and indexed. A metadata record `foxglove_json`
names the source file and its sha256. The original bag stays the record of truth.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path
from typing import Callable, Iterable, Optional

from rig_nodes.cdr import STRING_TYPE, cdr_string


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


def parse_envelope_raw(obj: dict, keep_raw: bool = False) -> dict:
    """Generic enrichment: an envelope whose `raw` is JSON text gets `msg` = the parsed object
    (the `raw` text is dropped unless keep_raw — it is the same data twice)."""
    raw = obj.get("raw")
    if isinstance(raw, str) and raw[:1] in "{[":
        try:
            obj["msg"] = json.loads(raw)
            if not keep_raw:
                del obj["raw"]
        except ValueError:
            pass
    return obj


Enrich = Callable[[str, dict], Optional[dict]]
# derive(records) with records = [(topic, log_time_ns, obj)] in log-time order (every message the
# per-message pass kept, after enrich) → an iterable of (topic, time_ns, obj, schema_name | None):
# new messages, written at time_ns as both log and publish time; schema_name None = inferred from
# the messages like every recorded topic, else a WELL_KNOWN_SCHEMAS key.
Derive = Callable[[list], Iterable[tuple]]

# Foxglove's built-in schemas a derived topic may use by name, so the panels that need a specific
# type (Image, …) accept it: https://docs.foxglove.dev/docs/visualization/message-schemas
WELL_KNOWN_SCHEMAS: dict[str, dict] = {
    "foxglove.CompressedImage": {
        "title": "foxglove.CompressedImage", "type": "object", "properties": {
            "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}}},
            "frame_id": {"type": "string"}, "data": {"type": "string", "contentEncoding": "base64"}, "format": {"type": "string"}}},
}


def convert(src: Path, dst: Path, enrich: Enrich | None = None, keep_raw: bool = False,
            library: str = "rig_nodes foxglove_json", metadata: dict | None = None,
            derive: Derive | None = None) -> dict:
    """Rewrite `src` (a bag directory or one .mcap) as `dst`. `enrich(topic, obj)` may add
    derived fields or return None to drop the message; it runs after the generic envelope
    parsing. `derive(records)` then runs once over everything kept and adds new messages (see
    `Derive`). `metadata` adds key/values to the `foxglove_json` metadata record."""
    from mcap.writer import CompressionType, Writer

    if src.is_dir():
        src = next(p for p in sorted(src.glob("*.mcap")))
    msgs, torn = read_messages(src)
    rows, schemas, skipped = [], {}, 0
    records: list[tuple[str, int, dict]] = []  # for derive
    for topic, sname, log_t, pub_t, seq, data in msgs:
        if sname != STRING_TYPE:
            skipped += 1
            continue
        try:
            obj = json.loads(cdr_string(data))
        except (ValueError, UnicodeDecodeError, struct.error):
            skipped += 1
            continue
        if not isinstance(obj, dict):
            obj = {"value": obj}
        obj = parse_envelope_raw(obj, keep_raw)
        if enrich is not None:
            obj = enrich(topic, obj)
            if obj is None:
                skipped += 1
                continue
        obj = finite(obj)
        schemas[topic] = merge_schema(schemas.get(topic, {}), obj)
        rows.append((topic, log_t, pub_t, seq, json.dumps(obj, separators=(",", ":")).encode()))
        if derive is not None:
            records.append((topic, log_t, obj))

    fixed: dict[str, str] = {}  # derived topic → well-known schema name
    derived = 0
    if derive is not None:
        records.sort(key=lambda r: r[1])
        for topic, t_ns, obj, schema_name in derive(records):
            obj = finite(obj)
            if schema_name:
                if schema_name not in WELL_KNOWN_SCHEMAS:
                    raise ValueError(f"derive: unknown schema {schema_name!r} for {topic} (WELL_KNOWN_SCHEMAS)")
                fixed[topic] = schema_name
            else:
                schemas[topic] = merge_schema(schemas.get(topic, {}), obj)
            rows.append((topic, int(t_ns), int(t_ns), 0, json.dumps(obj, separators=(",", ":")).encode()))
            derived += 1
    for topic in fixed:  # a well-known schema wins over an inferred one for the same topic
        schemas.pop(topic, None)

    sha = hashlib.sha256(src.read_bytes()).hexdigest()
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "wb") as f:
        w = Writer(f, compression=CompressionType.ZSTD)
        w.start(profile="", library=library)
        chan = {}
        sid_by_name = {}
        for topic in sorted(fixed):
            name = fixed[topic]
            if name not in sid_by_name:
                sid_by_name[name] = w.register_schema(name=name, encoding="jsonschema", data=json.dumps(WELL_KNOWN_SCHEMAS[name]).encode())
            chan[topic] = w.register_channel(topic=topic, message_encoding="json", schema_id=sid_by_name[name])
        for topic in sorted(schemas):
            sc = finish_schema(schemas[topic]) or {"type": "object"}
            sc = {"$schema": "https://json-schema.org/draft/2020-12/schema", "title": topic, **sc}
            sid = w.register_schema(name=topic.strip("/").replace("/", "_"), encoding="jsonschema",
                                    data=json.dumps(sc).encode())
            chan[topic] = w.register_channel(topic=topic, message_encoding="json", schema_id=sid)
        for topic, log_t, pub_t, seq, data in sorted(rows, key=lambda r: r[1]):
            w.add_message(channel_id=chan[topic], log_time=log_t, publish_time=pub_t, sequence=seq, data=data)
        meta = {"source": src.name, "source_sha256": sha, "raw_kept": "yes" if keep_raw else "no",
                "torn_source": torn or "no"}
        meta.update({k: str(v) for k, v in (metadata or {}).items()})
        w.add_metadata("foxglove_json", meta)
        w.finish()
    counts = {}
    for r in rows:
        counts[r[0]] = counts.get(r[0], 0) + 1
    out = {"source": src.name, "messages": len(rows), "skipped": skipped, "torn": torn, "topics": counts,
           "bytes": dst.stat().st_size}
    if derive is not None:
        out["derived"] = derived
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--keep-raw", action="store_true", help="keep the `raw` text beside the parsed `msg`")
    a = ap.parse_args(argv)
    print(json.dumps(convert(a.src, a.dst, keep_raw=a.keep_raw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
