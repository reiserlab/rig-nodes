"""compare.py — size one behavior_v2 run log (.jsonl or .jsonl.gz) as NDJSON vs MCAP layouts.

    pixi run size-study -- path/to/run.jsonl[.gz]

Size comparison: our behavior_v2 NDJSON run log vs the same content in MCAP.

Variants:
  ndjson            raw / gzip-9 (what the bridge writes) / gzip-6 / zstd-3 / zstd-19 / lz4
  mcap-json         one channel per stream or event type, JSON-encoded messages with a JSON schema,
                    chunk compression none / lz4 / zstd   (what the Foxglove SDK would produce)
  mcap-packed       high-rate rows as fixed little-endian structs (≈ what typed ROS 2 CDR messages
                    cost; CDR adds a 4-byte header + alignment), events still JSON; zstd
"""
import gzip, io, json, struct, sys, math, zlib
import zstandard, lz4.frame
from mcap.writer import Writer, CompressionType

path = sys.argv[1]
raw = open(path, "rb").read()
if raw[:2] == b"\x1f\x8b":
    raw = gzip.decompress(raw)
objs = [json.loads(l) for l in raw.decode().splitlines() if l.strip()]
t0 = next(o["ms"] for o in objs if isinstance(o, dict) and o.get("type") == "session")

def kinds(o):
    if isinstance(o, list):
        return "fictrac/frame" if isinstance(o[0], (int, float)) else f"stream/{o[0]}"
    t = o.get("type")
    return f"event/{t}" + (f"/{o.get('event')}" if t == "log" else "")

def log_time_ns(o):
    if isinstance(o, list):
        if isinstance(o[0], (int, float)):
            rel = o[0]
        elif o[0] == "a":
            rel = o[1]
        else:
            rel = o[1] if isinstance(o[1], (int, float)) else 0
        return int((t0 + rel) * 1e6)
    if isinstance(o.get("ms"), (int, float)):
        ms = o["ms"]
        return int((ms if ms > 1e11 else t0 + ms) * 1e6)
    if isinstance(o.get("rx_ms"), (int, float)):
        return int(o["rx_ms"] * 1e6)
    return int(t0 * 1e6)


import mcap.writer as MW

def write_mcap(level=3, msg_index=True, batch_streams=(), batch_s=1.0):
    """JSON-schema channels, one per stream/event type; optional 1 s batching of named streams;
    zstd chunk compression at `level` (the library hard-codes 3, so it is swapped underneath)."""
    saved = MW.zstandard.compress
    MW.zstandard.compress = lambda d, _l=level: zstandard.ZstdCompressor(level=_l).compress(d)
    try:
        buf = io.BytesIO()
        from mcap.writer import IndexType
        w = Writer(buf, compression=CompressionType.ZSTD, index_types=IndexType.ALL if msg_index else (IndexType.ALL & ~IndexType.MESSAGE))
        w.start(profile="", library="compare.py")
        chans = {}
        def chan(name):
            if name not in chans:
                chans[name] = w.register_channel(name, "json", w.register_schema(name, "jsonschema", b"{}"))
            return chans[name]
        seq = 0; buckets = {}
        for o in objs:
            k = kinds(o); ts = log_time_ns(o)
            if k in batch_streams:
                buckets.setdefault((k, int(ts // int(batch_s * 1e9))), []).append(o); continue
            seq += 1
            w.add_message(chan(k), log_time=ts, publish_time=ts, data=json.dumps(o, separators=(",", ":")).encode(), sequence=seq)
        for (k, b), rows in sorted(buckets.items(), key=lambda kv: kv[0][1]):
            ts = log_time_ns(rows[0]); seq += 1
            w.add_message(chan(k + "/batch"), log_time=ts, publish_time=ts, data=json.dumps(rows, separators=(",", ":")).encode(), sequence=seq)
        w.finish()
        return len(buf.getvalue())
    finally:
        MW.zstandard.compress = saved

n = len(objs)
g9 = len(gzip.compress(raw, 9))
print(f"{path.rsplit('/', 1)[-1]}: {n} records")
print(f"{'variant':62s} {'bytes':>9s} {'B/rec':>6s} {'vs gzip-9':>9s}")
for name, b in [
    ("NDJSON raw", len(raw)),
    ("NDJSON gzip-9 (bridge 3.4 writes this)", g9),
    ("NDJSON zstd-19", len(zstandard.ZstdCompressor(level=19).compress(raw))),
    ("MCAP per record, rosbag2/SDK defaults (zstd-3, message index)", write_mcap()),
    ("MCAP per record, no message index, zstd-3", write_mcap(msg_index=False)),
    ("MCAP per record, no message index, zstd-19", write_mcap(level=19, msg_index=False)),
    ("MCAP tuned: arena echoes batched 1 s, no msg index, zstd-19", write_mcap(level=19, msg_index=False, batch_streams=("stream/a",))),
    ("MCAP all high-rate batched 1 s, zstd-19", write_mcap(level=19, msg_index=False, batch_streams=("stream/a", "stream/kf", "fictrac/frame"))),
]:
    print(f"{name:62s} {b:9d} {b/n:6.1f} {b/g9:8.2f}×")
