"""columns.py — size a bridge-node bag vs its jsonl.gz under the FicTrac-record layouts.

    pixi run size-columns -- runlogs/<run>.bag [runlogs/<run>.jsonl] [--all]

The study behind docs/development/fictrac-columns-in-the-bag.md (decided 2026-10-18): ONE record per
FicTrac frame on /arena/frame carrying 10 of the 25 FicTrac columns, float64, one MCAP message per
frame, chunk-only index, zstd 19 — measured on the two 2026-10-14 bench bags at 2.24× / 2.48× the
jsonl.gz. Every variant is written by the pure-Python `mcap` writer (the same one mcap_index.reindex
uses for the final layout), so the numbers are what a cleanly closed run ends up with.

Without --all: the reference jsonl.gz, the bag as it is on disk, and the ADOPTED layout rebuilt from
the bag's records. With --all: the alternatives that were measured and declined (two-topic layouts,
column subsets, 1 s batching, int32 fixed-point), as in the doc's §3 table. A bag recorded before
2026-10-18 holds the raw FicTrac topics (/fictrac/frame or /fictrac/raw), so every variant can be
rebuilt from it; a bag recorded after holds the kept columns in /arena/frame only, and the
25-column rows are reported as "n/a".

Pure Python (mcap, zstandard); no ROS process needed.
"""

from __future__ import annotations

import argparse
import collections
import gzip
import io
import json
import math
import os
import struct
import sys
from pathlib import Path

import zstandard
from mcap.reader import make_reader
from mcap.writer import CompressionType, IndexType, Writer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
from mcap_convert import decode_cdr  # noqa: E402  (the converter's CDR decoder; reads the schema text in the bag)
from mcap_index import ZSTD_LEVEL, _zstd_level  # noqa: E402

TOPIC_FRAME = "/arena/frame"
FICTRAC_TOPICS = ("/fictrac/frame", "/fictrac/raw")

# 1-based FicTrac columns (doc §1)
BEHAVIOR_5 = [1, 15, 16, 17, 22]
KEPT_10 = [1, 5, 6, 7, 8, 15, 16, 17, 22, 23]
ALL_25 = list(range(1, 26))
EXTRA_COLS = [5, 6, 7, 8, 23]  # kept beyond the behavior row (col 22 native rides as ft_ns)
INT_COLS = {1, 22, 23}  # packed as int64 (col 22 = ns on our rigs)

ARENA_FLOATS = ("ft", "x", "y", "hd", "bias_deg", "turning_dps", "forward_mms", "err", "dr_lab_x", "dr_lab_y", "dr_lab_z")


def load(path: Path) -> list[tuple]:
    """(topic, schema name, log_ns, pub_ns, data, decoded) per record, file order."""
    files = sorted(path.glob("*.mcap")) if path.is_dir() else [path]
    recs = []
    for f in files:
        with open(f, "rb") as fh:
            for schema, ch, m in make_reader(fh).iter_messages(log_time_order=False):
                sn = schema.name if schema else None
                st = schema.data.decode() if schema and schema.data else None
                recs.append((ch.topic, sn, m.log_time, m.publish_time, m.data, decode_cdr(sn, st, m.data)))
    return recs


def fictrac_fields(sn: str | None, msg: dict) -> list[float] | None:
    if sn == "rig_interfaces/msg/FicTracFrame":
        return list(msg["fields"])
    try:
        env = json.loads(msg["data"])
        parts = env["raw"].strip().split(",")
    except (KeyError, TypeError, json.JSONDecodeError):
        return None
    if parts and parts[0].strip()[:2].upper() == "FT":
        parts = parts[1:]
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def arena_fields(sn: str | None, msg: dict) -> dict:
    """The /arena/frame record as a flat dict, from the typed message or the String-path JSON."""
    if sn == "rig_interfaces/msg/ArenaFrame":
        return dict(msg)
    o = json.loads(msg["data"])
    return {"t_ms": o["t"], "ms": o["ms"], "fc": o["fc"], "idx": o["idx"], "has_ft": o.get("ft") is not None,
            "ft": o.get("ft") if o.get("ft") is not None else math.nan, "x": o.get("x"), "y": o.get("y"), "hd": o.get("hd"),
            "epoch": o.get("epoch", 0), "bias_deg": o.get("bias", 0.0) or 0.0, "seq": o.get("wseq", 0),
            "turning_dps": math.nan, "forward_mms": math.nan,
            "err": o.get("err", math.nan), "dr_lab_x": o.get("dr_lab_x", math.nan), "dr_lab_y": o.get("dr_lab_y", math.nan),
            "dr_lab_z": o.get("dr_lab_z", math.nan), "ft_seq": o.get("ft_seq", 0), "ft_ns": o.get("ft_ns", 0), "flags": o.get("flags", 0)}


def _f(v) -> float:
    return float(v) if v is not None else math.nan


def pack_num(v, enc: str) -> bytes:
    if enc == "f64":
        return struct.pack("<d", _f(v))
    if enc == "f32":
        return struct.pack("<f", _f(v))
    if enc == "fix5":  # int32 × 1e-5 (the jsonl's 5-decimal rounding); NaN → INT_MIN
        x = _f(v)
        return struct.pack("<i", -(2**31) if math.isnan(x) else int(round(x * 1e5)))
    raise ValueError(enc)


def pack_fictrac(fields: list[float], cols: list[int], enc: str) -> bytes:
    out = b""
    for c in cols:
        v = fields[c - 1] if len(fields) >= c else math.nan
        out += struct.pack("<q", int(v) if not math.isnan(_f(v)) else 0) if c in INT_COLS else pack_num(v, enc)
    return out


def pack_arena(a: dict, enc: str, extras: bool, dedup: bool = False, derived: bool = False) -> bytes:
    """≈ the typed ArenaFrame's CDR cost: ints as declared, floats per `enc`; `extras` adds the
    kept FicTrac columns (the adopted record); `dedup` drops x/y/hd/ft (the join-by-fc variant);
    `derived` keeps the live turning_dps/forward_mms values — the adopted record stores NaN there
    (a constant, free), which is what the node records since 2026-10-18."""
    out = struct.pack("<qiqi", int(a["t_ms"]), int(a["ms"]), int(a["fc"]), int(a["idx"]))
    if not dedup:
        # `ft` is a ms clock (grows past int32 × 1e-5 within 21 s) and `bias_deg` a degree angle: both
        # stay float64 in every encoding; the fixed-point variant quantizes the radian columns only.
        out += struct.pack("<?", bool(a["has_ft"])) + pack_num(a["ft"], "f64")
        out += b"".join(pack_num(a[k], enc) for k in ("x", "y", "hd"))
    out += struct.pack("<I", int(a["epoch"])) + pack_num(a["bias_deg"], "f64") + struct.pack("<Q", int(a["seq"]))
    if derived:
        out += struct.pack("<dd", _f(a.get("turning_dps")) if a.get("turning_dps") == a.get("turning_dps") else 0.0,
                           _f(a.get("forward_mms")) if a.get("forward_mms") == a.get("forward_mms") else 0.0)
    else:
        out += struct.pack("<dd", 0.0, 0.0)  # NaN in the record: a constant
    if extras:
        out += b"".join(pack_num(a.get(k), enc) for k in ("err", "dr_lab_x", "dr_lab_y", "dr_lab_z"))
        out += struct.pack("<qqB", int(a.get("ft_seq") or 0), int(a.get("ft_ns") or 0), int(a.get("flags") or 0))
    return out


def write_variant(recs: list[tuple], *, layout: str, cols: list[int] | None, enc: str, batch_ft: bool, batch_arena: bool,
                  level: int = ZSTD_LEVEL) -> int | None:
    """Rewrite the bag's records in one layout; returns the MCAP's size in bytes.
    layout: 'merged' (the adopted record: /arena/frame + extras, no FicTrac topic) · 'split' (a FicTrac
    topic with `cols` beside today's ArenaFrame) · 'dedup' (split, ArenaFrame without x/y/hd/ft) ·
    'floor' (ArenaFrame alone, no extras). None when the bag lacks what the variant needs."""
    ft_by_fc: dict[int, list[float]] = {}
    have_ft_topic = any(r[0] in FICTRAC_TOPICS for r in recs)
    if layout in ("split", "dedup") and not have_ft_topic:
        return None
    if have_ft_topic:
        for topic, sn, lt, pt, data, msg in recs:
            if topic in FICTRAC_TOPICS:
                f = fictrac_fields(sn, msg)
                if f and len(f) >= 1:
                    ft_by_fc[int(f[0])] = f
    buf = io.BytesIO()
    with _zstd_level(level):
        w = Writer(buf, compression=CompressionType.ZSTD, index_types=IndexType.ALL & ~IndexType.MESSAGE)
        w.start(profile="ros2", library="columns.py")
        chans: dict[str, int] = {}

        def chan(name: str) -> int:
            if name not in chans:
                chans[name] = w.register_channel(name, "cdr", w.register_schema(name, "ros2msg", b""))
            return chans[name]

        seq = 0
        buckets: dict[tuple, list] = collections.OrderedDict()

        def emit(topic: str, lt: int, pt: int, payload: bytes, batch: bool) -> None:
            nonlocal seq
            if batch:
                buckets.setdefault((topic, lt // 1_000_000_000), []).append((lt, payload))
            else:
                seq += 1
                w.add_message(chan(topic), log_time=lt, publish_time=pt, data=payload, sequence=seq)

        for topic, sn, lt, pt, data, msg in recs:
            if topic in FICTRAC_TOPICS:
                if layout in ("split", "dedup") and cols:
                    f = fictrac_fields(sn, msg)
                    if f is not None:
                        emit("/fictrac/frame", lt, pt, pack_fictrac(f, cols, enc), batch_ft)
                continue  # merged / floor: the raw topic is not recorded
            if topic == TOPIC_FRAME:
                a = arena_fields(sn, msg)
                if layout == "merged":
                    # The extras come from the record itself (a bag after 2026-10-18) or, for an old
                    # bag, from the raw topic joined by fc — what the node now writes directly.
                    if math.isnan(_f(a.get("err"))) and int(a["fc"]) in ft_by_fc:
                        f = ft_by_fc[int(a["fc"])]
                        a.update({"err": f[4] if len(f) > 4 else math.nan, "dr_lab_x": f[5] if len(f) > 5 else math.nan,
                                  "dr_lab_y": f[6] if len(f) > 6 else math.nan, "dr_lab_z": f[7] if len(f) > 7 else math.nan,
                                  "ft_seq": int(f[22]) if len(f) > 22 else 0, "ft_ns": int(f[21]) if len(f) > 21 else 0})
                    payload = pack_arena(a, enc, extras=True)
                elif layout == "dedup":
                    payload = pack_arena(a, enc, extras=False, dedup=True)
                else:
                    payload = pack_arena(a, enc, extras=False, derived=True)  # the pre-2026-10-18 record as it was
                emit(topic, lt, pt, payload, batch_arena)
            else:
                emit(topic, lt, pt, data, False)
        for (topic, _b), items in sorted(buckets.items(), key=lambda kv: kv[0][1]):
            body = struct.pack("<I", len(items)) + b"".join(struct.pack("<q", t) + struct.pack("<H", len(p)) + p for t, p in items)
            seq += 1
            w.add_message(chan(topic + "/batch"), log_time=items[0][0], publish_time=items[0][0], data=body, sequence=seq)
        w.finish()
    return len(buf.getvalue())


ADOPTED = dict(layout="merged", cols=KEPT_10, enc="f64", batch_ft=False, batch_arena=False)
ALTERNATIVES = [
    ("two topics per frame, all 25 cols float64, both batched 1 s", dict(layout="split", cols=ALL_25, enc="f64", batch_ft=True, batch_arena=True)),
    ("two topics per frame, behavior 5 cols, batched", dict(layout="split", cols=BEHAVIOR_5, enc="f64", batch_ft=True, batch_arena=True)),
    ("two topics per frame, kept 10 cols, batched", dict(layout="split", cols=KEPT_10, enc="f64", batch_ft=True, batch_arena=True)),
    ("two topics, kept 10, ArenaFrame without x/y/hd/ft (join by fc), batched", dict(layout="dedup", cols=KEPT_10, enc="f64", batch_ft=True, batch_arena=True)),
    ("one record per frame, float64, batched 1 s (declined)", dict(layout="merged", cols=KEPT_10, enc="f64", batch_ft=False, batch_arena=True)),
    ("one record per frame, int32 fixed-point x1e-5, batched (declined)", dict(layout="merged", cols=KEPT_10, enc="fix5", batch_ft=False, batch_arena=True)),
    ("one record per frame, int32 fixed-point x1e-5, per record (declined)", dict(layout="merged", cols=KEPT_10, enc="fix5", batch_ft=False, batch_arena=False)),
    ("floor: the pre-2026-10-18 ArenaFrame alone (derived channels recorded, no FicTrac extras), batched", dict(layout="floor", cols=None, enc="f64", batch_ft=False, batch_arena=True)),
]


def study(bag: Path, jsonl: Path | None, all_variants: bool) -> dict:
    recs = load(bag)
    n_frames = sum(1 for r in recs if r[0] == TOPIC_FRAME)
    on_disk = sum(f.stat().st_size for f in (bag.glob("*.mcap") if bag.is_dir() else [bag]))
    rows: list[tuple[str, int | None]] = []
    gz = None
    if jsonl is None:
        cand = bag.parent / (bag.name[:-4] + ".jsonl") if bag.name.endswith(".bag") else None
        if cand and cand.exists():
            jsonl = cand
    if jsonl and jsonl.exists():
        raw = jsonl.read_bytes()
        gz = len(raw) if jsonl.suffix == ".gz" else len(gzip.compress(raw, 9))
        rows.append(("jsonl.gz (reference)", gz))
    rows.append(("bag as on disk", on_disk))
    rows.append(("ADOPTED: one record per frame, kept 10 cols, float64, one message per frame, chunk-only index, zstd 19",
                 write_variant(recs, **ADOPTED)))
    if all_variants:
        for label, kw in ALTERNATIVES:
            rows.append((label, write_variant(recs, **kw)))
    return {"bag": str(bag), "frames": n_frames, "jsonl_gz": gz, "rows": rows}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="a rosbag2 bag directory or a single .mcap file")
    ap.add_argument("jsonl", nargs="?", help="the run's exported .jsonl or .jsonl.gz (default: <bag stem>.jsonl beside the bag)")
    ap.add_argument("--all", action="store_true", help="also the declined alternatives (doc §3 table)")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    a = ap.parse_args(argv)
    res = study(Path(a.bag), Path(a.jsonl) if a.jsonl else None, a.all)
    if a.json:
        print(json.dumps(res))
        return 0
    gz = res["jsonl_gz"]
    print(f"{Path(res['bag']).name}: {res['frames']} frames" + (f"; jsonl.gz {gz} B" if gz else "; no jsonl beside the bag (ratios omitted)"))
    print(f"{'variant':100s} {'bytes':>9s} {'x jsonl.gz':>10s} {'B/frame':>8s}")
    for label, n in res["rows"]:
        if n is None:
            print(f"{label:100s} {'n/a':>9s} {'':>10s} {'':>8s}   (needs the raw FicTrac topics)")
            continue
        ratio = f"{n / gz:9.2f}x" if gz else ""
        per = f"{n / res['frames']:8.1f}" if res["frames"] else ""
        print(f"{label:100s} {n:9d} {ratio:>10s} {per:>8s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
