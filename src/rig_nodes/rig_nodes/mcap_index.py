#!/usr/bin/env python3
"""mcap_index.py — rewrite a finished bag as an indexed, chunked, compressed MCAP; recover a torn one.

    pixi run reindex -- runlogs/<run>.bag            # in place (keeps <file>.unindexed.mcap until verified, then deletes it)
    pixi run reindex -- runlogs/<run>.bag --keep     # keep the original beside the new file
    pixi run reindex -- runlogs/<run>.bag --recover  # a bag the bridge never closed (crash): salvage, index, metadata.yaml

Why: the bridge node records with the `crashsafe` profile — no chunking, no cache, every record
reaches the disk as it is written — so a SIGKILL loses at most the record in progress. The price is
an MCAP without a chunk/message index: readers must scan it ("Problems: file is unindexed" in
Foxglove; `ros2 bag` and our converter do not mind). Once a run has CLOSED cleanly there is no
crash left to survive, so the bridge calls `reindex()` from Recorder.close(): the same records,
same schemas/channels/metadata, written again by the `mcap` library with chunking, zstd and the
full index. Pure Python; the result is a standard MCAP (rosbag2's `metadata.yaml` is untouched).
The rewrite is verified by re-reading and counting messages per channel before the original is
removed; on any mismatch the original stays and the function reports it.

Recovery (`recover_bag`, LAB-303 adoption week): a bag whose writer was killed has no
`metadata.yaml` and an MCAP without footer/summary (a 0-byte file with rosbag2's default profile).
`recover_bag` (1) rewrites every `.mcap` with `reindex(recover=True)`, which stops at the torn
tail instead of raising, appends ONE `/arena/session` record `{"event":"recovered",…}` so the bag
itself says it was salvaged, and keeps the torn original as `<name>.torn.bak` (not `.mcap`, so no
reader picks it up); then (2) asks rosbag2's own `Reindexer` for `metadata.yaml`. Afterwards the
bag opens in Foxglove, `ros2 bag info`, and the converter WITHOUT `--recover`. The bridge node runs
this itself when a `log_export` meets such a bag (`needs_recovery`), so the Studio's export after
a bridge restart gets the salvaged log and a `recovered: true` flag.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

# The topic the recovery marker is appended to. The default is the arena gateway's session
# topic; another application passes its own (`recover_bag(..., session_topic=…)`).
SESSION_TOPIC = "/arena/session"
STRING_SCHEMA = ("std_msgs/msg/String", "ros2msg", b"string data")
RECOVERED_SEQ = 1 << 40  # sorts after every real emission seq (the converter orders by seq)


def cdr_string(s: str) -> bytes:
    """A std_msgs/String payload as rosbag2 writes it: XCDR1 LE header + uint32 length (incl.
    the NUL) + bytes + NUL, padded to 4 — what nodes/mcap_convert.py's decoder reads back."""
    raw = s.encode("utf-8") + b"\x00"
    body = struct.pack("<I", len(raw)) + raw
    return b"\x00\x01\x00\x00" + body + b"\x00" * ((-len(body)) % 4)


def _scan(src: Path, recover: bool):
    """Yield the records of `src` in file order. A torn tail raises unless `recover`, in which
    case the generator returns (torn:bool, error:str|None) via StopIteration.value."""
    from mcap.stream_reader import StreamReader

    torn, error = False, None
    with open(src, "rb") as fin:
        try:
            for rec in StreamReader(fin, emit_chunks=False).records:
                yield rec
        except Exception as exc:  # EndOfFile / a half-written record: the crash point
            if not recover:
                raise
            torn, error = True, f"{type(exc).__name__}: {exc}"
    return torn, error


ZSTD_LEVEL = 19  # the final layout's compression level (docs/development/fictrac-columns-in-the-bag.md §3)


class _zstd_level:
    """The `mcap` writer calls `zstandard.compress(data)` — level 3, no knob. Swap the module
    function for the duration of one rewrite (the same trick as sizing/compare.py); restored in
    __exit__ even on an exception. Process-wide, so never nest two rewrites concurrently."""

    def __init__(self, level: int) -> None:
        self.level = level

    def __enter__(self):
        import zstandard

        self._mod = zstandard
        self._saved = zstandard.compress
        lvl = self.level
        zstandard.compress = lambda data, _l=lvl: zstandard.ZstdCompressor(level=_l).compress(data)
        return self

    def __exit__(self, *exc) -> None:
        self._mod.compress = self._saved


def reindex(mcap_path: str | os.PathLike, keep_original: bool = False, compression: str = "zstd",
            recover: bool = False, marker: dict | None = None, level: int = ZSTD_LEVEL) -> dict:
    """Rewrite one MCAP indexed + compressed. `recover=True` tolerates a torn tail (the records
    read up to it are kept), appends the `recovered` session marker (`marker` adds fields) and
    keeps the original as `<name>.torn.bak`. Returns a report; `verified` False = nothing replaced.

    The final layout (2026-10-18): a CHUNK index only — no per-message index, which costs more
    than the messages of a 50 Hz stream compress to — and zstd at `level` (19; the mcap writer's
    own 3 is a streaming default). Readers seek by chunk; Foxglove and `ros2 bag` are unaffected."""
    from mcap.records import Attachment, Channel, Header, Message, Metadata, Schema
    from mcap.writer import CompressionType, IndexType, Writer

    src = Path(mcap_path)
    tmp = src.with_suffix(".indexed.tmp")
    comp = {"zstd": CompressionType.ZSTD, "lz4": CompressionType.LZ4, "none": CompressionType.NONE}[compression]
    t0 = time.perf_counter()
    schema_map: dict[int, int] = {}
    channel_map: dict[int, int] = {}
    topic_of: dict[int, str] = {}
    want: dict[str, int] = {}
    n = 0
    torn, error = False, None
    marker_written = False
    with open(tmp, "wb") as fout, _zstd_level(level if compression == "zstd" else 3):
        w = Writer(fout, compression=comp, index_types=IndexType.ALL & ~IndexType.MESSAGE)
        started = False
        gen = _scan(src, recover)
        while True:
            try:
                rec = next(gen)
            except StopIteration as stop:
                if stop.value:
                    torn, error = stop.value
                break
            if isinstance(rec, Header):
                w.start(profile=rec.profile, library=f"{rec.library} + mcap_index.py")
                started = True
            elif isinstance(rec, Schema):
                if rec.id not in schema_map:
                    schema_map[rec.id] = w.register_schema(name=rec.name, encoding=rec.encoding, data=rec.data)
            elif isinstance(rec, Channel):
                if rec.id not in channel_map:
                    channel_map[rec.id] = w.register_channel(topic=rec.topic, message_encoding=rec.message_encoding,
                                                             schema_id=schema_map.get(rec.schema_id, 0), metadata=rec.metadata)
                    topic_of[rec.id] = rec.topic
            elif isinstance(rec, Message):
                if not started:
                    raise ValueError("message before header")
                w.add_message(channel_id=channel_map[rec.channel_id], log_time=rec.log_time, data=rec.data,
                              publish_time=rec.publish_time, sequence=rec.sequence)
                want[topic_of[rec.channel_id]] = want.get(topic_of[rec.channel_id], 0) + 1
                n += 1
            elif isinstance(rec, Metadata):
                w.add_metadata(name=rec.name, data=rec.metadata)
            elif isinstance(rec, Attachment):
                w.add_attachment(create_time=rec.create_time, log_time=rec.log_time, name=rec.name, media_type=rec.media_type, data=rec.data)
        if recover and started:
            # The bag says so itself: one session record, after everything salvaged. The
            # converter turns it into a `{"type":"session","event":"recovered",…}` line.
            ch = next((cid for sid, cid in channel_map.items() if topic_of.get(sid) == SESSION_TOPIC), None)
            if ch is None:
                sid = w.register_schema(name=STRING_SCHEMA[0], encoding=STRING_SCHEMA[1], data=STRING_SCHEMA[2])
                ch = w.register_channel(topic=SESSION_TOPIC, message_encoding="cdr", schema_id=sid)
            now_ns = time.time_ns()
            body = {"seq": RECOVERED_SEQ, "event": "recovered", "ms": now_ns // 1_000_000, "recovered": True,
                    "salvaged_records": n, "torn": torn, "torn_error": error, "file": src.name, **(marker or {})}
            w.add_message(channel_id=ch, log_time=now_ns, publish_time=now_ns, sequence=0,
                          data=cdr_string(json.dumps(body, separators=(",", ":"))))
            want[SESSION_TOPIC] = want.get(SESSION_TOPIC, 0) + 1
            marker_written = True
        if started:
            w.finish()
    if not started:
        # Nothing readable (rosbag2's default profile leaves a 0-byte file when killed early).
        tmp.unlink(missing_ok=True)
        return {"file": str(src), "messages": 0, "bytes_before": src.stat().st_size, "verified": False,
                "recovered": recover, "torn": True, "torn_error": error or "no MCAP header",
                "error": "nothing salvageable: the file holds no complete record",
                "seconds": round(time.perf_counter() - t0, 2)}
    # Verify: the new file must yield the same per-topic counts, through the indexed reader.
    from mcap.reader import make_reader

    with open(tmp, "rb") as fh:
        rd = make_reader(fh)
        summ = rd.get_summary()
        got: dict[str, int] = {}
        for _, chn, _ in rd.iter_messages(log_time_order=False):
            got[chn.topic] = got.get(chn.topic, 0) + 1
    ok = got == want and summ is not None and bool(summ.chunk_indexes)
    res = {"file": str(src), "messages": n, "bytes_before": src.stat().st_size, "bytes_after": tmp.stat().st_size,
           "chunks": len(summ.chunk_indexes) if summ else 0, "compression": compression,
           "level": level if compression == "zstd" else None, "message_index": False, "verified": ok,
           "seconds": round(time.perf_counter() - t0, 2)}
    if recover:
        res.update({"recovered": True, "torn": torn, "torn_error": error, "marker": marker_written})
    if not ok:
        tmp.unlink(missing_ok=True)
        res["error"] = f"verification failed: per-topic counts {got} vs {want}"
        return res
    # Replace ATOMICALLY (os.replace: rename(2) / MoveFileEx on the same filesystem). A kill
    # between an unlink and a rename used to leave a bag directory with metadata.yaml and no
    # .mcap at all — for the record of truth that is the one unacceptable state. With
    # `recover` / `keep_original` the source is moved aside first, so at every instant either
    # the original or the verified rewrite is in place under a name a reader will find.
    if recover:
        src.rename(src.with_name(src.name + ".torn.bak"))  # never a `.mcap` name: no reader must find it
    elif keep_original:
        src.rename(src.with_name(src.stem + ".unindexed.mcap"))
    os.replace(tmp, src)
    return res


def bag_mcaps(bag: Path) -> list[Path]:
    return sorted(f for f in bag.glob("*.mcap") if not f.name.endswith((".unindexed.mcap", ".indexed.tmp")))


def needs_recovery(bag_dir: str | os.PathLike) -> str | None:
    """Why a bag needs `recover_bag`, or None when it is whole: no `metadata.yaml` (the writer never
    closed), or an `.mcap` without a readable summary/footer (a torn last chunk)."""
    bag = Path(bag_dir)
    if not bag.is_dir():
        return None
    files = bag_mcaps(bag)
    if not files:
        return None
    if not (bag / "metadata.yaml").exists():
        return "no metadata.yaml (the writer never closed this bag)"
    from mcap.reader import make_reader

    for f in files:
        try:
            with open(f, "rb") as fh:
                if make_reader(fh, validate_crcs=False).get_summary() is None:
                    return f"{f.name}: no summary section (torn)"
        except Exception as exc:
            return f"{f.name}: {type(exc).__name__} reading the summary (torn)"
    return None


def write_metadata(bag_dir: Path) -> dict:
    """rosbag2's own Reindexer → `metadata.yaml` for the bag (`ros2 bag info` then works)."""
    try:
        import rosbag2_py
    except Exception as exc:  # pure-Python environment: the MCAP is still whole without it
        return {"written": False, "error": f"rosbag2_py unavailable: {exc!r}"}
    try:
        rosbag2_py.Reindexer().reindex(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"))
        return {"written": (bag_dir / "metadata.yaml").exists()}
    except Exception as exc:
        return {"written": False, "error": f"{type(exc).__name__}: {exc}"}


def recover_bag(bag_dir: str | os.PathLike, compression: str = "zstd", marker: dict | None = None) -> dict:
    """Salvage a bag the bridge never closed: every `.mcap` rewritten whole + indexed (torn tail
    dropped, `recovered` session marker appended, original kept as `.torn.bak`), then
    `metadata.yaml`. Idempotent: a bag that needs nothing is left alone (`recovered: False`)."""
    bag = Path(bag_dir)
    t0 = time.perf_counter()
    reason = needs_recovery(bag)
    if reason is None:
        return {"bag": str(bag), "recovered": False, "reason": None}
    files = []
    for f in bag_mcaps(bag):
        try:
            files.append(reindex(f, compression=compression, recover=True, marker=marker))
        except Exception as exc:
            files.append({"file": str(f), "verified": False, "error": f"{type(exc).__name__}: {exc}"})
    salvaged = sum(int(r.get("messages") or 0) for r in files if r.get("verified"))
    meta = write_metadata(bag) if salvaged else {"written": False, "error": "nothing salvaged"}
    return {"bag": str(bag), "recovered": True, "reason": reason, "files": files, "salvaged_records": salvaged,
            "metadata_yaml": meta, "whole": all(r.get("verified") for r in files) and bool(meta.get("written")),
            "seconds": round(time.perf_counter() - t0, 2)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="a bag directory or a single .mcap file")
    ap.add_argument("--keep", action="store_true", help="keep the original as <name>.unindexed.mcap")
    ap.add_argument("--recover", action="store_true",
                    help="a bag the writer never closed: salvage the records up to the tear, index, write metadata.yaml")
    ap.add_argument("--compression", choices=("zstd", "lz4", "none"), default="zstd")
    a = ap.parse_args(argv)
    p = Path(a.bag)
    if a.recover:
        if not p.is_dir():
            ap.error("--recover takes the bag DIRECTORY")
        r = recover_bag(p, compression=a.compression)
        print(json.dumps(r))
        return 0 if (r.get("whole") or not r.get("recovered")) else 1
    files = bag_mcaps(p) if p.is_dir() else [p]
    rc = 0
    for f in files:
        r = reindex(f, keep_original=a.keep, compression=a.compression)
        print(json.dumps(r))
        rc |= 0 if r.get("verified") else 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
