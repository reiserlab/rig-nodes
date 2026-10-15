#!/usr/bin/env python3
"""mcap_index.py — rewrite a finished bag as an indexed, chunked, compressed MCAP.

    pixi run reindex -- runlogs/<run>.bag            # in place (keeps <file>.unindexed.mcap until verified, then deletes it)
    pixi run reindex -- runlogs/<run>.bag --keep     # keep the original beside the new file

Why: the bridge node records with the `crashsafe` profile — no chunking, no cache, every record
reaches the disk as it is written — so a SIGKILL loses at most the record in progress. The price is
an MCAP without a chunk/message index: readers must scan it ("Problems: file is unindexed" in
Foxglove; `ros2 bag` and our converter do not mind). Once a run has CLOSED cleanly there is no
crash left to survive, so the bridge calls `reindex()` from Recorder.close(): the same records,
same schemas/channels/metadata, written again by the `mcap` library with chunking, zstd and the
full index. Pure Python; the result is a standard MCAP (rosbag2's `metadata.yaml` is untouched).
The rewrite is verified by re-reading and counting messages per channel before the original is
removed; on any mismatch the original stays and the function reports it.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


def reindex(mcap_path: str | os.PathLike, keep_original: bool = False, compression: str = "zstd") -> dict:
    from mcap.records import Attachment, Channel, Header, Message, Metadata, Schema
    from mcap.stream_reader import StreamReader
    from mcap.writer import CompressionType, Writer

    src = Path(mcap_path)
    tmp = src.with_suffix(".indexed.tmp")
    comp = {"zstd": CompressionType.ZSTD, "lz4": CompressionType.LZ4, "none": CompressionType.NONE}[compression]
    t0 = time.monotonic()
    schema_map: dict[int, int] = {}
    channel_map: dict[int, int] = {}
    counts: dict[int, int] = {}
    n = 0
    with open(src, "rb") as fin, open(tmp, "wb") as fout:
        w = Writer(fout, compression=comp)
        started = False
        for rec in StreamReader(fin, emit_chunks=False).records:
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
            elif isinstance(rec, Message):
                if not started:
                    raise ValueError("message before header")
                w.add_message(channel_id=channel_map[rec.channel_id], log_time=rec.log_time, data=rec.data,
                              publish_time=rec.publish_time, sequence=rec.sequence)
                counts[rec.channel_id] = counts.get(rec.channel_id, 0) + 1
                n += 1
            elif isinstance(rec, Metadata):
                w.add_metadata(name=rec.name, data=rec.metadata)
            elif isinstance(rec, Attachment):
                w.add_attachment(create_time=rec.create_time, log_time=rec.log_time, name=rec.name, media_type=rec.media_type, data=rec.data)
        w.finish()
    # Verify: the new file must yield the same per-topic counts, through the indexed reader.
    from mcap.reader import make_reader

    with open(tmp, "rb") as fh:
        rd = make_reader(fh)
        summ = rd.get_summary()
        got = {}
        for _, ch, _ in rd.iter_messages(log_time_order=False):
            got[ch.topic] = got.get(ch.topic, 0) + 1
    want: dict[str, int] = {}
    with open(src, "rb") as fin:
        chans = {}
        for rec in StreamReader(fin, emit_chunks=False).records:
            if isinstance(rec, Channel):
                chans[rec.id] = rec.topic
            elif isinstance(rec, Message):
                want[chans[rec.channel_id]] = want.get(chans[rec.channel_id], 0) + 1
    ok = got == want and summ is not None and bool(summ.chunk_indexes)
    res = {"file": str(src), "messages": n, "bytes_before": src.stat().st_size, "bytes_after": tmp.stat().st_size,
           "chunks": len(summ.chunk_indexes) if summ else 0, "compression": compression, "verified": ok,
           "seconds": round(time.monotonic() - t0, 2)}
    if not ok:
        tmp.unlink(missing_ok=True)
        res["error"] = f"verification failed: per-topic counts {got} vs {want}"
        return res
    if keep_original:
        src.rename(src.with_name(src.stem + ".unindexed.mcap"))
    else:
        src.unlink()
    tmp.rename(src)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="a bag directory or a single .mcap file")
    ap.add_argument("--keep", action="store_true", help="keep the original as <name>.unindexed.mcap")
    ap.add_argument("--compression", choices=("zstd", "lz4", "none"), default="zstd")
    a = ap.parse_args(argv)
    p = Path(a.bag)
    files = sorted(p.glob("*.mcap")) if p.is_dir() else [p]
    files = [f for f in files if not f.name.endswith(".unindexed.mcap")]
    rc = 0
    for f in files:
        r = reindex(f, keep_original=a.keep, compression=a.compression)
        print(json.dumps(r))
        rc |= 0 if r.get("verified") else 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
