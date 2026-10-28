#!/usr/bin/env python3
"""zmq_latency_report.py — latency, gaps and queue depth of the ScanImage ZMQ frame stream (Block R).

Reads what rig_nodes.scanimage_frames_node published — from a run's MCAP bag (the bridge node records
/scanimage/frame_meta, /scanimage/frame_stats, /scanimage/preview/meta and the preview images) or from
the node's own `--dump` JSONL — and, per acquisition segment, reports:

  stream delay      rx_mono − header frameTimestamp, RELATIVE to the fastest frame of the segment: the
                    acquisition clock (vDAQ, since acquisition start) and the host clock share no origin,
                    so without the TIFF this is the jitter/backlog part of the latency, not its constant.
                    p50 / p99 / max, plus the drift of the two clocks (ms per s, a linear fit).
  TIFF match        with --tiff: which segment the TIFF is (choose_tiff_segment: frame count + epoch, an
                    error when none or several fit; --segment overrides), every TIFF frame found in it,
                    and the per-frame difference between the ZMQ header timestamp and the TIFF's (one
                    constant: the same acquisition clock; 0 on the Bergamo, 2026-10-20).
  epoch offset      with --tiff: rx_epoch_ms − (the TIFF's `epoch` + `frameTimestamps_sec`). NOT a latency:
                    ScanImage's epoch is when it armed the acquisition, before the shutter's openTime_s and
                    the first line (~0.56 s on 2026-10-20), so it has no pass bar.
  hops              receive → publish inside the node (`pub_mono_ns − rx_mono_ns`) and, from a bag, node →
                    bridge record (MCAP log_time − rx_epoch_ms), like FicTrac's connector_status.
  preview           encode+publish per image (preview meta) and, from a bag, frame receive → preview
                    record (the image's header.stamp is the frame's receive time); achieved rate per channel.
  gaps              frameNumber discontinuities in the meta stream (what the publisher sent and the node
                    never received), the slow-joiner loss (first frame − 1), and the node's own
                    dropped_by_queue / bad_frames from the last frame_stats.
  queue depth       histogram and p99 of the node's queue at publish time.

Pass bars (the procedure's): meta p99 < one frame period (--meta-p99-ms, default = the measured period),
preview p99 < --preview-p99-ms (250), 0 gaps after the first frame, 0 dropped_by_queue. Printed as a
markdown table; --json writes everything; --strict exits 1 when a bar fails.

Warnings (printed and in the JSON, never a bar): frames that never reached the node before a segment's
first frame (missed_before_first > 0: a late join, or a reconnect mid-acquisition), and a TIFF `epoch`
that is not a wall time (ScanImage's simulated vDAQ writes [1601 1 1 0 0 0]), which is then ignored:
no epoch offset, and the TIFF's segment is chosen by frame count alone.

    pixi run zmq-latency -- --bag runlogs/<run>.bag --json latency_focus60.json
    pixi run zmq-latency -- --bag runlogs/<run>.bag --tiff "F:\\data\\run_00003.tif" --segment 2
    pixi run zmq-latency -- --jsonl frames.jsonl
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import struct
import sys
from collections import Counter
from pathlib import Path

META_TOPIC = "/scanimage/frame_meta"
STATS_TOPIC = "/scanimage/frame_stats"
PREVIEW_META_TOPIC = "/scanimage/preview/meta"
PREVIEW_PREFIX = "/scanimage/preview/ch"
FRAME_ID_RE = re.compile(r"^scanimage/ch(\d+)/s(\d+)/f(\d+)$")


# ── small statistics ──────────────────────────────────────────────────────────────────────
def pct(sorted_vals: list[float], q: float) -> float:
    """Nearest-rank percentile of an ascending list."""
    if not sorted_vals:
        return math.nan
    k = max(0, min(len(sorted_vals) - 1, int(math.ceil(q / 100.0 * len(sorted_vals))) - 1))
    return sorted_vals[k]


def summarize(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0}
    s = sorted(vals)
    return {"n": len(s), "min": round(s[0], 3), "p50": round(pct(s, 50), 3), "p99": round(pct(s, 99), 3),
            "max": round(s[-1], 3), "mean": round(sum(s) / len(s), 3)}


def slope(xs: list[float], ys: list[float]) -> float | None:
    """Least-squares slope of ys on xs (None below 10 points or without spread)."""
    n = len(xs)
    if n < 10:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx


# ── sources: the node's JSONL dump, or a bag ─────────────────────────────────────────────
def load_jsonl(path: Path) -> list[dict]:
    recs = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            topic = obj.pop("topic", None)
            if topic:
                recs.append({"topic": topic, "obj": obj, "log_ns": None})
    return recs


def cdr_string(data: bytes) -> str:
    """std_msgs/String in CDR (XCDR1): 4-byte encapsulation, uint32 length, bytes (NUL-terminated)."""
    le = data[1] == 1
    n = struct.unpack_from("<I" if le else ">I", data, 4)[0]
    return data[8:8 + n].rstrip(b"\x00").decode("utf-8", errors="replace")


def cdr_compressed_image(data: bytes) -> dict:
    """sensor_msgs/CompressedImage header + format + payload length (the PNG itself is not needed)."""
    le = data[1] == 1
    e = "<" if le else ">"
    p = 4

    def align(n: int) -> None:
        nonlocal p
        p += (-(p - 4)) % n

    sec = struct.unpack_from(e + "i", data, p)[0]
    p += 4
    nsec = struct.unpack_from(e + "I", data, p)[0]
    p += 4
    n = struct.unpack_from(e + "I", data, p)[0]
    p += 4
    frame_id = data[p:p + n].rstrip(b"\x00").decode("utf-8", errors="replace")
    p += n
    align(4)
    n = struct.unpack_from(e + "I", data, p)[0]
    p += 4
    fmt = data[p:p + n].rstrip(b"\x00").decode("utf-8", errors="replace")
    p += n
    align(4)
    n = struct.unpack_from(e + "I", data, p)[0]
    return {"stamp_ns": sec * 1_000_000_000 + nsec, "frame_id": frame_id, "format": fmt, "bytes": n}


def load_bag(path: Path) -> list[dict]:
    from mcap.reader import make_reader

    files = sorted(f for f in path.glob("*.mcap") if not f.name.endswith((".unindexed.mcap", ".indexed.tmp"))) if path.is_dir() else [path]
    if not files:
        raise SystemExit(f"no .mcap files in {path}")
    recs = []
    for f in files:
        with open(f, "rb") as fh:
            reader = make_reader(fh, validate_crcs=False)
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                t = channel.topic
                if t in (META_TOPIC, STATS_TOPIC, PREVIEW_META_TOPIC):
                    try:
                        obj = json.loads(cdr_string(message.data))
                    except (json.JSONDecodeError, struct.error, UnicodeDecodeError):
                        continue
                    recs.append({"topic": t, "obj": obj, "log_ns": message.log_time})
                elif t.startswith(PREVIEW_PREFIX):
                    try:
                        img = cdr_compressed_image(message.data)
                    except struct.error:
                        continue
                    recs.append({"topic": t, "obj": img, "log_ns": message.log_time})
    return recs


# ── the TIFF side (stdlib; ScanImage writes one page per channel per frame) ──────────────
def tiff_descriptions(path: Path):
    """Yield (page index, ImageDescription text) for every IFD; classic and BigTIFF, both byte orders."""
    with open(path, "rb") as f:
        data = f.read()
    bo = "<" if data[:2] == b"II" else ">"
    big = struct.unpack_from(bo + "H", data, 2)[0] == 43
    if big:
        off = struct.unpack_from(bo + "Q", data, 8)[0]
        cnt_fmt, cnt_size, ent, ent_fmt, off_fmt = "Q", 8, 20, "HHQQ", "Q"
    else:
        off = struct.unpack_from(bo + "I", data, 4)[0]
        cnt_fmt, cnt_size, ent, ent_fmt, off_fmt = "H", 2, 12, "HHII", "I"
    page = 0
    seen = set()
    while off and off not in seen and off + cnt_size <= len(data):
        seen.add(off)
        n = struct.unpack_from(bo + cnt_fmt, data, off)[0]
        pos = off + cnt_size
        desc = ""
        for i in range(n):
            tag, typ, count, val = struct.unpack_from(bo + ent_fmt, data, pos + i * ent)
            if tag == 270:
                inline = 8 if big else 4
                if count <= inline:  # value stored in the entry itself
                    raw = struct.pack(bo + off_fmt, val)[:count]
                else:
                    raw = data[val:val + count]
                desc = raw.decode("latin1", "replace").rstrip("\x00")
        yield page, desc
        page += 1
        off = struct.unpack_from(bo + off_fmt, data, pos + n * ent)[0]


def parse_fields(desc: str) -> dict:
    out: dict = {}
    for key in ("frameNumbers", "frameTimestamps_sec", "acqTriggerTimestamps_sec", "endOfAcquisition"):
        m = re.search(re.escape(key) + r"\s*=\s*([-\d.eE+]+)", desc)
        out[key] = float(m.group(1)) if m else None
    m = re.search(r"epoch\s*=\s*\[([^\]]*)\]", desc)
    out["epoch"] = [float(x) for x in re.split(r"[\s,;]+", m.group(1).strip()) if x] if m else None
    return out


def tiff_frames(path: Path) -> dict:
    """{frameNumber: {"ts": frameTimestamps_sec, "pages": n}} + the acquisition epoch of the first page."""
    frames: dict[int, dict] = {}
    epoch = None
    pages = 0
    for _, desc in tiff_descriptions(path):
        pages += 1
        f = parse_fields(desc)
        if f["frameNumbers"] is None:
            continue
        k = int(f["frameNumbers"])
        if epoch is None and f["epoch"]:
            epoch = f["epoch"]
        if k in frames:
            frames[k]["pages"] += 1
        else:
            frames[k] = {"ts": f["frameTimestamps_sec"], "pages": 1}
    return {"frames": frames, "epoch": epoch, "pages": pages, "path": str(path)}


EPOCH_MIN_YEAR = 2000  # an earlier TIFF `epoch` is not a wall time (the simulated vDAQ writes 1601-01-01)


def epoch_wall_ms(epoch: list, utc_offset_h: float | None) -> int | None:
    """ScanImage's `epoch = [y m d H M S.fff]` (PC local time) → wall ms. The offset defaults to this
    machine's local zone (the tool normally runs on the ScanImage PC). None when the epoch is not a
    usable wall time: a year before EPOCH_MIN_YEAR (ScanImage 2026's simulated vDAQ writes
    [1601 1 1 0 0 0], the Windows FILETIME zero; Bergamo PC 2026-10-22) or a date this platform cannot
    convert (on Windows, `astimezone()` raises OSError for anything before 1970)."""
    y, mo, d, h, mi, s = epoch[:6]
    if y < EPOCH_MIN_YEAR:
        return None
    sec = int(s)
    frac = s - sec
    try:
        naive = dt.datetime(int(y), int(mo), int(d), int(h), int(mi), sec)
        if utc_offset_h is None:
            t = naive.astimezone()  # interpret as local time
        else:
            t = naive.replace(tzinfo=dt.timezone(dt.timedelta(hours=utc_offset_h)))
        return int(t.timestamp() * 1000) + int(round(frac * 1000))
    except (OverflowError, OSError, ValueError):
        return None


# ── analysis ──────────────────────────────────────────────────────────────────────────────
EPOCH_WINDOW_S = 5.0  # the TIFF's segment starts this soon after its epoch (Bergamo 2026-10-20: 0.56 s)


class SegmentChoiceError(ValueError):
    """The TIFF matches no segment, or more than one; the message lists them all."""


def choose_tiff_segment(by_seg: dict, tiff: dict, utc_offset_h: float | None = None,
                        segment: int | None = None, window_s: float = EPOCH_WINDOW_S) -> tuple[int, list[dict]]:
    """Which acquisition segment a TIFF belongs to → (segment id, one row per segment).
    Every segment numbers its frames from 1, so frame numbers alone match the first segment, not the
    right one (Bergamo 2026-10-20: a FOCUS instead of the GRAB). A candidate holds every TIFF frame
    number, has exactly as many frames as the TIFF and, when the TIFF has an epoch, its first frame
    reached the node 0..window_s after that epoch. Exactly one candidate is the answer; none or
    several raise SegmentChoiceError. `segment` overrides the choice."""
    n_tiff = len(tiff["frames"])
    e_ms = epoch_wall_ms(tiff["epoch"], utc_offset_h) if tiff.get("epoch") else None
    rows = []
    for sid in sorted(by_seg):
        recs = by_seg[sid]
        nums = {int(r["obj"]["frameNumber"]) for r in recs}
        found = sum(1 for k in tiff["frames"] if k in nums)
        off_s = (min(r["obj"]["rx_epoch_ms"] for r in recs) - e_ms) / 1000.0 if e_ms is not None else None
        rows.append({"segment": sid, "frames": len(recs), "tiff_frames_found": found,
                     "first_rx_after_epoch_s": round(off_s, 3) if off_s is not None else None,
                     "candidate": found == n_tiff and len(recs) == n_tiff and (off_s is None or 0.0 <= off_s <= window_s)})
    listing = "; ".join(f"segment {r['segment']}: {r['frames']} frames, {r['tiff_frames_found']}/{n_tiff} TIFF frames found, "
                        f"first frame {r['first_rx_after_epoch_s']} s after the epoch" for r in rows)
    if segment is not None:
        if segment not in by_seg:
            raise SegmentChoiceError(f"--segment {segment} is not in the stream ({listing})")
        return segment, rows
    cands = [r["segment"] for r in rows if r["candidate"]]
    if len(cands) == 1:
        return cands[0], rows
    raise SegmentChoiceError(f"{'no segment matches' if not cands else f'{len(cands)} segments match'} the TIFF "
                             f"({n_tiff} frames{', epoch window 0..%g s' % window_s if e_ms is not None else ', no epoch'}); "
                             f"pass --segment N. {listing}")


def analyze(records: list[dict], tiff: dict | None = None, utc_offset_h: float | None = None,
            frame_period_ms: float | None = None, segment_filter: int | None = None,
            meta_p99_ms: float | None = None, preview_p99_ms: float = 250.0,
            epoch_window_s: float = EPOCH_WINDOW_S) -> dict:
    metas = [r for r in records if r["topic"] == META_TOPIC and isinstance(r["obj"].get("seq"), int)]
    stats = [r["obj"] for r in records if r["topic"] == STATS_TOPIC]
    pmetas = [r for r in records if r["topic"] == PREVIEW_META_TOPIC]
    pimgs = [r for r in records if r["topic"].startswith(PREVIEW_PREFIX)]
    by_seg: dict[int, list[dict]] = {}
    for r in metas:
        by_seg.setdefault(int(r["obj"]["segment"]), []).append(r)
    last_stats = stats[-1] if stats else {}
    stats_segments = {s["id"]: s for s in last_stats.get("segment_list", [])} if last_stats else {}

    warnings: list[str] = []
    if tiff and tiff.get("epoch") and epoch_wall_ms(tiff["epoch"], utc_offset_h) is None:
        warnings.append(f"TIFF epoch {[round(v, 3) for v in tiff['epoch'][:6]]} is not a usable wall time "
                        "(ScanImage's simulated vDAQ writes 1601-01-01): ignored, so there is no epoch offset "
                        "and the TIFF's segment is chosen by frame count alone")

    # which segment the TIFF belongs to: by frame count and epoch, never by frame numbers alone
    tiff_seg, choice = None, None
    if tiff and tiff["frames"]:
        tiff_seg, choice = choose_tiff_segment(by_seg, tiff, utc_offset_h, segment_filter, epoch_window_s)

    segments = []
    for sid in sorted(by_seg):
        if segment_filter and sid != segment_filter:
            continue
        rows = sorted(by_seg[sid], key=lambda r: r["obj"]["seq"])
        objs = [r["obj"] for r in rows]
        nums = [int(o["frameNumber"]) for o in objs]
        ts = [float(o["frameTimestamp_s"]) for o in objs]
        rx_mono_s = [o["rx_mono_ns"] / 1e9 for o in objs]
        gaps = [{"after": a, "next": b, "missing": b - a - 1} for a, b in zip(nums, nums[1:]) if b > a + 1]
        d = [rm - t for rm, t in zip(rx_mono_s, ts)]
        off = min(d) if d else 0.0
        rel_ms = [(x - off) * 1000.0 for x in d]
        dts = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
        period_ms = frame_period_ms or (pct(dts, 50) * 1000.0 if dts else None)
        hop_ms = [(o["pub_mono_ns"] - o["rx_mono_ns"]) / 1e6 for o in objs if "pub_mono_ns" in o]
        bridge_ms = [(r["log_ns"] - o["rx_epoch_ms"] * 1_000_000) / 1e6 for r, o in zip(rows, objs) if r["log_ns"] is not None]
        depths = [int(o.get("queue_depth", 0)) for o in objs]
        hist = dict(sorted(Counter(depths).items()))
        pm = [r["obj"] for r in pmetas if int(r["obj"].get("segment", -1)) == sid]
        enc_ms = [(o["pub_mono_ns"] - o["rx_mono_ns"]) / 1e6 for o in pm if "pub_mono_ns" in o]
        per_ch: dict = {}
        for o in pm:
            c = per_ch.setdefault(str(o.get("channel")), {"images": 0, "first_rx_ms": o["rx_epoch_ms"], "last_rx_ms": o["rx_epoch_ms"], "mapping": o.get("mapping")})
            c["images"] += 1
            c["last_rx_ms"] = o["rx_epoch_ms"]
        for c in per_ch.values():
            span = (c["last_rx_ms"] - c["first_rx_ms"]) / 1000.0
            c["achieved_hz"] = round((c["images"] - 1) / span, 2) if c["images"] > 1 and span > 0 else None
        e2e_ms = []
        for r in pimgs:
            m = FRAME_ID_RE.match(r["obj"].get("frame_id", ""))
            if m and int(m.group(2)) == sid and r["log_ns"] is not None:
                e2e_ms.append((r["log_ns"] - r["obj"]["stamp_ns"]) / 1e6)
        seg = {
            "id": sid, "frames": len(objs), "first_frame": nums[0], "last_frame": nums[-1],
            "missed_before_first": max(nums[0] - 1, 0), "expected_from_first": nums[-1] - nums[0] + 1,
            "gaps": len(gaps), "gap_frames": sum(g["missing"] for g in gaps), "gap_list": gaps[:200],
            "dims": objs[0].get("dims"), "nchan": objs[0].get("nchan"),
            "frame_period_ms": round(period_ms, 3) if period_ms else None,
            "rate_hz": round(1000.0 / period_ms, 3) if period_ms else None,
            "span_s": round(rx_mono_s[-1] - rx_mono_s[0], 3),
            "stream_delay_rel_ms": summarize(rel_ms),
            "clock_drift_ms_per_s": (lambda m: round(m * 1000.0, 4) if m is not None else None)(slope(ts, d)),
            "node_hop_ms": summarize(hop_ms),
            "node_to_bridge_ms": summarize(bridge_ms),
            "queue_depth": {"hist": hist, "p99": pct(sorted(depths), 99) if depths else None, "max": max(depths) if depths else None},
            "preview": {"images": len(pm), "encode_publish_ms": summarize(enc_ms), "per_channel": per_ch,
                        "frame_to_record_ms": summarize(e2e_ms)},
            "from_stats": stats_segments.get(sid),
        }
        if tiff and sid == tiff_seg:
            pairs = [(o, tiff["frames"][int(o["frameNumber"])]) for o in objs if int(o["frameNumber"]) in tiff["frames"]]
            delta_ms = [(float(o["frameTimestamp_s"]) - t["ts"]) * 1000.0 for o, t in pairs if t["ts"] is not None]
            t_align = {"tiff": tiff["path"], "tiff_frames": len(tiff["frames"]), "tiff_pages": tiff["pages"],
                       "matched_frames": len(pairs), "zmq_minus_tiff_ts_ms": summarize(delta_ms),
                       # the same frames carry the same acquisition clock: one constant offset (0 on the rig)
                       "timestamps_agree": bool(delta_ms) and len(delta_ms) == len(pairs) and max(delta_ms) - min(delta_ms) < 1.0,
                       "pages_per_frame": Counter(t["pages"] for t in tiff["frames"].values()).most_common(1)[0][0] if tiff["frames"] else None,
                       "frame_meta_count_equals_tiff_frames": len(objs) == len(tiff["frames"]),
                       "segment_choice": choice}
            if tiff["epoch"]:
                e_ms = epoch_wall_ms(tiff["epoch"], utc_offset_h)
                t_align.update({"epoch": tiff["epoch"], "epoch_usable": e_ms is not None})
                if e_ms is not None:
                    # NOT a latency: ScanImage's epoch is when it armed the acquisition, before the shutter's
                    # openTime_s and the first line, so this holds that start-up gap (~0.56 s on 2026-10-20)
                    off_ms = [o["rx_epoch_ms"] - (e_ms + t["ts"] * 1000.0) for o, t in pairs if t["ts"] is not None]
                    t_align.update({"epoch_wall_ms": e_ms, "utc_offset_h": utc_offset_h, "epoch_offset_ms": summarize(off_ms)})
            seg["tiff"] = t_align
        if seg["missed_before_first"]:
            warnings.append(f"segment {sid}: the first {seg['missed_before_first']} frame(s) never reached the node "
                            f"(its first frame is {seg['first_frame']}): a late join (start the node before Grab) "
                            "or a reconnect mid-acquisition")
        segments.append(seg)

    # pass bars
    bars = []
    for seg in segments:
        period = meta_p99_ms or seg["frame_period_ms"] or 34.0
        lat = seg["stream_delay_rel_ms"]  # the TIFF's epoch offset holds the start-up gap: no bar on it
        if lat.get("n"):
            bars.append({"segment": seg["id"], "bar": f"meta latency p99 < {period:.1f} ms (relative (fastest frame = 0))", "value": lat["p99"], "pass": lat["p99"] < period})
        if seg.get("tiff"):
            t = seg["tiff"]
            bars.append({"segment": seg["id"], "bar": "TIFF frames all in the stream, timestamps agree (constant offset)",
                         "value": f"{t['matched_frames']}/{t['tiff_frames']}, ZMQ − TIFF p50 {t['zmq_minus_tiff_ts_ms'].get('p50')} ms",
                         "pass": t["matched_frames"] == t["tiff_frames"] and t["timestamps_agree"]})
        pv = seg["preview"]["frame_to_record_ms"] if seg["preview"]["frame_to_record_ms"].get("n") else seg["preview"]["encode_publish_ms"]
        if pv.get("n"):
            bars.append({"segment": seg["id"], "bar": f"preview p99 < {preview_p99_ms:g} ms", "value": pv["p99"], "pass": pv["p99"] < preview_p99_ms})
        bars.append({"segment": seg["id"], "bar": "0 gaps after the first frame", "value": seg["gap_frames"], "pass": seg["gap_frames"] == 0})
    if last_stats:
        bars.append({"segment": None, "bar": "0 dropped_by_queue (node back-pressure)", "value": last_stats.get("dropped_by_queue"), "pass": not last_stats.get("dropped_by_queue")})
        bars.append({"segment": None, "bar": "0 bad frames", "value": last_stats.get("bad_frames"), "pass": not last_stats.get("bad_frames")})
    return {
        "frame_meta": len(metas), "frame_stats": len(stats), "preview_meta": len(pmetas), "preview_images": len(pimgs),
        "last_stats": {k: last_stats.get(k) for k in ("frames", "published", "dropped_by_queue", "bad_frames", "gaps", "gap_frames",
                                                     "segments", "connects", "disconnects", "queue_limit", "uptime_s")} if last_stats else None,
        "segments": segments, "bars": bars, "all_pass": all(b["pass"] for b in bars) if bars else None,
        "warnings": warnings,
    }


# ── report ────────────────────────────────────────────────────────────────────────────────
def fmt(s: dict) -> str:
    return "—" if not s or not s.get("n") else f"{s['p50']:.2f} / {s['p99']:.2f} / {s['max']:.2f} (n={s['n']})"


def markdown(rep: dict, source: str) -> str:
    out = [f"# ZMQ frame-stream report — {source}", "",
           f"frame_meta {rep['frame_meta']}, frame_stats {rep['frame_stats']}, preview meta {rep['preview_meta']}, preview images {rep['preview_images']}"]
    if rep["last_stats"]:
        out.append("node totals (last frame_stats): " + ", ".join(f"{k}={v}" for k, v in rep["last_stats"].items()))
    for w in rep.get("warnings", []):
        out.append(f"- WARNING: {w}")
    out += ["", "| segment | frames (first–last) | missed before first | gaps (frames) | rate Hz | stream delay rel p50/p99/max ms | node hop p50/p99/max ms | node→bridge p50/p99/max ms | queue p99/max | preview enc p50/p99/max ms | preview frame→record p50/p99/max ms |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for s in rep["segments"]:
        q = s["queue_depth"]
        out.append(f"| {s['id']} | {s['frames']} ({s['first_frame']}–{s['last_frame']}) | {s['missed_before_first']} | {s['gaps']} ({s['gap_frames']}) | {s['rate_hz']} | "
                   f"{fmt(s['stream_delay_rel_ms'])} | {fmt(s['node_hop_ms'])} | {fmt(s['node_to_bridge_ms'])} | {q['p99']}/{q['max']} | "
                   f"{fmt(s['preview']['encode_publish_ms'])} | {fmt(s['preview']['frame_to_record_ms'])} |")
    for s in rep["segments"]:
        if s.get("clock_drift_ms_per_s") is not None:
            out.append(f"- segment {s['id']}: acquisition clock vs host monotonic drift {s['clock_drift_ms_per_s']:+.4f} ms/s; queue-depth histogram {s['queue_depth']['hist']}")
        if s["gap_list"]:
            out.append(f"- segment {s['id']} gaps: " + ", ".join(f"{g['after']}→{g['next']} (−{g['missing']})" for g in s["gap_list"][:20]) + (" …" if len(s["gap_list"]) > 20 else ""))
        for ch, c in s["preview"]["per_channel"].items():
            out.append(f"- segment {s['id']} preview ch{ch}: {c['images']} images, {c['achieved_hz']} Hz achieved, mapping {c['mapping']}")
        t = s.get("tiff")
        if t:
            out.append(f"- segment {s['id']} vs TIFF {t['tiff']}: {t['matched_frames']} of {t['tiff_frames']} TIFF frames matched ({t['tiff_pages']} pages, {t['pages_per_frame']} per frame); "
                       f"frame_meta count == TIFF frames: {t['frame_meta_count_equals_tiff_frames']}; ZMQ − TIFF timestamp p50/p99/max ms {fmt(t['zmq_minus_tiff_ts_ms'])}"
                       + (f"; epoch offset (topic − (epoch + frame time); includes ScanImage's arming → first-line gap, NOT a latency) p50/p99/max ms {fmt(t['epoch_offset_ms'])}"
                          if t.get("epoch_offset_ms") else "; the TIFF's `epoch` is not a usable wall time → no epoch offset"
                          if t.get("epoch") else "; no `epoch` in the TIFF → no epoch offset"))
    out += ["", "| pass bar | segment | value | result |", "|---|---|---|---|"]
    for b in rep["bars"]:
        out.append(f"| {b['bar']} | {b['segment'] if b['segment'] is not None else 'all'} | {b['value']} | {'PASS' if b['pass'] else 'FAIL'} |")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="a run's bag directory (or one .mcap) recorded by the bridge node")
    src.add_argument("--jsonl", help="the node's --dump file")
    ap.add_argument("--tiff", help="the acquisition's ScanImage TIFF (frameNumbers / frameTimestamps_sec / epoch in its page descriptions)")
    ap.add_argument("--utc-offset-h", type=float, default=None, help="the ScanImage PC's UTC offset for the TIFF epoch (default: this machine's local zone)")
    ap.add_argument("--segment", type=int, default=None, help="only this acquisition segment (with --tiff: the TIFF's segment, overriding the choice)")
    ap.add_argument("--epoch-window-s", type=float, default=EPOCH_WINDOW_S,
                    help=f"with --tiff: the TIFF's segment starts 0..this many s after its epoch (default {EPOCH_WINDOW_S:g})")
    ap.add_argument("--frame-period-ms", type=float, default=None, help="override the measured frame period for the pass bar")
    ap.add_argument("--meta-p99-ms", type=float, default=None, help="pass bar for meta latency p99 (default: one frame period)")
    ap.add_argument("--preview-p99-ms", type=float, default=250.0)
    ap.add_argument("--json", help="write the full report here")
    ap.add_argument("--strict", action="store_true", help="exit 1 when a pass bar fails")
    a = ap.parse_args(argv)

    if a.bag:
        records = load_bag(Path(a.bag))
        source = a.bag
    else:
        records = load_jsonl(Path(a.jsonl))
        source = a.jsonl
    tiff = tiff_frames(Path(a.tiff)) if a.tiff else None
    try:
        rep = analyze(records, tiff, a.utc_offset_h, a.frame_period_ms, a.segment, a.meta_p99_ms, a.preview_p99_ms, a.epoch_window_s)
    except SegmentChoiceError as exc:
        print(f"which segment is the TIFF? {exc}", file=sys.stderr)
        return 2
    rep["source"] = source
    if not rep["segments"]:
        print(f"no /scanimage/frame_meta records in {source}", file=sys.stderr)
        return 2
    print(markdown(rep, source))
    for w in rep["warnings"]:
        print(f"warning: {w}", file=sys.stderr)
    if a.json:
        Path(a.json).write_text(json.dumps(rep, indent=1), encoding="utf-8")
        print(f"\nwritten {a.json}")
    return 1 if (a.strict and rep["all_pass"] is False) else 0


if __name__ == "__main__":
    raise SystemExit(main())
