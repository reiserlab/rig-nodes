#!/usr/bin/env python3
"""scanimage_frames_node.py — ScanImage's ZeroMQ frame stream as ROS 2 topics (LAB-303, Block R).

Moved from webDisplayTools `rig/nodes/scanimage_frames_node.py` (PR #261) into rig-nodes 0.3.0: paths and
imports only. Bench-validated against ScanImage 2026 on the Bergamo PC on 2026-10-22 (start the node BEFORE
GRAB; frames queue up to libzmq's default high-water mark). The stand-in publisher is
`rig_nodes.sims.zmq_frames_mock`; the harness `tests/test_scanimage_frames.py`; the report
`rig_nodes.zmq_latency_report`.

ScanImage 2026 Premium publishes every acquired frame on a ZeroMQ PUB socket
(`hSI.hScan2D.hAcq.ZMQAddress`, tcp://127.0.0.1:65179 on the Bergamo rig; bound only while an
acquisition runs). Each frame is a 2-part message: a 40-byte header of five little-endian doubles
[pixelsPerLine, linesPerFrame, nChannels, frameTimestamp_s (since acquisition start),
totalAcquiredFrames] and the int16 planes (x fastest, then y, then channel). PUB drops silently
when a subscriber falls behind, and a subscriber that joins after the bind misses the first
frames (the "slow joiner"). This node is that subscriber, and its job is to say exactly what it
saw and what it lost — the raw pixels stay in ScanImage's TIFF.

  /scanimage/frame_meta      OUT  every frame: {"seq", "segment", "frameNumber", "frameTimestamp_s",
                                  "dims": [ppl, lpf], "nchan", "bytes", "gap_before", "rx_epoch_ms",
                                  "rx_mono_ns", "pub_mono_ns", "queue_depth", "preview"}
  /scanimage/frame_stats     OUT  every --stats-every s: {"connected", "connects", "frames", "published",
                                  "dropped_by_queue", "bad_frames", "gaps", "gap_frames", "gap_list",
                                  "segments", "segment_list": [{"id", "first_frame", "missed_before_first",
                                  "frames", "last_frame", "dims", "nchan", …}], "queue_depth", "queue_max",
                                  "rate_hz", "preview": {…}}
  /scanimage/preview/ch<id>  OUT  sensor_msgs/CompressedImage (PNG, --preview-size², ≤ --preview-hz per
                                  channel; 0 = off). Bounded on purpose: live imaging is not v1.
  /scanimage/preview/meta    OUT  one JSON line per preview image: which frame, which channel, and the
                                  int16 → 8-bit display mapping used ({"mode", "lo", "hi"}).

Design: a dedicated receive thread owns the SUB socket (plus its monitor socket, which reports
connect/disconnect, so a publisher that goes away and comes back is logged and the next frames open a
new segment). It parses the header, stamps the frame (`rx_epoch_ms` wall / `rx_mono_ns` monotonic,
_common's precise clocks on Windows too), does the loss accounting — all of it on the receive thread,
so every frame that reaches this process is counted — and hands the frame to a BOUNDED queue. The
ROS side publishes from that queue; when it falls behind, the newest frame is dropped and counted
as `dropped_by_queue` (never a gap). Loss accounting, by header frame number:

  gap            frameNumber jumped by more than 1 inside a segment — frames the publisher sent and
                 this process never received (PUB dropped them: slow subscriber, SIGSTOP, HWM);
  segment        a new acquisition: the frame counter restarted (ScanImage numbers each acquisition
                 from 1) or the socket reconnected; `missed_before_first` = frameNumber − 1 of its
                 first frame = the slow-joiner loss;
  dropped_by_queue  received, not published (this node's own back-pressure);
  bad_frames     a message whose header or pixel length is not the documented shape.
  received == published + dropped_by_queue + queue_depth, always.

    pixi run scanimage-frames                                  # tcp://127.0.0.1:65179, preview 5 Hz
    pixi run scanimage-frames -- --preview-hz 0 --dump meta.jsonl
    pixi run rig -- --scanimage-frames                         # supervised with the bridge node (bag records the topics)
"""

from __future__ import annotations

import argparse
import json
import queue
import struct
import sys
import threading
import time
import zlib

import numpy as np
import zmq
from zmq.utils.monitor import recv_monitor_message

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from rig_nodes._common import RELIABLE_QOS, mono_ns, raise_timer_resolution, spin, wall_ns

SRC = "scanimage_frames"
META_TOPIC = "/scanimage/frame_meta"
STATS_TOPIC = "/scanimage/frame_stats"
PREVIEW_TOPIC_PREFIX = "/scanimage/preview/ch"
PREVIEW_META_TOPIC = "/scanimage/preview/meta"
DEFAULT_ADDRESS = "tcp://127.0.0.1:65179"
HEADER = struct.Struct("<5d")  # pixelsPerLine, linesPerFrame, nChannels, frameTimestamp_s, totalAcquiredFrames
QOS = RELIABLE_QOS
GAP_LIST_MAX = 200  # gap events kept in memory / sent in frame_stats (the count is always exact)
STATS_GAP_LIST = 50  # the most recent gap events per frame_stats message


# ── pure helpers (Node-free; tests import them) ────────────────────────────────────────────
def parse_header(hdr: bytes) -> dict:
    """The 40-byte header → {"ppl", "lpf", "nchan", "frameTimestamp_s", "frameNumber"}. Raises
    ValueError on the wrong length or non-integer geometry."""
    if len(hdr) != HEADER.size:
        raise ValueError(f"header is {len(hdr)} B, expected {HEADER.size}")
    ppl, lpf, nch, ts, n = HEADER.unpack(hdr)
    for name, v in (("pixelsPerLine", ppl), ("linesPerFrame", lpf), ("nChannels", nch), ("totalAcquiredFrames", n)):
        if not (v == v) or v != int(v) or v < 0 or v > 1e9:  # NaN, fractional, negative, absurd
            raise ValueError(f"header {name}={v!r} is not a non-negative integer")
    if not (ts == ts):
        raise ValueError("header frameTimestamp is NaN")
    return {"ppl": int(ppl), "lpf": int(lpf), "nchan": int(nch), "frameTimestamp_s": float(ts), "frameNumber": int(n)}


def expected_bytes(h: dict) -> int:
    return h["ppl"] * h["lpf"] * h["nchan"] * 2


def planes(buf, h: dict) -> np.ndarray:
    """The pixel part as an int16 array [channel, y, x] (a view; no copy)."""
    return np.frombuffer(buf, dtype="<i2").reshape(h["nchan"], h["lpf"], h["ppl"])


def downsample(plane: np.ndarray, size: int) -> np.ndarray:
    """(H, W) → (size, size) float32: block mean when both axes divide evenly (512 → 128), else
    nearest-neighbour sampling (also upsamples a 96-line mock frame)."""
    h, w = plane.shape
    if h % size == 0 and w % size == 0 and h >= size and w >= size:
        return plane.reshape(size, h // size, size, w // size).astype(np.float32).mean(axis=(1, 3))
    ys = (np.arange(size) * h) // size
    xs = (np.arange(size) * w) // size
    return plane[np.ix_(ys, xs)].astype(np.float32)


def auto_range(img: np.ndarray) -> tuple[float, float]:
    """A display mapping from the data: 1st–99.5th percentile, widened to at least 1 count."""
    lo, hi = (float(x) for x in np.percentile(img, (1.0, 99.5)))
    if hi <= lo:
        hi = lo + 1.0
    return lo, hi


def to_uint8(img: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.clip((img - lo) * (255.0 / (hi - lo)), 0.0, 255.0).astype(np.uint8)


def png_gray(img: np.ndarray) -> bytes:
    """8-bit grayscale (H, W) → PNG bytes (stdlib zlib; no imaging library in the env)."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    raw = b"".join(b"\x00" + img[i].tobytes() for i in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def parse_range(text: str) -> tuple[float, float] | None:
    """--preview-range: 'auto' → None, 'lo:hi' → (lo, hi)."""
    if text.strip().lower() == "auto":
        return None
    lo, hi = (float(x) for x in text.split(":", 1))
    if hi <= lo:
        raise argparse.ArgumentTypeError("--preview-range lo:hi needs hi > lo")
    return lo, hi


# ── the receive side ───────────────────────────────────────────────────────────────────────
class Segment:
    """One acquisition as seen on the stream."""

    def __init__(self, sid: int, h: dict, rx_epoch_ms: int, rx_mono_ns: int) -> None:
        self.id = sid
        self.first_frame = h["frameNumber"]
        self.missed_before_first = max(h["frameNumber"] - 1, 0)
        self.last_frame = h["frameNumber"]
        self.first_ts = self.last_ts = h["frameTimestamp_s"]
        self.dims = [h["ppl"], h["lpf"]]
        self.nchan = h["nchan"]
        self.dims_changes = 0
        self.frames = 0
        self.gaps = 0
        self.gap_frames = 0
        self.t_first_ms = self.t_last_ms = rx_epoch_ms
        self.mono_first_ns = self.mono_last_ns = rx_mono_ns

    def as_dict(self) -> dict:
        span_s = (self.mono_last_ns - self.mono_first_ns) / 1e9
        return {
            "id": self.id, "first_frame": self.first_frame, "missed_before_first": self.missed_before_first,
            "last_frame": self.last_frame, "frames": self.frames, "gaps": self.gaps, "gap_frames": self.gap_frames,
            "dims": self.dims, "nchan": self.nchan, "dims_changes": self.dims_changes,
            "first_ts_s": self.first_ts, "last_ts_s": self.last_ts, "t_first_ms": self.t_first_ms, "t_last_ms": self.t_last_ms,
            "rate_hz": round((self.frames - 1) / span_s, 3) if self.frames > 1 and span_s > 0 else None,
        }


class FrameReceiver:
    """SUB socket + monitor on its own thread → bounded queue of (meta, pixels-or-None)."""

    def __init__(self, address: str, depth: int, rcv_hwm: int, preview_hz: float, log) -> None:
        self.address = address
        self.q: queue.Queue = queue.Queue(maxsize=max(1, depth))
        self.rcv_hwm = rcv_hwm
        self.preview_period_ns = int(1e9 / preview_hz) if preview_hz > 0 else 0
        self.log = log
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="zmq-rx", daemon=True)
        # counters (under lock)
        self.frames = 0
        self.bytes = 0
        self.bad_frames = 0
        self.dropped_by_queue = 0
        self.gaps = 0
        self.gap_frames = 0
        self.gap_list: list[dict] = []
        self.segments: list[Segment] = []
        self.connected = False
        self.connects = 0
        self.disconnects = 0
        self.reconnected = False  # a (re)connect since the last frame → next frame opens a segment
        self.queue_max = 0
        self._last_preview_ns = 0
        self.started_mono = mono_ns()

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=3.0)

    # ── thread body ──
    def _run(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVHWM, self.rcv_hwm)
        sock.setsockopt(zmq.RECONNECT_IVL, 100)
        sock.setsockopt(zmq.RECONNECT_IVL_MAX, 500)  # ScanImage rebinds per acquisition; a localhost SYN every ≤ 0.5 s is cheap
        mon = sock.get_monitor_socket(zmq.EVENT_CONNECTED | zmq.EVENT_DISCONNECTED | zmq.EVENT_CLOSED)
        sock.connect(self.address)
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        poller.register(mon, zmq.POLLIN)
        try:
            while not self.stop.is_set():
                for s, _ in poller.poll(100):
                    if s is mon:
                        self._monitor_event(recv_monitor_message(mon))
                    else:
                        self._frame(sock.recv_multipart(copy=False))
        finally:
            try:
                sock.disable_monitor()
            except Exception:
                pass
            mon.close(0)
            sock.close(0)

    def _monitor_event(self, ev: dict) -> None:
        kind = ev.get("event")
        with self.lock:
            if kind == zmq.EVENT_CONNECTED:
                was = self.connects
                self.connects += 1
                self.connected = True
                if was > 0:
                    self.reconnected = True
                self.log.info(f"{'re' if was else ''}connected to {self.address} (#{self.connects})")
            elif kind in (zmq.EVENT_DISCONNECTED, zmq.EVENT_CLOSED):
                if self.connected:
                    self.disconnects += 1
                    self.log.info(f"publisher at {self.address} went away (waiting for the next bind)")
                self.connected = False

    def _frame(self, parts: list) -> None:
        rx_mono = mono_ns()
        rx_wall = wall_ns()
        try:
            if len(parts) != 2:
                raise ValueError(f"{len(parts)}-part message, expected 2")
            h = parse_header(bytes(parts[0].buffer))
            n = expected_bytes(h)
            if len(parts[1].buffer) != n:
                raise ValueError(f"{len(parts[1].buffer)} pixel bytes, header says {n}")
        except ValueError as exc:
            with self.lock:
                self.bad_frames += 1
                bad = self.bad_frames
            if bad <= 5 or bad % 100 == 0:
                self.log.warning(f"bad frame #{bad}: {exc}")
            return
        want_preview = False
        if self.preview_period_ns and rx_mono - self._last_preview_ns >= self.preview_period_ns:
            want_preview = True
            self._last_preview_ns = rx_mono
        with self.lock:
            self.frames += 1
            self.bytes += n
            gap_before = 0
            seg = self.segments[-1] if self.segments else None
            new = seg is None or self.reconnected or h["frameNumber"] <= seg.last_frame or h["frameTimestamp_s"] < seg.last_ts - 1e-6
            if new:
                seg = Segment(len(self.segments) + 1, h, rx_wall // 1_000_000, rx_mono)
                self.segments.append(seg)
                self.reconnected = False
                self.log.info(f"segment {seg.id}: first frame {seg.first_frame} ({seg.missed_before_first} missed before it), "
                              f"{h['ppl']}x{h['lpf']} x {h['nchan']} ch")
            else:
                if h["frameNumber"] > seg.last_frame + 1:
                    gap_before = h["frameNumber"] - seg.last_frame - 1
                    seg.gaps += 1
                    seg.gap_frames += gap_before
                    self.gaps += 1
                    self.gap_frames += gap_before
                    if len(self.gap_list) < GAP_LIST_MAX:
                        self.gap_list.append({"segment": seg.id, "after": seg.last_frame, "next": h["frameNumber"], "missing": gap_before,
                                              "rx_epoch_ms": rx_wall // 1_000_000})
                    if seg.gaps <= 5 or seg.gaps % 100 == 0:
                        self.log.warning(f"gap: {gap_before} frame(s) missing between {seg.last_frame} and {h['frameNumber']} (segment {seg.id})")
                if [h["ppl"], h["lpf"]] != seg.dims or h["nchan"] != seg.nchan:
                    seg.dims_changes += 1
                    seg.dims, seg.nchan = [h["ppl"], h["lpf"]], h["nchan"]
                seg.last_frame = h["frameNumber"]
                seg.last_ts = h["frameTimestamp_s"]
            seg.frames += 1
            seg.t_last_ms = rx_wall // 1_000_000
            seg.mono_last_ns = rx_mono
            meta = {"seq": self.frames, "segment": seg.id, "frameNumber": h["frameNumber"], "frameTimestamp_s": h["frameTimestamp_s"],
                    "dims": [h["ppl"], h["lpf"]], "nchan": h["nchan"], "bytes": n, "gap_before": gap_before,
                    "rx_epoch_ms": rx_wall // 1_000_000, "rx_wall_ns": rx_wall, "rx_mono_ns": rx_mono, "preview": want_preview}
            try:
                # copy the pixels only for a preview frame; the queue entry is otherwise tiny
                self.q.put_nowait((meta, bytes(parts[1].buffer) if want_preview else None))
                depth = self.q.qsize()
                if depth > self.queue_max:
                    self.queue_max = depth
            except queue.Full:
                self.dropped_by_queue += 1
                if self.dropped_by_queue <= 5 or self.dropped_by_queue % 100 == 0:
                    self.log.warning(f"queue full ({self.q.maxsize}): dropped frame {h['frameNumber']} ({self.dropped_by_queue} dropped so far)")

    def snapshot(self) -> dict:
        with self.lock:
            qmax, self.queue_max = self.queue_max, self.q.qsize()
            return {
                "address": self.address, "connected": self.connected, "connects": self.connects, "disconnects": self.disconnects,
                "frames": self.frames, "rx_bytes": self.bytes, "bad_frames": self.bad_frames, "dropped_by_queue": self.dropped_by_queue,
                "gaps": self.gaps, "gap_frames": self.gap_frames, "gap_list": list(self.gap_list[-STATS_GAP_LIST:]),
                "gap_list_truncated": len(self.gap_list) >= GAP_LIST_MAX,
                "segments": len(self.segments), "current_segment": self.segments[-1].id if self.segments else None,
                "segment_list": [s.as_dict() for s in self.segments],
                "queue_depth": self.q.qsize(), "queue_max": qmax, "queue_limit": self.q.maxsize,
            }


# ── the ROS side ───────────────────────────────────────────────────────────────────────────
class ScanImageFramesNode(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("scanimage_frames")
        self.a = a
        self.meta_pub = self.create_publisher(String, META_TOPIC, QOS)
        self.stats_pub = self.create_publisher(String, STATS_TOPIC, QOS)
        self.preview_meta_pub = self.create_publisher(String, PREVIEW_META_TOPIC, QOS) if a.preview_hz > 0 else None
        self.preview_pubs: dict[int, object] = {}
        self.channel_ids = [int(x) for x in a.channel_ids.split(",") if x.strip()] if a.channel_ids else []
        self.preview_channels: set[int] = set()
        if a.preview_hz > 0:
            # Declared up front: creating a publisher on the first frame of the first acquisition held
            # the receive side ~0.85 s (zenoh declaration under the GIL) in the off-rig smoke run.
            for ch in (self.channel_ids or list(range(1, a.preview_channels + 1))):
                self._preview_pub(ch)
        self.fixed_range = parse_range(a.preview_range)
        self._auto_ranges: dict[tuple[int, int], tuple[float, float]] = {}  # (segment, channel) → (lo, hi)
        self.published = 0
        self.previews = 0
        self._hops_ms: list[float] = []  # in-node hop (publish − receive) since the last stats message
        self._dump = open(a.dump, "a", encoding="utf-8") if a.dump else None
        self._stats_prev = (mono_ns(), 0)
        self.rx = FrameReceiver(a.address, a.queue_depth, a.rcv_hwm, a.preview_hz, self.get_logger())
        self.rx.start()
        self._stop = threading.Event()
        self._pub_thread = threading.Thread(target=self._publish_loop, name="ros-pub", daemon=True)
        self._pub_thread.start()
        self.create_timer(max(0.1, a.stats_every), self._stats)
        prev = f"preview ≤ {a.preview_hz:g} Hz/channel {a.preview_size}² PNG → {PREVIEW_TOPIC_PREFIX}<id>" if a.preview_hz > 0 else "preview off"
        self.get_logger().info(f"SUB {a.address} → {META_TOPIC}, {STATS_TOPIC} every {a.stats_every:g}s, {prev}, queue {a.queue_depth}")

    # ── plumbing ──
    def _send(self, pub, obj: dict, dump_topic: str | None = None) -> None:
        m = String()
        m.data = json.dumps(obj, separators=(",", ":"))
        pub.publish(m)
        if self._dump is not None and dump_topic:
            self._dump.write(json.dumps({"topic": dump_topic, **obj}, separators=(",", ":")) + "\n")

    def _publish_loop(self) -> None:
        a = self.a
        while not self._stop.is_set():
            try:
                meta, pixels = self.rx.q.get(timeout=0.1)
            except queue.Empty:
                continue
            if a.debug_stall_s > 0 and meta["seq"] == a.debug_stall_at:
                self.get_logger().warning(f"debug: stalling the publisher for {a.debug_stall_s:g}s at frame seq {meta['seq']}")
                time.sleep(a.debug_stall_s)
            meta["queue_depth"] = self.rx.q.qsize()
            meta["pub_mono_ns"] = mono_ns()
            if len(self._hops_ms) < 100_000:
                self._hops_ms.append((meta["pub_mono_ns"] - meta["rx_mono_ns"]) / 1e6)
            out = {"src": SRC, **{k: v for k, v in meta.items() if k != "rx_wall_ns"}}
            try:
                self._send(self.meta_pub, out, META_TOPIC)
                self.published += 1
                if pixels is not None:
                    self._preview(meta, pixels)
            except Exception as exc:  # never let one frame kill the publisher thread
                self.get_logger().error(f"publish failed: {exc!r}")

    def _preview_pub(self, ch: int):
        pub = self.preview_pubs.get(ch)
        if pub is None:
            pub = self.preview_pubs[ch] = self.create_publisher(CompressedImage, f"{PREVIEW_TOPIC_PREFIX}{ch}", QOS)
            self.preview_channels.add(ch)
        return pub

    def _preview(self, meta: dict, pixels: bytes) -> None:
        a = self.a
        h = {"ppl": meta["dims"][0], "lpf": meta["dims"][1], "nchan": meta["nchan"]}
        arr = planes(pixels, h)
        stamp_s, stamp_ns = divmod(int(meta["rx_wall_ns"]), 1_000_000_000)
        for plane_i in range(h["nchan"]):
            ch = self.channel_ids[plane_i] if plane_i < len(self.channel_ids) else plane_i + 1
            small = downsample(arr[plane_i], a.preview_size)
            if self.fixed_range is not None:
                lo, hi = self.fixed_range
                mode = "fixed"
            else:
                key = (meta["segment"], ch)
                if key not in self._auto_ranges:
                    self._auto_ranges[key] = auto_range(small)
                    for old in [k for k in self._auto_ranges if k[0] < meta["segment"] - 2]:
                        del self._auto_ranges[old]
                lo, hi = self._auto_ranges[key]
                mode = "auto"
            png = png_gray(to_uint8(small, lo, hi))
            pub = self._preview_pub(ch)
            img = CompressedImage()
            img.header.stamp.sec = int(stamp_s)
            img.header.stamp.nanosec = int(stamp_ns)
            img.header.frame_id = f"scanimage/ch{ch}/s{meta['segment']}/f{meta['frameNumber']}"
            img.format = "png"
            img.data = png
            pub.publish(img)
            self.previews += 1
            self._send(self.preview_meta_pub, {
                "src": SRC, "image_topic": f"{PREVIEW_TOPIC_PREFIX}{ch}", "channel": ch, "plane": plane_i, "segment": meta["segment"],
                "frameNumber": meta["frameNumber"], "frameTimestamp_s": meta["frameTimestamp_s"], "dims": meta["dims"],
                "size": [a.preview_size, a.preview_size],
                "mapping": {"mode": mode, "lo": lo, "hi": hi, "to": "uint8 linear, clipped", "downsample": "block mean" if (h["lpf"] % a.preview_size == 0 and h["ppl"] % a.preview_size == 0 and h["lpf"] >= a.preview_size and h["ppl"] >= a.preview_size) else "nearest"},
                "png_bytes": len(png), "rx_epoch_ms": meta["rx_epoch_ms"], "rx_mono_ns": meta["rx_mono_ns"], "pub_mono_ns": mono_ns(),
            }, PREVIEW_META_TOPIC)

    def _stats(self) -> None:
        now = mono_ns()
        snap = self.rx.snapshot()
        t_prev, f_prev = self._stats_prev
        dt = (now - t_prev) / 1e9
        self._stats_prev = (now, snap["frames"])
        hops, self._hops_ms = self._hops_ms, []
        hops.sort()
        hop = ({"n": len(hops), "p50_ms": round(hops[len(hops) // 2], 3), "p99_ms": round(hops[min(len(hops) - 1, int(len(hops) * 0.99))], 3),
                "max_ms": round(hops[-1], 3)} if hops else {"n": 0})
        obj = {
            "src": SRC, "t_ms": wall_ns() // 1_000_000, "uptime_s": round((now - self.rx.started_mono) / 1e9, 1),
            **snap, "published": self.published,
            "rate_hz": round((snap["frames"] - f_prev) / dt, 2) if dt > 0 else None,
            "hop": hop,  # receive → publish inside this node, over the last interval
            "preview": {"hz": self.a.preview_hz, "size": self.a.preview_size, "published": self.previews,
                        "channels": sorted(self.preview_channels), "range": self.a.preview_range},
        }
        self._send(self.stats_pub, obj, STATS_TOPIC)
        if self._dump is not None:
            self._dump.flush()

    def destroy_node(self):
        self.close()  # stop the receive + publish threads before the publishers go away
        return super().destroy_node()

    def close(self) -> None:
        if getattr(self, "_closed", False):  # destroy_node() and main()'s finally both call it
            return
        self._closed = True
        self._stop.set()
        for _ in range(3):  # rclpy forwards SIGINT to Python's handler too: its KeyboardInterrupt may land in a join
            try:
                self.rx.close()
                self._pub_thread.join(timeout=2.0)
                break
            except KeyboardInterrupt:
                continue
        if self._dump is not None:
            self._dump.flush()
            self._dump.close()


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default=DEFAULT_ADDRESS, help=f"ScanImage's ZMQAddress (default {DEFAULT_ADDRESS})")
    p.add_argument("--queue-depth", type=int, default=64, help="frames buffered between the receive thread and the ROS publisher; a full queue drops the newest frame (counted; default 64)")
    p.add_argument("--rcv-hwm", type=int, default=32, help="ZeroMQ receive high-water mark in messages (default 32; beyond it libzmq stops reading and the PUBLISHER drops)")
    p.add_argument("--stats-every", type=float, default=1.0, help=f"seconds between {STATS_TOPIC} messages (default 1)")
    p.add_argument("--preview-hz", type=float, default=5.0, help="preview images per channel per second at most (default 5; 0 = no preview)")
    p.add_argument("--preview-size", type=int, default=128, help="preview edge in pixels (default 128)")
    p.add_argument("--preview-range", default="auto", help="int16 → 8-bit display mapping: 'auto' (1–99.5 percentile of the first preview frame of each segment, then held) or 'lo:hi' fixed (default auto)")
    p.add_argument("--preview-channels", type=int, default=4, help="preview publishers declared at start, ch1..chN (default 4 = ScanImage's channels); more are added on demand")
    p.add_argument("--channel-ids", default="", help="comma list naming the planes, e.g. '1,3' when ScanImage's channelSave is [1 3] (default: 1..nChannels)")
    p.add_argument("--dump", default=None, help="also append every frame_meta / frame_stats / preview meta line to this JSONL file (rig_nodes.zmq_latency_report reads it when there is no bag)")
    p.add_argument("--debug-stall-at", type=int, default=0, help="(harness) stall the ROS publisher at this received-frame seq …")
    p.add_argument("--debug-stall-s", type=float, default=0.0, help="(harness) … for this many seconds, to exercise dropped_by_queue")
    a = p.parse_args(argv)
    if a.preview_size < 8 or a.preview_size > 1024:
        raise SystemExit("--preview-size must be 8..1024")
    parse_range(a.preview_range)
    raise_timer_resolution()
    rclpy.init()
    node = ScanImageFramesNode(a)
    try:
        spin(node)
    finally:
        node.close()


if __name__ == "__main__":
    main()
