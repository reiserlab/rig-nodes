#!/usr/bin/env python3
"""test_scanimage_frames.py — the ScanImage ZMQ frame-stream node, off-rig (LAB-303 Block R).

    pixi run test-frames                  # from rig/ (≈ 45 s); also the first half of `pixi run test`
    pixi run test-frames -- --keep --mock ../scanimage-2026-bergamo/tests/zmq-frames/mock_zmq_publisher.py
    python tests/test_scanimage_frames.py     # anywhere rig_nodes is installed (the acceptance job)

Brings up a Zenoh router, rig_nodes.scanimage_frames_node and an in-process ROS subscriber, then
drives the node with the frame publisher stand-in (rig_nodes.sims.zmq_frames_mock: Hannah's
mock_zmq_publisher.py format and CLI; `--mock` runs the original) at 29.08 Hz × 512 × 512 × 2 ch:

  A  one acquisition (87 frames)          every frame counted, dims/nchan right, frame numbers contiguous,
                                          segment 1 from frame 1, previews ≤ 5 Hz per channel, 128² PNG,
                                          display mapping recorded, in-node hop small
  B  gap injection (--drop-every 10)      a NEW segment (frame counter restarted), the 9 visible gaps
                                          counted (a dropped LAST frame is invisible to any subscriber)
  C  publisher restart, no start delay    the slow-joiner loss recorded as the segment's
                                          missed_before_first, reconnects counted
  D1 slow consumer, publisher HWM 1000    the node SIGSTOPped 1.5 s mid-stream: the publisher's timing is
                                          unaffected, nothing lost (it buffered — what ScanImage's memory
                                          does), the backlog visible as queue depth / hop (POSIX only)
  D2 slow consumer, publisher HWM 20      same pause: the publisher DROPS, the node's accounting names
                                          every missing frame (gaps / tail), publisher timing unaffected
  E  node restart + queue back-pressure   clean SIGINT exit; a second node with a tiny queue and a stalled
                                          publisher thread counts dropped_by_queue exactly
  F  rig_nodes.zmq_latency_report        on the node's --dump JSONL and on a bag written with
                                          rig_nodes.recorder from the received messages (the bridge's
                                          record path), with a synthetic ScanImage-style TIFF for the
                                          TIFF-clock alignment
  G  bridge_node registration             the stream topics are in the bag's topic map; nothing derives them

Exit 0 when every check passes; artifacts kept with --keep.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable
BUNDLED_MOCK = "rig_nodes.sims.zmq_frames_mock"  # the stand-in publisher, installed with the package
NODE = [PY, "-m", "rig_nodes.scanimage_frames_node"]
TOOL = [PY, "-m", "rig_nodes.zmq_latency_report"]
from rig_nodes._common import RELIABLE_QOS, wall_ns  # noqa: E402
from rig_nodes.supervise import SIGKILL, popen_group, signal_group, wait_port  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import String  # noqa: E402

META_TOPIC = "/scanimage/frame_meta"
STATS_TOPIC = "/scanimage/frame_stats"
PREVIEW_META_TOPIC = "/scanimage/preview/meta"
PREVIEW_PREFIX = "/scanimage/preview/ch"
RATE = 29.08
DIMS = (512, 512)
NCH = 2

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, info: str = "") -> bool:
    results.append((name, bool(ok), info))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'' if ok else ' — ' + info}", file=sys.stderr)
    return bool(ok)


def note(text: str) -> None:
    print(f"  info  {text}", file=sys.stderr)


def free_tcp_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class Proc:
    def __init__(self, name: str, argv: list[str], logdir: Path) -> None:
        self.name = name
        self.argv = argv
        self.log = open(logdir / f"{name}.log", "a", encoding="utf-8")
        self.p: subprocess.Popen | None = None
        self.t_start = 0.0

    def start(self) -> "Proc":
        self.t_start = time.perf_counter()
        self.p = popen_group(self.argv, stdout=self.log, stderr=subprocess.STDOUT, text=True)
        return self

    def alive(self) -> bool:
        return self.p is not None and self.p.poll() is None

    def wait(self, timeout: float) -> float:
        """Wait for exit → elapsed seconds since start."""
        assert self.p
        self.p.wait(timeout)
        return time.perf_counter() - self.t_start

    def stop(self, grace: float = 5.0) -> int | None:
        if not self.alive():
            return self.p.returncode if self.p else None
        try:
            signal_group(self.p, signal.SIGINT)
            self.p.wait(grace)
        except subprocess.TimeoutExpired:
            signal_group(self.p, signal.SIGTERM)
            try:
                self.p.wait(3)
            except subprocess.TimeoutExpired:
                signal_group(self.p, SIGKILL)
        except (ProcessLookupError, OSError, ValueError):
            pass
        self.log.flush()
        return self.p.returncode


class Collector(Node):
    """The harness's own subscriber: everything the frames node publishes, with receive stamps."""

    def __init__(self) -> None:
        super().__init__("frames_harness")
        self.lock = threading.Lock()
        self.meta: list[tuple[int, dict, String]] = []  # (wall_ns, obj, msg)
        self.stats: list[tuple[int, dict, String]] = []
        self.pmeta: list[tuple[int, dict, String]] = []
        self.images: dict[int, list[tuple[int, CompressedImage]]] = {}
        self.create_subscription(String, META_TOPIC, lambda m: self._json(self.meta, m), RELIABLE_QOS)
        self.create_subscription(String, STATS_TOPIC, lambda m: self._json(self.stats, m), RELIABLE_QOS)
        self.create_subscription(String, PREVIEW_META_TOPIC, lambda m: self._json(self.pmeta, m), RELIABLE_QOS)
        for ch in (1, 2, 3, 4):
            self.create_subscription(CompressedImage, f"{PREVIEW_PREFIX}{ch}", lambda m, ch=ch: self._img(ch, m), RELIABLE_QOS)

    def _json(self, dest: list, m: String) -> None:
        t = wall_ns()
        try:
            obj = json.loads(m.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            dest.append((t, obj, m))

    def _img(self, ch: int, m: CompressedImage) -> None:
        t = wall_ns()
        with self.lock:
            self.images.setdefault(ch, []).append((t, m))

    # snapshots
    def metas(self) -> list[dict]:
        with self.lock:
            return [o for _, o, _ in self.meta]

    def last_stats(self) -> dict | None:
        with self.lock:
            return self.stats[-1][1] if self.stats else None

    def n_images(self, ch: int) -> int:
        with self.lock:
            return len(self.images.get(ch, []))

    def clear_images(self) -> None:
        with self.lock:
            self.images.clear()
            self.pmeta.clear()


def wait_for(pred, timeout: float, period: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


def wait_settled(col: Collector, timeout: float = 12.0) -> dict | None:
    """Wait until the node's stats say the queue is drained and the frame count stopped moving."""
    last = None
    same_since = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = col.last_stats()
        if st is not None:
            key = (st["frames"], st["published"], st["dropped_by_queue"])
            if st["published"] + st["dropped_by_queue"] == st["frames"] and st["queue_depth"] == 0:
                if key == last:
                    if same_since is not None and time.monotonic() - same_since >= 1.2:
                        return st
                else:
                    same_since = time.monotonic()
                last = key
        time.sleep(0.1)
    return col.last_stats()


def segment(st: dict | None, sid: int) -> dict | None:
    if not st:
        return None
    for s in st.get("segment_list", []):
        if s["id"] == sid:
            return s
    return None


def png_size(data: bytes) -> tuple[int, int] | None:
    if data[:8] != b"\x89PNG\r\n\x1a\n" or data[12:16] != b"IHDR":
        return None
    return struct.unpack(">II", data[16:24])


def write_si_tiff(path: Path, frames: list[tuple[int, float]], nch: int, epoch: list[float]) -> None:
    """A classic little-endian TIFF the way ScanImage lays it out: one 8×8 int16 page per channel per
    frame, each page's ImageDescription carrying frameNumbers / frameTimestamps_sec / epoch. Enough for
    tools/zmq_latency_report.py's stdlib reader (tifffile is not in the env)."""
    w = h = 8
    pixels = bytes(w * h * 2)
    out = bytearray(b"II*\x00" + struct.pack("<I", 8))
    ifd_offsets = []
    for k, ts in frames:
        for c in range(nch):
            desc = (f"frameNumbers = {k}\nacquisitionNumbers = 1\nframeNumberAcquisition = {k}\nframeTimestamps_sec = {ts:.6f}\n"
                    f"acqTriggerTimestamps_sec = 0.000000\nepoch = [{' '.join(f'{v:g}' for v in epoch)}]\nendOfAcquisition = 0\nchannel = {c + 1}\n").encode("ascii") + b"\x00"
            data_off = len(out)
            out += pixels
            desc_off = len(out)
            out += desc
            if len(out) % 2:
                out += b"\x00"
            ifd_offsets.append(len(out))
            tags = [(256, 3, 1, w), (257, 3, 1, h), (258, 3, 1, 16), (259, 3, 1, 1), (262, 3, 1, 1),
                    (270, 2, len(desc), desc_off), (273, 4, 1, data_off), (277, 3, 1, 1), (278, 3, 1, h),
                    (279, 4, 1, len(pixels)), (339, 3, 1, 2)]
            out += struct.pack("<H", len(tags))
            for tag, typ, count, val in tags:
                out += struct.pack("<HHI", tag, typ, count) + (struct.pack("<HH", val, 0) if typ == 3 else struct.pack("<I", val))
            out += struct.pack("<I", 0)  # next IFD, patched below
    # the header's first-IFD pointer, then chain the IFDs
    struct.pack_into("<I", out, 4, ifd_offsets[0])
    for a, b in zip(ifd_offsets, ifd_offsets[1:]):
        n = struct.unpack_from("<H", out, a)[0]
        struct.pack_into("<I", out, a + 2 + n * 12, b)
    path.write_bytes(bytes(out))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", default=BUNDLED_MOCK, help="the frame publisher to drive the node with: a path, or the bundled rig_nodes.sims.zmq_frames_mock (default)")
    ap.add_argument("--keep", action="store_true", help="keep the work dir")
    ap.add_argument("--workdir", default=None)
    ap.add_argument("--no-router", action="store_true", help="a Zenoh router is already running on 127.0.0.1:7447")
    ap.add_argument("--pause-s", type=float, default=1.5, help="how long the node is SIGSTOPped in D1/D2")
    a = ap.parse_args(argv)

    work = Path(a.workdir) if a.workdir else Path(tempfile.mkdtemp(prefix="si-frames-test-"))
    work.mkdir(parents=True, exist_ok=True)
    logdir = work / "proc-logs"
    logdir.mkdir(exist_ok=True)
    bundled = a.mock == BUNDLED_MOCK
    mock = Path(a.mock)
    if not bundled and not mock.exists():
        print(f"[test] mock publisher not found: {mock}", file=sys.stderr)
        return 2
    mock_argv = [PY, "-m", BUNDLED_MOCK] if bundled else [PY, str(mock)]
    print(f"[test] work dir {work}; publisher {mock}{'' if bundled else ' (external — D2 needs the bundled --sndhwm, skipped)'}", file=sys.stderr)
    procs: list[Proc] = []
    port = free_tcp_port()
    address = f"tcp://127.0.0.1:{port}"
    dump_a = work / "node-a.jsonl"
    dump_b = work / "node-b.jsonl"
    posix = os.name != "nt"

    def start(name: str, argv: list[str]) -> Proc:
        pr = Proc(name, argv, logdir).start()
        procs.append(pr)
        return pr

    def run_mock(name: str, frames: int, **kw) -> Proc:
        argv = [*mock_argv, "--address", address, "--frames", str(frames), "--rate", str(RATE),
                "--ppl", str(DIMS[0]), "--lpf", str(DIMS[1]), "--nch", str(NCH)]
        for k, v in kw.items():
            argv += [f"--{k.replace('_', '-')}", str(v)]
        return start(name, argv)

    node_a = node_b = None
    col = ex = None
    try:
        # the router FIRST: creating any rclpy node (our subscriber included) blocks until it answers
        if not a.no_router:
            start("router", [PY, "-m", "rig_nodes.zenoh"])
            check("zenoh router listening on 127.0.0.1:7447", wait_port("127.0.0.1", 7447, 20.0))
        rclpy.init()
        col = Collector()
        ex = SingleThreadedExecutor()
        ex.add_node(col)
        threading.Thread(target=ex.spin, name="harness-spin", daemon=True).start()
        node_a = start("node-a", [*NODE, "--address", address, "--stats-every", "0.5",
                                  "--preview-hz", "5", "--preview-size", "128", "--dump", str(dump_a)])
        check("node up: frame_stats arrives (0 frames, not connected)",
              wait_for(lambda: col.last_stats() is not None, 20.0) and col.last_stats()["frames"] == 0, json.dumps(col.last_stats()))
        time.sleep(0.5)

        # ── A: one acquisition ──────────────────────────────────────────────────────────
        n_a = 87
        m = run_mock("mock-a", n_a, start_delay=1.0)
        el = m.wait(30)
        st = wait_settled(col)
        metas = col.metas()
        seg1 = segment(st, 1)
        check(f"A: all {n_a} frames received and published", st and st["frames"] == n_a and st["published"] == n_a and len(metas) == n_a,
              f"stats frames={st and st['frames']} published={st and st['published']} meta msgs={len(metas)}")
        check("A: dims 512×512 and nchan 2 on every frame_meta", all(tuple(o["dims"]) == DIMS and o["nchan"] == NCH and o["bytes"] == DIMS[0] * DIMS[1] * NCH * 2 for o in metas),
              str({(tuple(o["dims"]), o["nchan"]) for o in metas}))
        check("A: frame numbers 1..87 contiguous, one segment from frame 1, 0 gaps",
              [o["frameNumber"] for o in metas] == list(range(1, n_a + 1)) and seg1 and seg1["first_frame"] == 1 and seg1["missed_before_first"] == 0 and st["gaps"] == 0 and st["segments"] == 1,
              json.dumps(seg1))
        check("A: node connected to the publisher (monitor) and saw it go away", st["connects"] == 1 and st["disconnects"] >= 1, f"connects={st['connects']} disconnects={st['disconnects']}")
        check("A: header timestamps advance at the publisher's rate", seg1 and seg1["rate_hz"] and abs(seg1["rate_hz"] - RATE) < 1.0, f"rate_hz={seg1 and seg1['rate_hz']}")
        hops = sorted((o["pub_mono_ns"] - o["rx_mono_ns"]) / 1e6 for o in metas[3:])
        hop_p99 = hops[int(len(hops) * 0.99) - 1] if hops else None
        check("A: in-node hop p99 (receive → publish) < 100 ms after the first 3 frames", hop_p99 is not None and hop_p99 < 100.0, f"p99={hop_p99} ms, max={hops[-1] if hops else None}")
        note(f"A: in-node hop p50 {hops[len(hops) // 2]:.2f} ms, p99 {hop_p99:.2f} ms, max {hops[-1]:.2f} ms; queue max {max(o['queue_depth'] for o in metas)}")
        span = (metas[-1]["rx_mono_ns"] - metas[0]["rx_mono_ns"]) / 1e9
        for ch in (1, 2):
            n_img = col.n_images(ch)
            check(f"A: preview ch{ch} published and bounded: 1 ≤ {n_img} ≤ 5 Hz × {span:.1f} s + 1", 1 <= n_img <= int(5 * span) + 1, f"{n_img} images in {span:.2f} s")
        with col.lock:
            imgs = list(col.images.get(1, []))
            pm = [o for _, o, _ in col.pmeta]
        sizes = {png_size(bytes(m.data)) for _, m in imgs}
        check("A: preview images are 128×128 PNG (format 'png', frame_id names channel/segment/frame)",
              sizes == {(128, 128)} and all(m.format == "png" and m.header.frame_id.startswith("scanimage/ch1/s1/f") for _, m in imgs), f"sizes={sizes}")
        check("A: preview meta records the display mapping (lo < hi) for every image and names the image topic",
              len(pm) == sum(col.n_images(c) for c in (1, 2)) and all(o["mapping"]["lo"] < o["mapping"]["hi"] and o["image_topic"].startswith(PREVIEW_PREFIX) and o["size"] == [128, 128] for o in pm),
              f"{len(pm)} preview meta vs {sum(col.n_images(c) for c in (1, 2))} images")
        check("A: publisher ran at its own pace (no back-pressure from the subscriber)", el < n_a / RATE + 1.0 + 1.5, f"{el:.2f} s for {n_a} frames + 1 s delay")

        # ── B: gap injection + a restarted frame counter ────────────────────────────────
        n_b = 100
        m = run_mock("mock-b", n_b, start_delay=1.0, drop_every=10)
        m.wait(30)
        st = wait_settled(col)
        seg2 = segment(st, 2)
        check("B: the restarted frame counter opened segment 2 from frame 1", st and st["segments"] == 2 and seg2 and seg2["first_frame"] == 1 and seg2["missed_before_first"] == 0, json.dumps(seg2))
        check("B: 9 gaps of 1 frame (10, 20, … 90) counted; frame 100 — the dropped LAST frame — is invisible to a subscriber",
              seg2 and seg2["gaps"] == 9 and seg2["gap_frames"] == 9 and seg2["frames"] == 90 and seg2["last_frame"] == 99, json.dumps(seg2))
        gl = [g for g in st["gap_list"] if g["segment"] == 2]
        check("B: gap_list names each gap (after/next/missing)", [(g["after"], g["next"], g["missing"]) for g in gl] == [(k - 1, k + 1, 1) for k in range(10, 100, 10)], json.dumps(gl))
        metas_b = [o for o in col.metas() if o["segment"] == 2]
        check("B: frame_meta carries gap_before on the frame after each gap", [o["frameNumber"] for o in metas_b if o["gap_before"] == 1] == list(range(11, 100, 10)), "")

        # ── C: slow joiner (publisher binds and sends at once) ──────────────────────────
        n_c = 60
        m = run_mock("mock-c", n_c, start_delay=0.0)
        m.wait(30)
        st = wait_settled(col)
        seg3 = segment(st, 3)
        check("C: a new segment after the publisher restart; the socket reconnected (connects = 3)", st and st["segments"] == 3 and st["connects"] == 3, f"segments={st and st['segments']} connects={st and st['connects']}")
        check("C: the slow-joiner loss is recorded: missed_before_first = first_frame − 1 ≥ 1, no gaps after it",
              seg3 and seg3["missed_before_first"] == seg3["first_frame"] - 1 >= 1 and seg3["gaps"] == 0 and seg3["frames"] == n_c - seg3["missed_before_first"], json.dumps(seg3))
        note(f"C: first frame seen {seg3 and seg3['first_frame']} ({seg3 and seg3['missed_before_first']} missed before it)")

        # ── D: slow consumer (the node paused) ──────────────────────────────────────────
        def paused_run(name: str, frames: int, **kw) -> tuple[float, dict | None, dict | None]:
            m = run_mock(name, frames, start_delay=1.0, **kw)
            sid = (col.last_stats() or {}).get("segments", 0) + 1
            wait_for(lambda: len([o for o in col.metas() if o["segment"] == sid]) >= 20, 10.0)
            os.kill(node_a.p.pid, signal.SIGSTOP)
            time.sleep(a.pause_s)
            os.kill(node_a.p.pid, signal.SIGCONT)
            el = m.wait(60)
            st = wait_settled(col, 20.0)
            return el, st, segment(st, sid)

        if posix:
            n_d = 150
            budget = n_d / RATE + 1.0 + 1.5
            el, st, seg4 = paused_run("mock-d1", n_d)
            check(f"D1: publisher (HWM 1000) finished on time while the node was paused {a.pause_s:g} s", el < budget, f"{el:.2f} s vs budget {budget:.2f} s")
            check("D1: nothing lost — the publisher buffered the pause (what ScanImage's memory does); accounting exact",
                  seg4 and seg4["frames"] == n_d and seg4["gap_frames"] == 0 and seg4["missed_before_first"] == 0 and st["dropped_by_queue"] == 0, json.dumps(seg4))
            metas_d = [o for o in col.metas() if o["segment"] == 4]
            worst = max(((o["pub_mono_ns"] - o["rx_mono_ns"]) / 1e6 for o in metas_d), default=0)
            # The backlog's signature depends on the machine: a node that drains slower than the burst shows
            # queue depth ≥ 2; one that keeps up (CI's macOS runner, 2026-10-28) shows the burst itself — many
            # frames received within a short span of rx_mono_ns, far above the 29 Hz pace (≈ 6 per 200 ms).
            rx = sorted(o["rx_mono_ns"] for o in metas_d)
            burst = max((sum(1 for t in rx[i:] if t - rx[i] <= 300_000_000) for i in range(len(rx))), default=0)
            qmax = max((o["queue_depth"] for o in metas_d), default=0)
            check("D1: the backlog is visible after the pause: queue depth ≥ 2, or a burst of ≥ 20 frames within 300 ms",
                  metas_d and (qmax >= 2 or burst >= 20), f"queue max {qmax}, burst {burst} frames / 300 ms, hop max {worst:.0f} ms")
            note(f"D1: queue max {qmax}, burst {burst} frames within 300 ms, in-node hop max {worst:.0f} ms")
            if bundled:
                el, st, seg5 = paused_run("mock-d2", n_d, sndhwm=20)
                check(f"D2: publisher (HWM 20) finished on time while the node was paused {a.pause_s:g} s", el < budget, f"{el:.2f} s vs budget {budget:.2f} s")
                lost = seg5 and (seg5["gap_frames"] + (n_d - seg5["last_frame"]) + seg5["missed_before_first"])
                check("D2: the publisher dropped frames during the pause and the node counted them (gaps and/or a missing tail)",
                      seg5 and lost and lost > 0 and seg5["frames"] + seg5["gap_frames"] == seg5["last_frame"] - seg5["first_frame"] + 1 and st["dropped_by_queue"] == 0,
                      json.dumps(seg5))
                note(f"D2: {seg5 and seg5['frames']} received, {seg5 and seg5['gap_frames']} in {seg5 and seg5['gaps']} gaps, tail {seg5 and n_d - seg5['last_frame']} missing → {lost} lost of {n_d}")
        else:
            note("D1/D2 skipped: no SIGSTOP on Windows (the lab-day procedure pauses the node by hand)")

        # ── E: node restart (clean exit) + queue back-pressure ──────────────────────────
        rc = node_a.stop()
        check("E: node exits cleanly on SIGINT (code 0)", rc == 0, f"rc={rc}")
        check("E: the --dump file holds the frame_meta and frame_stats lines", dump_a.exists() and sum(1 for l in dump_a.read_text().splitlines() if '"/scanimage/frame_meta"' in l) == st["frames"],
              f"{dump_a}")
        prev_frames = st["frames"]
        node_b = start("node-b", [*NODE, "--address", address, "--stats-every", "0.5", "--preview-hz", "0",
                                  "--queue-depth", "8", "--debug-stall-at", "20", "--debug-stall-s", "1.0", "--dump", str(dump_b)])
        col.clear_images()
        check("E: restarted node up (fresh counters)", wait_for(lambda: (col.last_stats() or {}).get("frames") == 0 and (col.last_stats() or {}).get("uptime_s", 99) < 5, 20.0), json.dumps(col.last_stats()))
        time.sleep(0.5)
        n_e = 100
        m = run_mock("mock-e", n_e, start_delay=1.0)
        m.wait(30)
        st = wait_settled(col)
        check("E: a stalled publisher thread + queue 8 → dropped_by_queue counted, received == published + dropped exactly",
              st and st["frames"] == n_e and st["dropped_by_queue"] >= 1 and st["published"] + st["dropped_by_queue"] == st["frames"] and st["gaps"] == 0,
              json.dumps({k: st and st[k] for k in ("frames", "published", "dropped_by_queue", "gaps", "queue_limit")}))
        check("E: --preview-hz 0 publishes no preview", st["preview"]["published"] == 0 and col.n_images(1) == 0 and col.n_images(2) == 0, json.dumps(st["preview"]))
        note(f"E: {st['dropped_by_queue']} dropped by the queue (limit {st['queue_limit']}) during a 1 s stall at 29 Hz")

        # ── F: the latency report (dump + bag + TIFF) ───────────────────────────────────
        metas1 = [o for o in col.metas() if o["segment"] == 1][:n_a]
        tiff = work / "mock_a.tif"
        epoch_ms = metas1[0]["rx_epoch_ms"] - 20  # "acquisition started 20 ms before the first frame reached the node"
        e = dt.datetime.fromtimestamp(epoch_ms / 1000.0, dt.timezone.utc)
        write_si_tiff(tiff, [(o["frameNumber"], o["frameTimestamp_s"] - 0.005) for o in metas1], NCH, [e.year, e.month, e.day, e.hour, e.minute, e.second + e.microsecond / 1e6])
        rep_json = work / "report-a.json"
        r = subprocess.run([*TOOL, "--jsonl", str(dump_a), "--tiff", str(tiff), "--utc-offset-h", "0", "--json", str(rep_json)], capture_output=True, text=True, timeout=120)
        ok = r.returncode == 0 and rep_json.exists()
        check("F: zmq_latency_report runs on the node's --dump JSONL (+ TIFF)", ok, (r.stdout + r.stderr)[-600:])
        if ok:
            rep = json.loads(rep_json.read_text())
            s1 = next((s for s in rep["segments"] if s["id"] == 1), None)
            s2 = next((s for s in rep["segments"] if s["id"] == 2), None)
            check("F: report segment 1 = 87 frames, stream-delay/hop percentiles, queue histogram",
                  s1 and s1["frames"] == n_a and s1["stream_delay_rel_ms"]["n"] == n_a and s1["node_hop_ms"]["p99"] is not None and sum(s1["queue_depth"]["hist"].values()) == n_a, json.dumps(s1 and {k: s1[k] for k in ("frames", "stream_delay_rel_ms", "queue_depth")}))
            check("F: report segment 2 lists the 9 injected gaps", s2 and s2["gaps"] == 9 and len(s2["gap_list"]) == 9, json.dumps(s2 and s2["gap_list"]))
            t = s1 and s1.get("tiff")
            check("F: TIFF alignment: 87/87 frames matched, 2 pages per frame, ZMQ − TIFF timestamp = +5 ms",
                  t and t["matched_frames"] == n_a and t["pages_per_frame"] == NCH and t["frame_meta_count_equals_tiff_frames"] and abs(t["zmq_minus_tiff_ts_ms"]["p50"] - 5.0) < 0.01, json.dumps(t))
            check("F: epoch offset from the TIFF epoch is finite and plausible (20 ms + the stream delay)",
                  t and t.get("epoch_offset_ms", {}).get("n") == n_a and 15.0 <= t["epoch_offset_ms"]["min"] <= 2000.0, json.dumps(t and t.get("epoch_offset_ms")))
            check("F: the report's pass bars include the meta-latency and gap bars per segment", any("meta latency" in b["bar"] for b in rep["bars"]) and any(b["bar"].startswith("0 gaps") and b["segment"] == 2 and not b["pass"] for b in rep["bars"]), json.dumps(rep["bars"]))
            check("F: the TIFF's segment is chosen by frame count + epoch (segment 1 the only candidate; timestamps agree)",
                  t and [c["segment"] for c in t["segment_choice"] if c["candidate"]] == [1] and t["timestamps_agree"], json.dumps(t and t["segment_choice"]))
            print(r.stdout[-1500:], file=sys.stderr)
        # three acquisitions that each number frames from 1, with the TIFF = the LAST one (2026-10-20's wrong match)
        syn = work / "three_segments.jsonl"
        base = metas1[0]["rx_epoch_ms"]
        with open(syn, "w", encoding="utf-8") as fh:
            for sid in (1, 2, 3):
                for k in range(1, 21):
                    t_ms = sid * 10_000 + (k - 1) * 1000 // 29
                    fh.write(json.dumps({"topic": "/scanimage/frame_meta", "seq": sid * 100 + k, "segment": sid, "frameNumber": k,
                                         "frameTimestamp_s": (k - 1) / 29.0, "rx_epoch_ms": base + t_ms, "rx_mono_ns": t_ms * 1_000_000,
                                         "pub_mono_ns": t_ms * 1_000_000 + 100_000, "queue_depth": 0, "dims": [8, 8, 1], "nchan": 1}) + "\n")

        def syn_tiff(name: str, epoch_ms: int) -> Path:
            p = work / name
            e = dt.datetime.fromtimestamp(epoch_ms / 1000.0, dt.timezone.utc)
            write_si_tiff(p, [(k, (k - 1) / 29.0) for k in range(1, 21)], 1, [e.year, e.month, e.day, e.hour, e.minute, e.second + e.microsecond / 1e6])
            return p

        def syn_report(tif: Path, *extra: str):
            out = work / f"{tif.stem}{'-'.join(extra)}.json"
            rr = subprocess.run([*TOOL, "--jsonl", str(syn), "--tiff", str(tif), "--utc-offset-h", "0", "--json", str(out), *extra],
                                capture_output=True, text=True, timeout=60)
            seg_t = {s["id"]: s.get("tiff") for s in json.loads(out.read_text())["segments"]} if rr.returncode == 0 and out.exists() else {}
            return rr, seg_t

        rr, seg_t = syn_report(syn_tiff("syn_last.tif", base + 30_000 - 560))  # armed 0.56 s before segment 3's first frame
        check("F: three segments numbered from 1, TIFF = the last: the report picks segment 3, all 20 frames, timestamps agree",
              rr.returncode == 0 and seg_t.get(3) and not seg_t.get(1) and seg_t[3]["matched_frames"] == 20 and seg_t[3]["timestamps_agree"],
              (rr.stdout + rr.stderr)[-400:])
        rr, _ = syn_report(syn_tiff("syn_none.tif", base - 60_000))
        check("F: a TIFF no segment fits (epoch 60 s early) → exit 2, listing the segments, asking for --segment",
              rr.returncode == 2 and "--segment" in rr.stderr and "segment 3:" in rr.stderr, (rr.stdout + rr.stderr)[-400:])
        rr, seg_t = syn_report(work / "syn_none.tif", "--segment", "2")
        check("F: --segment overrides the choice (segment 2)", rr.returncode == 0 and seg_t.get(2) and seg_t[2]["matched_frames"] == 20,
              (rr.stdout + rr.stderr)[-400:])
        # the simulated vDAQ's TIFF epoch [1601 1 1 0 0 0] in the local-zone path (no --utc-offset-h): it crashed
        # the report on Windows (astimezone() before 1970, Bergamo PC 2026-10-22); now ignored with a warning
        sim_tif = work / "syn_sim1601.tif"
        write_si_tiff(sim_tif, [(k, (k - 1) / 29.0) for k in range(1, 21)], 1, [1601, 1, 1, 0, 0, 0])
        sim_out = work / "syn_sim1601.json"
        rr = subprocess.run([*TOOL, "--jsonl", str(syn), "--tiff", str(sim_tif), "--segment", "2", "--json", str(sim_out)],
                            capture_output=True, text=True, timeout=60)
        rep_s = json.loads(sim_out.read_text()) if rr.returncode == 0 and sim_out.exists() else {}
        t2 = next((s.get("tiff") for s in rep_s.get("segments", []) if s["id"] == 2), None)
        check("F: a 1601 TIFF epoch (simulated vDAQ) is ignored with a warning: no crash, no epoch offset, 20/20 frames matched",
              rr.returncode == 0 and t2 and t2["matched_frames"] == 20 and t2.get("epoch_usable") is False and "epoch_offset_ms" not in t2
              and any("not a usable wall time" in w for w in rep_s.get("warnings", [])) and "warning: TIFF epoch" in rr.stderr,
              (rr.stdout + rr.stderr)[-400:])
        rr = subprocess.run([*TOOL, "--jsonl", str(syn), "--tiff", str(sim_tif)], capture_output=True, text=True, timeout=60)
        check("F: with the epoch ignored, three equal segments are ambiguous → exit 2 asking for --segment (no traceback)",
              rr.returncode == 2 and "3 segments match" in rr.stderr and "Traceback" not in rr.stderr, (rr.stdout + rr.stderr)[-400:])
        # a late join: the node's first frame is 6 → missed_before_first 5, reported as a warning (never a bar)
        late = work / "late_join.jsonl"
        with open(late, "w", encoding="utf-8") as fh:
            for k in range(6, 21):
                t_ms = (k - 1) * 1000 // 29
                fh.write(json.dumps({"topic": "/scanimage/frame_meta", "seq": k - 5, "segment": 1, "frameNumber": k,
                                     "frameTimestamp_s": (k - 1) / 29.0, "rx_epoch_ms": base + t_ms, "rx_mono_ns": t_ms * 1_000_000,
                                     "pub_mono_ns": t_ms * 1_000_000 + 100_000, "queue_depth": 0, "dims": [8, 8, 1], "nchan": 1}) + "\n")
        late_out = work / "late_join.json"
        rr = subprocess.run([*TOOL, "--jsonl", str(late), "--json", str(late_out)], capture_output=True, text=True, timeout=60)
        rep_l = json.loads(late_out.read_text()) if rr.returncode == 0 and late_out.exists() else {}
        check("F: a late join (first frame 6) → missed_before_first 5, a warning in the JSON, the markdown and on stderr",
              bool(rep_l) and rep_l["segments"][0]["missed_before_first"] == 5
              and any("first 5 frame(s) never reached the node" in w for w in rep_l["warnings"])
              and "WARNING: segment 1" in rr.stdout and "warning: segment 1" in rr.stderr, (rr.stdout + rr.stderr)[-400:])
        # a bag the way the bridge records it (raw CDR, arrival stamps), then the report's bag path
        from rig_nodes.recorder import Recorder  # noqa: E402

        bag = work / "frames.bag"
        topics = {META_TOPIC: "std_msgs/msg/String", STATS_TOPIC: "std_msgs/msg/String", PREVIEW_META_TOPIC: "std_msgs/msg/String",
                  **{f"{PREVIEW_PREFIX}{c}": "sensor_msgs/msg/CompressedImage" for c in (1, 2, 3, 4)}}
        rec = Recorder(bag, topics, profile="default")
        rec.open()
        with col.lock:
            # node A's run only: node B's frame_meta seqs restart at 1 — stop at the first seq that goes back
            prev_seq = 0
            for t_ns, o, msg in col.meta:
                if o["seq"] <= prev_seq:
                    break
                prev_seq = o["seq"]
                rec.write_raw(META_TOPIC, serialize_message(msg), t_ns)
            for t_ns, o, msg in col.stats[:1] + [x for x in col.stats if x[1]["frames"] == prev_frames][-1:]:
                rec.write_raw(STATS_TOPIC, serialize_message(msg), t_ns)
            for t_ns, o, msg in col.pmeta:
                rec.write_raw(PREVIEW_META_TOPIC, serialize_message(msg), t_ns)
        for ch, lst in list(col.images.items()):
            for t_ns, msg in lst:
                rec.write_raw(f"{PREVIEW_PREFIX}{ch}", serialize_message(msg), t_ns)
        info = rec.close(reindex=False)
        rep_json2 = work / "report-bag.json"
        r = subprocess.run([*TOOL, "--bag", str(bag), "--json", str(rep_json2)], capture_output=True, text=True, timeout=120)
        ok = r.returncode == 0 and rep_json2.exists()
        check(f"F: zmq_latency_report runs on a bag ({info.get('records')} records)", ok, (r.stdout + r.stderr)[-600:])
        if ok:
            rep2 = json.loads(rep_json2.read_text())
            s1 = next((s for s in rep2["segments"] if s["id"] == 1), None)
            check("F: bag path decodes the String topics (segment 1 = 87 frames) and measures the node→record hop",
                  s1 and s1["frames"] == n_a and s1["node_to_bridge_ms"]["n"] == n_a and 0 <= s1["node_to_bridge_ms"]["p50"] < 5000, json.dumps(s1 and {k: s1[k] for k in ("frames", "node_to_bridge_ms")}))
            note(f"F: node→record hop p50 {s1 and s1['node_to_bridge_ms']['p50']} ms, p99 {s1 and s1['node_to_bridge_ms']['p99']} ms (the harness subscriber stands in for the bridge)")

        # The gateway's bag topic map and the converter's neutrality toward these streams are the arena
        # application's concern: webDisplayTools rig/tests/test_ros2_poc.py checks them (ex section G).
    finally:
        for pr in reversed(procs):
            pr.stop()
        for pr in procs:
            pr.log.close()
        try:
            if ex is not None:
                ex.shutdown(timeout_sec=1.0)
            if col is not None:
                col.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass

    n_ok = sum(1 for _, ok, _ in results if ok)
    print(f"\n{n_ok} / {len(results)} checks passed", file=sys.stderr)
    summary = ["| check | result |", "|---|---|"] + [f"| {n} | {'PASS' if ok else 'FAIL — ' + i.replace('|', '/')[:160]} |" for n, ok, i in results]
    (work / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary))
    if not a.keep and n_ok == len(results):
        shutil.rmtree(work, ignore_errors=True)
    else:
        print(f"[test] artifacts kept in {work}", file=sys.stderr)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
