#!/usr/bin/env python3
"""zmq_frames_mock.py — a stand-in for ScanImage's ZeroMQ frame publisher, for the off-rig harness.

The frame FORMAT is reiserlab/scanimage-2026-bergamo `tests/zmq-frames/mock_zmq_publisher.py`
(Hannah Marie Santos): a 2-part message per frame — a 40-byte header of five little-endian doubles
[pixelsPerLine, linesPerFrame, nChannels, frameTimestamp_s since acquisition start,
totalAcquiredFrames] and int16 pixels in C order of [channel, y, x] (x fastest, then y, then
channel) — and the same CLI, so `tests/test_scanimage_frames.py -- --mock <that file>` runs the
harness against the original. Two additions for the harness only: `--sndhwm` (the publisher's
high-water mark in frames; libzmq's default 1000 ≈ 1 GB at 512×512×2 ch is what a paused subscriber
makes ScanImage buffer — Block Z's estimate) and `--seed`. Like the original, this models the
format only, nothing about ScanImage's timing or memory behaviour.

    python -m rig_nodes.sims.zmq_frames_mock --frames 120 --rate 29.08 --ppl 512 --lpf 512 --nch 2
    python -m rig_nodes.sims.zmq_frames_mock --frames 300 --drop-every 50      # simulate dropped frames
    python -m rig_nodes.sims.zmq_frames_mock --frames 150 --sndhwm 20          # a publisher that drops when the subscriber stalls
"""
import argparse
import struct
import time

import numpy as np
import zmq


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="tcp://127.0.0.1:65179")
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--rate", type=float, default=29.08)
    ap.add_argument("--ppl", type=int, default=128)
    ap.add_argument("--lpf", type=int, default=96)
    ap.add_argument("--nch", type=int, default=2)
    ap.add_argument("--drop-every", type=int, default=0, help="skip publishing every Nth frame")
    ap.add_argument("--tiff", help="also write the frames to this TIFF (needs tifffile)")
    ap.add_argument("--start-delay", type=float, default=1.0, help="seconds between bind and the first frame (0 = provoke the slow-joiner loss)")
    ap.add_argument("--sndhwm", type=int, default=1000, help="publisher high-water mark in messages (libzmq default 1000)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    ctx = zmq.Context()
    s = ctx.socket(zmq.PUB)
    s.setsockopt(zmq.SNDHWM, a.sndhwm)
    s.bind(a.address)
    time.sleep(a.start_delay)  # let subscribers connect (ZeroMQ "slow joiner")
    rng = np.random.default_rng(a.seed)
    writer = None
    if a.tiff:
        import tifffile

        writer = tifffile.TiffWriter(a.tiff)
    t0 = time.perf_counter()
    for k in range(1, a.frames + 1):
        img = rng.integers(-200, 2000, size=(a.nch, a.lpf, a.ppl), dtype=np.int16)  # [c, y, x]
        if writer:
            for c in range(a.nch):
                writer.write(img[c], description=f"frameNumbers = {k}\nframeTimestamps_sec = {(k - 1) / a.rate:.6f}")
        if not (a.drop_every and k % a.drop_every == 0):
            hdr = struct.pack("<5d", a.ppl, a.lpf, a.nch, (k - 1) / a.rate, k)
            s.send_multipart([hdr, img.tobytes()])  # C order of [c,y,x] == x fastest, then y, then c
        nxt = t0 + k / a.rate
        time.sleep(max(0, nxt - time.perf_counter()))
    if writer:
        writer.close()
    time.sleep(0.5)
    s.close(linger=1000)
    ctx.term()
    print(f"published {a.frames} frames")


if __name__ == "__main__":
    main()
