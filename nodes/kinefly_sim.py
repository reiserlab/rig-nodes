#!/usr/bin/env python3
"""kinefly_sim.py — a synthetic KF/1 producer (stands in for Kinefly, like fictrac_sim.py for FicTrac).

Emits one KF/1 datagram per frame at --rate Hz to --host:--port, byte-compatible with
mbreiser/Kinefly-MacOS `kinefly/core/udp.py` (`protocol: "Kinefly"`, `version: 1`, a fresh
session UUID, `sequence` from 1, `source`, `timestamp_s`, `host_monotonic_s`, five `parts`).
The wings are tracked (`edge` mode, one angle each); head/abdomen/aux are disabled with the
empty-array/null shape the real encoder uses for disabled parts.

Kinematics: a steering fly. Left and right wing-stroke amplitudes sit at `--mean-deg` and
the DIFFERENCE L−R follows a slow sine of `--diff-deg` peak at `--diff-hz` (plus a little
noise), so a downstream L−R → yaw integrator has something smooth and known to chase.
--seed makes the bytes reproducible.

    pixi run kinefly-sim -- --rate 100 --diff-deg 10 --diff-hz 0.1 --seed 1
"""

from __future__ import annotations

import argparse
import json
import math
import random
import socket
import sys
import time
import uuid

DEFAULT_PORT = 60001
PARTS = ("head", "abdomen", "left", "right", "aux")


def disabled_part(mode: str | None) -> dict:
    return dict(enabled=False, valid=False, mode=mode, angles_rad=[], radii_px=[], gradients=[],
                intensity=None, frequency_hz=None, frequency_valid=False)


def wing_part(angle_rad: float, gradient: float) -> dict:
    return dict(enabled=True, valid=True, mode="edge", angles_rad=[angle_rad], radii_px=[],
                gradients=[gradient], intensity=None, frequency_hz=None, frequency_valid=False)


def kf1_packet(session: str, seq: int, t_s: float, left_rad: float, right_rad: float, fps: float, grad: float) -> dict:
    return dict(
        protocol="Kinefly", version=1, session=session, sequence=seq,
        source=dict(kind="synthetic", id="kinefly_sim.py", clock="host_monotonic_after_capture",
                    fps=fps, timestamp_units="s"),
        timestamp_s=t_s, host_monotonic_s=time.monotonic(),
        parts=dict(head=disabled_part("area"), abdomen=disabled_part("area"),
                   left=wing_part(left_rad, grad), right=wing_part(right_rad, grad),
                   aux=disabled_part("intensity")),
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--rate", type=float, default=100.0, help="frames per second (default: 100)")
    p.add_argument("--count", type=int, default=0, help="emit N frames then exit (0 = forever)")
    p.add_argument("--mean-deg", type=float, default=70.0, help="mean wing-stroke angle, both wings (default: 70)")
    p.add_argument("--diff-deg", type=float, default=10.0, help="peak L−R difference (default: 10)")
    p.add_argument("--diff-hz", type=float, default=0.1, help="frequency of the L−R sine (default: 0.1)")
    p.add_argument("--noise-deg", type=float, default=0.5, help="per-frame Gaussian noise per wing (default: 0.5)")
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args(argv)

    rng = random.Random(args.seed)
    session = str(uuid.UUID(int=rng.getrandbits(128))) if args.seed is not None else str(uuid.uuid4())
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)
    period = 1.0 / args.rate
    t0 = time.perf_counter()
    seq = 0
    print(f"[kinefly-sim] KF/1 → udp://{dst[0]}:{dst[1]} at {args.rate:g} Hz, session {session}", file=sys.stderr)
    try:
        while args.count == 0 or seq < args.count:
            seq += 1
            t_s = (seq - 1) * period
            half = 0.5 * args.diff_deg * math.sin(2 * math.pi * args.diff_hz * t_s)
            left = args.mean_deg + half + rng.gauss(0, args.noise_deg)
            right = args.mean_deg - half + rng.gauss(0, args.noise_deg)
            pkt = kf1_packet(session, seq, t_s, math.radians(left), math.radians(right), args.rate, 0.8)
            sock.sendto(json.dumps(pkt, allow_nan=False, separators=(",", ":")).encode("utf-8"), dst)
            target = t0 + seq * period
            delay = target - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        pass
    print(f"[kinefly-sim] sent {seq} frames", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
