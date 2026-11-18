#!/usr/bin/env python3
"""ez_bench.py — ezmsg on the cells of shm_bench_qpc.py, for comparison with rmw_zenoh (docs/ezmsg-bench.md).

One publisher unit and one subscriber unit exchange a fixed-size message at a fixed rate.
Latency is stamp-before-yield → subscriber coroutine entry, both time.perf_counter_ns(), so it
includes pickling, the transport, unpickling and asyncio dispatch: the span shm_bench_qpc.py
measures for rclpy (publish call → subscriber callback).

modes:    proc  = publisher and subscriber in separate processes (ezmsg's shared-memory channel)
          tcp   = separate processes, shared memory off (EZMSG_FORCE_TCP)
          local = both units in one process (ezmsg hands the object over by reference)
payloads: bytes   = a `bytes` field, pickled in-band (what shm_bench's msg.data is)
          ndarray = a numpy uint8 array, pickle protocol 5 out-of-band buffer

ezmsg is NOT a dependency of this workspace (it is not on conda-forge and nothing here uses it).
Run the bench from a throwaway environment:

    uv run --no-project --python 3.12 --with ezmsg==3.9.0 --with numpy bench/ez_bench.py run --modes proc,tcp,local --out ez.json

Each cell is its own `cell` subprocess; ezmsg starts its graph server on 127.0.0.1:25978 for the
cell and stops it at the end. Like the other benches it loads the CPU for several minutes: never
run it while an experiment is running on the PC.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib.metadata
import json
import os
import platform
import statistics
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from typing import Any

import ezmsg.core as ez


def _pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


@dataclass
class Msg:
    seq: int  # -1 = warmup, -2 = end
    t_ns: int
    data: Any


class PubSettings(ez.Settings):
    size: int
    rate: float
    duration: float
    warmup: float
    payload: str
    out: str

class SubSettings(ez.Settings):
    size: int
    out: str

class Pub(ez.Unit):
    SETTINGS = PubSettings
    OUT = ez.OutputStream(Msg)

    @ez.publisher(OUT)
    async def run(self):
        s = self.SETTINGS
        raw = os.urandom(s.size)
        if s.payload == "ndarray":
            import numpy as np

            data = np.frombuffer(raw, dtype=np.uint8)
        else:
            data = raw
        await asyncio.sleep(1.0)  # let the graph connect
        warm = max(1, int(s.rate * s.warmup))
        n = max(1, int(s.rate * s.duration))
        period = 1.0 / s.rate
        call_ns = []
        next_t = time.monotonic()
        cpu0 = wall0 = None
        for i in range(warm + n):
            dt = next_t - time.monotonic()
            if dt > 0:
                await asyncio.sleep(dt)
            if i == warm:
                cpu0, wall0 = time.process_time(), time.monotonic()
            t = time.perf_counter_ns()
            yield self.OUT, Msg(-1 if i < warm else i - warm, t, data)
            if i >= warm:
                call_ns.append(time.perf_counter_ns() - t)
            next_t += period
        cpu1, wall1 = time.process_time(), time.monotonic()
        with open(s.out, "w") as f:
            json.dump(
                {
                    "sent": n,
                    "rate_hz": n / (wall1 - wall0),
                    "cpu_pct": 100 * (cpu1 - cpu0) / (wall1 - wall0),
                    "publish_call_ms_p50": statistics.median(call_ns) / 1e6,
                    "publish_call_ms_p99": _pct(call_ns, 99) / 1e6,
                    "pid": os.getpid(),
                },
                f,
            )
        yield self.OUT, Msg(-2, 0, b"")

class Sub(ez.Unit):
    SETTINGS = SubSettings
    IN = ez.InputStream(Msg)

    async def initialize(self):
        self.lat, self.seqs, self.bad = [], [], 0
        self.cpu0 = self.wall0 = None

    @ez.subscriber(IN)
    async def on(self, msg: Msg):
        now = time.perf_counter_ns()
        if msg.seq == -2:
            cpu1, wall1 = time.process_time(), time.monotonic()
            lat, seqs = self.lat, self.seqs
            out = {"received": len(lat), "bad_size": self.bad, "pid": os.getpid()}
            if lat:
                out.update(
                    {
                        "lost": (max(seqs) + 1) - len(set(seqs)),
                        "lat_ms_p50": statistics.median(lat) / 1e6,
                        "lat_ms_p95": _pct(lat, 95) / 1e6,
                        "lat_ms_p99": _pct(lat, 99) / 1e6,
                        "lat_ms_max": max(lat) / 1e6,
                        "cpu_pct": 100 * (cpu1 - self.cpu0) / max(1e-9, wall1 - self.wall0),
                    }
                )
            with open(self.SETTINGS.out, "w") as f:
                json.dump(out, f)
            raise ez.NormalTermination
        if msg.seq < 0:
            return
        if self.cpu0 is None:
            self.cpu0, self.wall0 = time.process_time(), time.monotonic()
        if len(msg.data) != self.SETTINGS.size:
            self.bad += 1
        self.lat.append(now - msg.t_ns)
        self.seqs.append(msg.seq)


def cmd_cell(args):
    pub = Pub(PubSettings(args.size, args.rate, args.duration, args.warmup, args.payload, args.pub_out))
    sub = Sub(SubSettings(args.size, args.sub_out))
    ez.run(
        PUB=pub,
        SUB=sub,
        connections=((pub.OUT, sub.IN),),
        process_components=() if args.mode == "local" else (pub, sub),
    )
    return 0


def _human(n):
    for unit, k in (("MB", 1 << 20), ("kB", 1 << 10)):
        if n >= k:
            return f"{n / k:g} {unit}"
    return f"{n} B"


def cmd_run(args):
    me = os.path.abspath(__file__)
    sizes = [int(s) for s in args.sizes.split(",")]
    rates = [float(r) for r in args.rates.split(",")]
    if len(rates) == 1:
        rates *= len(sizes)
    results = []
    print(
        f"{'mode':>5} {'payload':>7} {'size':>8} {'Hz':>5} {'got Hz':>6} {'recv/sent':>11} {'p50 ms':>7} "
        f"{'p99 ms':>7} {'max ms':>7} {'pub()':>6} {'pubCPU':>6} {'subCPU':>6}",
        flush=True,
    )
    tmp = tempfile.mkdtemp(prefix="ezbench_")
    for mode in args.modes.split(","):
        for payload in args.payloads.split(","):
            for size, rate in zip(sizes, rates):
                po, so = os.path.join(tmp, "pub.json"), os.path.join(tmp, "sub.json")
                for p in (po, so):
                    if os.path.exists(p):
                        os.remove(p)
                env = dict(os.environ)
                if mode == "tcp":
                    env["EZMSG_FORCE_TCP"] = "1"
                row = {"mode": mode, "payload": payload, "size": size, "rate": rate}
                try:
                    subprocess.run(
                        [sys.executable, me, "cell", "--mode", mode, "--payload", payload,
                         "--size", str(size), "--rate", str(rate), "--duration", str(args.duration),
                         "--warmup", str(args.warmup), "--pub-out", po, "--sub-out", so],
                        env=env, timeout=args.duration + args.warmup + 90,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    )
                except subprocess.TimeoutExpired:
                    row["timeout"] = True
                for role, p in (("pub", po), ("sub", so)):
                    if os.path.exists(p):
                        with open(p) as f:
                            row.update({f"{role}_{k}": v for k, v in json.load(f).items()})
                results.append(row)
                g = row.get
                nan = float("nan")
                print(
                    f"{mode:>5} {payload:>7} {_human(size):>8} {rate:>5g} {g('pub_rate_hz', nan):>6.1f} "
                    f"{g('sub_received', 0):>5}/{g('pub_sent', 0):<5} {g('sub_lat_ms_p50', nan):>7.2f} "
                    f"{g('sub_lat_ms_p99', nan):>7.2f} {g('sub_lat_ms_max', nan):>7.2f} "
                    f"{g('pub_publish_call_ms_p50', nan):>6.2f} {g('pub_cpu_pct', nan):>5.0f}% "
                    f"{g('sub_cpu_pct', nan):>5.0f}%" + ("  TIMEOUT" if g("timeout") else ""),
                    flush=True,
                )
    meta = {
        "host": platform.node(), "platform": platform.platform(), "python": platform.python_version(),
        "ezmsg": importlib.metadata.version("ezmsg"), "duration_s": args.duration,
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if args.out:
        with open(args.out, "w") as f:
            json.dump({"meta": meta, "results": results}, f, indent=1)
        print(f"wrote {args.out}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    r = sp.add_parser("run")
    r.add_argument("--sizes", default="1024,65536,1048576,2097152,8388608,2097152")
    r.add_argument("--rates", default="100,100,30,30,30,135")
    r.add_argument("--modes", default="proc,local")
    r.add_argument("--payloads", default="bytes,ndarray")
    r.add_argument("--out")
    c = sp.add_parser("cell")
    c.add_argument("--mode", required=True)
    c.add_argument("--payload", required=True)
    c.add_argument("--size", type=int, required=True)
    c.add_argument("--rate", type=float, required=True)
    c.add_argument("--pub-out", required=True)
    c.add_argument("--sub-out", required=True)
    for p in (r, c):
        p.add_argument("--duration", type=float, default=8.0)
        p.add_argument("--warmup", type=float, default=1.0)
    args = ap.parse_args()
    return {"run": cmd_run, "cell": cmd_cell}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
