#!/usr/bin/env python3
"""mcap_bench.py — rosbag2 (C++) recording an image topic to MCAP, rmw_zenoh shared memory off vs on (LAB-303).

The publisher is shm_bench_qpc.py's `pub` (rclpy, sensor_msgs/Image of a fixed size at a fixed
rate, incompressible payload). The only subscriber is a rosbag2 Recorder: a C++ generic
subscription that never deserializes, writing MCAP with the default storage profile. It is
driven through rosbag2_py the way `ros2 bag record` drives it, but stopped by a line on stdin
instead of SIGINT (Windows cannot deliver Ctrl+C to one child process).

    pixi run python mcap_bench.py run --out ww18_mcap.json
    pixi run python mcap_bench.py run --cells 2097152@135 --modes on --duration 5

Per cell it reports what was sent, what ended up in the bag (by the frame_id sequence numbers in
the recorded CDR), the recorder's CPU per message (including the flush at stop), how long the
stop flush took, the recorder's mapped shared memory, and the rmw publish -> recorder receive
latency from each MCAP record's publish_time and log_time (both the system clock; rmw_zenoh
stamps the source timestamp after serialization, so rclpy's conversion is not included).
Bags go to --bag-dir and are deleted after they are counted unless --keep-bags.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import platform
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile
import time

import shm_bench as sb

HERE = os.path.dirname(os.path.abspath(__file__))
PUB_SCRIPT = os.path.join(HERE, "shm_bench_qpc.py")


# ── the recorder process ──────────────────────────────────────────────────────────────────────


def cmd_rec(args):
    import datetime

    from ros2cli.node import NODE_NAME_PREFIX
    from rosbag2_py import Recorder, RecordOptions, StorageOptions

    storage = StorageOptions(
        uri=args.uri,
        storage_id="mcap",
        max_cache_size=args.cache_mb << 20,
        storage_preset_profile=args.preset,
    )
    ro = RecordOptions()
    ro.topics = [sb.TOPIC]
    ro.rmw_serialization_format = ""
    ro.topic_polling_interval = datetime.timedelta(milliseconds=100)
    ro.node_prefix = NODE_NAME_PREFIX
    ro.disable_keyboard_controls = True
    rec = Recorder(storage, ro, "info", "mcap_bench_recorder")
    rec.start_spin()
    rec.record()
    print("READY", flush=True)

    cpu0, wall0 = time.process_time(), time.monotonic()
    for line in sys.stdin:
        cmd = line.strip()
        if cmd == "mark":  # the publisher is being started
            cpu0, wall0 = time.process_time(), time.monotonic()
        elif cmd == "stop":
            break
    cpu1, wall1 = time.process_time(), time.monotonic()
    rec.stop()  # flushes the cache and closes the bag
    cpu2, wall2 = time.process_time(), time.monotonic()
    shm = sb._shm_mapped()
    rec.stop_spin()
    print(
        json.dumps(
            {
                "role": "rec",
                "cpu_s": cpu2 - cpu0,
                "wall_s": wall1 - wall0,
                "stop_s": wall2 - wall1,
                "shm_mapped": shm,
            }
        ),
        flush=True,
    )
    return 0


# ── reading the bag ───────────────────────────────────────────────────────────────────────────


def _read_bag(uri, size):
    """Count what the recorder wrote: measured ("m<seq>") messages by sequence number, plus the
    rmw publish -> recorder receive latency from MCAP publish_time / log_time."""
    from mcap.reader import make_reader

    files = sorted(glob.glob(os.path.join(uri, "*.mcap")))
    out = {"bag_files": len(files), "bag_bytes": sum(os.path.getsize(f) for f in files)}
    seqs, lat, n_all, short = [], [], 0, 0
    same_t = 0
    for f in files:
        with open(f, "rb") as fh:
            for _schema, _chan, msg in make_reader(fh).iter_messages(log_time_order=False):
                n_all += 1
                d = msg.data
                # CDR_LE: 4-byte encapsulation, stamp sec/nanosec, then frame_id length + chars + NUL
                (n,) = struct.unpack_from("<I", d, 12)
                fid = d[16 : 16 + n - 1].decode("ascii", "replace")
                if not fid.startswith("m"):
                    continue
                seqs.append(int(fid[1:]))
                if len(d) < size:
                    short += 1
                if msg.publish_time and msg.publish_time != msg.log_time:
                    lat.append(msg.log_time - msg.publish_time)
                else:
                    same_t += 1
    out.update({"bag_msgs": n_all, "bagged": len(set(seqs)), "short": short})
    if lat:
        out.update(
            {
                "lat_ms_p50": statistics.median(lat) / 1e6,
                "lat_ms_p95": sb._pct(lat, 95) / 1e6,
                "lat_ms_p99": sb._pct(lat, 99) / 1e6,
                "lat_ms_max": max(lat) / 1e6,
            }
        )
    out["no_publish_time"] = same_t
    return out


# ── the orchestrator ──────────────────────────────────────────────────────────────────────────


def cmd_run(args):
    me = os.path.abspath(__file__)
    base = dict(os.environ)
    base["ROS_DOMAIN_ID"] = str(args.domain)
    base["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    base.setdefault("RUST_LOG", "error")
    endpoint = f'["tcp/127.0.0.1:{args.port}"]'
    os.makedirs(args.bag_dir, exist_ok=True)

    router = None
    if not sb._port_open(args.port):
        renv = dict(base, ZENOH_CONFIG_OVERRIDE=f"listen/endpoints={endpoint}")
        router = subprocess.Popen(
            sb._router_argv(), env=renv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        t0 = time.monotonic()
        while not sb._port_open(args.port):
            if time.monotonic() - t0 > 20:
                sys.exit(f"router did not come up on :{args.port}")
            time.sleep(0.2)

    cells = []
    for c in args.cells.split(","):
        size, rate = c.split("@")
        cells.append((int(size), float(rate)))

    pub_args = [
        "--depth", str(args.depth),
        "--duration", str(args.duration),
        "--warmup", str(args.warmup),
    ] + (["--reliable"] if args.reliable else ["--no-reliable"])

    results = []
    try:
        for mode in args.modes.split(","):
            override = f"connect/endpoints={endpoint};transport/shared_memory/enabled={'true' if mode == 'on' else 'false'}"
            env = dict(base, ZENOH_CONFIG_OVERRIDE=override)
            for size, rate in cells:
                uri = os.path.join(args.bag_dir, f"{mode}_{size}_{rate:g}")
                shutil.rmtree(uri, ignore_errors=True)  # rosbag2 refuses an existing directory
                with open(uri + ".log", "w") as log:
                    rec = subprocess.Popen(
                        [sys.executable, me, "rec", "--uri", uri, "--cache-mb", str(args.cache_mb), "--preset", args.preset],
                        env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True,
                    )
                    for _ in range(200):
                        line = rec.stdout.readline()
                        if not line or line.strip() == "READY":
                            break
                        print(f"  [rec] {line.rstrip()}", file=sys.stderr)
                    if not line:
                        sys.exit(f"recorder did not start (see {uri}.log)")
                    rec.stdin.write("mark\n")
                    rec.stdin.flush()
                    pub = subprocess.Popen(
                        [sys.executable, PUB_SCRIPT, "pub", "--size", str(size), "--rate", str(rate)] + pub_args,
                        env=env, stdout=subprocess.PIPE, text=True,
                    )
                    pout, _ = pub.communicate(timeout=args.duration + 60)
                    time.sleep(args.drain)
                    rec.stdin.write("stop\n")
                    rec.stdin.flush()
                    rout, _ = rec.communicate(timeout=180)
                row = {"mode": mode, "size": size, "rate": rate}
                for line in (pout + rout).splitlines():
                    if line.startswith("{"):
                        d = json.loads(line)
                        role = d.pop("role")
                        row.update({f"{role}_{k}": v for k, v in d.items()})
                if os.path.isdir(uri):
                    row.update(_read_bag(uri, size))
                    if row.get("bag_msgs") and row.get("rec_cpu_s") is not None:
                        row["rec_cpu_ms_per_msg"] = 1000 * row["rec_cpu_s"] / row["bag_msgs"]
                if row.get("pub_sent") is not None and row.get("bagged") is not None:
                    row["lost"] = row["pub_sent"] - row["bagged"]
                with open(uri + ".log") as f:
                    row["rec_log"] = [
                        ln.rstrip() for ln in f if any(k in ln for k in ("lost", "WARN", "ERROR", "dropped"))
                    ]
                if not args.keep_bags:
                    shutil.rmtree(uri, ignore_errors=True)
                results.append(row)
                _print_row(row)
                for ln in row["rec_log"]:
                    print(f"    rec log: {ln}", flush=True)
    finally:
        if router:
            router.terminate()
            try:
                router.wait(5)
            except subprocess.TimeoutExpired:
                router.kill()

    meta = {
        "host": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "rmw": "rmw_zenoh_cpp",
        "recorder": "rosbag2_py Recorder, storage mcap",
        "preset": args.preset or "default",
        "cache_mb": args.cache_mb,
        "bag_dir": os.path.abspath(args.bag_dir),
        "reliable": args.reliable,
        "depth": args.depth,
        "duration_s": args.duration,
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if args.out:
        with open(args.out, "w") as f:
            json.dump({"meta": meta, "results": results}, f, indent=1)
        print(f"wrote {args.out}")
    return 0


_HDR_DONE = False


def _print_row(r):
    global _HDR_DONE
    if not _HDR_DONE:
        print(
            f"{'shm':>3} {'size':>8} {'Hz':>5} {'MB/s':>6} {'bagged/sent':>11} {'lost':>4} {'pub()':>6} "
            f"{'pubCPU':>6} {'recCPU':>6} {'stop s':>6} {'p50 ms':>7} {'p99 ms':>7} {'max ms':>7} {'shm map pub/rec':>16}",
            flush=True,
        )
        _HDR_DONE = True
    g = r.get
    nan = float("nan")
    rec_pct = g("rec_cpu_ms_per_msg", nan) * g("pub_rate_hz", nan) / 10  # ms/message x achieved Hz
    shm_s = f"{g('pub_shm_mapped') or 'n/a'}/{g('rec_shm_mapped') or 'n/a'}"
    mbps = r["size"] * g("pub_rate_hz", 0) / 1e6
    print(
        f"{r['mode']:>3} {sb._human(r['size']):>8} {r['rate']:>5g} {mbps:>6.1f} "
        f"{g('bagged', 0):>5}/{g('pub_sent', 0):<5} {g('lost', -1):>4} "
        f"{g('pub_publish_call_ms_p50', nan):>6.2f} {g('pub_cpu_pct', nan):>5.0f}% {rec_pct:>5.0f}% "
        f"{g('rec_stop_s', nan):>6.2f} {g('lat_ms_p50', nan):>7.2f} {g('lat_ms_p99', nan):>7.2f} "
        f"{g('lat_ms_max', nan):>7.2f} {shm_s:>16}",
        flush=True,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("rec")
    p.add_argument("--uri", required=True)
    p.add_argument("--cache-mb", type=int, default=100)
    p.add_argument("--preset", default="")
    p = sp.add_parser("run")
    p.add_argument(
        "--cells",
        default="2097152@135,8388608@30",
        help="size@rate list (default: BIAS-rate 2 MP frame, 8 MB at 30 Hz)",
    )
    p.add_argument("--modes", default="off,on")
    p.add_argument("--duration", type=float, default=8.0, help="measured seconds per cell")
    p.add_argument("--warmup", type=float, default=1.0)
    p.add_argument("--depth", type=int, default=10)
    p.add_argument("--reliable", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--drain", type=float, default=1.0, help="seconds between the publisher exiting and stop")
    p.add_argument("--cache-mb", type=int, default=100, help="rosbag2 max_cache_size (ros2 bag record default 100)")
    p.add_argument("--preset", default="", help="MCAP storage preset ('' = default, fastwrite, zstd_fast, ...)")
    p.add_argument("--bag-dir", default=os.path.join(tempfile.gettempdir(), "mcap_bench"))
    p.add_argument("--keep-bags", action="store_true")
    p.add_argument("--port", type=int, default=7457)
    p.add_argument("--domain", type=int, default=42)
    p.add_argument("--out")
    args = ap.parse_args()
    return {"rec": cmd_rec, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
