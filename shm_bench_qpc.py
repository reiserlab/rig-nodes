#!/usr/bin/env python3
"""shm_bench.py — rmw_zenoh shared memory on vs off, for image-sized ROS 2 messages (LAB-303).

One publisher process and one subscriber process exchange sensor_msgs/Image messages of a fixed
size at a fixed rate. Latency is publish-call → subscriber callback, both stamped with
time.perf_counter_ns() (one system-wide clock on macOS, Linux and Windows), so it includes rclpy's
Python→C conversion, CDR serialization, the transport, deserialization and callback dispatch.

    pixi run python shm_bench.py run                       # the default matrix, shm off then on
    pixi run python shm_bench.py run --sizes 1048576 --rates 30 --modes on --duration 5
    pixi run python shm_bench.py run --out results.json

`run` starts its own Zenoh router on --port (default 7457, not the rig's 7447) and a separate
ROS_DOMAIN_ID, so it never touches a running rig. Shared memory is switched with rmw_zenoh's
ZENOH_CONFIG_OVERRIDE (transport/shared_memory/enabled) on both sessions; the pool size can be
raised with --pool-mb (rmw_zenoh's ZENOH_SHM_ALLOC_SIZE, default 48 MiB). After its measured
window each process reports how much shared memory it has mapped (macOS: vmmap; Linux: /dev/shm;
Windows: committed MEM_MAPPED regions): the direct evidence that the Zenoh pool was used. With
shared memory on, it is about 48 MB higher than with it off.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import socket
import statistics
import subprocess
import sys
import time

TOPIC = "/bench/image"


# ── the two ROS processes ──────────────────────────────────────────────────────────────────────


def _qos(args):
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=args.depth,
        reliability=ReliabilityPolicy.RELIABLE if args.reliable else ReliabilityPolicy.BEST_EFFORT,
    )


def cmd_pub(args):
    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("shm_bench_pub")
    pub = node.create_publisher(Image, TOPIC, _qos(args))
    msg = Image()
    msg.height, msg.width, msg.step, msg.encoding = 1, args.size, args.size, "mono8"
    msg.data = os.urandom(args.size)  # incompressible, built once

    t_wait = time.monotonic()
    while pub.get_subscription_count() == 0:
        if time.monotonic() - t_wait > 15:
            print(json.dumps({"role": "pub", "error": "no subscriber after 15 s"}), flush=True)
            return 1
        rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)

    warm = max(1, int(args.rate * args.warmup))
    n = max(1, int(args.rate * args.duration))
    period = 1.0 / args.rate
    call_ns = []
    next_t = time.monotonic()
    cpu0 = wall0 = None
    for i in range(warm + n):
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        if i == warm:
            cpu0, wall0 = time.process_time(), time.monotonic()
        msg.header.frame_id = f"w{i}" if i < warm else f"m{i - warm}"
        t = time.perf_counter_ns()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t, 1_000_000_000)
        pub.publish(msg)
        if i >= warm:
            call_ns.append(time.perf_counter_ns() - t)
        next_t += period
    cpu1, wall1 = time.process_time(), time.monotonic()
    msg.header.frame_id = "end"
    for _ in range(3):
        pub.publish(msg)
        time.sleep(0.05)
    print(
        json.dumps(
            {
                "role": "pub",
                "sent": n,
                "rate_hz": n / (wall1 - wall0),
                "cpu_pct": 100 * (cpu1 - cpu0) / (wall1 - wall0),
                "publish_call_ms_p50": statistics.median(call_ns) / 1e6,
                "publish_call_ms_p99": _pct(call_ns, 99) / 1e6,
                "shm_mapped": _shm_mapped(),
            }
        ),
        flush=True,
    )
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def cmd_sub(args):
    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("shm_bench_sub")
    st = {"lat": [], "seqs": [], "bad": 0, "cpu0": None, "wall0": None, "done": False}

    def cb(msg):
        now = time.perf_counter_ns()
        fid = msg.header.frame_id
        if fid == "end":
            st["done"] = True
            return
        if fid[0] != "m":
            return
        if st["cpu0"] is None:
            st["cpu0"], st["wall0"] = time.process_time(), time.monotonic()
        if len(msg.data) != args.size:
            st["bad"] += 1
        st["lat"].append(now - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec))
        st["seqs"].append(int(fid[1:]))

    node.create_subscription(Image, TOPIC, cb, _qos(args))
    print("READY", flush=True)
    deadline = time.monotonic() + args.warmup + args.duration + 30
    while not st["done"] and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    cpu1, wall1 = time.process_time(), time.monotonic()
    lat = st["lat"]
    out = {"role": "sub", "received": len(lat), "bad_size": st["bad"], "timed_out": not st["done"]}
    if lat:
        seqs = st["seqs"]
        out.update(
            {
                "lost": (max(seqs) + 1) - len(set(seqs)),
                "lat_ms_p50": statistics.median(lat) / 1e6,
                "lat_ms_p95": _pct(lat, 95) / 1e6,
                "lat_ms_p99": _pct(lat, 99) / 1e6,
                "lat_ms_max": max(lat) / 1e6,
                "cpu_pct": 100 * (cpu1 - st["cpu0"]) / max(1e-9, wall1 - st["wall0"]),
            }
        )
    out["shm_mapped"] = _shm_mapped()
    print(json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _shm_mapped():
    """Shared memory this process has mapped — the direct evidence that the Zenoh pool is in use.

    Called only AFTER the measured window: macOS `vmmap` suspends the process it inspects.
    macOS: vmmap's "shared memory" virtual size (about 48 kB without the pool, 49 MB with the
    default 48 MiB pool). Linux: the /dev/shm mappings. Windows: committed MEM_MAPPED regions.
    Anything else: None (not measured)."""
    try:
        if sys.platform == "darwin":
            out = subprocess.run(
                ["vmmap", "-summary", str(os.getpid())], capture_output=True, text=True, timeout=20
            ).stdout
            for ln in out.splitlines():
                if ln.startswith("shared memory"):
                    return ln.split()[2]
        elif sys.platform.startswith("linux"):
            total = 0
            with open("/proc/self/maps") as f:
                for ln in f:
                    if "/dev/shm/" in ln:
                        a, b = ln.split()[0].split("-")
                        total += int(b, 16) - int(a, 16)
            return f"{total / (1 << 20):.1f}M"
        elif sys.platform == "win32":
            # Committed MEM_MAPPED regions (section views: the Zenoh pool, plus a few MB of other
            # mapped files). Compare off vs on: the pool shows up as a ~48 MB step.
            import ctypes
            from ctypes import wintypes

            class MBI(ctypes.Structure):
                _fields_ = [
                    ("BaseAddress", ctypes.c_void_p),
                    ("AllocationBase", ctypes.c_void_p),
                    ("AllocationProtect", wintypes.DWORD),
                    ("PartitionId", wintypes.WORD),
                    ("RegionSize", ctypes.c_size_t),
                    ("State", wintypes.DWORD),
                    ("Protect", wintypes.DWORD),
                    ("Type", wintypes.DWORD),
                ]

            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.VirtualQuery.argtypes = [ctypes.c_void_p, ctypes.POINTER(MBI), ctypes.c_size_t]
            k32.VirtualQuery.restype = ctypes.c_size_t
            mbi, addr, total = MBI(), 0, 0
            while addr < (1 << 47) and k32.VirtualQuery(addr, ctypes.byref(mbi), ctypes.sizeof(mbi)):
                if mbi.State == 0x1000 and mbi.Type == 0x40000:  # MEM_COMMIT, MEM_MAPPED
                    total += mbi.RegionSize
                addr = (mbi.BaseAddress or 0) + mbi.RegionSize
            return f"{total / (1 << 20):.1f}M"
    except Exception:
        pass
    return None


def _pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


# ── the orchestrator ──────────────────────────────────────────────────────────────────────────


def _port_open(port):
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


def _router_argv():
    """The rmw_zenohd binary itself, so terminate() stops the router and not just a wrapper
    (`ros2` is a launcher script on Windows). Falls back to `ros2 run`."""
    try:
        from ament_index_python.packages import get_package_prefix

        libdir = os.path.join(get_package_prefix("rmw_zenoh_cpp"), "lib", "rmw_zenoh_cpp")
        for name in ("rmw_zenohd.exe", "rmw_zenohd"):
            exe = os.path.join(libdir, name)
            if os.path.isfile(exe):
                return [exe]
    except Exception:
        pass
    return [shutil.which("ros2") or "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]


def _human(n):
    for unit, k in (("MB", 1 << 20), ("kB", 1 << 10)):
        if n >= k:
            v = n / k
            return f"{v:g} {unit}"
    return f"{n} B"


def cmd_run(args):
    me = os.path.abspath(__file__)
    base = dict(os.environ)
    base["ROS_DOMAIN_ID"] = str(args.domain)
    base["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    base.setdefault("RUST_LOG", "error")  # silence zenoh_config deprecation warnings
    endpoint = f'["tcp/127.0.0.1:{args.port}"]'

    router = None
    if not _port_open(args.port):
        renv = dict(base, ZENOH_CONFIG_OVERRIDE=f"listen/endpoints={endpoint}")
        router = subprocess.Popen(
            _router_argv(), env=renv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        t0 = time.monotonic()
        while not _port_open(args.port):
            if time.monotonic() - t0 > 20:
                sys.exit(f"router did not come up on :{args.port}")
            time.sleep(0.2)

    sizes = [int(s) for s in args.sizes.split(",")]
    rates = [float(r) for r in args.rates.split(",")]
    if len(rates) == 1:
        rates *= len(sizes)
    if len(rates) != len(sizes):
        sys.exit("--rates must be one value or one per size")

    common = [
        "--depth", str(args.depth),
        "--duration", str(args.duration),
        "--warmup", str(args.warmup),
    ] + (["--reliable"] if args.reliable else [])

    results = []
    try:
        for mode in args.modes.split(","):
            override = f"connect/endpoints={endpoint};transport/shared_memory/enabled={'true' if mode == 'on' else 'false'}"
            env = dict(base, ZENOH_CONFIG_OVERRIDE=override)
            if args.pool_mb:
                env["ZENOH_SHM_ALLOC_SIZE"] = str(int(args.pool_mb * (1 << 20)))
            for size, rate in zip(sizes, rates):
                argv = common + ["--size", str(size), "--rate", str(rate)]
                sub = subprocess.Popen(
                    [sys.executable, me, "sub"] + argv, env=env, stdout=subprocess.PIPE, text=True
                )
                for _ in range(200):  # rmw/zenoh may log to stdout before the marker
                    line = sub.stdout.readline()
                    if not line or line.strip() == "READY":
                        break
                    print(f"  [sub] {line.rstrip()}", file=sys.stderr)
                if not line:
                    sys.exit("subscriber did not start")
                pub = subprocess.Popen(
                    [sys.executable, me, "pub"] + argv, env=env, stdout=subprocess.PIPE, text=True
                )
                pout, _ = pub.communicate(timeout=args.duration + 60)
                sout, _ = sub.communicate(timeout=60)
                row = {"mode": mode, "size": size, "rate": rate}
                for line in (pout + sout).splitlines():
                    if line.startswith("{"):
                        d = json.loads(line)
                        role = d.pop("role")
                        row.update({f"{role}_{k}": v for k, v in d.items()})
                results.append(row)
                _print_row(row)
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
        "machine": platform.machine(),
        "python": platform.python_version(),
        "rmw": "rmw_zenoh_cpp",
        "reliable": args.reliable,
        "depth": args.depth,
        "duration_s": args.duration,
        "pool_mb": args.pool_mb or 48,
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
            f"{'shm':>3} {'size':>8} {'Hz':>5} {'MB/s':>6} {'recv/sent':>11} {'p50 ms':>7} {'p99 ms':>7} "
            f"{'max ms':>7} {'pub()':>6} {'pubCPU':>6} {'subCPU':>6} {'shm map pub/sub':>16}",
            flush=True,
        )
        _HDR_DONE = True
    g = r.get
    shm_s = f"{g('pub_shm_mapped') or 'n/a'}/{g('sub_shm_mapped') or 'n/a'}"
    mbps = r["size"] * g("pub_rate_hz", 0) / 1e6
    print(
        f"{r['mode']:>3} {_human(r['size']):>8} {r['rate']:>5g} {mbps:>6.1f} "
        f"{g('sub_received', 0):>5}/{g('pub_sent', 0):<5} "
        f"{g('sub_lat_ms_p50', float('nan')):>7.2f} {g('sub_lat_ms_p99', float('nan')):>7.2f} "
        f"{g('sub_lat_ms_max', float('nan')):>7.2f} {g('pub_publish_call_ms_p50', float('nan')):>6.2f} "
        f"{g('pub_cpu_pct', float('nan')):>5.0f}% {g('sub_cpu_pct', float('nan')):>5.0f}% {shm_s:>16}",
        flush=True,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    for name in ("pub", "sub", "run"):
        p = sp.add_parser(name)
        p.add_argument("--depth", type=int, default=10)
        p.add_argument("--reliable", action=argparse.BooleanOptionalAction, default=True)
        p.add_argument("--duration", type=float, default=8.0, help="measured seconds per cell")
        p.add_argument("--warmup", type=float, default=1.0)
        if name == "run":
            p.add_argument(
                "--sizes",
                default="1024,65536,1048576,2097152,8388608,2097152",
                help="bytes per message (default: FicTrac-ish, preview, 2P frame, 2 MP frame, 8 MB, BIAS-rate 2 MP)",
            )
            p.add_argument("--rates", default="100,100,30,30,30,135", help="Hz, one value or one per size")
            p.add_argument("--modes", default="off,on")
            p.add_argument("--pool-mb", type=float, default=0, help="shared-memory pool (0 = rmw_zenoh default 48)")
            p.add_argument("--port", type=int, default=7457)
            p.add_argument("--domain", type=int, default=42)
            p.add_argument("--out")
        else:
            p.add_argument("--size", type=int, required=True)
            p.add_argument("--rate", type=float, required=True)
    args = ap.parse_args()
    return {"pub": cmd_pub, "sub": cmd_sub, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
