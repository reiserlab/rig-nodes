#!/usr/bin/env python3
"""host_monitor.py — host "pressure" of the rig PC on /rig/host, one sample per window (default 1 s).

    pixi run rig                                  # starts it (opt out: --no-host-monitor)
    pixi run python nodes/host_monitor.py         # alone, beside a running graph (from rig/)
    pixi run python nodes/host_sampler.py 5       # five samples on the console, no ROS

What a sample holds (rig_interfaces/HostStats; the request with every field:
rig/docs/host-stats-request.md): per-core CPU as the window mean AND the busiest of 10 sub-windows
(cycle-exact on Windows, the atlas-bench cpu_sampler.py method), P-/E-core class, how late the
sampler woke (a PC-wide stall), context switches / interrupts / DPCs per s, memory and page file,
hard page faults, free space on the log volume, every physical disk, every network adapter that is
up, every NVIDIA GPU (incl. NVENC/NVDEC), and the processes that matter: the top 3 by CPU, by
memory and by disk I/O, plus one summed row per pinned group (FicTrac, BIAS, MATLAB, the browser,
Foxglove, the router, each graph node as py:<script>). `monitor_cpu_cores` is this node's own cost.

Message type: rig_interfaces/HostStats when the installed rig_interfaces has it (rig-nodes ≥ 0.3.0,
the `typed` environment), else std_msgs/String JSON with the same field names (NaN → null, floats
to 3 decimals) — `--typed auto|on|off`, the same pattern as the FicTrac adapter. The arena gateway
records /rig/host into each run's MCAP bag. Generic on purpose: this node and host_sampler.py are
rig-nodes material (`rig_nodes.host_monitor`, since 0.3.0), prototyped in webDisplayTools on ww18. Design, budget, measured
cost: docs/development/rig-host-monitor.md.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import signal
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from rig_nodes.host_sampler import DEFAULT_PINS, HostSampler, summary_line, to_json_safe

TOPIC = "/rig/host"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                 durability=QoSDurabilityPolicy.VOLATILE)
HOST_STATS_TYPE = "rig_interfaces/msg/HostStats"
NESTED = {"disks": "DiskStats", "nets": "NetStats", "gpus": "GpuStats", "procs": "ProcStats"}


def load_typed():
    """The rig_interfaces host messages, or None when this build has no HostStats (rig-nodes < 0.3.0)."""
    try:
        from rig_interfaces import msg as m

        return {"HostStats": m.HostStats, **{v: getattr(m, v) for v in NESTED.values()}}
    except (ImportError, AttributeError):
        return None


def to_msg(sample: dict, types: dict):
    """HostStats fields (host_sampler dict) → the typed message; one builder for every field."""
    m = types["HostStats"]()
    for k, v in sample.items():
        if k in NESTED:
            cls = types[NESTED[k]]
            items = []
            for d in v:
                it = cls()
                for kk, vv in d.items():
                    setattr(it, kk, vv)
                items.append(it)
            setattr(m, k, items)
        elif k == "core_class":
            m.core_class = bytes(int(c) & 0xFF for c in v)
        else:
            setattr(m, k, v)
    return m


def to_json(sample: dict) -> str:
    def r(o):
        if isinstance(o, float):
            return round(o, 3) if math.isfinite(o) else None
        if isinstance(o, dict):
            return {k: r(v) for k, v in o.items()}
        if isinstance(o, list):
            return [r(v) for v in o]
        return o

    return json.dumps(r(to_json_safe(sample)), separators=(",", ":"))


def parse_pins(items: list[str]) -> list[tuple[str, str]]:
    out = []
    for it in items:
        name, sep, rx = it.partition("=")
        if not sep or not name or not rx:
            raise SystemExit(f"--pin expects NAME=REGEX, got {it!r}")
        re.compile(rx)
        out.append((name, rx))
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--period", type=float, default=1.0, help="window length in s = publish period (default: 1)")
    p.add_argument("--sub", type=int, default=10, help="CPU sub-windows per window for the peak (default: 10 → 100 ms at 1 s)")
    p.add_argument("--top", type=int, default=3, help="processes per top list (CPU, memory, disk I/O; default: 3)")
    p.add_argument("--pin", action="append", default=[], help="extra pinned group NAME=REGEX on the executable name (repeatable; checked first)")
    p.add_argument("--log-dir", default="runlogs", help="where run logs / bags are written: its volume's free space is reported (default: ./runlogs; the arena rig passes its own)")
    p.add_argument("--py-root", action="append", default=[], help="a directory whose Python scripts are pinned as py:<script> (repeatable; `-m rig_nodes.<module>` processes are grouped without it; the arena rig passes <webDisplayTools>/rig)")
    p.add_argument("--no-gpu", action="store_true", help="skip NVIDIA GPU sampling (NVML)")
    p.add_argument("--typed", choices=("auto", "on", "off"), default="auto",
                   help="publish rig_interfaces/HostStats (auto: when the installed rig_interfaces has it; default) or String JSON")
    p.add_argument("--print-every", type=float, default=0.0, help="print a summary line every N s (0 = only the first sample; default)")
    args, ros_args = p.parse_known_args(argv)
    if args.period <= 0 or args.sub < 1:
        p.error("--period must be > 0 and --sub ≥ 1")
    types = load_typed() if args.typed != "off" else None
    if args.typed == "on" and types is None:
        raise SystemExit("--typed on, but the installed rig_interfaces has no HostStats (rig-nodes ≥ 0.3.0, `pixi install -e typed`)")
    try:
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        sampler = HostSampler(args.period, args.sub, log_dir, args.top, parse_pins(args.pin) + DEFAULT_PINS, gpu=not args.no_gpu,
                              py_roots=args.py_root)
    except RuntimeError as exc:  # no psutil
        print(f"[host] {exc} — no host samples (the rig keeps running)", file=sys.stderr)
        return 0

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    if hasattr(signal, "SIGBREAK"):  # rig.py / the harness stop children with CTRL_BREAK_EVENT on Windows
        signal.signal(signal.SIGBREAK, lambda *_: stop.set())

    rclpy.init(args=ros_args)
    node = Node("rig_host_monitor")
    pub = node.create_publisher(types["HostStats"] if types else String, TOPIC, QOS)
    gpus = sampler.gpu.names if sampler.gpu is not None and sampler.gpu.available else []
    print(f"[host] {TOPIC} every {args.period:g} s ({args.sub} CPU sub-windows), "
          f"{'rig_interfaces/HostStats' if types else 'std_msgs/String JSON'}; {sampler.ncpu} logical CPUs"
          f"{' (efficiency classes ' + ''.join(str(c) for c in sampler.core_class) + ')' if sampler.core_class else ''}, "
          f"GPU {', '.join(gpus) or 'none'}, log volume {sampler.log_volume}", file=sys.stderr)
    n, next_print = 0, 0.0
    try:
        while not stop.is_set():
            s = sampler.window(stop=stop.is_set)
            if stop.is_set():
                break
            if types:
                pub.publish(to_msg(s, types))
            else:
                m = String()
                m.data = to_json(s)
                pub.publish(m)
            n += 1
            elapsed = n * args.period
            if n == 1 or (args.print_every > 0 and elapsed >= next_print):
                print(f"[host] {summary_line(s)}", file=sys.stderr, flush=True)
                next_print = elapsed + args.print_every
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
    print(f"[host] {n} samples published", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
