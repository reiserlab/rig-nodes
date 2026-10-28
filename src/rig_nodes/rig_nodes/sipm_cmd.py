#!/usr/bin/env python3
"""sipm_cmd.py — send one command to sipm_node.py and print its reply (all commands are read-only).

    pixi run sipm-cmd -- status
    pixi run sipm-cmd -- get --name PARAM_PMT1_ENABLE
    pixi run sipm-cmd -- get --id 1201
    pixi run sipm-cmd -- info --id 1201
    pixi run sipm-cmd -- scan --from 0 --to 2000 --out sipm_scan.json
    pixi run sipm-cmd -- disconnect      # free the COM port (e.g. to reopen ThorSiPM_Control.exe)
    pixi run sipm-cmd -- reconnect

Exit 0 on `ok`, 1 on an error reply, 2 on no reply.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", help="ping | status | get | info | scan | disconnect | reconnect")
    p.add_argument("--id", type=int, help="get/info: parameter ID")
    p.add_argument("--name", help="get/info: a --params-file name or a PARAM_* name")
    p.add_argument("--from", dest="first", type=int, help="scan: first ID (default 0)")
    p.add_argument("--to", dest="last", type=int, help="scan: last ID (default 2000)")
    p.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for the reply (default: 30; a full scan takes a while)")
    p.add_argument("--json", action="store_true", help="print the raw reply JSON")
    p.add_argument("--out", help="also write the reply JSON to this file")
    args, ros_args = p.parse_known_args(argv)
    params = {k: v for k, v in (("id", args.id), ("name", args.name), ("from", args.first), ("to", args.last)) if v is not None}
    cid = str(uuid.uuid4())[:8]

    rclpy.init(args=ros_args)
    node = Node("sipm_cmd_" + cid)
    got: dict = {}
    done = threading.Event()

    def on_reply(msg: String) -> None:
        try:
            rep = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(rep, dict) and rep.get("id") == cid:
            got.update(rep)
            done.set()

    node.create_subscription(String, "/sipm/reply", on_reply, QOS)
    pub = node.create_publisher(String, "/sipm/cmd", QOS)
    time.sleep(0.5)  # discovery
    m = String()
    m.data = json.dumps({"id": cid, "command": args.command, "params": params})
    pub.publish(m)
    deadline = time.monotonic() + args.timeout
    while not done.is_set() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:  # rclpy's own SIGINT handler may already have shut the context down (races in Jazzy)
        pass
    if not got:
        print(f"[sipm-cmd] no reply within {args.timeout:g}s (is `pixi run sipm-node` running?)", file=sys.stderr)
        return 2
    if args.out:
        Path(args.out).write_text(json.dumps(got, indent=1), encoding="utf-8")
    if args.json:
        print(json.dumps(got, indent=1))
    else:
        v = got.get("value")
        if args.command == "scan" and isinstance(v, dict):
            for r in v.get("available", []):
                print(f"  {r['id']:5d}  {r.get('name') or '?':40s}  value={r.get('value')!r}")
            print(f"{len(v.get('available', []))} available; not in the GUI enum: {v.get('not_in_enum')}")
        print(f"{got.get('command')} → {got.get('outcome')} ({got.get('rtt_ms')} ms)"
              + (f"  value={json.dumps(v)[:300]}" if v not in (None, "") and args.command != "scan" else "")
              + (f"  error={got.get('error')}" if got.get("error") else ""))
    return 0 if got.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
