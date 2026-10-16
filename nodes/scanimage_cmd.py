#!/usr/bin/env python3
"""scanimage_cmd.py — send one plugin command to scanimage_node.py and print its reply.

    pixi run scanimage-cmd -- ping
    pixi run scanimage-cmd -- status --json
    pixi run scanimage-cmd -- setFileStem --stem violet_d7
    pixi run scanimage-cmd -- set --property hStackManager.framesPerSlice --value 750
    pixi run scanimage-cmd -- grab
    pixi run scanimage-cmd -- waitIdle --timeout-s 40
    pixi run scanimage-cmd -- abort

Exit 0 on `ok`, 1 on an error reply, 3 on an UNKNOWN outcome (timeout), 2 on no reply.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=50, durability=QoSDurabilityPolicy.VOLATILE)


def parse_value(text: str):
    """--value as JSON when it parses (750, true, "x"), else the plain string."""
    try:
        return json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return text


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", help="ping | status | setFileStem | set | grab | waitIdle | abort | get")
    p.add_argument("--stem", help="setFileStem: the log file stem")
    p.add_argument("--property", help="get/set: property path, e.g. hStackManager.framesPerSlice")
    p.add_argument("--value", help="set: the value (JSON if it parses, else a string)")
    p.add_argument("--timeout-s", type=float, help="waitIdle: seconds to wait for acqState idle")
    p.add_argument("--timeout", type=float, default=45.0, help="seconds to wait for the reply (default: 45)")
    p.add_argument("--json", action="store_true", help="print the raw reply JSON")
    args, ros_args = p.parse_known_args(argv)
    params: dict = {}
    if args.stem is not None:
        params["stem"] = args.stem
    if args.property is not None:
        params["property"] = args.property
    if args.value is not None:
        params["value"] = parse_value(args.value)
    if args.timeout_s is not None:
        params["timeout_s"] = args.timeout_s
    cid = str(uuid.uuid4())[:8]

    rclpy.init(args=ros_args)
    node = Node("scanimage_cmd_" + cid)
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

    node.create_subscription(String, "/scanimage/reply", on_reply, QOS)
    pub = node.create_publisher(String, "/scanimage/cmd", QOS)
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
    except Exception:
        pass
    if not got:
        print(f"[scanimage-cmd] no reply within {args.timeout:g}s (is `pixi run scanimage-node` running?)", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(got, indent=1))
    else:
        v = got.get("value")
        print(f"{got.get('command')} → {got.get('outcome')} ({got.get('rtt_ms')} ms)" + (f"  value={v!r}" if v not in (None, "") else "") + (f"  error={got.get('error')}" if got.get("error") else ""))
        if got.get("artifact"):
            print(f"artifact: {got['artifact']}")
        if got.get("reconciled"):
            print(f"reconciled status: {got['reconciled']}")
    return 0 if got.get("ok") else (3 if got.get("outcome") == "unknown" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
