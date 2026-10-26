#!/usr/bin/env python3
"""bias_cmd.py — send one plugin command to bias_node.py and print its reply.

    pixi run bias-cmd -- connect
    pixi run bias-cmd -- startRecording --filename trial_001.avi
    pixi run bias-cmd -- stopRecording
    pixi run bias-cmd -- getTimestamp --json

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

QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", help="connect | startRecording | stopRecording | startPreview | stopCapture | getTimestamp | disconnect")
    p.add_argument("--filename", help="startRecording: video file name/path")
    p.add_argument("--ip", help="connect: BIAS host")
    p.add_argument("--port", type=int, help="connect: BIAS command port")
    p.add_argument("--config-file", help="connect: BIAS configuration file to load")
    p.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the reply (default: 10)")
    p.add_argument("--json", action="store_true", help="print the raw reply JSON")
    args, ros_args = p.parse_known_args(argv)
    params = {k: v for k, v in (("filename", args.filename), ("ip", args.ip), ("port", args.port), ("config_file", args.config_file)) if v}
    cid = str(uuid.uuid4())[:8]

    rclpy.init(args=ros_args)
    node = Node("bias_cmd_" + cid)
    got = {}
    done = threading.Event()

    def on_reply(msg: String) -> None:
        try:
            rep = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(rep, dict) and rep.get("id") == cid:
            got.update(rep)
            done.set()

    node.create_subscription(String, "/bias/reply", on_reply, QOS)
    pub = node.create_publisher(String, "/bias/cmd", QOS)
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
        print(f"[bias-cmd] no reply within {args.timeout:g}s (is `pixi run bias-node` running?)", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(got, indent=1))
    else:
        print(f"{got.get('command')} → {got.get('outcome')} ({got.get('rtt_ms')} ms)" + (f"  value={got.get('value')!r}" if got.get("value") not in (None, "") else "") + (f"  error={got.get('error')}" if got.get("error") else ""))
        if got.get("artifact"):
            print(f"artifact: {got['artifact']}")
    return 0 if got.get("ok") else (3 if got.get("outcome") == "unknown" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
