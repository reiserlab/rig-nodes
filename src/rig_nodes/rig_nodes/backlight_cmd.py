#!/usr/bin/env python3
"""backlight_cmd.py — send one plugin command to backlight_node.py and print its reply.

    pixi run backlight-cmd -- setIRLEDPower --power 50
    pixi run backlight-cmd -- setRedLEDPower --power 5 --panel-num 2 --pattern 1010
    pixi run backlight-cmd -- turnOnLED
    pixi run backlight-cmd -- setVisibleBacklightsOff --json
    pixi run backlight-cmd -- turnOnLED --timeout-s 2        # each write may block at most 2 s

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

from rig_nodes.backlight import PLUGIN_COMMANDS

QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", help=" | ".join(PLUGIN_COMMANDS))
    p.add_argument("--power", type=float, help="setIRLEDPower / set<Color>LEDPower: 0-100")
    p.add_argument("--panel-num", type=int, help="set<Color>LEDPower: panel number")
    p.add_argument("--pattern", help="set<Color>LEDPower: on/off pattern, e.g. 1010 (needs --panel-num)")
    p.add_argument("--timeout-s", type=float, help="the command's timeout_s: the most each controller write may block (the node's --write-timeout-s otherwise)")
    p.add_argument("--timeout", type=float, default=10.0, help="seconds this tool waits for the reply (default: 10)")
    p.add_argument("--json", action="store_true", help="print the raw reply JSON")
    args, ros_args = p.parse_known_args(argv)
    # `is not None`, not truthiness: power 0 and panel 0 are real values.
    params = {k: v for k, v in (("power", args.power), ("panel_num", args.panel_num), ("pattern", args.pattern), ("timeout_s", args.timeout_s))
              if v is not None}
    cid = str(uuid.uuid4())[:8]

    rclpy.init(args=ros_args)
    node = Node("backlight_cmd_" + cid)
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

    node.create_subscription(String, "/backlight/reply", on_reply, QOS)
    pub = node.create_publisher(String, "/backlight/cmd", QOS)
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
        print(f"[backlight-cmd] no reply within {args.timeout:g}s (is `pixi run backlight-node` running?)", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(got, indent=1))
    else:
        lines = ", ".join(repr(s.get("command")) for s in got.get("steps", []))
        print(f"{got.get('command')} → {got.get('outcome')} ({got.get('rtt_ms')} ms)" + (f"  sent {lines}" if lines else "")
              + (f"  error={got.get('error')}" if got.get("error") else ""))
        if got.get("warning"):
            print(f"warning: {got['warning']}")
    return 0 if got.get("ok") else (3 if got.get("outcome") == "unknown" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
