#!/usr/bin/env python3
"""kinefly_udp_adapter.py — Kinefly KF/1 UDP → ROS 2 topic, one std_msgs/String per frame.

Michael's Kinefly fork (mbreiser/Kinefly-MacOS, `--udp HOST:PORT`) sends one UTF-8 JSON
datagram per processed frame: the KF/1 record documented in its docs/udp-protocol.md
(`protocol: "Kinefly"`, `version: 1`, `session`, `sequence`, `source`, `timestamp_s`,
`host_monotonic_s`, `parts.{head,abdomen,left,right,aux}`). Kinefly is untouched; this
node owns the receiving socket and republishes each datagram verbatim on `/kinefly/raw`
inside the same envelope the FicTrac adapter uses:

    {"src": "kinefly", "seq": <adapter counter>, "rx_epoch_ms": …, "rx_mono_ns": …,
     "raw": "<the KF/1 datagram text, verbatim>"}

Protocol discrimination happens HERE, before anything downstream decodes: a datagram
that is not JSON, or whose `protocol`/`version` are not `Kinefly`/1, is counted and
dropped (Kinefly's bridge plan, step 1: discriminate protocol before decoding, never pad
KF data into FicTrac fields). Nothing else is interpreted — the bridge node derives the
logged `["kf", …]` row and (later) the L−R → yaw integrator from the parsed record, so
the mapping lives in one place.

    pixi run kinefly-adapter -- --port 60001
    pixi run kinefly-sim -- --port 60001 --rate 100     # synthetic KF/1 producer
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from _common import RELIABLE_QOS, mono_ns, raise_timer_resolution, spin, wall_ms, wrap_line

TOPIC = "/kinefly/raw"
DEFAULT_PORT = 60001
MAX_DATAGRAM = 16384  # KF/1 encoder bound (kinefly/core/udp.py)
RAW_QOS = RELIABLE_QOS


def kf1_reject_reason(data: bytes) -> str | None:
    """Why a datagram is NOT a KF/1 record (None = accept). Cheap and strict on the
    identity fields only; full field validation is the bridge node's job."""
    if len(data) > MAX_DATAGRAM:
        return "oversize"
    try:
        obj = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return "not JSON"
    if not isinstance(obj, dict):
        return "not an object"
    if obj.get("protocol") != "Kinefly" or obj.get("version") != 1:
        return f"protocol/version {obj.get('protocol')!r}/{obj.get('version')!r}"
    if not isinstance(obj.get("sequence"), int) or not isinstance(obj.get("parts"), dict):
        return "missing sequence/parts"
    return None



class KineflyUdpAdapter(Node):
    def __init__(self, host: str, port: int, status_every_s: float) -> None:
        super().__init__("kinefly_udp_adapter")
        self.pub = self.create_publisher(String, TOPIC, RAW_QOS)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)
        self.seq = 0
        self.published = 0
        self.rejected = 0
        self._reject_reasons: dict[str, int] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._recv_loop, name="udp-recv", daemon=True)
        self._thread.start()
        self.get_logger().info(f"UDP {host}:{port} (KF/1) → {TOPIC} (std_msgs/String JSON, reliable)")
        if status_every_s > 0:
            self._last = (0, time.monotonic())
            self.create_timer(status_every_s, self._status)

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            rx_epoch_ms, rx_mono_ns = wall_ms(), mono_ns()  # precise on Windows too (_common "clocks")
            why = kf1_reject_reason(data)
            if why is not None:
                self.rejected += 1
                self._reject_reasons[why] = self._reject_reasons.get(why, 0) + 1
                if self._reject_reasons[why] <= 3:
                    self.get_logger().warning(f"dropped non-KF/1 datagram ({why})")
                continue
            self.seq += 1
            msg = String()
            msg.data = wrap_line("kinefly", self.seq, data.decode("utf-8"), rx_epoch_ms, rx_mono_ns)
            self.pub.publish(msg)
            self.published += 1

    def _status(self) -> None:
        n0, t0 = self._last
        now = time.monotonic()
        rate = (self.published - n0) / max(1e-9, now - t0)
        self._last = (self.published, now)
        self.get_logger().info(f"{self.published} KF/1 frames published ({rate:.1f}/s), {self.rejected} rejected")

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="UDP bind host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"UDP bind port — Kinefly's --udp destination (default: {DEFAULT_PORT})")
    p.add_argument("--status-every", type=float, default=5.0, help="stderr rate line interval in s; 0 = off (default: 5)")
    args, ros_args = p.parse_known_args(argv)
    raise_timer_resolution()  # Windows: 1 ms system timer for this process (_common "clocks")
    rclpy.init(args=ros_args)
    node = KineflyUdpAdapter(args.host, args.port, args.status_every)
    spin(node)
    print(f"[kinefly-adapter] {node.published} published, {node.rejected} rejected", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
