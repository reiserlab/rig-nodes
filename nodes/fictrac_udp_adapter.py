#!/usr/bin/env python3
"""fictrac_udp_adapter.py — FicTrac UDP → ROS 2 topic, one std_msgs/String per record.

FicTrac itself is untouched: it keeps sending its 25-field CSV line per camera frame to
a UDP port (the same `sock_host`/`sock_port` it has today, 127.0.0.1:60000). This node
owns that socket and republishes every datagram line as JSON on `/fictrac/raw`:

    {"src": "fictrac", "seq": <adapter counter>,
     "rx_epoch_ms": <ms, wall clock at receipt>, "rx_mono_ns": <ns, monotonic at receipt>,
     "raw": "<the FicTrac line, verbatim, no newline>"}

The line is NOT parsed here. The bridge node parses it with bridge.py's Pipeline, so the
closed-loop mapping, the heading tare and the logged row are byte-for-byte what bridge
3.4 produces — the adapter only moves bytes and stamps when it saw them. No custom
message types (LAB-303 PoC rule): std_msgs/String carrying JSON.

QoS: RELIABLE, KEEP_LAST 200. The run log must hold EVERY FicTrac frame (bridge.py
write_frame), so the topic is reliable; the bridge node's Hub coalesces for the browser
separately, as before. ~50-100 Hz × ~250 B is nothing for Zenoh on loopback.

    pixi run fictrac-adapter -- --port 60000
    pixi run sim -- --proto udp --rate 50            # the unmodified simulator feeds it

PLAN B (LAB-303, typed topic): with `--typed` (default `auto` = when arena_interfaces imports)
the adapter ALSO publishes `/fictrac/frame` as arena_interfaces/FicTracFrame — the parsed
columns, the FT tag, the receive stamps AND the verbatim line — so the closed-loop node can
subscribe to a typed topic while the mapping math still parses exactly the bytes FicTrac sent.
`/fictrac/raw` keeps being published (`--typed only` drops it) so `ros2 topic echo` and older
consumers see no change.
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

from _common import RELIABLE_QOS, mono_ns, probe_interfaces, raise_timer_resolution, spin, wall_ms, wrap_line

_IFACES = probe_interfaces()  # import + serialize; None → String only
HAVE_IFACES = _IFACES is not None
FicTracFrame = _IFACES[1] if _IFACES else None  # type: ignore[assignment]

TOPIC = "/fictrac/raw"
TYPED_TOPIC = "/fictrac/frame"
RAW_QOS = RELIABLE_QOS


def parse_fictrac_line(raw: str) -> tuple[str, list[float]]:
    """(tag, fields) the way bridge.py's Pipeline parses: a leading FT* token is the tag
    ('' when absent, as in offline .dat lines and the simulator); fields = the floats after
    it, or [] when the line is not numeric (a bad frame has no usable data)."""
    parts = raw.strip().split(",")
    tag = ""
    if parts and parts[0].strip()[:2].upper() == "FT":
        tag = parts[0].strip()
        parts = parts[1:]
    try:
        return tag, [float(p) for p in parts]
    except ValueError:
        return tag, []



class FicTracUdpAdapter(Node):
    def __init__(self, host: str, port: int, status_every_s: float, typed: str = "auto") -> None:
        super().__init__("fictrac_udp_adapter")
        want_typed = HAVE_IFACES if typed == "auto" else typed in ("on", "only")
        if typed in ("on", "only") and not HAVE_IFACES:
            raise SystemExit("--typed on/only, but arena_interfaces is not importable (pixi install builds it)")
        self.typed_pub = self.create_publisher(FicTracFrame, TYPED_TOPIC, RAW_QOS) if want_typed else None
        self.pub = None if typed == "only" else self.create_publisher(String, TOPIC, RAW_QOS)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)  # lets the thread notice shutdown
        self.seq = 0
        self.lines = 0
        self.datagrams = 0
        self._stop = threading.Event()
        self._buf = b""
        self._thread = threading.Thread(target=self._recv_loop, name="udp-recv", daemon=True)
        self._thread.start()
        self.get_logger().info(f"UDP {host}:{port} → {TOPIC if self.pub else '-'}" + (f" + {TYPED_TOPIC} (arena_interfaces/FicTracFrame)" if self.typed_pub else " (std_msgs/String JSON, reliable)"))
        if status_every_s > 0:
            self._last = (0, time.monotonic())
            self.create_timer(status_every_s, self._status)

    def _publish(self, raw: str) -> None:
        self.seq += 1
        rx_ms, rx_ns = wall_ms(), mono_ns()  # precise on Windows too (_common "clocks")
        if self.pub is not None:
            msg = String()
            msg.data = wrap_line("fictrac", self.seq, raw, rx_ms, rx_ns)
            self.pub.publish(msg)
        if self.typed_pub is not None:
            tag, fields = parse_fictrac_line(raw)
            self.typed_pub.publish(FicTracFrame(seq=self.seq, rx_epoch_ms=rx_ms, rx_mono_ns=rx_ns, tag=tag, fields=fields, raw=raw))
        self.lines += 1

    def _recv_loop(self) -> None:
        # One record per datagram is FicTrac's behaviour; buffer + split on newlines anyway
        # (same tolerance as bridge.py's _UdpProtocol) so a coalesced or split datagram
        # never produces a half line.
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.datagrams += 1
            self._buf += data
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                if line.strip():
                    self._publish(line.decode("ascii", errors="replace"))
            if self._buf and b"\n" not in data:
                self._publish(self._buf.decode("ascii", errors="replace"))
                self._buf = b""

    def _status(self) -> None:
        n0, t0 = self._last
        now = time.monotonic()
        rate = (self.lines - n0) / max(1e-9, now - t0)
        self._last = (self.lines, now)
        self.get_logger().info(f"{self.lines} lines published ({rate:.1f}/s), {self.datagrams} datagrams")

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
    p.add_argument("--port", type=int, default=60000, help="UDP bind port — FicTrac's sock_port (default: 60000)")
    p.add_argument("--status-every", type=float, default=5.0, help="stderr rate line interval in s; 0 = off (default: 5)")
    p.add_argument("--typed", choices=("auto", "on", "off", "only"), default="auto",
                   help="also publish /fictrac/frame as arena_interfaces/FicTracFrame (auto: when the package imports; only: drop /fictrac/raw)")
    # ros2 run / launch pass --ros-args; keep them for rclpy.
    args, ros_args = p.parse_known_args(argv)
    raise_timer_resolution()  # Windows: 1 ms system timer for this process (_common "clocks")
    rclpy.init(args=ros_args)
    node = FicTracUdpAdapter(args.host, args.port, args.status_every, args.typed)
    spin(node)
    print(f"[fictrac-adapter] {node.lines} lines, {node.datagrams} datagrams", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
