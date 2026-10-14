"""_common.py — the few things every PoC node shares (kept tiny on purpose).

* `wrap_line`  — THE adapter envelope the bridge node parses. One definition, so a field
                 added for one source cannot silently differ for another.
* `RELIABLE_QOS` — reliable / keep-last 200: the run log must hold every frame.
* `spin`       — rclpy.spin + the Jazzy shutdown boilerplate (Ctrl-C raises
                 ExternalShutdownException from spin, and rclpy's own handler may already
                 have shut the context down, so try_shutdown can raise too).
* `wait_port`  — poll a TCP port (the Zenoh router on 127.0.0.1:7447).
"""

from __future__ import annotations

import json
import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def wrap_line(src: str, seq: int, raw: str, rx_epoch_ms: int, rx_mono_ns: int) -> str:
    """Adapter envelope: {"src","seq","rx_epoch_ms","rx_mono_ns","raw"} as compact JSON."""
    return json.dumps(
        {"src": src, "seq": seq, "rx_epoch_ms": rx_epoch_ms, "rx_mono_ns": rx_mono_ns, "raw": raw},
        separators=(",", ":"),
    )


def spin(node) -> None:
    """rclpy.spin(node) until Ctrl-C / external shutdown, then destroy + shut down quietly."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:  # rclpy's SIGINT handler may already have shut the context down
            pass


def wait_port(host: str, port: int, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False
