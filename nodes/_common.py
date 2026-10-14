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
import os
import signal
import socket
import subprocess
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


# Windows' signal module has no SIGKILL; every "kill it" site uses this constant so the
# escalation paths (rig.py, the test harness, signal_group) do not raise AttributeError there.
SIGKILL = getattr(signal, "SIGKILL", 9)


def popen_group(argv: list, **kwargs) -> subprocess.Popen:
    """Popen with the child in its OWN process group, on every platform, so a stop signal
    reaches wrappers and what they spawn (`ros2 run … rmw_zenohd` does not forward SIGINT
    to the router binary). POSIX: setsid; Windows: CREATE_NEW_PROCESS_GROUP, which is also
    what makes CTRL_BREAK_EVENT deliverable to that child."""
    if os.name == "nt":
        kwargs.setdefault("creationflags", subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs.setdefault("start_new_session", True)
    return subprocess.Popen(argv, **kwargs)


def signal_group(proc: subprocess.Popen, sig: int) -> None:
    """Send SIGINT / SIGTERM / SIGKILL to the child's whole group, portably.
    Windows has no SIGINT for other processes (send_signal raises ValueError): SIGINT maps to
    CTRL_BREAK_EVENT on the child's group, SIGTERM and SIGKILL to TerminateProcess."""
    if os.name == "nt":
        if sig == signal.SIGINT:
            try:
                proc.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
                return
            except (ValueError, OSError):
                pass
        proc.kill() if sig == SIGKILL else proc.terminate()
        return
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        proc.send_signal(sig)


def wait_port(host: str, port: int, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False
