"""_common.py — the few things every PoC node shares (kept tiny on purpose).

* `wrap_line`  — THE adapter envelope the bridge node parses. One definition, so a field
                 added for one source cannot silently differ for another.
* `RELIABLE_QOS` — reliable / keep-last 200: the run log must hold every frame.
* `spin`       — rclpy.spin + the Jazzy shutdown boilerplate (Ctrl-C raises
                 ExternalShutdownException from spin, and rclpy's own handler may already
                 have shut the context down, so try_shutdown can raise too).
* `wait_port`  — poll a TCP port (the Zenoh router on 127.0.0.1:7447).
* `wall_ns` / `wall_ms` / `mono_ns` / `raise_timer_resolution` — sub-ms clocks on Windows too.
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

# ── clocks ───────────────────────────────────────────────────────────────────────────────
# Python 3.12 on Windows (RoboStack Jazzy pins it): time.time() is GetSystemTimeAsFileTime,
# which advances once per timer interrupt — 15.625 ms unless some process has raised the rate —
# and time.monotonic() is GetTickCount64, 15.625 ms always. Measured on ww18 2026-10-15: idle
# PC, run-log `ms` stamps would come in 15.6 ms steps, 2.3 ms behind the true time.
# GetSystemTimePreciseAsFileTime is the wall clock interpolated with the performance counter
# (~1 µs; what Python 3.13 made time.time()), and perf_counter is QueryPerformanceCounter.
# Elsewhere these are the stdlib clocks unchanged.
if os.name == "nt":
    import ctypes
    from ctypes import wintypes

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _k32.GetSystemTimePreciseAsFileTime.argtypes = [ctypes.POINTER(ctypes.c_ulonglong)]
    _k32.GetSystemTimePreciseAsFileTime.restype = None
    _k32.GetCurrentProcess.restype = wintypes.HANDLE
    _k32.SetProcessInformation.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    _k32.SetProcessInformation.restype = wintypes.BOOL
    _FILETIME_UNIX_EPOCH = 116444736000000000  # 100 ns intervals from 1601-01-01 to 1970-01-01

    def wall_ns() -> int:
        ft = ctypes.c_ulonglong()
        _k32.GetSystemTimePreciseAsFileTime(ctypes.byref(ft))
        return (ft.value - _FILETIME_UNIX_EPOCH) * 100

    mono_ns = time.perf_counter_ns
else:
    wall_ns = time.time_ns
    mono_ns = time.monotonic_ns


def wall_ms() -> int:
    """Wall-clock ms since the epoch, like bridge.py's now_ms() but precise on Windows."""
    return wall_ns() // 1_000_000


def raise_timer_resolution(period_ms: int = 1) -> bool:
    """Windows: a `period_ms` system timer for THIS process (timeBeginPeriod; per process since
    Windows 10 2004), kept when its console is hidden (Windows 11 otherwise may ignore the
    request for an occluded or minimized process). Sharpens waits and timeouts — socket,
    asyncio, rcl; the stamps do not depend on it (wall_ns). Windows releases it at exit.
    Returns True when granted; a no-op elsewhere."""
    if os.name != "nt":
        return False
    try:
        granted = ctypes.WinDLL("winmm").timeBeginPeriod(period_ms) == 0  # TIMERR_NOERROR

        class _PowerThrottling(ctypes.Structure):
            _fields_ = [("Version", wintypes.ULONG), ("ControlMask", wintypes.ULONG), ("StateMask", wintypes.ULONG)]

        # ProcessPowerThrottling = 4; PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION = 0x4,
        # StateMask 0 = never ignore this process's timer request.
        st = _PowerThrottling(1, 0x4, 0)
        _k32.SetProcessInformation(_k32.GetCurrentProcess(), 4, ctypes.byref(st), ctypes.sizeof(st))
        return granted
    except (AttributeError, OSError):
        return False


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


# ── typed interfaces (LAB-303 Plan B item 3) ──────────────────────────────────────────
INTERFACES_INSTALL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "interfaces", "install")


def _prepend_env(name: str, value: str) -> None:
    cur = os.environ.get(name, "")
    parts = cur.split(os.pathsep) if cur else []
    if value not in parts:
        os.environ[name] = os.pathsep.join([value] + parts)


def setup_interfaces(prefix: str | None = None) -> bool:
    """Make a colcon-built `interfaces/install` visible to THIS process and its children —
    the equivalent of sourcing install/setup.sh, done in Python so Windows needs no shell
    script. Returns True when the prefix exists. Call it BEFORE importing rig_interfaces."""
    import sys as _sys

    prefix = prefix or INTERFACES_INSTALL
    if not os.path.isdir(prefix):
        return False
    _prepend_env("AMENT_PREFIX_PATH", prefix)
    libdir = os.path.join(prefix, "lib")
    bindir = os.path.join(prefix, "bin")
    _prepend_env("PATH", bindir if os.name == "nt" else libdir)
    # rcpputils::find_library_path searches these for the typesupport libraries.
    _prepend_env("DYLD_LIBRARY_PATH" if _sys.platform == "darwin" else "LD_LIBRARY_PATH", libdir)
    if os.name == "nt":
        _prepend_env("PATH", libdir)
        try:
            os.add_dll_directory(libdir)
        except (AttributeError, OSError):
            pass
    sp = None
    if os.name == "nt":
        cand = os.path.join(prefix, "Lib", "site-packages")
        sp = cand if os.path.isdir(cand) else None
    if sp is None:
        pyver = f"python{_sys.version_info.major}.{_sys.version_info.minor}"
        cand = os.path.join(libdir, pyver, "site-packages")
        sp = cand if os.path.isdir(cand) else None
    if sp:
        _prepend_env("PYTHONPATH", sp)
        if sp not in _sys.path:
            _sys.path.insert(0, sp)
    return True


def probe_interfaces():
    """(ArenaFrame, FicTracFrame) when rig_interfaces imports AND its typesupport actually
    loads — else None. An import alone is not enough: a package that is importable but whose
    typesupport library cannot be dlopen'ed (seen with a colcon install visible from the
    default env: `RMWError: … Could not load library librig_interfaces__rosidl_typesupport_
    fastrtps_c.dylib`) would otherwise make a node pick typed topics and die at its first
    message. One serialize_message() is the test.

    The ENVIRONMENT's package (pixi-build, `pixi install -e typed`) is tried FIRST and a colcon
    `interfaces/install` is consulted only when nothing imports without it. Prepending a stale
    colcon prefix on top of the env mixed two builds (2026-10-14, after ArenaFrame gained two
    fields): the old Python class + C struct from the prefix, the new typesupport library from
    the env — rclpy serialized 16 bytes PAST the struct, and every recorded frame carried
    garbage in the new fields while the harness stayed green. One build per process, always."""
    found = _try_interfaces()
    if found is not None:
        return found
    if setup_interfaces():
        return _try_interfaces()
    return None


def _try_interfaces():
    try:
        from rig_interfaces.msg import ArenaFrame, FicTracFrame
        from rclpy.serialization import serialize_message

        serialize_message(ArenaFrame())
        serialize_message(FicTracFrame())
        return ArenaFrame, FicTracFrame
    except Exception:
        return None


def interfaces_available() -> bool:
    """True when rig_interfaces imports and serializes (after setup_interfaces)."""
    return probe_interfaces() is not None
