"""_common.py — the few things every node shares (kept tiny on purpose).

* `wrap_line`  — THE adapter envelope (CONTRACT.md "Envelope"). One definition, so a field
                 added for one source cannot silently differ for another.
* `RELIABLE_QOS` — reliable / keep-last 200: a run record must hold every frame.
* `spin`       — rclpy.spin + the Jazzy shutdown boilerplate (Ctrl-C raises
                 ExternalShutdownException from spin, and rclpy's own handler may already
                 have shut the context down, so try_shutdown can raise too).
* `wall_ns` / `wall_ms` / `mono_ns` / `raise_timer_resolution` — sub-ms clocks on Windows too.
* `probe_interfaces` / `setup_interfaces` — is the typed `rig_interfaces` package usable here?
* the Zenoh loopback configuration is applied to the environment at import (`rig_nodes.zenoh`).
* the process helpers (`SIGKILL`, `popen_group`, `signal_group`, `wait_port`) are re-exported
  from `rig_nodes.supervise`, their home since 2026-10-26, for the callers that import them here.
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from rig_nodes.supervise import SIGKILL, popen_group, signal_group, wait_port  # noqa: F401  (re-exported)
from rig_nodes.zenoh import apply_env_defaults

# Every node that imports this module runs with the package's loopback Zenoh configuration
# unless the environment already names one (rig_nodes/zenoh.py) — so an installed consumer
# needs no checkout to find the json5 files. Done at import, before any rclpy.init().
apply_env_defaults()

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
    """Wall-clock ms since the epoch, precise on Windows."""
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


# ── typed interfaces ─────────────────────────────────────────────────────────────────────
# The installed `rig_interfaces` conda package is the normal case. A colcon-built install prefix
# (`rig_nodes.build_interfaces`, the .msg author's path) is consulted only when named by the
# RIG_INTERFACES_INSTALL environment variable or passed to setup_interfaces() explicitly — the
# module no longer guesses a prefix from its own location, since it lives in site-packages.
INTERFACES_INSTALL_ENV = "RIG_INTERFACES_INSTALL"


def _prepend_env(name: str, value: str) -> None:
    cur = os.environ.get(name, "")
    parts = cur.split(os.pathsep) if cur else []
    if value not in parts:
        os.environ[name] = os.pathsep.join([value] + parts)


def setup_interfaces(prefix: str | None = None) -> bool:
    """Make a colcon-built install prefix visible to THIS process and its children — the
    equivalent of sourcing install/setup.sh, done in Python so Windows needs no shell script.
    Returns True when the prefix exists. Call it BEFORE importing rig_interfaces."""
    import sys as _sys

    prefix = prefix or os.environ.get(INTERFACES_INSTALL_ENV)
    if not prefix or not os.path.isdir(prefix):
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

    The ENVIRONMENT's package (the installed conda package) is tried FIRST and a colcon prefix
    (RIG_INTERFACES_INSTALL) only when nothing imports without it. Prepending a stale colcon
    prefix on top of the env mixed two builds (2026-10-14, after ArenaFrame gained two fields):
    the old Python class + C struct from the prefix, the new typesupport library from the env —
    rclpy serialized 16 bytes PAST the struct, and every recorded frame carried garbage in the
    new fields while the harness stayed green. One build per process, always."""
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
