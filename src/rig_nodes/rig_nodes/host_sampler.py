"""host_sampler.py (rig_nodes; moved from webDisplayTools rig/nodes/host_sampler.py on 2026-10-28, prototyped on ww18) — host "pressure" samples for the rig graph (NO ROS imports; host_monitor.py publishes them).

One `HostSampler.window()` call = one window (default 1 s) and returns a dict whose keys are the
fields of rig_interfaces/HostStats (src/rig_interfaces/msg/HostStats.msg), with nested lists of dicts for
`disks` / `nets` / `gpus` / `procs`. Absent values are NaN (float) / 0 (int), as the contract says.

CPU follows atlas-bench `tools/cpu_sampler.py` (the method the lab has used on ww18): on Windows,
per-core busy = 1 − Δidle cycles / (cycle rate × Δt) from QueryIdleProcessorCycleTime every
1/sub_samples of the window — exact, where Task Manager's tick-based numbers step at 15.6 ms — and
each window reports the mean AND the busiest sub-window, so a 100 ms burst on one core is not
averaged away. How late the sampler woke for a sub-window (`sampler_late_ms_max`) shows a PC-wide
stall. Every process comes from ONE NtQuerySystemInformation(SystemProcessInformation) call per
window (cycles, working set, I/O bytes, hard faults); a psutil scan of ~300 processes took 0.7 s on
ww18, this takes a few ms. Python processes get their script name (`py:<stem>`) from their command
line, read once per new pid. Elsewhere (Linux/macOS) psutil supplies the same fields, tick-based.

Memory, disks, network adapters and the log volume come from psutil on every platform; NVIDIA GPUs
from the driver's own NVML library through ctypes (no Python package). Design, budget, measured
cost: docs/development/rig-host-monitor.md.
"""

from __future__ import annotations

import ctypes
import math
import os
import re
import socket
import struct
import sys
import time
from pathlib import Path

try:
    import psutil
except ImportError:  # pragma: no cover - psutil ships in the rig env
    psutil = None

NAN = float("nan")
MB = 1024.0 * 1024.0
WIN = os.name == "nt"

# Pinned process groups: (label, regex on the executable image name, case-insensitive). A group's
# processes are summed into ONE procs row (pid = its busiest process). Python processes are grouped by
# script instead (py_group). First match wins; --pin adds groups in front.
DEFAULT_PINS = [
    ("router", r"^rmw_zenohd"),
    ("foxglove_bridge", r"^foxglove_bridge"),
    ("fictrac", r"^fictrac"),
    ("bias", r"^(test_gui|bias)"),  # BIAS's GUI binary is test_gui.exe (atlas-bench)
    ("matlab", r"^matlab"),
    ("foxglove_app", r"^foxglove(\.exe)?$"),
    ("browser", r"^(chrome|msedge|google chrome)(\.exe)?$"),
]
PY_EXE = re.compile(r"^python[0-9.]*w?(\.exe)?$", re.I)
REASON_CPU, REASON_MEM, REASON_IO, REASON_PINNED = 1, 2, 4, 8


def _f(v) -> float:
    return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else NAN


def py_group(cmdline: list[str], roots: tuple[Path, ...] = ()) -> str | None:
    """`py:<stem>` for a Python process running a `rig_nodes.<module>` (always), or a script under one
    of `roots` (absolute, or relative to that root — a rig passes its workspace, e.g. `--py-root <repo>/rig`);
    None for any other Python process."""
    args = list(cmdline[1:])
    for i, a in enumerate(args):
        if a == "-m" and i + 1 < len(args):
            mod = args[i + 1]
            return f"py:{mod.split('.')[-1]}" if mod.startswith("rig_nodes") else None
        if a.lower().endswith(".py"):
            p = Path(a.strip('"'))
            for root in roots:
                for c in ([p] if p.is_absolute() else [root / p]):
                    try:
                        c.resolve().relative_to(root.resolve())
                    except (OSError, ValueError):
                        continue
                    if p.is_absolute() or c.exists():  # a relative path is relative to ITS cwd: must exist here
                        return f"py:{p.stem}"
            return None
        if not a.startswith("-"):
            return None
    return None


# ── Windows: cycle counters + one-call process table (ported from atlas-bench cpu_sampler.py) ─────
if WIN:
    from ctypes import wintypes as W

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _ntdll = ctypes.WinDLL("ntdll")
    _k32.QueryIdleProcessorCycleTime.argtypes = [ctypes.POINTER(W.ULONG), ctypes.c_void_p]
    _k32.QueryThreadCycleTime.argtypes = [W.HANDLE, ctypes.POINTER(ctypes.c_uint64)]
    _k32.QueryProcessCycleTime.argtypes = [W.HANDLE, ctypes.POINTER(ctypes.c_uint64)]
    _k32.GetCurrentThread.restype = W.HANDLE
    _k32.GetCurrentProcess.restype = W.HANDLE
    _k32.GetSystemCpuSetInformation.argtypes = [ctypes.c_void_p, W.ULONG, ctypes.POINTER(W.ULONG), W.HANDLE, W.ULONG]
    _ntdll.NtQuerySystemInformation.argtypes = [W.ULONG, ctypes.c_void_p, W.ULONG, ctypes.POINTER(W.ULONG)]

    def win_ncpu() -> int:
        n = W.ULONG(0)
        _k32.QueryIdleProcessorCycleTime(ctypes.byref(n), None)
        return n.value // 8

    def win_idle_cycles(n: int) -> list[int]:
        buf = (ctypes.c_uint64 * n)()
        size = W.ULONG(8 * n)
        _k32.QueryIdleProcessorCycleTime(ctypes.byref(size), buf)
        return list(buf)

    def _thread_cycles() -> int:
        c = ctypes.c_uint64()
        _k32.QueryThreadCycleTime(_k32.GetCurrentThread(), ctypes.byref(c))
        return c.value

    def win_self_cycles() -> int:
        c = ctypes.c_uint64()
        _k32.QueryProcessCycleTime(_k32.GetCurrentProcess(), ctypes.byref(c))
        return c.value

    def win_calibrate_hz(rounds: int = 3, spin_s: float = 0.1) -> float:
        """Cycle-counter rate: spin this thread and compare its cycles with wall time (best of N)."""
        best = 0.0
        for _ in range(rounds):
            c0, t0 = _thread_cycles(), time.perf_counter()
            while time.perf_counter() - t0 < spin_s:
                pass
            c1, t1 = _thread_cycles(), time.perf_counter()
            best = max(best, (c1 - c0) / (t1 - t0))
        return best

    def win_core_classes(n: int) -> list[int]:
        """EfficiencyClass per logical processor (higher = faster core); [] when unknown."""
        need = W.ULONG(0)
        _k32.GetSystemCpuSetInformation(None, 0, ctypes.byref(need), None, 0)
        if need.value == 0:
            return []
        buf = ctypes.create_string_buffer(need.value)
        if not _k32.GetSystemCpuSetInformation(buf, need, ctypes.byref(need), None, 0):
            return []
        raw, off, out = buf.raw, 0, {}
        while off < need.value:
            size = int.from_bytes(raw[off:off + 4], "little")
            if size == 0:
                break
            out[raw[off + 14]] = raw[off + 18]  # LogicalProcessorIndex, EfficiencyClass
            off += size
        return [out.get(i, 0) for i in range(n)]

    _spi = [ctypes.create_string_buffer(1 << 20)]
    # x64 SYSTEM_PROCESS_INFORMATION: NextEntryOffset @0, HardFaultCount @16, CycleTime @24,
    # ImageName.Length @56 / .Buffer @64, UniqueProcessId @80, WorkingSetSize @144,
    # ReadTransferCount @232, WriteTransferCount @240 (cpu_sampler.py reads the same cycles/name/pid).
    _SPI = struct.Struct("<I12xI4xQ24xH6xQ8xQ56xQ80xQQ")
    _NAMES: dict[tuple, str] = {}  # (pid, UTF-16 bytes) → decoded image name

    def _ntquery(cls: int, holder: list) -> ctypes.Array | None:
        need = W.ULONG(0)
        while True:
            buf = holder[0]
            st = _ntdll.NtQuerySystemInformation(cls, buf, len(buf), ctypes.byref(need)) & 0xFFFFFFFF
            if st == 0xC0000004:  # STATUS_INFO_LENGTH_MISMATCH
                holder[0] = ctypes.create_string_buffer(max(need.value, len(buf)) * 2)
                continue
            return buf if st == 0 else None

    def win_processes() -> dict[int, tuple]:
        """{pid: (image name, cycles, working set B, read B, write B, hard faults)} for every
        process, from one NtQuerySystemInformation(SystemProcessInformation) call."""
        buf = _ntquery(5, _spi)
        if buf is None:
            return {}
        mv = memoryview(buf).cast("B")  # no copy of the (~1 MB) buffer
        base = ctypes.addressof(buf)
        unpack = _SPI.unpack_from
        out, off, n = {}, 0, len(mv)
        while True:
            nxt, hard, cyc, nlen, nptr, pid, ws, rd, wr = unpack(mv, off)
            if nptr and nlen:
                # the strings live inside this buffer: decode from it, not through a pointer read
                rel = nptr - base
                key = (pid, bytes(mv[rel:rel + nlen])) if 0 <= rel < n else None
                name = _NAMES.get(key) if key else None
                if name is None:
                    name = key[1].decode("utf-16-le", "replace") if key else ctypes.wstring_at(nptr, nlen // 2)
                    if key:
                        if len(_NAMES) > 4096:
                            _NAMES.clear()
                        _NAMES[key] = name
            else:
                name = "Idle"
            out[pid] = (name, cyc, ws, rd, wr, hard)
            if nxt == 0 or off + nxt >= n:
                return out
            off += nxt

    _sii = [ctypes.create_string_buffer(64 * 24)]

    def win_dpc_count() -> int | None:
        """Σ DpcCount over CPUs (SystemInterruptInformation: 24 B per CPU, DpcCount @4)."""
        buf = _ntquery(23, _sii)
        if buf is None:
            return None
        raw = buf.raw
        return sum(struct.unpack_from("<I", raw, o + 4)[0] for o in range(0, len(raw) - 23, 24))

    # Network adapters: GetIfEntry2 per adapter (µs) instead of psutil's GetAdaptersAddresses
    # walk on every read (5.3 ms on ww18). The adapter list (hardware, up, not loopback) comes from
    # GetIfTable2, refreshed every 10 s. MIB_IF_ROW2 (1352 B): InterfaceIndex @8, Alias @28,
    # Type @1128, InterfaceAndOperStatusFlags @1152 (bit 0 HardwareInterface), OperStatus @1156,
    # InOctets @1208, InDiscards @1232, InErrors @1240, OutOctets @1280, OutDiscards @1304, OutErrors @1312.
    _iphlp = ctypes.WinDLL("iphlpapi")
    _iphlp.GetIfTable2.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    _iphlp.FreeMibTable.argtypes = [ctypes.c_void_p]
    _iphlp.GetIfEntry2.argtypes = [ctypes.c_void_p]
    ROW2 = 1352
    _IF_CTR = struct.Struct("<Q16xQQ32xQ16xQQ")  # from @1208: InOctets, InDiscards, InErrors, OutOctets, OutDiscards, OutErrors

    def win_adapters() -> list[tuple[int, str]]:
        tbl = ctypes.c_void_p()
        if _iphlp.GetIfTable2(ctypes.byref(tbl)) != 0 or not tbl.value:
            return []
        try:
            n = ctypes.c_ulong.from_address(tbl.value).value
            raw = ctypes.string_at(tbl.value + 8, n * ROW2)
        finally:
            _iphlp.FreeMibTable(tbl)
        out = []
        for k in range(n):
            o = k * ROW2
            idx, = struct.unpack_from("<I", raw, o + 8)
            typ, = struct.unpack_from("<I", raw, o + 1128)
            hw = raw[o + 1152] & 1
            oper, = struct.unpack_from("<I", raw, o + 1156)
            if hw and oper == 1 and typ != 24:  # IfOperStatusUp, not IF_TYPE_SOFTWARE_LOOPBACK
                alias = raw[o + 28:o + 28 + 514].decode("utf-16-le", "replace").split("\x00", 1)[0]
                out.append((idx, alias))
        return out

    _row = ctypes.create_string_buffer(ROW2)

    def win_adapter_counters(idx: int) -> tuple | None:
        ctypes.memset(_row, 0, ROW2)
        struct.pack_into("<I", _row, 8, idx)
        if _iphlp.GetIfEntry2(_row) != 0:
            return None
        return _IF_CTR.unpack_from(_row.raw, 1208)


# ── NVIDIA GPUs through the driver's NVML (ctypes; no Python package) ──────────────────────────────
class _Util(ctypes.Structure):
    _fields_ = [("gpu", ctypes.c_uint), ("memory", ctypes.c_uint)]


class _Mem(ctypes.Structure):
    _fields_ = [("total", ctypes.c_ulonglong), ("free", ctypes.c_ulonglong), ("used", ctypes.c_ulonglong)]


class Nvml:
    """The few NVML calls HostStats needs. `available` is False without an NVIDIA driver."""

    def __init__(self) -> None:
        self.lib = None
        self.handles: list = []
        self.names: list[str] = []
        try:
            if WIN:
                path = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32", "nvml.dll")
                self.lib = ctypes.CDLL(path)
            else:
                self.lib = ctypes.CDLL("libnvidia-ml.so.1")
        except OSError:
            self.lib = None
            return
        try:
            if self.lib.nvmlInit_v2() != 0:
                self.lib = None
                return
            n = ctypes.c_uint(0)
            self.lib.nvmlDeviceGetCount_v2(ctypes.byref(n))
            for i in range(n.value):
                h = ctypes.c_void_p()
                if self.lib.nvmlDeviceGetHandleByIndex_v2(i, ctypes.byref(h)) == 0:
                    name = ctypes.create_string_buffer(96)
                    self.lib.nvmlDeviceGetName(h, name, 96)
                    self.handles.append(h)
                    self.names.append(name.value.decode(errors="replace"))
        except AttributeError:
            self.lib = None
            self.handles = []

    @property
    def available(self) -> bool:
        return self.lib is not None and bool(self.handles)

    def sample(self) -> list[dict]:
        out = []
        for i, h in enumerate(self.handles):
            g = {"index": i, "name": self.names[i], "util_frac": NAN, "mem_util_frac": NAN, "mem_used_mb": NAN,
                 "mem_total_mb": NAN, "encoder_util_frac": NAN, "decoder_util_frac": NAN, "temperature_c": NAN,
                 "power_w": NAN, "pstate": 255, "throttle_reasons": 0}
            u, m = _Util(), _Mem()
            if self.lib.nvmlDeviceGetUtilizationRates(h, ctypes.byref(u)) == 0:
                g["util_frac"], g["mem_util_frac"] = u.gpu / 100.0, u.memory / 100.0
            if self.lib.nvmlDeviceGetMemoryInfo(h, ctypes.byref(m)) == 0:
                g["mem_used_mb"], g["mem_total_mb"] = m.used / MB, m.total / MB
            v, period = ctypes.c_uint(0), ctypes.c_uint(0)
            if self.lib.nvmlDeviceGetEncoderUtilization(h, ctypes.byref(v), ctypes.byref(period)) == 0:
                g["encoder_util_frac"] = v.value / 100.0
            if self.lib.nvmlDeviceGetDecoderUtilization(h, ctypes.byref(v), ctypes.byref(period)) == 0:
                g["decoder_util_frac"] = v.value / 100.0
            if self.lib.nvmlDeviceGetTemperature(h, 0, ctypes.byref(v)) == 0:  # NVML_TEMPERATURE_GPU
                g["temperature_c"] = float(v.value)
            if self.lib.nvmlDeviceGetPowerUsage(h, ctypes.byref(v)) == 0:
                g["power_w"] = v.value / 1000.0
            ps = ctypes.c_int(0)
            if self.lib.nvmlDeviceGetPerformanceState(h, ctypes.byref(ps)) == 0 and 0 <= ps.value <= 15:
                g["pstate"] = ps.value
            r = ctypes.c_ulonglong(0)
            if self.lib.nvmlDeviceGetCurrentClocksThrottleReasons(h, ctypes.byref(r)) == 0:
                g["throttle_reasons"] = int(r.value)
            out.append(g)
        return out


class HostSampler:
    def __init__(self, window_s: float = 1.0, sub_samples: int = 10, log_volume: str | os.PathLike = ".",
                 top_n: int = 3, pins: list[tuple[str, str]] | None = None, gpu: bool = True,
                 py_roots: list[str | os.PathLike] | tuple = ()) -> None:
        if psutil is None:
            raise RuntimeError("psutil is not installed")
        self.window_s = float(window_s)
        self.sub_samples = max(1, int(sub_samples))
        self.log_volume = str(Path(log_volume).resolve().anchor or log_volume) if WIN else str(log_volume)
        self._log_path = str(log_volume)
        self.top_n = int(top_n)
        self.pins = [(n, re.compile(rx, re.I)) for n, rx in (pins if pins is not None else DEFAULT_PINS)]
        self.py_roots = tuple(Path(r) for r in py_roots)  # script roots for py_group (a rig's own nodes)
        self.hostname = socket.gethostname()
        self.gpu = Nvml() if gpu else None
        self._groups: dict[int, tuple] = {}  # pid → (image name, group label or None), cached per pid
        if WIN:
            self.ncpu = win_ncpu()
            self.hz = win_calibrate_hz()
            self.core_class = win_core_classes(self.ncpu)
            self._idle = win_idle_cycles(self.ncpu)
            self._procs = win_processes()
            self._self_cyc = win_self_cycles()
        else:
            self.ncpu = psutil.cpu_count() or 1
            self.hz = None
            self.core_class = []
            psutil.cpu_percent(None, percpu=True)
            self._procs = {}
            self._psproc: dict[int, "psutil.Process"] = {}
            self._self = psutil.Process()
            self._self_cpu = sum(self._self.cpu_times()[:2])
        self._t_procs = time.perf_counter()
        self._cpu_stats = psutil.cpu_stats()
        self._dpc = win_dpc_count() if WIN else None
        self._disk = psutil.disk_io_counters(perdisk=True) or {}
        self._net = {} if WIN else (psutil.net_io_counters(pernic=True) or {})
        self._net_up: set[str] = set()
        self._net_up_t = 0.0
        self._t = time.perf_counter()

    # ── CPU sub-windows ─────────────────────────────────────────────────────────────────────
    def _core_busy(self, dt_s: float) -> list[float]:
        if WIN:
            idle = win_idle_cycles(self.ncpu)
            busy = [max(0.0, min(1.0, 1.0 - (i1 - i0) / (self.hz * dt_s))) for i0, i1 in zip(self._idle, idle)]
            self._idle = idle
            return busy
        return [p / 100.0 for p in psutil.cpu_percent(None, percpu=True)]

    def window(self, stop=None) -> dict:
        """Sample for one window (blocks for window_s) and return the HostStats fields."""
        sub = self.window_s / self.sub_samples
        mean = [0.0] * self.ncpu
        peak = [0.0] * self.ncpu
        tot_sum, tot_peak, late_max = 0.0, 0.0, 0.0
        t_prev = time.perf_counter()
        target = t_prev + sub
        n = 0
        for _ in range(self.sub_samples):
            wait = target - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            now = time.perf_counter()
            if wait > 0:
                late_max = max(late_max, (now - target) * 1000.0)
            target = now + sub  # no catch-up: every sub-window is >= sub (cpu_sampler's rule)
            busy = self._core_busy(now - t_prev)
            t_prev = now
            n += 1
            s = sum(busy)
            tot_sum += s
            tot_peak = max(tot_peak, s)
            for i, b in enumerate(busy[: self.ncpu]):
                mean[i] += b
                if b > peak[i]:
                    peak[i] = b
            if stop is not None and stop():
                break
        mean = [m / n for m in mean]
        rest = self._rest()
        return {"t_ms": int(time.time() * 1000), "window_s": round(self.window_s, 3), "sub_samples": n,
                "hostname": self.hostname,
                "cpu_busy_cores_mean": tot_sum / n, "cpu_busy_cores_peak": tot_peak,
                "core_busy_mean": mean, "core_busy_peak": peak, "core_class": list(self.core_class),
                "sampler_late_ms_max": late_max, **rest}

    # ── everything else, once per window ─────────────────────────────────────────────────────
    def _rest(self) -> dict:
        now = time.perf_counter()
        dt = max(1e-3, now - self._t)
        self._t = now
        cs = psutil.cpu_stats()
        p = self._cpu_stats
        self._cpu_stats = cs
        if WIN:  # psutil reports soft_interrupts = 0 on Windows; the kernel's per-CPU DpcCount instead
            dpc, prev_dpc = win_dpc_count(), self._dpc
            dpcs = (dpc - prev_dpc) / dt if dpc is not None and prev_dpc is not None else NAN
            self._dpc = dpc
        else:
            dpcs = (cs.soft_interrupts - p.soft_interrupts) / dt
        vm = psutil.virtual_memory()
        sw = psutil.swap_memory()
        try:
            free_gb = psutil.disk_usage(self._log_path).free / (1024.0 ** 3)
        except OSError:
            free_gb = NAN
        procs, hard_faults, self_cores = self._processes()
        return {"context_switches_per_s": (cs.ctx_switches - p.ctx_switches) / dt,
                "interrupts_per_s": (cs.interrupts - p.interrupts) / dt,
                "dpcs_per_s": dpcs,
                "mem_total_mb": vm.total / MB, "mem_used_mb": (vm.total - vm.available) / MB,
                "mem_available_mb": vm.available / MB, "pagefile_used_mb": sw.used / MB,
                "hard_page_faults_per_s": hard_faults / dt if hard_faults is not None else NAN,
                "log_volume": self.log_volume, "log_volume_free_gb": free_gb,
                "disks": self._disks(dt), "nets": self._nets(dt),
                "gpus": self.gpu.sample() if self.gpu is not None and self.gpu.available else [],
                "procs": procs, "monitor_cpu_cores": self_cores}

    def _disks(self, dt: float) -> list[dict]:
        cur = psutil.disk_io_counters(perdisk=True) or {}
        prev, self._disk = self._disk, cur
        out = []
        for name, c in sorted(cur.items()):
            q = prev.get(name)
            if q is None:
                continue
            if hasattr(c, "busy_time"):  # Linux
                busy = (c.busy_time - q.busy_time) / (dt * 1000.0)
            else:  # Windows/macOS: read + write time, ms; > 1 with queued I/O → clamped
                busy = ((c.read_time - q.read_time) + (c.write_time - q.write_time)) / (dt * 1000.0)
            out.append({"name": name, "read_mb_per_s": (c.read_bytes - q.read_bytes) / MB / dt,
                        "write_mb_per_s": (c.write_bytes - q.write_bytes) / MB / dt,
                        "reads_per_s": (c.read_count - q.read_count) / dt, "writes_per_s": (c.write_count - q.write_count) / dt,
                        "busy_frac": max(0.0, min(1.0, busy))})
        return out

    def _nets(self, dt: float) -> list[dict]:
        if WIN:
            return self._nets_win(dt)
        now = time.perf_counter()
        if now - self._net_up_t > 10.0:  # adapter up/down list: every 10 s, not every window
            self._net_up_t = now
            try:
                self._net_up = {k for k, v in psutil.net_if_stats().items() if v.isup}
            except OSError:
                self._net_up = set()
        cur = psutil.net_io_counters(pernic=True) or {}
        prev, self._net = self._net, cur
        out = []
        for name, c in sorted(cur.items()):
            q = prev.get(name)
            if q is None or (self._net_up and name not in self._net_up) or "loopback" in name.lower() or re.match(r"^lo\d*$", name):
                continue
            err = (c.errin + c.errout + c.dropin + c.dropout) - (q.errin + q.errout + q.dropin + q.dropout)
            out.append({"name": name, "rx_mb_per_s": (c.bytes_recv - q.bytes_recv) / MB / dt,
                        "tx_mb_per_s": (c.bytes_sent - q.bytes_sent) / MB / dt, "errors": max(0, int(err))})
        return out

    def _nets_win(self, dt: float) -> list[dict]:
        now = time.perf_counter()
        if now - self._net_up_t > 10.0 or not hasattr(self, "_adapters"):
            self._net_up_t = now
            self._adapters = win_adapters()
        out = []
        for idx, name in self._adapters:
            c = win_adapter_counters(idx)
            q = self._net.get(idx)
            self._net[idx] = c
            if c is None or q is None:
                continue
            err = (c[1] + c[2] + c[4] + c[5]) - (q[1] + q[2] + q[4] + q[5])
            out.append({"name": name, "rx_mb_per_s": max(0, c[0] - q[0]) / MB / dt,
                        "tx_mb_per_s": max(0, c[3] - q[3]) / MB / dt, "errors": max(0, int(err))})
        return out

    # ── processes ───────────────────────────────────────────────────────────────────────────
    def classify(self, pid: int, name: str) -> str | None:
        hit = self._groups.get(pid)
        if hit is not None and hit[0] == name:  # cached per pid; a reused pid has another image name
            return hit[1]
        g = None
        if PY_EXE.match(name or ""):
            try:
                g = py_group(psutil.Process(pid).cmdline(), self.py_roots)
            except Exception:  # AccessDenied / gone
                g = None
        else:
            for label, rx in self.pins:
                if rx.search(name or ""):
                    g = label
                    break
        self._groups[pid] = (name, g)
        return g

    def _processes(self) -> tuple[list[dict], float | None, float]:
        """procs rows (top N by CPU, memory, disk I/O + one row per pinned group), the system's hard
        faults in the window (Σ per process; Windows), and this monitor's own cores."""
        now = time.perf_counter()
        dt = max(1e-3, now - self._t_procs)
        self._t_procs = now
        me = os.getpid()
        rows = []  # (pid, name, cpu cores, rss MB, read MB/s, write MB/s)
        hard = None
        if WIN:
            cur = win_processes()
            prev, self._procs = self._procs, cur
            hard = 0
            for pid, (name, cyc, ws, rd, wr, hf) in cur.items():
                q = prev.get(pid)
                if pid == 0 or q is None or q[0] != name:
                    continue
                hard += max(0, hf - q[5])
                rows.append((pid, name, max(0.0, (cyc - q[1]) / (self.hz * dt)), ws / MB,
                             max(0, rd - q[3]) / MB / dt, max(0, wr - q[4]) / MB / dt))
            c = win_self_cycles()
            self_cores = (c - self._self_cyc) / (self.hz * dt)
            self._self_cyc = c
        else:
            seen = set()
            for p in psutil.process_iter(["name"]):
                seen.add(p.pid)
                q = self._psproc.setdefault(p.pid, p)
                try:
                    with q.oneshot():
                        cpu = q.cpu_percent(None) / 100.0
                        rss = q.memory_info().rss / MB
                except Exception:
                    continue
                rows.append((p.pid, p.info.get("name") or "", cpu, rss, NAN, NAN))
            for pid in list(self._psproc):
                if pid not in seen:
                    self._psproc.pop(pid, None)
            t = sum(self._self.cpu_times()[:2])
            self_cores = (t - self._self_cpu) / dt
            self._self_cpu = t
        live = {r[0] for r in rows}
        for pid in [p for p in self._groups if p not in live]:
            self._groups.pop(pid, None)
        return self._select(rows, me), hard, max(0.0, self_cores)

    def _select(self, rows: list[tuple], me: int) -> list[dict]:
        out: dict = {}

        def row(pid, name, group, cpu, rss, rd, wr, reason):
            key = ("g", group) if group else ("p", pid)
            r = out.get(key)
            if r is None:
                out[key] = {"pid": pid, "name": name, "group": group or "", "cpu_cores": cpu, "rss_mb": rss,
                            "io_read_mb_per_s": rd, "io_write_mb_per_s": wr, "reasons": reason}
            else:
                r["reasons"] |= reason

        groups: dict[str, list] = {}
        for r in rows:
            g = "py:host_monitor" if r[0] == me else self.classify(r[0], r[1])
            if g:
                groups.setdefault(g, []).append(r)
        for g, rs in groups.items():
            busiest = max(rs, key=lambda r: r[2])
            row(busiest[0], busiest[1], g, sum(r[2] for r in rs), sum(r[3] for r in rs),
                _nansum(r[4] for r in rs), _nansum(r[5] for r in rs), REASON_PINNED)
        pid_group = {r[0]: g for g, rs in groups.items() if len(rs) == 1 for r in rs}
        for reason, key in ((REASON_CPU, lambda r: r[2]), (REASON_MEM, lambda r: r[3]),
                            (REASON_IO, lambda r: (r[4] if math.isfinite(r[4]) else 0) + (r[5] if math.isfinite(r[5]) else 0))):
            for r in sorted(rows, key=key, reverse=True)[: self.top_n]:
                if reason == REASON_IO and key(r) <= 0:
                    continue
                g = pid_group.get(r[0])
                if g:  # a one-process pinned group: the group row IS this process
                    out[("g", g)]["reasons"] |= reason
                else:
                    row(r[0], r[1], "", r[2], r[3], r[4], r[5], reason)
        return sorted(out.values(), key=lambda d: -d["cpu_cores"])


def _nansum(vals) -> float:
    vals = [v for v in vals if math.isfinite(v)]
    return sum(vals) if vals else NAN


def to_json_safe(obj):
    """NaN → None, recursively (the String-JSON form must stay strict JSON)."""
    if isinstance(obj, float):
        return None if not math.isfinite(obj) else obj
    if isinstance(obj, dict):
        return {k: to_json_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [to_json_safe(v) for v in obj]
    return obj


def summary_line(s: dict) -> str:
    """One console line for a sample: the pressure at a glance."""
    top = ", ".join(f"{p['group'] or p['name']} {p['cpu_cores']:.2f}" for p in s["procs"][:3])
    gpu = s["gpus"][0] if s["gpus"] else None
    disk = sum(d["read_mb_per_s"] + d["write_mb_per_s"] for d in s["disks"])
    net = sum(n["rx_mb_per_s"] + n["tx_mb_per_s"] for n in s["nets"])
    return (f"CPU {s['cpu_busy_cores_mean']:.2f} cores (peak {s['cpu_busy_cores_peak']:.2f}, busiest core "
            f"{max(s['core_busy_peak'] or [0]) * 100:.0f} %), late {s['sampler_late_ms_max']:.1f} ms · RAM "
            f"{s['mem_used_mb'] / 1024:.1f}/{s['mem_total_mb'] / 1024:.0f} GB · disk {disk:.1f} MB/s · net {net:.2f} MB/s"
            + (f" · GPU {gpu['util_frac'] * 100:.0f} % enc {gpu['encoder_util_frac'] * 100:.0f} %" if gpu else "")
            + f" · top {top} · monitor {s['monitor_cpu_cores'] * 100:.2f} % of a core")


if __name__ == "__main__":  # quick look: python host_sampler.py [n]
    import json

    hs = HostSampler()
    for _ in range(int(sys.argv[1]) if len(sys.argv) > 1 else 3):
        s = hs.window()
        print(summary_line(s))
    print(json.dumps(to_json_safe(s), indent=1)[:4000])
