#!/usr/bin/env python3
"""Offline tests for rig_nodes.host_sampler (ex webDisplayTools tests/test-host-sampler.py) — the rig's host-pressure sampler (no ROS, no psutil
needed: a fake psutil with known counters drives the portable code path), wired into the root
`pixi run test`. Checks:
  (a) the CONTRACT: every sample field, and every disks/nets/gpus/procs field, is exactly the
      `.msg` definition in rig_interfaces/msg/*.msg, with a matching Python type
  (b) rates from counter deltas (disk MB/s, IOPS, busy, network MB/s + errors, context switches)
  (c) CPU: window mean vs busiest sub-window, per core
  (d) processes: pinned groups summed into one row, top-N by CPU / memory / I/O with reason bits,
      a one-process group merged with its top-N row, py:<script> for this repo's Python processes
  (e) JSON form: NaN → null, strict JSON; the console summary line

Run: python tests/test-host-sampler.py
"""
import collections
import json
import math
import os
import re
import sys
import types

import tempfile

import rig_nodes.host_sampler as H  # the installed package (the acceptance job) or the in-place build

HERE = os.path.dirname(os.path.abspath(__file__))
# The .msg files: the installed rig_interfaces share directory first (what a rig has), else the source tree.
MSG_DIRS = []
try:
    from ament_index_python import get_package_share_directory

    MSG_DIRS.append(os.path.join(get_package_share_directory("rig_interfaces"), "msg"))
except Exception:
    pass
MSG_DIRS.append(os.path.join(HERE, "..", "src", "rig_interfaces", "msg"))
MSG_DIR = next((d for d in MSG_DIRS if os.path.isdir(d)), None)
# A stand-in for a rig workspace whose scripts py_group pins (the arena rig passes `--py-root <repo>/rig`).
WORK = tempfile.mkdtemp(prefix="host-sampler-test-")
os.makedirs(os.path.join(WORK, "nodes"))
open(os.path.join(WORK, "nodes", "bridge_node.py"), "w").close()

total = 0
failures = 0


def check(name, got, expected):
    global total, failures
    total += 1
    if got == expected:
        print(f"  ok   {name}")
    else:
        failures += 1
        print(f"  FAIL {name}: got {got!r}, expected {expected!r}")


# ── the fake psutil: counters that advance by fixed steps per call ────────────────────────────
class FakePsutil(types.SimpleNamespace):
    pass


CpuStats = collections.namedtuple("CpuStats", "ctx_switches interrupts soft_interrupts syscalls")
Disk = collections.namedtuple("Disk", "read_count write_count read_bytes write_bytes read_time write_time")
Nic = collections.namedtuple("Nic", "bytes_sent bytes_recv packets_sent packets_recv errin errout dropin dropout")
VM = collections.namedtuple("VM", "total available percent")
Swap = collections.namedtuple("Swap", "used")
Usage = collections.namedtuple("Usage", "free")
IfStat = collections.namedtuple("IfStat", "isup")
Mem = collections.namedtuple("Mem", "rss")
CALLS = collections.Counter()


def _step(name):
    CALLS[name] += 1
    return CALLS[name]


class FakeProc:
    TABLE = {}  # pid → (name, cmdline, cpu %, rss MB)

    def __init__(self, pid=None):
        self.pid = os.getpid() if pid is None else pid
        name = self.TABLE.get(self.pid, ("python.exe", ["python"], 0.0, 1.0))[0]
        self.info = {"name": name}

    def oneshot(self):
        import contextlib

        return contextlib.nullcontext()

    def cpu_percent(self, _):
        _step(("cpu", self.pid))
        return self.TABLE[self.pid][2]

    def memory_info(self):
        return Mem(int(self.TABLE[self.pid][3] * H.MB))

    def cmdline(self):
        return self.TABLE[self.pid][1]

    def cpu_times(self):
        k = _step("self")
        return (0.001 * k, 0.0)


fake = FakePsutil(
    cpu_count=lambda: 4,
    cpu_percent=lambda _i, percpu=True: [10.0, 20.0, 30.0, 40.0] if _step("cpu") % 2 else [30.0, 20.0, 10.0, 80.0],
    cpu_stats=lambda: (lambda k: CpuStats(1000 * k, 500 * k, 50 * k, 0))(_step("stats")),
    virtual_memory=lambda: VM(16 * 1024 * H.MB, 6 * 1024 * H.MB, 62.5),
    swap_memory=lambda: Swap(512 * H.MB),
    disk_usage=lambda p: Usage(100 * 1024 ** 3),
    disk_io_counters=lambda perdisk=True: (lambda k: {"disk0": Disk(10 * k, 20 * k, int(2 * H.MB) * k, int(4 * H.MB) * k, 100 * k, 200 * k)})(_step("disk")),
    net_io_counters=lambda pernic=True: (lambda k: {"en0": Nic(int(H.MB) * k, 3 * int(H.MB) * k, 0, 0, k, 0, 0, 0),
                                                    "lo0": Nic(9 * k, 9 * k, 0, 0, 0, 0, 0, 0)})(_step("net")),
    net_if_stats=lambda: {"en0": IfStat(True), "lo0": IfStat(True)},
    Process=FakeProc,
    process_iter=lambda attrs=None: (_step("iter"), [FakeProc(pid) for pid in FakeProc.TABLE])[1],
)
me = os.getpid()
script = os.path.join(WORK, "nodes", "bridge_node.py")
FakeProc.TABLE = {
    me: ("python.exe", ["python", "host_monitor.py"], 1.0, 40.0),
    101: ("fictrac.exe", [], 78.0, 300.0),
    102: ("chrome.exe", [], 5.0, 800.0),
    103: ("chrome.exe", [], 3.0, 900.0),
    104: ("python.exe", ["python", script, "--log-dir", "x"], 12.0, 75.0),
    105: ("python.exe", ["python", "/elsewhere/tool.py"], 50.0, 30.0),
    106: ("bigmem.exe", [], 0.5, 5000.0),
    107: ("idle.exe", [], 0.0, 1.0),
}
H.psutil = fake
H.WIN = False  # the portable path, on every OS
hs = H.HostSampler(window_s=0.1, sub_samples=2, log_volume=HERE, top_n=2, gpu=False, py_roots=[WORK])
hs.window()
s = hs.window()

print(f"(a) contract vs rig_interfaces/msg/*.msg ({MSG_DIR})")
msgs = {}
for name in ("DiskStats", "GpuStats", "HostStats", "NetStats", "ProcStats"):
    path = MSG_DIR and os.path.join(MSG_DIR, name + ".msg")
    if not path or not os.path.exists(path):
        continue
    body = open(path, encoding="utf-8").read()
    fields = []
    for line in body.splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            typ, fname = line.split()[:2]
            fields.append((typ, fname))
    msgs[name] = fields
check("five messages in rig_interfaces/msg", sorted(msgs), ["DiskStats", "GpuStats", "HostStats", "NetStats", "ProcStats"])


def pytype_ok(typ, v):
    if typ.endswith("[]"):
        return isinstance(v, list)
    if typ.startswith("float"):
        return isinstance(v, float)
    if typ.startswith(("int", "uint")):
        return isinstance(v, int) and not isinstance(v, bool)
    if typ == "string":
        return isinstance(v, str)
    return False


def conforms(msg, d):
    names = [f for _, f in msgs[msg]]
    return sorted(d) == sorted(names) and all(pytype_ok(t, d[f]) for t, f in msgs[msg])


check("HostStats: exactly the spec's fields and types", conforms("HostStats", s), True)
check("DiskStats", bool(s["disks"]) and all(conforms("DiskStats", d) for d in s["disks"]), True)
check("NetStats", bool(s["nets"]) and all(conforms("NetStats", d) for d in s["nets"]), True)
check("ProcStats", bool(s["procs"]) and all(conforms("ProcStats", d) for d in s["procs"]), True)


class FakeNvml:  # every NVML call fails → the documented absent values
    def __getattr__(self, _):
        return lambda *a: 999


g = H.Nvml.__new__(H.Nvml)
g.lib, g.handles, g.names = FakeNvml(), [object()], ["Fake GPU"]
gs = g.sample()
check("GpuStats (every NVML call failing): spec fields, NaN floats, pstate 255", conforms("GpuStats", gs[0]) and math.isnan(gs[0]["util_frac"]) and gs[0]["pstate"] == 255, True)
check("Nvml() never raises (driver present or not)", isinstance(H.Nvml().available, bool), True)

print("(b) rates")
d = s["disks"][0]
rate = d["reads_per_s"] / 10  # = 1 / the window the sampler measured (reads advance by 10 per window)
check("disk MB/s = Δbytes / Δt", (round(d["read_mb_per_s"] / rate, 3), round(d["write_mb_per_s"] / rate, 3)), (2.0, 4.0))
check("disk busy = (Δread_time + Δwrite_time) / window, clamped to 1", d["busy_frac"], 1.0)
n = s["nets"]
check("loopback adapter skipped", [x["name"] for x in n], ["en0"])
check("net rx/tx MB/s, errors in the window", (round(n[0]["rx_mb_per_s"] / rate, 3), round(n[0]["tx_mb_per_s"] / rate, 3), n[0]["errors"]), (3.0, 1.0, 1))
check("context switches per s", round(s["context_switches_per_s"] / rate), 1000)
check("memory used / available / page file", (s["mem_used_mb"], s["mem_available_mb"], s["pagefile_used_mb"]), (10240.0, 6144.0, 512.0))
check("hard faults absent off Windows → NaN", math.isnan(s["hard_page_faults_per_s"]), True)
check("log volume free space", s["log_volume_free_gb"], 100.0)

print("(c) CPU mean vs peak")
check("per-core mean of the two sub-windows", [round(x, 3) for x in s["core_busy_mean"]], [0.2, 0.2, 0.2, 0.6])
check("per-core peak", s["core_busy_peak"], [0.3, 0.2, 0.3, 0.8])
check("whole-PC mean / peak (cores)", (round(s["cpu_busy_cores_mean"], 3), round(s["cpu_busy_cores_peak"], 3)), (1.2, 1.4))
check("sub_samples", s["sub_samples"], 2)

print("(d) processes")
by = {(p["group"] or p["name"]): p for p in s["procs"]}
check("pinned: fictrac row", by["fictrac"]["cpu_cores"], 0.78)
check("browser = ONE summed row for its two processes", (by["browser"]["cpu_cores"], by["browser"]["rss_mb"], by["browser"]["pid"]), (0.08, 1700.0, 102))
check("this repo's Python script → py:bridge_node", "py:bridge_node" in by, True)
check("the monitor itself → py:host_monitor", "py:host_monitor" in by, True)
check("a Python script outside the repo is not pinned, but tops CPU", (by["python.exe"]["group"], by["python.exe"]["reasons"]), ("", H.REASON_CPU))
check("one-process group merged with its top-N row (fictrac: pinned + top CPU)", by["fictrac"]["reasons"], H.REASON_PINNED | H.REASON_CPU)
check("top memory picks the big process", by["bigmem.exe"]["reasons"] & H.REASON_MEM, H.REASON_MEM)
check("an idle unpinned process is left out", "idle.exe" in by, False)
check("rows sorted by CPU", [p["cpu_cores"] for p in s["procs"]] == sorted((p["cpu_cores"] for p in s["procs"]), reverse=True), True)
check("py_group: -m rig_nodes.<module>", H.py_group(["python", "-m", "rig_nodes.fictrac_udp_adapter"]), "py:fictrac_udp_adapter")
check("py_group: relative nodes/ script under a --py-root (pixi task cwd = rig/)", H.py_group(["python", "nodes/bridge_node.py"], (H.Path(WORK),)), "py:bridge_node")
check("py_group: a script outside every root is not pinned", H.py_group(["python", "/elsewhere/tool.py"], (H.Path(WORK),)), None)
check("py_group: other module", H.py_group(["python", "-m", "http.server"]), None)

print("(e) JSON + summary")
s["gpus"] = gs
j = json.dumps(H.to_json_safe(s), allow_nan=False)
check("strict JSON (NaN → null)", json.loads(j)["hard_page_faults_per_s"], None)
line = H.summary_line(s)
check("summary line", line.startswith("CPU 1.20 cores (peak 1.40, busiest core 80 %)") and "top fictrac 0.78" in line and "GPU" in line, True)

print("(f) typed quantization (host_sampler.quantize, applied by host_monitor.to_msg)")
import struct  # noqa: E402

before = json.dumps(H.to_json_safe(s), sort_keys=True)
qs = H.quantize(s)
f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]  # noqa: E731


def on_step(v, step):
    return isinstance(v, float) and (math.isnan(v) or abs(v / step - round(v / step)) < 1e-9)


check("steps are 1/1024 and 1/64", (H.FRAC_STEP, H.RATE_STEP), (1 / 1024, 1 / 64))
check("per-core fractions on the 1/1024 grid", all(on_step(v, H.FRAC_STEP) for v in qs["core_busy_mean"] + qs["core_busy_peak"]), True)
check("cores rounded to nearest 1/1024", qs["cpu_busy_cores_mean"], round(s["cpu_busy_cores_mean"] * 1024) / 1024)
check("rates / MB on the 1/64 grid (top level, disks, nets, procs)",
      all(on_step(v, H.RATE_STEP) for v in [qs["context_switches_per_s"], qs["mem_used_mb"], qs["log_volume_free_gb"]]
          + [d["read_mb_per_s"] for d in qs["disks"]] + [n["rx_mb_per_s"] for n in qs["nets"]] + [p["rss_mb"] for p in qs["procs"]]), True)
check("process CPU on the 1/1024 grid", all(on_step(p["cpu_cores"], H.FRAC_STEP) for p in qs["procs"]), True)
check("error ≤ half a step", abs(qs["mem_used_mb"] - s["mem_used_mb"]) <= H.RATE_STEP / 2 and abs(qs["cpu_busy_cores_peak"] - s["cpu_busy_cores_peak"]) <= H.FRAC_STEP / 2, True)
check("NaN stays NaN (absent value)", math.isnan(qs["hard_page_faults_per_s"]), True)
check("ints and strings untouched", (qs["sub_samples"], qs["t_ms"], qs["hostname"], [n["errors"] for n in qs["nets"]], [p["pid"] for p in qs["procs"]]),
      (s["sub_samples"], s["t_ms"], s["hostname"], [n["errors"] for n in s["nets"]], [p["pid"] for p in s["procs"]]))
check("the caller's sample is not mutated (nested rows included)", json.dumps(H.to_json_safe(s), sort_keys=True) == before and qs != s, True)
gq = H.quantize({"gpus": gs})["gpus"][0]
check("gpu fractions / NaN handled", math.isnan(gq["util_frac"]) and gq["pstate"] == 255, True)
check("quantized values survive float32 exactly (what the CDR carries)",
      all(f32(v) == v for v in qs["core_busy_mean"] + [qs["mem_used_mb"], qs["context_switches_per_s"]]), True)

print("(g) portable path: full process table every proc_rescan_s, watched pids in between")
check("default rescan period 5 s", hs.proc_rescan_s, 5.0)
check("two windows -> ONE full table read (the first)", CALLS["iter"], 1)
check("an unwatched process (idle.exe) was read only at the full table", CALLS[("cpu", 107)], 1)
check("a pinned process (fictrac) is read every window", CALLS[("cpu", 101)], 2)
check("watched = every pinned pid (both browser processes) + the top-N + the monitor",
      {101, 102, 103, 104, me} <= hs._watch and 107 not in hs._watch, True)
hs._ps_scan_t -= hs.proc_rescan_s  # the period has passed
s3 = hs.window()
check("after the period: the whole table again", (CALLS["iter"], CALLS[("cpu", 107)]), (2, 2))
check("in-between windows give the same rows as a full read",
      sorted(p["pid"] for p in s3["procs"]) == sorted(p["pid"] for p in s["procs"]), True)

print(f"\n{total - failures}/{total} passed")
sys.exit(1 if failures else 0)
