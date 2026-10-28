#!/usr/bin/env python3
"""thorsipm.py — READ-ONLY driver for the Thorlabs SiPM controller (Bergamo rig), no ROS.

EXPERIMENTAL (rig-nodes 0.3.0, LAB-319): never run against the real controller; Windows-only ctypes
binding of an unreleased Thorlabs DLL. Off-rig everything uses `FakeBackend`.

The controller's software is `ThorSiPM_Control.exe` (an unreleased .NET build, 2025-09-25) on top
of a native `ThorSiPM.dll` that talks to the controller over a serial port (COM66 on Bergamo, set
in `ThorSiPMSettings.xml` next to the DLL). The DLL has ThorImage's standard device interface;
the signatures below are the GUI's own P/Invoke declarations, read from its .NET metadata
(scanimage-2026-bergamo, 2026-10-19; nothing decompiled):

    int FindDevices(ref int deviceCount)            int SelectDevice(int device)
    int GetParam(int paramID, ref double value)     int GetParamString(int paramID, char* buf, int size)
    int GetParamInfo(int paramID, ref int type, ref int available, ref int readOnly,
                     ref double min, ref double max, ref double default)
    int TeardownDevice()                            (all return 1 = TRUE on success)

This module wraps ONLY those. It has no SetParam / Start/Setup/Preflight call anywhere, so it
cannot change a detector setting. The parameter IDs in KNOWN_PARAMS are the GUI's `Params` enum;
the SiPM-specific ones the native DLL also names (PARAM_SIPM1_BIAS, …_TEMPERATURE, …_TEMPSTATUS,
…_OUTPUT_OFFSET_HIGH_GAIN/LOW_GAIN) have no ID in that enum, so `scan` finds them on the rig:
every ID whose GetParamInfo says "available", with its value — then match against the GUI.

Only ONE program can hold the COM port: close ThorSiPM_Control.exe first (Block S, step S1).
The DLL opens `ThorSiPMSettings.xml` relative to the working directory, so `ThorSiPM.open()`
switches the process's working directory to the DLL folder (the node is a process of its own).

    python -m rig_nodes.thorsipm scan     --dll-dir "C:\\…\\ThorSiPM\\Release" --out sipm_scan.json
    python -m rig_nodes.thorsipm snapshot --dll-dir …  [--params-file sipm_params.json]
    python -m rig_nodes.thorsipm watch    --dll-dir … --period 1 --duration 60
    python -m rig_nodes.thorsipm scan --fake            # off-rig: the simulated controller below
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys
import time
from pathlib import Path

DLL_NAME = "ThorSiPM.dll"
MAX_DETECTORS = 4

# The GUI's `Params` enum (ThorSiPM_Control.dll metadata), the subset this controller can answer.
KNOWN_PARAMS: dict[int, str] = {
    700: "PARAM_PMT1_GAIN_POS", 701: "PARAM_PMT1_ENABLE", 702: "PARAM_PMT2_GAIN_POS", 703: "PARAM_PMT2_ENABLE",
    704: "PARAM_PMT3_GAIN_POS", 705: "PARAM_PMT3_ENABLE", 706: "PARAM_PMT4_GAIN_POS", 707: "PARAM_PMT4_ENABLE",
    713: "PARAM_PMT1_SAFETY", 714: "PARAM_PMT2_SAFETY", 715: "PARAM_PMT3_SAFETY", 716: "PARAM_PMT4_SAFETY",
    722: "PARAM_PMT1_BANDWIDTH_POS", 723: "PARAM_PMT2_BANDWIDTH_POS", 724: "PARAM_PMT3_BANDWIDTH_POS", 725: "PARAM_PMT4_BANDWIDTH_POS",
    743: "PARAM_CONTROL_UNIT_FIRMWAREUPDATE", 744: "PARAM_CONTROL_UNIT_FIRMWAREVERSION", 745: "PARAM_CONTROL_UNIT_SERIALNUMBER",
    750: "PARAM_PMT1_GAIN_POS_CURRENT_VOLTS", 751: "PARAM_PMT1_OUTPUT_OFFSET", 752: "PARAM_PMT1_OUTPUT_OFFSET_CURRENT", 753: "PARAM_PMT1_SERIALNUMBER",
    754: "PARAM_PMT2_GAIN_POS_CURRENT_VOLTS", 755: "PARAM_PMT2_OUTPUT_OFFSET", 756: "PARAM_PMT2_OUTPUT_OFFSET_CURRENT", 757: "PARAM_PMT2_SERIALNUMBER",
    758: "PARAM_PMT3_GAIN_POS_CURRENT_VOLTS", 759: "PARAM_PMT3_OUTPUT_OFFSET", 760: "PARAM_PMT3_OUTPUT_OFFSET_CURRENT", 761: "PARAM_PMT3_SERIALNUMBER",
    762: "PARAM_PMT4_GAIN_POS_CURRENT_VOLTS", 763: "PARAM_PMT4_OUTPUT_OFFSET", 764: "PARAM_PMT4_OUTPUT_OFFSET_CURRENT", 765: "PARAM_PMT4_SERIALNUMBER",
    782: "PARAM_CONNECTED_PMTS",
    783: "PARAM_PMT1_FIRMWAREUPDATE", 784: "PARAM_PMT2_FIRMWAREUPDATE", 785: "PARAM_PMT3_FIRMWAREUPDATE", 786: "PARAM_PMT4_FIRMWAREUPDATE",
    789: "PARAM_PMT1_TYPE", 790: "PARAM_PMT2_TYPE", 791: "PARAM_PMT3_TYPE", 792: "PARAM_PMT4_TYPE",
    920: "PARAM_PMT1_BANDWIDTH_POS_CURRENT", 921: "PARAM_PMT2_BANDWIDTH_POS_CURRENT", 922: "PARAM_PMT3_BANDWIDTH_POS_CURRENT", 923: "PARAM_PMT4_BANDWIDTH_POS_CURRENT",
    926: "PARAM_PMT1_SATURATIONS", 927: "PARAM_PMT2_SATURATIONS", 928: "PARAM_PMT3_SATURATIONS", 929: "PARAM_PMT4_SATURATIONS",
    932: "PARAM_PMT1_DETECTOR_TYPE", 933: "PARAM_PMT2_DETECTOR_TYPE", 934: "PARAM_PMT3_DETECTOR_TYPE", 935: "PARAM_PMT4_DETECTOR_TYPE",
    938: "PARAM_PMT1_OFFSET_STEP_SIZE", 939: "PARAM_PMT2_OFFSET_STEP_SIZE", 940: "PARAM_PMT3_OFFSET_STEP_SIZE", 941: "PARAM_PMT4_OFFSET_STEP_SIZE",
    944: "PARAM_PMT1_GAIN_STEP_SIZE", 945: "PARAM_PMT2_GAIN_STEP_SIZE", 946: "PARAM_PMT3_GAIN_STEP_SIZE", 947: "PARAM_PMT4_GAIN_STEP_SIZE",
    950: "PARAM_PMT1_AVAILABLE_BANDWIDTHS", 951: "PARAM_PMT2_AVAILABLE_BANDWIDTHS", 952: "PARAM_PMT3_AVAILABLE_BANDWIDTHS", 953: "PARAM_PMT4_AVAILABLE_BANDWIDTHS",
    956: "PARAM_PMT1_FIRMWAREVERSION", 957: "PARAM_PMT2_FIRMWAREVERSION", 958: "PARAM_PMT3_FIRMWAREVERSION", 959: "PARAM_PMT4_FIRMWAREVERSION",
}
# Never READ these either, even in a scan: their names say "firmware update", and a vendor
# GetParam with a side effect is not worth the risk.
NEVER_READ = frozenset({743, 783, 784, 785, 786})
# Read with GetParamString instead of GetParam.
STRING_PARAMS = frozenset(i for i, n in KNOWN_PARAMS.items() if n.endswith(("SERIALNUMBER", "FIRMWAREVERSION")))


def detector_params(n: int) -> dict[str, int]:
    """The status fields for detector n (1-based) → param ID, from the GUI enum."""
    base = {"enable": 701, "gain_pos": 700, "gain_volts": 750, "output_offset": 751, "output_offset_current": 752,
            "bandwidth_pos": 722, "bandwidth_pos_current": 920, "safety": 713, "saturations": 926, "detector_type": 932}
    step = {"enable": 2, "gain_pos": 2, "gain_volts": 4, "output_offset": 4, "output_offset_current": 4,
            "bandwidth_pos": 1, "bandwidth_pos_current": 1, "safety": 1, "saturations": 1, "detector_type": 1}
    return {k: v + step[k] * (n - 1) for k, v in base.items()}


class SiPMError(RuntimeError):
    pass


# ── backends: the real DLL (Windows) and a simulated controller (tests, off-rig) ─────────────────

class DllBackend:
    """ctypes binding of ThorSiPM.dll. Windows x64 only (the DLL is PE32+)."""

    def __init__(self, dll_dir: str | os.PathLike) -> None:
        if os.name != "nt":
            raise SiPMError("ThorSiPM.dll is a Windows DLL; use --fake off the rig")
        self.dll_dir = Path(dll_dir).resolve()
        if not (self.dll_dir / DLL_NAME).is_file():
            raise SiPMError(f"{DLL_NAME} not found in {self.dll_dir}")
        os.add_dll_directory(str(self.dll_dir))  # SerialPortAccess.dll sits next to it
        d = ctypes.CDLL(str(self.dll_dir / DLL_NAME))
        I, D, P = ctypes.c_int32, ctypes.c_double, ctypes.POINTER
        sig = {
            "FindDevices": [P(I)], "SelectDevice": [I], "TeardownDevice": [],
            "GetParam": [I, P(D)], "GetParamString": [I, ctypes.c_char_p, I],
            "GetParamInfo": [I, P(I), P(I), P(I), P(D), P(D), P(D)],
        }
        for name, argtypes in sig.items():
            f = getattr(d, name)
            f.argtypes, f.restype = argtypes, I
        self._d = d

    def find_devices(self) -> tuple[int, int]:
        n = ctypes.c_int32(0)
        return self._d.FindDevices(ctypes.byref(n)), n.value

    def select_device(self, i: int) -> int:
        return self._d.SelectDevice(i)

    def teardown(self) -> int:
        return self._d.TeardownDevice()

    def get_param(self, pid: int) -> tuple[int, float]:
        v = ctypes.c_double(0.0)
        return self._d.GetParam(pid, ctypes.byref(v)), v.value

    def get_param_string(self, pid: int, size: int = 256) -> tuple[int, str]:
        # The DLL writes UTF-16LE (wchar_t), not ANSI: "000027" arrives as 30 00 30 00 … (Bergamo Block S,
        # 2026-10-28). The buffer is 2 × size bytes in case `size` counts wide characters.
        buf = ctypes.create_string_buffer(2 * size)
        rc = self._d.GetParamString(pid, buf, size)
        return rc, buf.raw.decode("utf-16-le", errors="replace").split("\x00", 1)[0]

    def get_param_info(self, pid: int) -> tuple[int, dict]:
        t, a, ro = ctypes.c_int32(), ctypes.c_int32(), ctypes.c_int32()
        lo, hi, df = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        rc = self._d.GetParamInfo(pid, *(ctypes.byref(x) for x in (t, a, ro, lo, hi, df)))
        return rc, {"type": t.value, "available": a.value, "read_only": ro.value, "min": lo.value, "max": hi.value, "default": df.value}


class FakeBackend:
    """A simulated controller: two SiPMs (the Bergamo serial numbers) plus 'hidden' SiPM params
    at IDs the GUI enum doesn't name, the way the real bias/temperature IDs are unknown."""

    HIDDEN = {1201: 54.2, 1202: 54.6, 1211: 21.8, 1212: 22.3, 1221: 0.0, 1222: 0.0}  # bias V ×2, temp °C ×2, temp status ×2

    def __init__(self, busy: bool = False, n_detectors: int = 2) -> None:
        self.busy = busy  # True = the GUI holds the COM port: SelectDevice fails
        self.selected = False
        self.reads = 0
        self.vals: dict[int, float] = {782: n_detectors}
        self.strs: dict[int, str] = {744: "1.2.3", 745: "CU000123"}
        for n, sn in zip(range(1, n_detectors + 1), ("000027", "000026")):
            p = detector_params(n)
            self.vals.update({p["enable"]: 1, p["gain_pos"]: 0.80, p["gain_volts"]: 0.80, p["output_offset"]: 0.0,
                              p["output_offset_current"]: 0.0, p["bandwidth_pos"]: 2, p["bandwidth_pos_current"]: 2,
                              p["safety"]: 0, p["saturations"]: 0, p["detector_type"]: 3})
            self.strs[753 + 4 * (n - 1)] = sn
            self.strs[956 + (n - 1)] = "5.0.1"
        self.vals.update(self.HIDDEN)

    def find_devices(self) -> tuple[int, int]:
        return 1, 1

    def select_device(self, i: int) -> int:
        if self.busy:
            return 0
        self.selected = True
        return 1

    def teardown(self) -> int:
        self.selected = False
        return 1

    def _ok(self, pid: int) -> bool:
        return self.selected and (pid in self.vals or pid in self.strs or pid in NEVER_READ)

    def get_param(self, pid: int) -> tuple[int, float]:
        self.reads += 1
        if not self.selected or pid not in self.vals:
            return 0, 0.0
        v = self.vals[pid]
        if pid in (1211, 1212):  # a temperature that drifts a little
            v += 0.01 * (self.reads % 7)
        return 1, float(v)

    def get_param_string(self, pid: int, size: int = 256) -> tuple[int, str]:
        return (1, self.strs[pid][: size - 1]) if self.selected and pid in self.strs else (0, "")

    def get_param_info(self, pid: int) -> tuple[int, dict]:
        if not self._ok(pid):
            return 0, {"type": 0, "available": 0, "read_only": 0, "min": 0.0, "max": 0.0, "default": 0.0}
        t = 2 if pid in self.strs else 1
        return 1, {"type": t, "available": 1, "read_only": int(pid in self.strs or pid in self.HIDDEN), "min": 0.0, "max": 100.0, "default": 0.0}


# ── the driver ────────────────────────────────────────────────────────────────────────────────────

class ThorSiPM:
    """Read-only access to one SiPM controller. `open()` → reads → `close()` (frees the COM port)."""

    def __init__(self, backend) -> None:
        self.b = backend
        self.is_open = False
        self.n_devices = 0

    @classmethod
    def from_args(cls, dll_dir: str | None, fake: bool = False, fake_busy: bool = False) -> "ThorSiPM":
        if fake:
            return cls(FakeBackend(busy=fake_busy))
        if not dll_dir:
            raise SiPMError("--dll-dir is required (the folder with ThorSiPM.dll and ThorSiPMSettings.xml)")
        return cls(DllBackend(dll_dir))

    def open(self, device: int = 0) -> int:
        dll_dir = getattr(self.b, "dll_dir", None)
        if dll_dir is not None:
            os.chdir(dll_dir)  # the DLL reads ThorSiPMSettings.xml from the working directory
        rc, n = self.b.find_devices()
        if rc != 1 or n < 1:
            raise SiPMError(f"FindDevices found no controller (rc={rc}, count={n}): is it powered and on the COM port in ThorSiPMSettings.xml?")
        if self.b.select_device(device) != 1:
            raise SiPMError("SelectDevice failed: the COM port is probably held by ThorSiPM_Control.exe (close it), or the controller is off")
        self.is_open, self.n_devices = True, n
        return n

    def close(self) -> None:
        if self.is_open:
            self.b.teardown()
            self.is_open = False

    def __enter__(self) -> "ThorSiPM":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _need_open(self) -> None:
        if not self.is_open:
            raise SiPMError("not connected")

    def get(self, pid: int) -> float | str | None:
        """One parameter's value, or None if the DLL refuses it. Strings via GetParamString."""
        self._need_open()
        if pid in NEVER_READ:
            raise SiPMError(f"param {pid} ({KNOWN_PARAMS.get(pid)}) is never read (firmware update)")
        if pid in STRING_PARAMS:
            rc, s = self.b.get_param_string(pid)
            return s if rc == 1 else None
        rc, v = self.b.get_param(pid)
        return v if rc == 1 else None

    def info(self, pid: int) -> dict | None:
        self._need_open()
        rc, inf = self.b.get_param_info(pid)
        return inf if rc == 1 else None

    def scan(self, first: int = 0, last: int = 2000) -> list[dict]:
        """Every ID in [first, last] whose GetParamInfo answers and says available, with its value.
        GetParam is called only for those, and never for NEVER_READ."""
        self._need_open()
        out = []
        for pid in range(first, last + 1):
            inf = self.info(pid)
            if not inf or not inf.get("available"):
                continue
            row = {"id": pid, "name": KNOWN_PARAMS.get(pid), **inf}
            if pid in NEVER_READ:
                row["value"], row["note"] = None, "not read (firmware update)"
            else:
                try:
                    row["value"] = self.get(pid)
                except SiPMError as exc:
                    row["value"], row["error"] = None, str(exc)
                if pid not in STRING_PARAMS and row["value"] is None:
                    rc, s = self.b.get_param_string(pid)
                    if rc == 1 and s:
                        row["string"] = s
            out.append(row)
        return out

    def snapshot(self, extra: dict[str, int] | None = None, detectors: int | None = None) -> dict:
        """The status record: control unit, per-detector fields (GUI enum), and `extra` named IDs
        (e.g. {"sipm1_bias": <id from the scan>}). Values the DLL refuses are None."""
        self._need_open()
        n = detectors or int(self.get(782) or 0) or 2
        snap: dict = {"connected_detectors": n, "control_unit": {"serial": self.get(745), "firmware": self.get(744)}, "detectors": {}}
        for d in range(1, min(n, MAX_DETECTORS) + 1):
            rec = {k: self.get(pid) for k, pid in detector_params(d).items()}
            rec["serial"] = self.get(753 + 4 * (d - 1))
            snap["detectors"][str(d)] = rec
        if extra:
            snap["extra"] = {name: self.get(pid) for name, pid in extra.items()}
        return snap


def load_params_file(path: str | None) -> dict[str, int]:
    """{"sipm1_bias": 1201, …} — the IDs identified from a scan (Block S, step S2)."""
    if not path:
        return {}
    obj = json.loads(Path(path).read_text(encoding="utf-8"))
    return {str(k): int(v) for k, v in obj.items() if not str(k).startswith("_")}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=("scan", "snapshot", "watch"))
    p.add_argument("--dll-dir", help="folder with ThorSiPM.dll + ThorSiPMSettings.xml (the GUI's Release folder)")
    p.add_argument("--fake", action="store_true", help="use the simulated controller (off-rig)")
    p.add_argument("--from", dest="first", type=int, default=0, help="scan: first ID (default 0)")
    p.add_argument("--to", dest="last", type=int, default=2000, help="scan: last ID (default 2000)")
    p.add_argument("--params-file", help="JSON {name: id} of extra params to read (from the scan)")
    p.add_argument("--period", type=float, default=1.0, help="watch: seconds between snapshots")
    p.add_argument("--duration", type=float, default=60.0, help="watch: total seconds")
    p.add_argument("--out", help="write the result as JSON (scan/snapshot) or JSON lines (watch)")
    a = p.parse_args(argv)
    out_path = Path(a.out).resolve() if a.out else None  # before open() changes the working directory
    extra = load_params_file(a.params_file and str(Path(a.params_file).resolve()))
    dev = ThorSiPM.from_args(a.dll_dir, fake=a.fake)
    t0 = time.perf_counter()
    try:
        n = dev.open()
        print(f"[thorsipm] connected: {n} controller(s) found, device 0 selected ({(time.perf_counter() - t0) * 1000:.0f} ms)", file=sys.stderr)
        if a.action == "scan":
            rows = dev.scan(a.first, a.last)
            res = {"t_epoch": time.time(), "range": [a.first, a.last], "available": rows}
            unknown = [r for r in rows if not r["name"]]
            print(f"[thorsipm] {len(rows)} available params, {len(unknown)} not in the GUI enum (SiPM bias/temperature candidates):", file=sys.stderr)
            for r in rows:
                print(f"  {r['id']:5d}  {r['name'] or '?':40s}  value={r.get('value')!r}  type={r['type']} ro={r['read_only']} range=[{r['min']:g}, {r['max']:g}]", file=sys.stderr)
            text = json.dumps(res, indent=1)
            (out_path.write_text(text, encoding="utf-8") if out_path else print(text))
        elif a.action == "snapshot":
            text = json.dumps({"t_epoch": time.time(), **dev.snapshot(extra)}, indent=1)
            (out_path.write_text(text, encoding="utf-8") if out_path else print(text))
        else:
            fh = open(out_path, "a", encoding="utf-8") if out_path else None
            end = time.monotonic() + a.duration
            while time.monotonic() < end:
                t1 = time.perf_counter()
                rec = {"t_epoch": time.time(), **dev.snapshot(extra)}
                rec["read_ms"] = round((time.perf_counter() - t1) * 1000, 1)
                print(json.dumps(rec, separators=(",", ":")), file=fh or sys.stdout, flush=True)
                time.sleep(max(0.0, a.period - (time.perf_counter() - t1)))
            if fh:
                fh.close()
    except SiPMError as exc:
        print(f"[thorsipm] {exc}", file=sys.stderr)
        return 2
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
