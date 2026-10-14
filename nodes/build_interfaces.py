#!/usr/bin/env python3
"""build_interfaces.py — build interfaces/arena_interfaces inside this pixi env, and time it.

    pixi run build-interfaces            # colcon build → interfaces/install, result JSON printed + saved
    pixi run build-interfaces -- --clean # wipe build/ + install/ first (a cold build; the number to report)
    pixi run build-interfaces -- --check # just report: built? importable? which typesupports?

LAB-303 Plan B, item 3: typed messages mean an ament_cmake interfaces package, and rosidl
generates C typesupport + a Python C extension — so a C/C++ compiler is needed on every machine
that builds it (RoboStack's `cxx-compiler`: clang on macOS, GCC on Linux, MSVC 2022 on Windows,
which pixi cannot install). This script is the measurement: wall time, steps, failures, verbatim.
The result goes to interfaces/build-result.json and the README's "Plan B spike" table.

Nodes pick the built package up through `_common.setup_interfaces()` (prepends the install
prefix to AMENT_PREFIX_PATH / PYTHONPATH / the dynamic-library path in-process), so no shell
has to source `install/setup.*` — the same command works on Windows.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
POC = HERE.parent
IFACES = POC / "interfaces"
PKG = IFACES / "arena_interfaces"
BUILD = IFACES / "build"
INSTALL = IFACES / "install"
RESULT = IFACES / "build-result.json"

sys.path.insert(0, str(HERE))
from _common import setup_interfaces  # noqa: E402


def check() -> dict:
    out = {"install_prefix": str(INSTALL), "built": INSTALL.exists()}
    setup_interfaces()
    try:
        from arena_interfaces.msg import ArenaFrame, ConnectorReply, FicTracFrame  # noqa: F401

        out["importable"] = True
        out["msgs"] = ["FicTracFrame", "ArenaFrame", "ConnectorReply"]
        # Can rclpy serialize it (loads the C typesupport)?
        from rclpy.serialization import serialize_message

        f = FicTracFrame(seq=1, raw="FT,1,2", tag="FT", fields=[1.0, 2.0])
        out["serializes"] = len(serialize_message(f)) > 4
    except Exception as exc:  # ImportError, OSError (dlopen), …
        out["importable"] = False
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def build(clean: bool) -> dict:
    res: dict = {"platform": f"{platform.system()} {platform.machine()}", "python": sys.version.split()[0],
                 "steps": [], "ok": False}
    if clean:
        for d in (BUILD, INSTALL):
            shutil.rmtree(d, ignore_errors=True)
        res["clean"] = True
    for tool in ("colcon", "cmake", "ninja"):
        res.setdefault("tools", {})[tool] = shutil.which(tool)
    cxx = shutil.which("clang++") or shutil.which("c++") or shutil.which("cl")
    res["tools"]["cxx"] = cxx
    cmd = [
        "colcon", "build",
        "--base-paths", str(IFACES),
        "--build-base", str(BUILD),
        "--install-base", str(INSTALL),
        "--merge-install",
        "--event-handlers", "console_direct+",
        "--cmake-args", "-GNinja", "-DCMAKE_BUILD_TYPE=Release",
        f"-DPython3_EXECUTABLE={sys.executable}", f"-DPython_EXECUTABLE={sys.executable}",
        # RoboStack's toolchain sets the install rpath to the ENV's lib only; the generated
        # Python extension then cannot find libarena_interfaces__*.dylib next to itself, and
        # macOS SIP strips DYLD_LIBRARY_PATH on the way through pixi's shell (first build on
        # the Mac: `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c'`). Add
        # the package's own lib dir to the rpath.
        f"-DCMAKE_INSTALL_RPATH={INSTALL / 'lib'};{Path(sys.executable).resolve().parents[1] / 'lib'}",
        "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON",
    ]
    res["command"] = " ".join(cmd)
    t0 = time.monotonic()
    env = os.environ.copy()
    env.setdefault("PYTHONUTF8", "1")
    p = subprocess.run(cmd, cwd=str(IFACES), capture_output=True, text=True, env=env)
    res["wall_s"] = round(time.monotonic() - t0, 1)
    res["returncode"] = p.returncode
    tail = (p.stdout + p.stderr).strip().splitlines()
    res["log_tail"] = tail[-25:]
    (IFACES / "build.log").write_text(p.stdout + "\n--- stderr ---\n" + p.stderr, encoding="utf-8")
    res["steps"].append({"step": "colcon build", "s": res["wall_s"], "rc": p.returncode})
    if p.returncode == 0:
        t1 = time.monotonic()
        chk = check()
        res["check"] = chk
        res["steps"].append({"step": "import + serialize", "s": round(time.monotonic() - t1, 2), "ok": chk.get("serializes", False)})
        res["ok"] = bool(chk.get("serializes"))
        libs = sorted(p.name for p in (INSTALL / "lib").glob("libarena_interfaces__*")) if (INSTALL / "lib").exists() else []
        res["typesupport_libs"] = libs
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clean", action="store_true", help="remove interfaces/build + install first")
    ap.add_argument("--check", action="store_true", help="only report whether the package imports")
    a = ap.parse_args(argv)
    if a.check:
        r = check()
        print(json.dumps(r, indent=1))
        return 0 if r.get("importable") else 1
    r = build(a.clean)
    RESULT.write_text(json.dumps(r, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in r.items() if k != "log_tail"}, indent=1))
    if not r["ok"]:
        print("\n".join(r["log_tail"]), file=sys.stderr)
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
