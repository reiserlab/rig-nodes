#!/usr/bin/env python3
"""test_sipm.py — the read-only SiPM controller driver and node, off-rig (Bergamo Block S).

    pixi run test-sipm                    # in the rig-nodes checkout (≈ 30 s)
    python tests/test_sipm.py             # anywhere rig_nodes is installed (the acceptance job)
    pixi run test-sipm -- --keep

Everything runs against thorsipm.FakeBackend (two SiPMs + 'hidden' bias/temperature params at IDs
the GUI enum doesn't name, the way the real ones are unknown until the rig scan):

  A  driver, no ROS        scan lists every available ID, flags the 6 not in the enum, never reads
                           the firmware-update params; snapshot fields + serials; a busy COM port
                           (GUI open) fails open() with a clear reason; no write path exists
  B  node, connected       /sipm/status at ~1 Hz, connected, level ok, extra (params-file) values;
                           get by name / by id; info; scan finds the hidden IDs; disconnect holds
                           (no auto-reconnect), reconnect restores; clean exit
  C  node, port busy       status says not connected with the reason and keeps publishing
  D  recording             the received status messages round-trip through rig_nodes.recorder into an
                           MCAP (the arena gateway's registration of /sipm/* is webDisplayTools' check)

Exit 0 when every check passes.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable
NODE = [PY, "-m", "rig_nodes.sipm_node"]
from rig_nodes._common import RELIABLE_QOS  # noqa: E402
from rig_nodes.supervise import SIGKILL, popen_group, signal_group, wait_port  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import rig_nodes.sipm_node as sipm_node  # noqa: E402
import rig_nodes.thorsipm as thorsipm  # noqa: E402

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, info: str = "") -> bool:
    results.append((name, bool(ok), info))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'' if ok else ' — ' + info}", file=sys.stderr)
    return bool(ok)


def wait_for(pred, timeout: float, period: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return bool(pred())


class Proc:
    def __init__(self, name: str, argv: list[str], logdir: Path) -> None:
        self.name, self.argv = name, argv
        self.log = open(logdir / f"{name}.log", "a", encoding="utf-8")
        self.p = popen_group(argv, stdout=self.log, stderr=subprocess.STDOUT, text=True)

    def alive(self) -> bool:
        return self.p.poll() is None

    def stop(self, grace: float = 5.0) -> int | None:
        if self.alive():
            try:
                signal_group(self.p, signal.SIGINT)
                self.p.wait(grace)
            except subprocess.TimeoutExpired:
                signal_group(self.p, SIGKILL)
                self.p.wait(3)
            except (ProcessLookupError, OSError, ValueError):
                pass
        self.log.flush()
        return self.p.returncode


class Client(Node):
    """Subscribes /sipm/status + /sipm/reply; sends commands and waits for the matching reply."""

    def __init__(self) -> None:
        super().__init__("sipm_harness")
        self.lock = threading.Lock()
        self.status: list[tuple[int, dict, String]] = []
        self.replies: dict = {}
        self.create_subscription(String, "/sipm/status", self._st, RELIABLE_QOS)
        self.create_subscription(String, "/sipm/reply", self._rep, RELIABLE_QOS)
        self.pub = self.create_publisher(String, "/sipm/cmd", RELIABLE_QOS)

    def _st(self, m: String) -> None:
        with self.lock:
            self.status.append((time.time_ns(), json.loads(m.data), m))

    def _rep(self, m: String) -> None:
        o = json.loads(m.data)
        with self.lock:
            self.replies[o.get("id")] = o

    def last_status(self) -> dict | None:
        with self.lock:
            return self.status[-1][1] if self.status else None

    def cmd(self, command: str, timeout: float = 10.0, **params) -> dict | None:
        cid = str(uuid.uuid4())[:8]
        m = String()
        m.data = json.dumps({"id": cid, "command": command, "params": params})
        self.pub.publish(m)
        wait_for(lambda: cid in self.replies, timeout)
        with self.lock:
            return self.replies.get(cid)


def part_a() -> None:
    print("A  driver, no ROS", file=sys.stderr)
    fake = thorsipm.FakeBackend()
    reads: list[int] = []
    orig = fake.get_param
    fake.get_param = lambda pid: (reads.append(pid), orig(pid))[1]
    dev = thorsipm.ThorSiPM(fake)
    with dev:
        rows = dev.scan(0, 2000)
        snap = dev.snapshot({"sipm1_bias": 1201, "sipm1_temp_c": 1211})
    ids = {r["id"] for r in rows}
    unknown = sorted(r["id"] for r in rows if not r["name"])
    check("A: scan returns every available ID (fake: 38)", len(rows) == 38, f"{len(rows)}: {sorted(ids)}")
    check("A: scan flags the 6 IDs missing from the GUI enum (the bias/temperature candidates)", unknown == sorted(fake.HIDDEN), str(unknown))
    check("A: firmware-update params are listed but never read", 743 in ids and not (set(reads) & thorsipm.NEVER_READ), str(sorted(set(reads) & thorsipm.NEVER_READ)))
    d1, d2 = snap["detectors"].get("1", {}), snap["detectors"].get("2", {})
    check("A: snapshot has both detectors with enable/gain/offset/bandwidth and serials 000027/000026",
          d1.get("enable") == 1.0 and d1.get("gain_volts") == 0.8 and d1.get("serial") == "000027" and d2.get("serial") == "000026"
          and "bandwidth_pos" in d1 and "output_offset" in d1, json.dumps(snap)[:300])
    check("A: extra (params-file) values read", snap.get("extra", {}).get("sipm1_bias") == 54.2, json.dumps(snap.get("extra")))
    check("A: close() tears down (frees the COM port)", not dev.is_open and not fake.selected, "")
    busy = thorsipm.ThorSiPM(thorsipm.FakeBackend(busy=True))
    try:
        busy.open()
        check("A: busy COM port (GUI open) → open() fails", False, "opened")
    except thorsipm.SiPMError as exc:
        check("A: busy COM port (GUI open) → open() fails and names the GUI", "ThorSiPM_Control.exe" in str(exc), str(exc))
    src = Path(thorsipm.__file__).read_text(encoding="utf-8") + Path(sipm_node.__file__).read_text(encoding="utf-8")
    calls = re.findall(r"\.(SetParam\w*|StartPosition|SetupPosition|PreflightPosition|PostflightPosition)\s*\(", src)
    sig_names = re.search(r"sig = \{(.*?)\n        \}", src, re.S)
    check("A: no write path — no SetParam/Setup/Start call, and the ctypes binding declares none",
          not calls and sig_names is not None and "SetParam" not in sig_names.group(1) and "Position" not in sig_names.group(1)
          and not hasattr(thorsipm.ThorSiPM, "set"), f"calls={calls}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--keep", action="store_true", help="keep the work dir")
    ap.add_argument("--no-router", action="store_true", help="a Zenoh router is already running on 127.0.0.1:7447")
    a = ap.parse_args(argv)
    work = Path(tempfile.mkdtemp(prefix="sipm-test-"))
    logdir = work / "proc-logs"
    logdir.mkdir()
    params_file = work / "sipm_params.json"
    params_file.write_text(json.dumps({"_note": "IDs from the Block S scan", "sipm1_bias": 1201, "sipm2_bias": 1202,
                                       "sipm1_temp_c": 1211, "sipm2_temp_c": 1212, "sipm1_tempstatus": 1221}), encoding="utf-8")
    print(f"[test] work dir {work}", file=sys.stderr)

    part_a()

    procs: list[Proc] = []
    cl = ex = None
    try:
        if not a.no_router:
            procs.append(Proc("router", [PY, "-m", "rig_nodes.zenoh"], logdir))
            check("zenoh router listening on 127.0.0.1:7447", wait_port("127.0.0.1", 7447, 20.0))
        rclpy.init()
        cl = Client()
        ex = SingleThreadedExecutor()
        ex.add_node(cl)
        threading.Thread(target=ex.spin, name="harness-spin", daemon=True).start()

        # ── B: node, connected ───────────────────────────────────────────────────────────────
        print("B  node, connected", file=sys.stderr)
        node = Proc("sipm-node", [*NODE, "--fake", "--params-file", str(params_file), "--status-every", "0.5"], logdir)
        procs.append(node)
        got = wait_for(lambda: len(cl.status) >= 4, 25.0)
        st = cl.last_status() or {}
        check("B: /sipm/status arrives", got, f"{len(cl.status)} messages")
        with cl.lock:
            ts = [t for t, _, _ in cl.status[-4:]]
        gaps = [(b - c) / 1e9 for c, b in zip(ts, ts[1:])]
        check("B: status period ≈ --status-every (0.5 s)", gaps and all(0.3 < g < 0.9 for g in gaps), str(gaps))
        check("B: status connected, level ok, two detectors, serials", st.get("connected") is True and st.get("level") == "ok"
              and st.get("detectors", {}).get("1", {}).get("serial") == "000027" and st.get("connected_detectors") == 2, json.dumps(st)[:300])
        check("B: params-file values in status (bias, temperature)", st.get("extra", {}).get("sipm1_bias") == 54.2
              and 21.0 < (st.get("extra", {}).get("sipm2_temp_c") or 0) < 23.0, json.dumps(st.get("extra")))
        r = cl.cmd("get", name="PARAM_PMT2_ENABLE")
        check("B: get by PARAM_* name", r and r.get("ok") and r["value"]["id"] == 703 and r["value"]["value"] == 1.0, json.dumps(r))
        r = cl.cmd("get", name="sipm2_bias")
        check("B: get by params-file name", r and r.get("ok") and r["value"]["value"] == 54.6, json.dumps(r))
        r = cl.cmd("info", id=1201)
        check("B: info", r and r.get("ok") and r["value"]["info"]["available"] == 1, json.dumps(r))
        r = cl.cmd("scan", timeout=30.0, **{"from": 690, "to": 1300})
        check("B: scan finds the hidden IDs", r and r.get("ok") and r["value"]["not_in_enum"] == sorted(thorsipm.FakeBackend.HIDDEN), json.dumps(r)[:300])
        r = cl.cmd("get", name="no_such_param")
        check("B: an unknown name is an error reply, not a crash", r and r.get("ok") is False and "unknown parameter" in r.get("error", ""), json.dumps(r))
        r = cl.cmd("set", id=701, value=0)
        check("B: there is no set command", r and r.get("ok") is False and "unknown command" in r.get("error", ""), json.dumps(r))
        r = cl.cmd("disconnect")
        n0 = len(cl.status)
        wait_for(lambda: len(cl.status) >= n0 + 3, 5.0)
        st = cl.last_status() or {}
        check("B: disconnect → status not connected, held (no auto-reconnect)", r and r.get("ok") and st.get("connected") is False and st.get("held") is True, json.dumps(st))
        r = cl.cmd("reconnect")
        n0 = len(cl.status)
        wait_for(lambda: len(cl.status) >= n0 + 2, 5.0)
        check("B: reconnect → connected again", r and r.get("ok") and (cl.last_status() or {}).get("connected") is True, json.dumps(r))
        rc = node.stop()
        check("B: clean exit on SIGINT", rc == 0, f"rc={rc}; log: {(logdir / 'sipm-node.log').read_text()[-400:]}")

        # ── C: node, port busy ───────────────────────────────────────────────────────────────
        print("C  node, port busy", file=sys.stderr)
        with cl.lock:
            cl.status.clear()
        busy = Proc("sipm-node-busy", [*NODE, "--fake-busy", "--status-every", "0.5", "--reconnect-every", "1"], logdir)
        procs.append(busy)
        wait_for(lambda: len(cl.status) >= 3, 25.0)
        st = cl.last_status() or {}
        check("C: busy port → status keeps coming, not connected, reason names the GUI", len(cl.status) >= 3 and st.get("connected") is False
              and st.get("level") == "error" and "ThorSiPM_Control.exe" in (st.get("error") or ""), json.dumps(st))
        r = cl.cmd("get", id=701)
        check("C: a read while not connected is an error reply", r and r.get("ok") is False and "not connected" in r.get("error", ""), json.dumps(r))
        busy.stop()

        # ── D: recording ─────────────────────────────────────────────────────────────────────
        print("D  recording", file=sys.stderr)
        # The arena gateway's bag topic map and the converter's /sipm/* derivation are checked by
        # webDisplayTools' harness (rig/tests/test_ros2_poc.py); here: the recorder round trip.
        from rig_nodes.foxglove_json import read_messages  # noqa: E402
        from rig_nodes.recorder import Recorder  # noqa: E402

        bag = work / "sipm.bag"
        rec = Recorder(bag, {"/sipm/status": "std_msgs/msg/String"}, profile="default")
        rec.open()
        with cl.lock:
            sent = [(t, m) for t, _, m in cl.status]
        for t_ns, m in sent:
            rec.write_raw("/sipm/status", serialize_message(m), t_ns)
        rec.close(reindex=False)
        mcaps = sorted(bag.glob("*.mcap")) if bag.is_dir() else [bag]
        msgs, torn = read_messages(mcaps[0]) if mcaps else ([], "no mcap")
        back = [json.loads(deserialize_message(d, String).data) for topic, _, _, _, _, d in msgs if topic == "/sipm/status"]
        check("D: status messages round-trip through the recorder into an MCAP", len(back) == len(sent) and back and back[-1].get("level") == "error" and not torn,
              f"{len(back)}/{len(sent)} torn={torn}")
    finally:
        for pr in reversed(procs):
            pr.stop()
            pr.log.close()
        try:
            if ex is not None:
                ex.shutdown(timeout_sec=1.0)
            if cl is not None:
                cl.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass

    n_ok = sum(1 for _, ok, _ in results if ok)
    print(f"\n{n_ok} / {len(results)} checks passed", file=sys.stderr)
    print("\n".join(["| check | result |", "|---|---|"] + [f"| {n} | {'PASS' if ok else 'FAIL — ' + i.replace('|', '/')[:160]} |" for n, ok, i in results]))
    if not a.keep and n_ok == len(results):
        shutil.rmtree(work, ignore_errors=True)
    else:
        print(f"[test] artifacts kept in {work}", file=sys.stderr)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
