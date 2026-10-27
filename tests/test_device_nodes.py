#!/usr/bin/env python3
"""test_device_nodes.py — device-node conformance: simulator → node → topics → loss accounting.

    pixi run test                          # in the rig-nodes checkout (starts its own Zenoh router)
    python tests/test_device_nodes.py      # anywhere rig_nodes is installed (the acceptance job)
    … -- --no-router                       # a router is already up on 127.0.0.1:7447
    … -- --timeout-scale 2                 # slower machines (CI)

This is rig-nodes' half of the testing split (rig-software-repos-and-runners.md §7.8): every
device node is driven by its simulator exactly as a rig would, and what reaches the topics is
judged against CONTRACT.md — the envelope, the command/reply/status shapes, the outcome
vocabulary, the counter-reset rule, the recorder's accounting. Nothing here knows about the arena;
the arena gateway's harness in webDisplayTools (`rig/tests/test_ros2_poc.py`) runs the whole graph
against these installed nodes and is the cross-repository smoke.

Checks (one Zenoh graph, ≈ 40 s):
  FicTrac   sim --proto udp → fictrac adapter → /fictrac/raw (+ /fictrac/frame when rig_interfaces
            is importable): envelope keys/order, src, consecutive seq, 25 columns, stamps in range,
            typed frame == envelope; adapter SIGKILLed and restarted → seq restarts at 1 (contract §6)
  Kinefly   kinefly sim → adapter → /kinefly/raw: KF/1 payload, consecutive sequence; a non-KF/1
            datagram and a wrong-version record are dropped and counted, never published
  BIAS      bias_sim → bias_node ← bias_cmd: connect / startRecording (artifact) / getTimestamp /
            stopRecording all `ok`; status carries reachable + the device snapshot; sim killed →
            reply outcome `error`, status reachable false
  ScanImage si_json_mock (stock bug B2 on one allowlisted property) → scanimage_node ← scanimage_cmd:
            status / setFileStem / grab / waitIdle `ok`; the stalled get → outcome `unknown` (exit 3),
            then status `ok` again (reconciled)
  Recorder  in-process Recorder: raw CDR + JSON records into a crashsafe bag, closed → indexed;
            counts match; records written while closed are counted as dropped
Exit 0 when all checks pass. Process logs are kept with --keep.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from rig_nodes.supervise import SIGKILL, popen_group, signal_group, wait_port

PY = sys.executable
results: list[tuple[str, bool, str]] = []
SCALE = 1.0


def check(name: str, ok: bool, info: str = "") -> bool:
    results.append((name, bool(ok), info))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'' if ok else ' — ' + str(info)[:400]}", file=sys.stderr)
    return bool(ok)


def free_port(kind=socket.SOCK_STREAM) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_for(pred, timeout_s: float, step: float = 0.1) -> bool:
    end = time.monotonic() + timeout_s * SCALE
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return bool(pred())


class Proc:
    def __init__(self, name: str, argv: list[str], logdir: Path) -> None:
        self.name = name
        self.argv = argv
        self.logpath = logdir / f"{name}.log"
        self.log = open(self.logpath, "w", encoding="utf-8")
        self.p: subprocess.Popen | None = None

    def start(self) -> "Proc":
        self.p = popen_group(self.argv, stdout=self.log, stderr=subprocess.STDOUT, text=True)
        return self

    def alive(self) -> bool:
        return self.p is not None and self.p.poll() is None

    def kill9(self) -> None:
        if self.alive():
            signal_group(self.p, SIGKILL)
            self.p.wait(5)

    def stop(self) -> None:
        if not self.alive():
            return
        try:
            signal_group(self.p, __import__("signal").SIGINT)
            self.p.wait(5)
        except subprocess.TimeoutExpired:
            signal_group(self.p, SIGKILL)
        except (ProcessLookupError, OSError, ValueError):
            pass
        self.log.flush()

    def text(self) -> str:
        self.log.flush()
        return self.logpath.read_text(encoding="utf-8", errors="replace")


def run_capture(argv: list[str], timeout: float = 60.0) -> subprocess.CompletedProcess:
    return subprocess.run(argv, capture_output=True, text=True, timeout=timeout * SCALE)


def cmd_json(r: subprocess.CompletedProcess) -> dict:
    """The reply JSON a command tool printed with --json (indented; rclpy may print lines before it)."""
    out = r.stdout or ""
    i = out.find("{")
    if i < 0:
        return {}
    try:
        return json.loads(out[i:])
    except json.JSONDecodeError:
        return {}


class Listener:
    """One rclpy node subscribed to the contract topics, collecting every message."""

    def __init__(self, typed) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        from rig_nodes._common import RELIABLE_QOS

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("device_conformance")
        self.msgs: dict[str, list] = {}
        self.lock = threading.Lock()
        self.typed = typed
        for topic in ("/fictrac/raw", "/kinefly/raw", "/bias/reply", "/bias/status", "/scanimage/reply", "/scanimage/status"):
            self.node.create_subscription(String, topic, self._cb(topic, lambda m: json.loads(m.data)), RELIABLE_QOS)
        if typed is not None:
            _ArenaFrame, FicTracFrame = typed
            self.node.create_subscription(FicTracFrame, "/fictrac/frame",
                                          self._cb("/fictrac/frame", lambda m: {"seq": m.seq, "rx_epoch_ms": m.rx_epoch_ms, "rx_mono_ns": m.rx_mono_ns,
                                                                                "tag": m.tag, "fields": list(m.fields), "raw": m.raw}), RELIABLE_QOS)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _cb(self, topic, decode):
        def cb(msg):
            try:
                obj = decode(msg)
            except Exception as exc:  # a payload that is not JSON is itself a finding
                obj = {"_undecodable": repr(exc)}
            with self.lock:
                self.msgs.setdefault(topic, []).append(obj)
        return cb

    def _spin(self) -> None:
        while not self._stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def take(self, topic: str) -> list:
        with self.lock:
            return list(self.msgs.get(topic, []))

    def count(self, topic: str) -> int:
        with self.lock:
            return len(self.msgs.get(topic, []))

    def clear(self, topic: str) -> None:
        with self.lock:
            self.msgs[topic] = []

    def close(self) -> None:
        self._stop.set()
        self.thread.join(2)
        self.node.destroy_node()
        try:
            self.rclpy.try_shutdown()
        except Exception:
            pass


def judge_fictrac(L: Listener, start, work: Path, logdir: Path, typed) -> None:
    from rig_nodes.fictrac import parse_fictrac_line

    udp = free_port(socket.SOCK_DGRAM)
    adapter = start("fictrac-adapter", [PY, "-m", "rig_nodes.fictrac_udp_adapter", "--port", str(udp), "--status-every", "0"])
    time.sleep(1.0 * SCALE)
    sim = start("fictrac-sim", [PY, "-m", "rig_nodes.sims.fictrac_sim", "--proto", "udp", "--port", str(udp), "--rate", "100", "--seed", "1"])
    got = wait_for(lambda: L.count("/fictrac/raw") >= 150, 8.0)
    msgs = L.take("/fictrac/raw")
    check("fictrac: ≥ 150 envelopes within 8 s (sim 100 Hz → adapter → /fictrac/raw)", got, f"{len(msgs)} msgs; adapter log: {adapter.text()[-300:]}")
    if msgs:
        keys_ok = all(list(m) == ["src", "seq", "rx_epoch_ms", "rx_mono_ns", "raw"] for m in msgs)
        check("fictrac: every envelope has exactly the five contract keys in order", keys_ok, [list(m) for m in msgs if list(m) != ["src", "seq", "rx_epoch_ms", "rx_mono_ns", "raw"]][:2])
        check("fictrac: src == 'fictrac'", all(m.get("src") == "fictrac" for m in msgs))
        seqs = [m["seq"] for m in msgs]
        check("fictrac: seq is 1-based and consecutive (no loss on loopback)", seqs[0] == 1 and seqs == list(range(seqs[0], seqs[0] + len(seqs))), (seqs[:3], seqs[-3:]))
        parsed = [parse_fictrac_line(m["raw"]) for m in msgs]
        check("fictrac: every raw line parses to 25 numeric columns, verbatim (no newline)", all(len(f) == 25 for _, f in parsed) and all("\n" not in m["raw"] for m in msgs),
              [m["raw"][:80] for m, (_, f) in zip(msgs, parsed) if len(f) != 25][:2])
        now_ms = time.time() * 1000
        check("fictrac: rx_epoch_ms is the adapter's wall clock (within 30 s of now)", all(abs(m["rx_epoch_ms"] - now_ms) < 30_000 for m in msgs))
        monos = [m["rx_mono_ns"] for m in msgs]
        check("fictrac: rx_mono_ns is non-decreasing", all(b >= a for a, b in zip(monos, monos[1:])))
        fcs = [f[0] for _, f in parsed]
        check("fictrac: FicTrac frame counter (col 1) increases by 1 per record", all(b - a == 1 for a, b in zip(fcs, fcs[1:])), (fcs[:3], fcs[-3:]))
    if typed is not None:
        tmsgs = L.take("/fictrac/frame")
        check("fictrac typed: /fictrac/frame published beside /fictrac/raw (rig_interfaces importable)", len(tmsgs) >= 100, len(tmsgs))
        if tmsgs and msgs:
            by_seq = {m["seq"]: m for m in msgs}
            same = [t for t in tmsgs if t["seq"] in by_seq]
            check("fictrac typed: same seq → same raw, 25 fields, tag '' (simulator lines carry no FT tag)",
                  bool(same) and all(by_seq[t["seq"]]["raw"] == t["raw"] and len(t["fields"]) == 25 and t["tag"] == "" for t in same), same[:1])
    else:
        print("  info  typed topics: rig_interfaces not importable in this environment — /fictrac/frame not checked", file=sys.stderr)
    # ── loss accounting: the adapter dies and comes back; the device (sim) never stopped ──
    adapter.kill9()
    time.sleep(0.8 * SCALE)  # datagrams sent now are lost: no adapter owns the socket
    L.clear("/fictrac/raw")
    adapter2 = start("fictrac-adapter-2", [PY, "-m", "rig_nodes.fictrac_udp_adapter", "--port", str(udp), "--status-every", "0"])
    got = wait_for(lambda: L.count("/fictrac/raw") >= 20, 8.0)
    after = L.take("/fictrac/raw")
    check("fictrac: adapter SIGKILLed + restarted → publishing resumes", got, f"{len(after)} msgs; log: {adapter2.text()[-300:]}")
    if after:
        check("fictrac: seq restarts at 1 with the adapter process (contract §6: counters are per process)", after[0]["seq"] == 1, after[0]["seq"])
        fcs = [parse_fictrac_line(m["raw"])[1][0] for m in after]
        check("fictrac: the device's own counter (col 1) kept counting through the outage (gap visible)", fcs[0] > 1, fcs[:2])
    sim.stop()
    adapter2.stop()


def judge_kinefly(L: Listener, start, logdir: Path) -> None:
    udp = free_port(socket.SOCK_DGRAM)
    adapter = start("kinefly-adapter", [PY, "-m", "rig_nodes.kinefly_udp_adapter", "--port", str(udp), "--status-every", "0"])
    time.sleep(1.0 * SCALE)
    sim = start("kinefly-sim", [PY, "-m", "rig_nodes.sims.kinefly_sim", "--port", str(udp), "--rate", "100", "--seed", "1"])
    got = wait_for(lambda: L.count("/kinefly/raw") >= 100, 8.0)
    msgs = L.take("/kinefly/raw")
    check("kinefly: ≥ 100 envelopes within 8 s", got, f"{len(msgs)}; log: {adapter.text()[-300:]}")
    if msgs:
        check("kinefly: envelope keys + src", all(list(m) == ["src", "seq", "rx_epoch_ms", "rx_mono_ns", "raw"] and m["src"] == "kinefly" for m in msgs))
        pk = [json.loads(m["raw"]) for m in msgs]
        check("kinefly: raw is a KF/1 record (protocol Kinefly, version 1, parts, sequence)",
              all(p.get("protocol") == "Kinefly" and p.get("version") == 1 and isinstance(p.get("parts"), dict) for p in pk))
        ks = [p["sequence"] for p in pk]
        check("kinefly: the device's sequence is consecutive", all(b - a == 1 for a, b in zip(ks, ks[1:])), (ks[:3], ks[-3:]))
        wings = [p["parts"]["left"]["angles_rad"][0] for p in pk if p["parts"].get("left", {}).get("valid")]
        check("kinefly: wing angles present and finite (radians)", len(wings) == len(pk) and all(math.isfinite(w) for w in wings), len(wings))
    sim.stop()
    time.sleep(0.3)
    n0 = L.count("/kinefly/raw")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b"not json at all", ("127.0.0.1", udp))
    s.sendto(json.dumps({"protocol": "Kinefly", "version": 2, "sequence": 1, "parts": {}}).encode(), ("127.0.0.1", udp))
    s.sendto(json.dumps({"protocol": "FicTrac", "version": 1}).encode(), ("127.0.0.1", udp))
    s.close()
    time.sleep(1.0 * SCALE)
    check("kinefly: non-KF/1 datagrams are dropped, never published", L.count("/kinefly/raw") == n0, (n0, L.count("/kinefly/raw")))
    adapter.stop()
    text = adapter.text()
    if "dropped non-KF/1" in text or "rejected" in text:
        check("kinefly: the adapter counts what it rejected (stderr accounting)", True)
    elif os.name == "nt":
        # Windows: the C runtime buffers stderr when it is a file and a terminated process never
        # flushes it, so the reject lines may be lost with the process; the drop itself was checked above.
        print("  info  kinefly: reject accounting not visible in the log on Windows (stderr buffered to a file)", file=sys.stderr)
    else:
        check("kinefly: the adapter counts what it rejected (stderr accounting)", False, text[-400:])


def judge_bias(L: Listener, start, work: Path) -> None:
    port = free_port()
    videos = work / "bias-videos"
    sim = start("bias-sim", [PY, "-m", "rig_nodes.sims.bias_sim", "--port", str(port), "--video-dir", str(videos)])
    check("bias: simulator up", wait_port("127.0.0.1", port, 10.0 * SCALE))
    node = start("bias-node", [PY, "-m", "rig_nodes.bias_node", "--port", str(port), "--status-every", "0.5", "--video-dir", str(videos)])
    time.sleep(1.5 * SCALE)

    def cmd(*args: str) -> tuple[int, dict]:
        r = run_capture([PY, "-m", "rig_nodes.bias_cmd", *args, "--json", "--timeout", str(15 * SCALE)], timeout=40)
        return r.returncode, cmd_json(r) or {"rc": r.returncode, "err": (r.stderr or r.stdout)[-300:]}

    rc, rep = cmd("connect")
    check("bias: connect → ok (exit 0)", rc == 0 and rep.get("outcome") == "ok" and rep.get("ok") is True, rep)
    check("bias: reply carries id, command, steps, rtt_ms, t_ms (contract §4)",
          all(k in rep for k in ("id", "command", "ok", "outcome", "steps", "rtt_ms", "t_ms")) and rep.get("command") == "connect", sorted(rep))
    rc, rep = cmd("startRecording", "--filename", "trial_001.avi")
    check("bias: startRecording → ok with an artifact {kind: video, path}", rc == 0 and rep.get("artifact", {}).get("kind") == "video" and rep["artifact"].get("path"), rep.get("artifact"))
    rc, rep = cmd("getTimestamp")
    check("bias: getTimestamp → ok with a value", rc == 0 and rep.get("value") is not None, rep)
    time.sleep(0.6 * SCALE)
    rc, rep = cmd("stopRecording")
    check("bias: stopRecording → ok", rc == 0 and rep.get("outcome") == "ok", rep)
    st = [s for s in L.take("/bias/status") if s.get("reachable")]
    check("bias: status every 0.5 s with reachable + the device snapshot (connected, capturing, frameCount)",
          len(st) >= 2 and all("t_ms" in s and "connected" in s and "frameCount" in s for s in st[-2:]), st[-1:] if st else L.take("/bias/status")[-1:])
    rc, rep = cmd("bogusCommand")
    check("bias: an unknown command → outcome error (exit 1), never a crash", rc == 1 and rep.get("outcome") == "error" and node.alive(), rep)
    # the device program dies: replies say so, status says unreachable, the node stays up
    sim.kill9()
    time.sleep(0.5 * SCALE)
    rc, rep = cmd("getTimestamp")
    # A refused connect is `error`; Windows may let the connect time out instead, which the contract
    # (§4) defines as `unknown`. Either way: a message, the node alive, and status says unreachable.
    check("bias: device gone → reply outcome error|unknown with a message; node alive",
          rc in (1, 3) and rep.get("outcome") in ("error", "unknown") and rep.get("error") and node.alive(), rep)
    got = wait_for(lambda: any(s.get("reachable") is False for s in L.take("/bias/status")[-4:]), 4.0)
    check("bias: status reports reachable=false after the loss", got, L.take("/bias/status")[-1:])
    replies = L.take("/bias/reply")
    check("bias: exactly one reply per command on /bias/reply", len(replies) == 6, len(replies))
    check("bias: outcome vocabulary is closed (ok | error | unknown)", all(r.get("outcome") in ("ok", "error", "unknown") for r in replies), {r.get("outcome") for r in replies})
    node.stop()


def judge_scanimage(L: Listener, start) -> None:
    port = free_port()
    mock = start("si-mock", [PY, "-m", "rig_nodes.sims.si_json_mock", "--port", str(port), "--grab-s", "1.0", "--start-delay-s", "0.2",
                             "--stall-on", "hSI.hRoiManager.scanZoomFactor"])
    check("scanimage: JSON-server mock up", wait_port("127.0.0.1", port, 10.0 * SCALE))
    node = start("scanimage-node", [PY, "-m", "rig_nodes.scanimage_node", "--port", str(port), "--status-every", "1", "--timeout-s", str(1.5 * SCALE)])
    time.sleep(1.5 * SCALE)

    def cmd(*args: str) -> tuple[int, dict]:
        r = run_capture([PY, "-m", "rig_nodes.scanimage_cmd", *args, "--json", "--timeout", str(30 * SCALE)], timeout=60)
        return r.returncode, cmd_json(r) or {"rc": r.returncode, "err": (r.stderr or r.stdout)[-300:]}

    rc, rep = cmd("status")
    check("scanimage: status → ok with acqState", rc == 0 and rep.get("outcome") == "ok" and "acqState" in json.dumps(rep), rep)
    rc, rep = cmd("setFileStem", "--stem", "conformance")
    check("scanimage: setFileStem → ok", rc == 0, rep)
    rc, rep = cmd("grab")
    check("scanimage: grab → ok (acqState confirmed 'grab')", rc == 0 and rep.get("outcome") == "ok", rep)
    rc, rep = cmd("waitIdle", "--timeout-s", str(10 * SCALE))
    check("scanimage: waitIdle → ok once the grab ends", rc == 0, rep)
    rc, rep = cmd("get", "--property", "hSI.hRoiManager.scanZoomFactor")
    check("scanimage: a get the server never answers (stock bug B2) → outcome unknown, exit 3", rc == 3 and rep.get("outcome") == "unknown", (rc, rep))
    rc, rep = cmd("status")
    check("scanimage: the node reconciled after the unknown outcome → status ok again", rc == 0 and rep.get("outcome") == "ok", rep)
    rc, rep = cmd("get", "--property", "hSI.notOnTheAllowlist")
    check("scanimage: a property outside the allowlist is refused in the node (error), never sent", rc == 1 and rep.get("outcome") == "error", rep)
    st = L.take("/scanimage/status")
    check("scanimage: status lines carry reachable + acqState", len(st) >= 1 and all("reachable" in s for s in st), st[-1:])
    node.stop()
    mock.stop()


def judge_recorder(work: Path, typed) -> None:
    from std_msgs.msg import String
    from rclpy.serialization import serialize_message

    from rig_nodes.recorder import PROFILES, STRING_TYPE, Recorder

    bag = work / "conformance.bag"
    rec = Recorder(bag, {"/x/raw": STRING_TYPE, "/x/json": STRING_TYPE}, profile="crashsafe")
    dropped_before = rec.write_json("/x/json", {"too": "early"})
    check("recorder: a write while closed is refused and counted", dropped_before is False and rec.dropped == 1, rec.dropped)
    rec.open()
    n = 200
    for i in range(n):
        m = String()
        m.data = json.dumps({"src": "x", "seq": i + 1, "rx_epoch_ms": 1, "rx_mono_ns": i, "raw": f"{i}"})
        rec.write_raw("/x/raw", serialize_message(m), t_ns=1_000_000 * (i + 1), pub_ns=1_000_000 * (i + 1) - 500)
        rec.write_json("/x/json", {"seq": i + 1, "v": i * 0.5}, t_ns=1_000_000 * (i + 1))
    rep = rec.close()
    check("recorder: close() reports records and files", rep.get("records") == 2 * n and rep.get("files"), rep)
    check("recorder: crashsafe bag re-indexed on clean close", rep.get("indexed") and all("error" not in x for x in rep["indexed"]), rep.get("indexed"))
    from mcap.reader import make_reader

    counts: dict[str, int] = {}
    pub_lag_ok = True
    with open(bag / rep["files"][0], "rb") as f:
        r = make_reader(f)
        summ = r.get_summary()
        for _, ch, m in r.iter_messages():
            counts[ch.topic] = counts.get(ch.topic, 0) + 1
            if ch.topic == "/x/raw" and m.publish_time != m.log_time - 500:
                pub_lag_ok = False
    check("recorder: every record read back per topic, indexed (summary present)", counts == {"/x/raw": n, "/x/json": n} and summ is not None, counts)
    check("recorder: MCAP log_time = receive stamp, publish_time = the producer's own stamp", pub_lag_ok)
    check("recorder: both writer profiles are defined", set(PROFILES) == {"default", "crashsafe"}, list(PROFILES))


def main(argv: list[str] | None = None) -> int:
    global SCALE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-router", action="store_true", help="a Zenoh router is already running on 127.0.0.1:7447")
    ap.add_argument("--timeout-scale", type=float, default=1.0, help="multiply every wait (CI runners: 2)")
    ap.add_argument("--keep", action="store_true", help="keep the work dir with the process logs")
    ap.add_argument("--workdir", default=None)
    a = ap.parse_args(argv)
    SCALE = a.timeout_scale
    work = Path(a.workdir) if a.workdir else Path(tempfile.mkdtemp(prefix="rig-nodes-devtest-"))
    work.mkdir(parents=True, exist_ok=True)
    logdir = work / "proc-logs"
    logdir.mkdir(exist_ok=True)
    procs: list[Proc] = []
    print(f"[test] work dir {work}", file=sys.stderr)

    def start(name: str, argv: list[str]) -> Proc:
        pr = Proc(name, argv, logdir).start()
        procs.append(pr)
        return pr

    listener = None
    try:
        if not a.no_router:
            start("router", [PY, "-m", "rig_nodes.zenoh"])
            check("zenoh router listening on 127.0.0.1:7447", wait_port("127.0.0.1", 7447, 20.0 * SCALE))
        from rig_nodes._common import probe_interfaces

        typed = probe_interfaces()
        print(f"  info  typed topics: {typed is not None}", file=sys.stderr)
        listener = Listener(typed)
        time.sleep(1.0 * SCALE)  # discovery
        for name, fn in (("fictrac", lambda: judge_fictrac(listener, start, work, logdir, typed)),
                         ("kinefly", lambda: judge_kinefly(listener, start, logdir)),
                         ("bias", lambda: judge_bias(listener, start, work)),
                         ("scanimage", lambda: judge_scanimage(listener, start)),
                         ("recorder", lambda: judge_recorder(work, typed))):
            try:
                fn()
            except Exception as exc:
                check(f"{name}: group raised", False, repr(exc))
    finally:
        if listener is not None:
            listener.close()
        for pr in reversed(procs):
            pr.stop()
    n_ok = sum(1 for _, ok, _ in results if ok)
    print(f"\n[test] {n_ok}/{len(results)} checks passed", file=sys.stderr)
    for name, ok, info in results:
        if not ok:
            print(f"  FAIL  {name}: {str(info)[:400]}", file=sys.stderr)
    if not a.keep and n_ok == len(results):
        shutil.rmtree(work, ignore_errors=True)
    else:
        print(f"[test] logs kept in {logdir}", file=sys.stderr)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
