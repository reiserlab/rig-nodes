#!/usr/bin/env python3
"""scanimage_node.py — ScanImage as a ROS 2 node: protocol plugin commands in, replies + status out.

ScanImage (2026.0.0) has no ROS node. This adapter talks to its built-in JSON server
(`most.network.json.Server`, started in MATLAB with `hJ = most.network.json.Server(5555)`;
chosen over si_zmp on LAB-305's measurements: event-driven, no frame-timing disturbance,
~2 ms replies) and exposes it on three topics, all std_msgs/String JSON (no custom types):

  /scanimage/cmd     IN   {"id": <any>, "command": <plugin command>, "params": {…}}
  /scanimage/reply   OUT  {"id", "command", "params", "ok", "outcome": "ok"|"error"|"unknown", "value",
                           "error", "steps": [...per JSON request...], "rtt_ms", "t_ms", "rx_epoch_ms",
                           "artifact": {"kind":"tiff", "path", "stem", "counter"} (grab)}
  /scanimage/status  OUT  every --status-every s: {"reachable", "acqState", "lastFrameNumber",
                           "logFileCounter", "logFileStem", "logFilePath", "loggingEnable", "rtt_ms", "t_ms"}

The `command` vocabulary is the protocol's `scanimage` plugin (js/plugin-registry.js,
class "ScanImagePlugin" — MATLAB has no such class yet):

  setFileStem(stem)        hScan2D.logFileStem = stem (one TIFF series per stem)
  set(property, value)     ONE scalar property from SET_ALLOWLIST (framesPerSlice, logging on/off,
                           file path/stem/counter); anything else is refused here, never sent
  grab                     eval hSI.startGrab(), then confirm acqState == 'grab'
  waitIdle(timeout_s)      poll acqState until 'idle' (the grab finished) or timeout_s (default 30)
  abort                    eval hSI.abort(), then confirm acqState == 'idle'
  get(property)            ONE scalar property from GET_ALLOWLIST (also what status polls)
  ping / status            reachability + the status snapshot

Why the allowlists (LAB-305, stock server on the rig, patches reverted): B2 — a `get` of an
OBJECT-valued property gets no reply and the client hangs; B4 — a partial message blocks
MATLAB for ~10 s, so every request is one length-prefixed write; the server is single-client,
so this node is the ONLY client and status polling is coalesced; `eval` exists on the server
but this node sends exactly two fixed expressions (startGrab, abort) and never a string from
a protocol. The JSON client is ported from tests/remote-control/si_remote.py (JsonClient) in
reiserlab/scanimage-2026-bergamo (Hannah Marie Santos), with attribution.

LAB-303 semantics (as bias_node.py): `timeout_s` per request (`slow_timeout_s` for grab/abort/
startFocus — ScanImage's startGrab is synchronous and takes 1.2–2.4 s on the Bergamo PC); a
timeout is an UNKNOWN outcome (`outcome: "unknown"`), the TCP connection is dropped and
rebuilt (the stream is out of sync after a timeout), and the node reconciles with a status
read before anything else. `critical` / `on_loss` are the runner's decision — the node reports.

    pixi run scanimage-node -- --host 127.0.0.1 --port 5555
    pixi run scanimage-cmd -- status
    pixi run scanimage-cmd -- setFileStem --stem violet_d7
    pixi run scanimage-cmd -- grab && pixi run scanimage-cmd -- waitIdle --timeout-s 40
"""

from __future__ import annotations

import argparse
import json
import queue
import socket
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from _common import RELIABLE_QOS, raise_timer_resolution, spin, wall_ms

CMD_TOPIC = "/scanimage/cmd"
REPLY_TOPIC = "/scanimage/reply"
STATUS_TOPIC = "/scanimage/status"
QOS = RELIABLE_QOS
PLUGIN_COMMANDS = ("setFileStem", "set", "grab", "waitIdle", "abort", "get", "ping", "status")

# Scalar properties this node will read. Everything here encodes as a JSON scalar or a flat
# numeric vector in ScanImage 2026.0.0 — never an object (B2).
GET_ALLOWLIST = frozenset({
    "hSI.acqState",
    "hSI.acqsPerLoop",
    "hSI.hDisplay.lastFrameNumber",
    "hSI.hScan2D.logFileStem",
    "hSI.hScan2D.logFilePath",
    "hSI.hScan2D.logFileCounter",
    "hSI.hScan2D.logAverageFactor",
    "hSI.hScan2D.bidirectional",
    "hSI.hChannels.loggingEnable",
    "hSI.hChannels.channelSave",
    "hSI.hStackManager.framesPerSlice",
    "hSI.hStackManager.numSlices",
    "hSI.hRoiManager.scanFrameRate",
    "hSI.hRoiManager.scanZoomFactor",
    "hSI.hRoiManager.linesPerFrame",
    "hSI.hRoiManager.pixelsPerLine",
})
# Scalar properties a protocol may set. Deliberately small: scan geometry, zoom, channels,
# power, gate windows (B3) and anything saved to the MDF stay in ScanImage's own UI/config.
SET_ALLOWLIST = frozenset({
    "hSI.hScan2D.logFileStem",
    "hSI.hScan2D.logFilePath",
    "hSI.hScan2D.logFileCounter",
    "hSI.hStackManager.framesPerSlice",
    "hSI.hChannels.loggingEnable",
})
# The only two expressions ever sent as `eval` (plus startFocus for the CLI's bench check).
EVAL_GRAB = "hSI.startGrab()"
EVAL_ABORT = "hSI.abort()"
EVAL_FOCUS = "hSI.startFocus()"
SLOW_COMMANDS = frozenset({"grab", "abort", "focus"})
STATUS_PROPS = ("hSI.acqState", "hSI.hDisplay.lastFrameNumber", "hSI.hScan2D.logFileCounter",
                "hSI.hScan2D.logFileStem", "hSI.hScan2D.logFilePath", "hSI.hChannels.loggingEnable")


def canon(prop: str) -> str:
    """'hScan2D.logFileStem' and 'hSI.hScan2D.logFileStem' name the same property."""
    prop = str(prop or "").strip()
    return prop if prop.startswith("hSI.") or not prop else "hSI." + prop


def coerce_scalar(value):
    """A protocol's `set` value arrives as text from the designer ("640", "true", "F:\\data"):
    make it the JSON scalar the ScanImage property wants — int, float, bool — and leave real
    text (paths, stems) alone. Non-strings pass through unchanged."""
    if not isinstance(value, str):
        return value
    t = value.strip()
    low = t.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(t)
    except ValueError:
        pass
    try:
        f = float(t)
        return f if t.lower() not in ("nan", "inf", "-inf", "+inf") else value
    except ValueError:
        return value


class SiError(RuntimeError):
    """The server replied with an error."""


class SiTimeout(TimeoutError):
    """No (complete) reply within the timeout — an UNKNOWN outcome."""


class JsonClient:
    """ScanImage's JSON server over TCP: `<Q` little-endian byte count + UTF-8 JSON each way.
    Persistent connection (the server is single-client; one connect per node). Ported from
    si_remote.JsonClient (reiserlab/scanimage-2026-bergamo, tests/remote-control/si_remote.py)."""

    def __init__(self, host: str, port: int, timeout_s: float, slow_timeout_s: float | None = None) -> None:
        self.host, self.port = host, port
        self.timeout_s = timeout_s
        self.slow_timeout_s = max(timeout_s, slow_timeout_s if slow_timeout_s is not None else timeout_s)
        self._sock: socket.socket | None = None
        self.reconnects = 0

    @property
    def base(self) -> str:
        return f"json://{self.host}:{self.port}"

    def _connect(self) -> None:
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def drop(self) -> None:
        """Close the connection; the next request reconnects. REQUIRED after a timeout: the
        stream may hold a late reply that would be read as the answer to the next request."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self.reconnects += 1

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("ScanImage JSON server closed the connection")
            buf += chunk
        return buf

    def request(self, msg: dict, timeout_s: float | None = None) -> tuple[dict, float]:
        """→ (reply dict, rtt_ms). Raises SiTimeout (connection dropped), SiError, OSError."""
        data = json.dumps(msg, separators=(",", ":")).encode("utf-8")
        payload = struct.pack("<Q", len(data)) + data  # header + body in ONE write (B4)
        to = timeout_s if timeout_s is not None else self.timeout_s
        t0 = time.perf_counter()
        try:
            if self._sock is None:
                self._connect()
            assert self._sock is not None
            self._sock.settimeout(to)
            self._sock.sendall(payload)
            n = struct.unpack("<Q", self._recv_exact(8))[0]
            body = self._recv_exact(n)
        except socket.timeout as exc:
            self.drop()
            raise SiTimeout(f"no complete reply within {to:g}s") from exc
        except (ConnectionError, OSError) as exc:
            self.drop()
            raise OSError(f"ScanImage JSON server unreachable at {self.host}:{self.port}: {exc}") from exc
        rtt = (time.perf_counter() - t0) * 1000.0
        try:
            rep = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.drop()
            raise OSError(f"non-JSON reply from the ScanImage JSON server: {body[:80]!r}") from exc
        if not isinstance(rep, dict):
            raise OSError(f"unexpected reply from the ScanImage JSON server: {rep!r}")
        if rep.get("error"):
            raise SiError(str(rep["error"]))
        return rep, round(rtt, 2)

    def get(self, prop: str, timeout_s: float | None = None) -> tuple[object, float]:
        rep, rtt = self.request({"command": "get", "property": prop}, timeout_s)
        return rep.get("value"), rtt

    def set(self, prop: str, value, timeout_s: float | None = None) -> tuple[object, float]:
        rep, rtt = self.request({"command": "set", "property": prop, "value": value}, timeout_s)
        return rep.get("actual", value), rtt

    def eval(self, expr: str, timeout_s: float | None = None) -> float:
        _, rtt = self.request({"command": "eval", "function": expr, "num_outputs": 0}, timeout_s)
        return rtt


class ScanImageNode(Node):
    def __init__(self, cli: JsonClient, status_every_s: float, poll_s: float, default_wait_s: float) -> None:
        super().__init__("scanimage")
        self.cli = cli
        self.poll_s = poll_s
        self.default_wait_s = default_wait_s
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, QOS)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, QOS)
        self.create_subscription(String, CMD_TOPIC, self._on_cmd, QOS)
        self._q: queue.Queue = queue.Queue()
        self._status_pending = False  # at most ONE status poll waits in the queue (coalesced)
        self._worker = threading.Thread(target=self._work, name="scanimage-json", daemon=True)
        self._worker.start()
        if status_every_s > 0:
            self.create_timer(status_every_s, self._enqueue_status)
        self.commands = 0
        self.get_logger().info(f"ScanImage JSON server at {cli.host}:{cli.port} ← {CMD_TOPIC} → {REPLY_TOPIC}, {STATUS_TOPIC} every {status_every_s:g}s")

    # ── plumbing ────────────────────────────────────────────────────────────────────────
    def _publish(self, pub, obj: dict) -> None:
        obj.setdefault("rx_epoch_ms", wall_ms())
        m = String()
        m.data = json.dumps(obj, separators=(",", ":"), default=str)
        pub.publish(m)

    def _on_cmd(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish(self.reply_pub, {"id": None, "ok": False, "outcome": "error", "error": "command is not JSON"})
            return
        if isinstance(req, dict):
            self._q.put(("cmd", req))

    def _enqueue_status(self) -> None:
        if not self._status_pending:
            self._status_pending = True
            self._q.put(("status", None))

    def _work(self) -> None:
        while True:
            kind, payload = self._q.get()
            try:
                if kind == "status":
                    self._status_pending = False
                    self._do_status()
                else:
                    self._do_cmd(payload)
            except Exception as exc:  # must not kill the worker
                self.get_logger().error(f"{kind} failed: {exc!r}")
                if kind == "cmd":
                    self._publish(self.reply_pub, {"id": (payload or {}).get("id"), "command": (payload or {}).get("command"),
                                                   "ok": False, "outcome": "error", "error": f"node error: {exc!r}"})

    # ── status ──────────────────────────────────────────────────────────────────────────
    def _snapshot(self, steps: list | None = None) -> dict:
        """The status snapshot: one `get` per STATUS_PROPS (all scalar). acqState must read; any
        other property that ScanImage answers with an error is None + `errors[prop]` — e.g.
        hDisplay.lastFrameNumber throws "Index exceeds array bounds" until the first acquisition
        after ScanImage starts (Bergamo, 2026-10-16). Timeouts and socket errors still raise."""
        out: dict = {}
        errors: dict = {}
        rtt_total = 0.0
        for prop in STATUS_PROPS:
            try:
                v, rtt = self.cli.get(prop)
            except SiError as exc:
                if prop == "hSI.acqState":
                    raise
                out[prop.split(".")[-1]] = None
                errors[prop] = str(exc).splitlines()[0]
                if steps is not None:
                    steps.append({"command": "get", "property": prop, "ok": False, "error": errors[prop]})
                continue
            rtt_total += rtt
            out[prop.split(".")[-1]] = v
            if steps is not None:
                steps.append({"command": "get", "property": prop, "ok": True, "value": v, "rtt_ms": rtt})
        if errors:
            out["errors"] = errors
        out["rtt_ms"] = round(rtt_total, 2)
        return out

    def _do_status(self) -> dict | None:
        t_ms = wall_ms()
        try:
            st = self._snapshot()
            self._publish(self.status_pub, {"reachable": True, "t_ms": t_ms, **st})
            return st
        except (SiTimeout, SiError, OSError) as exc:
            self._publish(self.status_pub, {"reachable": False, "t_ms": t_ms, "error": str(exc)})
            return None

    # ── commands ────────────────────────────────────────────────────────────────────────
    def _do_cmd(self, req: dict) -> None:
        self.commands += 1
        cid = req.get("id")
        command = str(req.get("command", ""))
        params = req.get("params") if isinstance(req.get("params"), dict) else {}
        t_ms = wall_ms()
        t0 = time.perf_counter()
        out: dict = {"id": cid, "command": command, "params": params, "t_ms": t_ms, "steps": []}
        steps = out["steps"]
        try:
            if command not in PLUGIN_COMMANDS:
                raise ValueError(f"unknown plugin command {command!r} (one of {PLUGIN_COMMANDS})")
            value = self._run(command, params, steps)
            out.update({"ok": True, "outcome": "ok", "value": value})
            if command == "grab" and isinstance(value, dict):
                out["artifact"] = {"kind": "tiff", "program": "scanimage", "path": value.get("logFilePath"),
                                   "stem": value.get("logFileStem"), "counter": value.get("logFileCounter")}
        except SiTimeout as exc:
            # UNKNOWN: the request may or may not have taken effect (the eval may have started
            # the grab). The connection was dropped; reconcile with a status read, and say so.
            steps.append({"ok": None, "error": str(exc)})
            out.update({"ok": False, "outcome": "unknown", "error": str(exc)})
            st = self._do_status()
            if st is not None:
                out["reconciled"] = st
        except SiError as exc:
            steps.append({"ok": False, "error": str(exc)})
            out.update({"ok": False, "outcome": "error", "error": f"ScanImage: {exc}"})
        except (ValueError, TypeError) as exc:
            out.update({"ok": False, "outcome": "error", "error": str(exc)})
        except OSError as exc:
            steps.append({"ok": False, "error": str(exc)})
            out.update({"ok": False, "outcome": "error", "error": str(exc)})
        out["rtt_ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
        self._publish(self.reply_pub, out)
        if out["ok"]:
            self.get_logger().info(f"{command} → ok ({out['rtt_ms']} ms)")
        else:
            self.get_logger().warning(f"{command} → {out['outcome']} ({out['rtt_ms']} ms): {out.get('error')}")

    def _get(self, prop: str, steps: list):
        prop = canon(prop)
        if prop not in GET_ALLOWLIST:
            raise ValueError(f"{prop!r} is not a readable scalar property (allowlist: {sorted(GET_ALLOWLIST)})")
        v, rtt = self.cli.get(prop)
        steps.append({"command": "get", "property": prop, "ok": True, "value": v, "rtt_ms": rtt})
        return v

    def _set(self, prop: str, value, steps: list):
        prop = canon(prop)
        if prop not in SET_ALLOWLIST:
            raise ValueError(f"{prop!r} may not be set from a protocol (allowlist: {sorted(SET_ALLOWLIST)})")
        if isinstance(value, (dict, list)):
            raise ValueError(f"{prop}: only scalar values may be set")
        value = coerce_scalar(value)
        actual, rtt = self.cli.set(prop, value)
        steps.append({"command": "set", "property": prop, "value": value, "ok": True, "actual": actual, "rtt_ms": rtt})
        return actual

    def _acq_state(self, steps: list | None = None) -> str:
        v, rtt = self.cli.get("hSI.acqState")
        if steps is not None:
            steps.append({"command": "get", "property": "hSI.acqState", "ok": True, "value": v, "rtt_ms": rtt})
        return str(v)

    def _wait_state(self, want, timeout_s: float, steps: list) -> str:
        """Poll acqState every poll_s until it is in `want` (a set) or timeout_s elapses. Returns
        the last state. Polling, not blocking: each poll is one ~2 ms request (LAB-305: polling
        the JSON server at 1–20 Hz did not disturb frame timing)."""
        end = time.monotonic() + timeout_s
        polls = 0
        while True:
            st = self._acq_state()
            polls += 1
            if st in want or time.monotonic() >= end:
                steps.append({"command": "poll", "property": "hSI.acqState", "ok": st in want, "value": st, "polls": polls})
                return st
            time.sleep(self.poll_s)

    def _run(self, command: str, params: dict, steps: list):
        if command == "ping":
            st = self._acq_state(steps)
            return {"acqState": st}
        if command == "status":
            return self._snapshot(steps)
        if command == "get":
            return self._get(params.get("property", ""), steps)
        if command == "set":
            if "property" not in params or "value" not in params:
                raise ValueError("set needs params.property and params.value")
            return self._set(params["property"], params["value"], steps)
        if command == "setFileStem":
            stem = str(params.get("stem") or params.get("filename") or "").strip()
            if not stem:
                raise ValueError("setFileStem needs params.stem")
            actual = self._set("hSI.hScan2D.logFileStem", stem, steps)
            return {"logFileStem": actual, "logFileCounter": self._get("hSI.hScan2D.logFileCounter", steps)}
        if command == "grab":
            before = self._acq_state(steps)
            if before != "idle":
                raise SiError(f"cannot start a grab while acqState is '{before}' (abort first)")
            stem = self._get("hSI.hScan2D.logFileStem", steps)
            path = self._get("hSI.hScan2D.logFilePath", steps)
            counter = self._get("hSI.hScan2D.logFileCounter", steps)
            rtt = self.cli.eval(EVAL_GRAB, self.cli.slow_timeout_s)
            steps.append({"command": "eval", "function": EVAL_GRAB, "ok": True, "rtt_ms": rtt})
            # Confirm: ScanImage's startGrab is synchronous, so 'grab' should be immediate; allow
            # a short window (a busy GUI thread) before calling the outcome an error.
            st = self._wait_state({"grab", "loop"}, min(5.0, self.cli.slow_timeout_s), steps)
            if st not in ("grab", "loop"):
                raise SiError(f"startGrab returned but acqState is '{st}'")
            return {"acqState": st, "logFileStem": stem, "logFilePath": path, "logFileCounter": counter, "start_rtt_ms": rtt}
        if command == "waitIdle":
            to = params.get("timeout_s", self.default_wait_s)
            try:
                to = float(to)
            except (TypeError, ValueError):
                raise ValueError(f"waitIdle: timeout_s must be a number, got {to!r}")
            t0 = time.monotonic()
            st = self._wait_state({"idle"}, to, steps)
            waited = round(time.monotonic() - t0, 3)
            if st != "idle":
                raise SiError(f"still '{st}' after {waited:g} s (timeout_s {to:g}); use abort")
            # lastFrameNumber is informational: on the Bergamo (2026.0.0) it throws "Index exceeds
            # array bounds" even right after a grab, so it must not turn a reached idle into an error.
            try:
                last = self._get("hSI.hDisplay.lastFrameNumber", steps)
            except SiError as exc:
                last = None
                steps.append({"command": "get", "property": "hSI.hDisplay.lastFrameNumber", "ok": False,
                              "error": str(exc).splitlines()[0]})
            return {"acqState": st, "waited_s": waited, "lastFrameNumber": last,
                    "logFileCounter": self._get("hSI.hScan2D.logFileCounter", steps)}
        if command == "abort":
            rtt = self.cli.eval(EVAL_ABORT, self.cli.slow_timeout_s)
            steps.append({"command": "eval", "function": EVAL_ABORT, "ok": True, "rtt_ms": rtt})
            st = self._wait_state({"idle"}, min(5.0, self.cli.slow_timeout_s), steps)
            if st != "idle":
                raise SiError(f"abort returned but acqState is '{st}'")
            return {"acqState": st}
        raise ValueError(f"unhandled command {command!r}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="ScanImage JSON server host (default: 127.0.0.1 — keep it local; the server binds all interfaces)")
    p.add_argument("--port", type=int, default=5555, help="ScanImage JSON server port (default: 5555)")
    p.add_argument("--timeout-s", type=float, default=2.0, help="per-request timeout; a timeout is an UNKNOWN outcome and drops the connection (default: 2)")
    p.add_argument("--slow-timeout-s", type=float, default=15.0, help="timeout for grab/abort (ScanImage's startGrab is synchronous, 1.2–2.4 s on the Bergamo PC; default: 15)")
    p.add_argument("--status-every", type=float, default=1.0, help="status poll period in s; 0 = off (default: 1 — six ~2 ms gets)")
    p.add_argument("--poll-s", type=float, default=0.25, help="acqState poll period inside grab/waitIdle/abort (default: 0.25)")
    p.add_argument("--wait-s", type=float, default=30.0, help="waitIdle default timeout_s when the command carries none (default: 30)")
    args, ros_args = p.parse_known_args(argv)
    raise_timer_resolution()
    rclpy.init(args=ros_args)
    node = ScanImageNode(JsonClient(args.host, args.port, args.timeout_s, args.slow_timeout_s), args.status_every, args.poll_s, args.wait_s)
    spin(node)
    print(f"[scanimage-node] {node.commands} commands handled, {node.cli.reconnects} reconnects", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
