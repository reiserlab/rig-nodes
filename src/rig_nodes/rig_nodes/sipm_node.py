#!/usr/bin/env python3
"""sipm_node.py — the Thorlabs SiPM controller as a ROS 2 node, READ-ONLY (Bergamo rig).

EXPERIMENTAL (rig-nodes 0.3.0, LAB-319): moved from webDisplayTools PR #266 with paths and imports
changed only; it has run against `thorsipm.FakeBackend` on macOS, NEVER against the real controller or
on Windows. The parameter IDs for bias and temperature are unknown until the Block S scan on the Bergamo
PC. Treat the vocabulary below as provisional until then.

Neither ScanImage nor the TIFF header knows the SiPM settings (bias, enable, offsets, bandwidth,
temperature). This node reads them through the vendor DLL (rig_nodes.thorsipm, no SetParam
anywhere) and publishes them, so they land in the run's bag and run log next to ScanImage and the
arena. Same connector shape as bias_node.py / scanimage_node.py, all std_msgs/String JSON:

  /sipm/cmd     IN   {"id": <any>, "command": <cmd>, "params": {…}}
  /sipm/reply   OUT  {"id", "command", "params", "ok", "outcome": "ok"|"error", "value", "error",
                      "rtt_ms", "t_ms", "rx_epoch_ms"}
  /sipm/status  OUT  every --status-every s: {"reachable", "connected", "level": "ok"|"warn"|"error",
                      "warnings", "read_ms", "t_ms", "connected_detectors", "control_unit",
                      "detectors": {"1": {enable, gain_pos, gain_volts, output_offset, bandwidth_pos,
                      safety, saturations, detector_type, serial, …}, …}, "extra": {name: value}}

Commands (none changes a detector setting):
  ping / status            the status snapshot now
  get(id | name)           one parameter by ID, or by a name from --params-file
  info(id)                 GetParamInfo for one ID
  scan(from, to)           every available ID in the range with its value (Block S, step S2)
  disconnect               TeardownDevice: frees the COM port (e.g. to reopen the vendor GUI);
                           the node then stays disconnected until `reconnect`
  reconnect                open the controller again

The node OWNS the controller while connected: ThorSiPM_Control.exe must be closed (one program
per COM port). If the port is busy the node keeps publishing `connected: false` with the reason
and retries every --reconnect-every s. Every DLL call runs on one worker thread.

    pixi run sipm-node -- --dll-dir "C:\\…\\ThorSiPM_Unreleased_9_25_25_1\\Release" --params-file sipm_params.json
    pixi run sipm-node -- --fake                 # off-rig: the simulated controller
    pixi run sipm-cmd -- scan --from 0 --to 2000 --json
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rig_nodes._common import RELIABLE_QOS, raise_timer_resolution, spin, wall_ms
from rig_nodes.thorsipm import KNOWN_PARAMS, SiPMError, ThorSiPM, load_params_file

CMD_TOPIC = "/sipm/cmd"
REPLY_TOPIC = "/sipm/reply"
STATUS_TOPIC = "/sipm/status"
QOS = RELIABLE_QOS
COMMANDS = ("ping", "status", "get", "info", "scan", "disconnect", "reconnect")


def assess(snap: dict) -> tuple[str, list[str]]:
    """Health from a snapshot: 'warn' on a tripped/saturated detector or a non-zero temperature
    status (extra names containing 'tempstatus'); the raw values stay in the record."""
    warnings = []
    for d, rec in (snap.get("detectors") or {}).items():
        if rec.get("safety") not in (None, 0, 0.0):
            warnings.append(f"detector {d}: safety={rec['safety']}")
        if rec.get("saturations") not in (None, 0, 0.0):
            warnings.append(f"detector {d}: saturations={rec['saturations']}")
    for name, v in (snap.get("extra") or {}).items():
        if "tempstatus" in name.lower() and v not in (None, 0, 0.0):
            warnings.append(f"{name}={v}")
    return ("warn" if warnings else "ok"), warnings


class SiPMNode(Node):
    def __init__(self, dev: ThorSiPM, extra: dict[str, int], status_every_s: float, reconnect_every_s: float) -> None:
        super().__init__("sipm")
        self.dev = dev
        self.extra = extra
        self.reconnect_every_s = reconnect_every_s
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, QOS)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, QOS)
        self.create_subscription(String, CMD_TOPIC, self._on_cmd, QOS)
        self._q: queue.Queue = queue.Queue()
        self._status_pending = False
        self._hold = False            # True after `disconnect`: no automatic reconnect
        self._last_try = 0.0
        self._last_error: str | None = None
        self.commands = 0
        self._stopping = False        # set by shutdown(): the node is destroyed, publish nothing more
        self._worker = threading.Thread(target=self._work, name="sipm-dll", daemon=True)
        self._worker.start()
        self._q.put(("connect", None))
        if status_every_s > 0:
            self.create_timer(status_every_s, self._enqueue_status)
        self.get_logger().info(f"SiPM controller ← {CMD_TOPIC} → {REPLY_TOPIC}, {STATUS_TOPIC} every {status_every_s:g}s; "
                               f"{len(extra)} extra param(s): {', '.join(extra) or '-'}")

    # ── publishing ───────────────────────────────────────────────────────────────────────────
    def _publish(self, pub, obj: dict) -> None:
        if self._stopping:
            return
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
        if not self._status_pending:  # coalesce: one poll waiting at most
            self._status_pending = True
            self._q.put(("status", None))

    # ── the worker: every DLL call happens here ──────────────────────────────────────────────
    def _work(self) -> None:
        while True:
            kind, payload = self._q.get()
            try:
                if kind == "connect":
                    self._connect()
                elif kind == "status":
                    self._status_pending = False
                    self._do_status()
                elif kind == "stop":
                    self.dev.close()
                    return
                else:
                    self._do_cmd(payload)
            except Exception as exc:  # a DLL surprise must not kill the worker
                self.get_logger().error(f"{kind} failed: {exc!r}")
                if kind == "cmd":
                    self._publish(self.reply_pub, {"id": (payload or {}).get("id"), "command": (payload or {}).get("command"),
                                                   "ok": False, "outcome": "error", "error": f"node error: {exc!r}"})

    def _connect(self) -> bool:
        self._last_try = time.monotonic()
        try:
            n = self.dev.open()
        except SiPMError as exc:
            if str(exc) != self._last_error:
                self.get_logger().warning(f"not connected: {exc}")
            self._last_error = str(exc)
            return False
        self._last_error = None
        self.get_logger().info(f"connected ({n} controller(s))")
        return True

    def _snapshot(self) -> dict:
        t_ms = wall_ms()
        if not self.dev.is_open and not self._hold and time.monotonic() - self._last_try >= self.reconnect_every_s:
            self._connect()
        if not self.dev.is_open:
            return {"reachable": False, "connected": False, "held": self._hold, "level": "error",
                    "error": "disconnected by command" if self._hold else (self._last_error or "not connected"), "t_ms": t_ms}
        t0 = time.perf_counter()
        try:
            snap = self.dev.snapshot(self.extra)
        except SiPMError as exc:
            self.dev.close()
            self._last_error = str(exc)
            return {"reachable": False, "connected": False, "level": "error", "error": str(exc), "t_ms": t_ms}
        level, warnings = assess(snap)
        return {"reachable": True, "connected": True, "level": level, "warnings": warnings,
                "read_ms": round((time.perf_counter() - t0) * 1000.0, 2), "t_ms": t_ms, **snap}

    def _do_status(self) -> None:
        self._publish(self.status_pub, self._snapshot())

    def _resolve_id(self, params: dict) -> int:
        if "id" in params:
            return int(params["id"])
        name = str(params.get("name", ""))
        if name in self.extra:
            return self.extra[name]
        by_name = {v: k for k, v in KNOWN_PARAMS.items()}
        if name in by_name:
            return by_name[name]
        raise ValueError(f"unknown parameter {name!r}: give `id`, a --params-file name, or a PARAM_* name")

    def _do_cmd(self, req: dict) -> None:
        self.commands += 1
        cid, command, params = req.get("id"), str(req.get("command", "")), req.get("params") or {}
        out: dict = {"id": cid, "command": command, "params": params, "t_ms": wall_ms()}
        t0 = time.perf_counter()
        try:
            if command not in COMMANDS:
                raise ValueError(f"unknown command {command!r} (one of {COMMANDS})")
            if command == "disconnect":
                self.dev.close()
                self._hold = True
                value = {"connected": False}
            elif command == "reconnect":
                self._hold = False
                self.dev.close()
                if not self._connect():
                    raise SiPMError(self._last_error or "reconnect failed")
                value = {"connected": True}
            elif command in ("ping", "status"):
                value = self._snapshot()
                self._publish(self.status_pub, dict(value))
            else:
                if not self.dev.is_open:
                    raise SiPMError("not connected" + (f": {self._last_error}" if self._last_error else ""))
                if command == "get":
                    pid = self._resolve_id(params)
                    value = {"id": pid, "name": KNOWN_PARAMS.get(pid), "value": self.dev.get(pid)}
                elif command == "info":
                    pid = self._resolve_id(params)
                    value = {"id": pid, "name": KNOWN_PARAMS.get(pid), "info": self.dev.info(pid)}
                else:  # scan
                    first, last = int(params.get("from", 0)), int(params.get("to", 2000))
                    if last - first > 5000:
                        raise ValueError("scan range limited to 5000 IDs")
                    rows = self.dev.scan(first, last)
                    value = {"range": [first, last], "available": rows, "not_in_enum": [r["id"] for r in rows if not r["name"]]}
            out.update({"ok": True, "outcome": "ok", "value": value})
        except (SiPMError, ValueError) as exc:
            out.update({"ok": False, "outcome": "error", "error": str(exc)})
        out["rtt_ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
        self._publish(self.reply_pub, out)
        if out["ok"]:
            self.get_logger().info(f"{command} → ok ({out['rtt_ms']} ms)")
        else:
            self.get_logger().warning(f"{command} → error ({out['rtt_ms']} ms): {out.get('error')}")

    def shutdown(self, timeout_s: float = 3.0) -> None:
        """Free the COM port on exit (TeardownDevice on the worker thread)."""
        self._stopping = True
        self._q.put(("stop", None))
        self._worker.join(timeout_s)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dll-dir", help="folder with ThorSiPM.dll + ThorSiPMSettings.xml (the vendor GUI's Release folder)")
    p.add_argument("--fake", action="store_true", help="use the simulated controller (off-rig tests)")
    p.add_argument("--fake-busy", action="store_true", help="--fake with the COM port 'held by the GUI' (SelectDevice fails)")
    p.add_argument("--params-file", help="JSON {name: id} of extra params to publish, e.g. the SiPM bias/temperature IDs from a scan")
    p.add_argument("--status-every", type=float, default=1.0, help="status period in s; 0 = off (default: 1)")
    p.add_argument("--reconnect-every", type=float, default=5.0, help="retry period while not connected, s (default: 5)")
    args, ros_args = p.parse_known_args(argv)
    try:
        dev = ThorSiPM.from_args(args.dll_dir, fake=args.fake or args.fake_busy, fake_busy=args.fake_busy)
        extra = load_params_file(args.params_file)
    except (SiPMError, OSError, ValueError) as exc:
        print(f"[sipm-node] {exc}", file=sys.stderr)
        return 2
    raise_timer_resolution()
    rclpy.init(args=ros_args)
    node = SiPMNode(dev, extra, args.status_every, args.reconnect_every)
    try:
        spin(node)
    finally:
        node.shutdown()
    print(f"[sipm-node] {node.commands} commands handled; controller released", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
