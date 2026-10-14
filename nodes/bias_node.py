#!/usr/bin/env python3
"""bias_node.py — the BIAS camera as a ROS 2 node: protocol plugin commands in, replies + status out.

BIAS has no ROS node; this adapter talks to its external-control HTTP interface
(`http://host:port/?command[=value]`, JSON-array replies — see bias_sim.py) and exposes it
on three topics, all std_msgs/String JSON (no custom message types):

  /bias/cmd     IN   {"id": <any>, "command": <plugin command>, "params": {…}}
  /bias/reply   OUT  {"id", "command", "ok", "outcome": "ok"|"error"|"unknown", "value",
                      "error", "steps": [...per HTTP command...], "rtt_ms", "t_ms",
                      "rx_epoch_ms", "artifact": {"kind":"video","path":…} (startRecording)}
  /bias/status  OUT  every --status-every s: {"reachable", "connected", "capturing",
                      "logging", "frameCount", "framesPerSec", "videoFile", "rtt_ms", …}

The `command` vocabulary is the protocol's `camera` plugin (js/plugin-registry.js
BiasPlugin): connect(ip, port, config_file), startRecording(filename), stopRecording,
startPreview, stopCapture, getTimestamp, disconnect — each mapped to the BIAS HTTP
sequence `BiasPlugin.m` performs. The bridge node logs every reply and status line into the
run log as `{"type":"connector","src":"bias",…}` events, so the video file identity
(`artifact`) and every command/reply pair sit in the same file as the FicTrac rows.

LAB-303 semantics carried here: `timeout_s` per HTTP call; a timeout is an UNKNOWN outcome
(`outcome: "unknown"`), after which the node reconciles with `get-status` before anything
else. `critical` / `on_loss` are the runner's decision (the Studio side, shared by both
plans) — the node only reports.

    pixi run bias-node -- --host 127.0.0.1 --port 5010
    pixi run bias-cmd -- startRecording --filename trial_001.avi
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

CMD_TOPIC = "/bias/cmd"
REPLY_TOPIC = "/bias/reply"
STATUS_TOPIC = "/bias/status"
QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    durability=QoSDurabilityPolicy.VOLATILE,
)
PLUGIN_COMMANDS = ("connect", "startRecording", "stopRecording", "startPreview", "stopCapture", "getTimestamp", "disconnect")


class BiasHttp:
    """Blocking HTTP client for BIAS's command port. One command per request."""

    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s

    @property
    def base(self) -> str:
        return f"http://{self.host}:{self.port}/"

    def call(self, command: str, value: str | None = None) -> dict:
        """→ {"command", "ok", "message", "value", "rtt_ms"} or raises TimeoutError / OSError."""
        q = command if value is None else f"{command}={urllib.parse.quote(str(value), safe='/:\\')}"
        url = f"{self.base}?{q}"
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                body = resp.read()
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, TimeoutError) or "timed out" in str(exc.reason):
                raise TimeoutError(str(exc)) from exc
            raise OSError(str(exc)) from exc
        except TimeoutError:
            raise
        rtt = (time.monotonic() - t0) * 1000.0
        try:
            arr = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise OSError(f"non-JSON reply from BIAS: {body[:80]!r}")
        rep = arr[0] if isinstance(arr, list) and arr else (arr if isinstance(arr, dict) else {})
        return {
            "command": command,
            "ok": bool(rep.get("success", False)),
            "message": rep.get("message", ""),
            "value": rep.get("value", None),
            "rtt_ms": round(rtt, 2),
        }


def plan(command: str, params: dict) -> list[tuple[str, str | None]]:
    """Plugin command → BIAS HTTP steps (the sequence BiasPlugin.m performs)."""
    params = params or {}
    if command == "connect":
        steps: list[tuple[str, str | None]] = [("connect", None)]
        if params.get("config_file"):
            steps.append(("load-configuration", str(params["config_file"])))
        return steps
    if command == "startRecording":
        steps = []
        if params.get("filename"):
            steps.append(("set-video-file", str(params["filename"])))
        steps += [("enable-logging", None), ("start-capture", None), ("get-video-file", None)]
        return steps
    if command == "stopRecording":
        return [("stop-capture", None), ("disable-logging", None)]
    if command == "startPreview":
        return [("disable-logging", None), ("start-capture", None)]
    if command == "stopCapture":
        return [("stop-capture", None)]
    if command == "getTimestamp":
        return [("get-time-stamp", None)]
    if command == "disconnect":
        return [("disconnect", None)]
    raise ValueError(f"unknown plugin command {command!r} (one of {PLUGIN_COMMANDS})")


class BiasNode(Node):
    def __init__(self, http: BiasHttp, status_every_s: float) -> None:
        super().__init__("bias_camera")
        self.http = http
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, QOS)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, QOS)
        self.create_subscription(String, CMD_TOPIC, self._on_cmd, QOS)
        self._q: queue.Queue = queue.Queue()
        self._worker = threading.Thread(target=self._work, name="bias-http", daemon=True)
        self._worker.start()
        self._status_every = status_every_s
        if status_every_s > 0:
            self.create_timer(status_every_s, lambda: self._q.put(("status", None)))
        self.commands = 0
        self.get_logger().info(f"BIAS at {http.base} ← {CMD_TOPIC} → {REPLY_TOPIC}, {STATUS_TOPIC} every {status_every_s:g}s")

    def _publish(self, pub, obj: dict) -> None:
        obj.setdefault("rx_epoch_ms", int(time.time() * 1000))
        m = String()
        m.data = json.dumps(obj, separators=(",", ":"), default=str)
        pub.publish(m)

    def _on_cmd(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish(self.reply_pub, {"id": None, "ok": False, "outcome": "error", "error": "command is not JSON"})
            return
        if not isinstance(req, dict):
            return
        self._q.put(("cmd", req))

    def _work(self) -> None:
        while True:
            kind, payload = self._q.get()
            if kind == "status":
                self._do_status()
            else:
                self._do_cmd(payload)

    def _do_status(self) -> None:
        t_ms = int(time.time() * 1000)
        try:
            r = self.http.call("get-status")
            st = r["value"] if isinstance(r["value"], dict) else {}
            self._publish(self.status_pub, {"reachable": True, "t_ms": t_ms, "rtt_ms": r["rtt_ms"], **st})
        except (TimeoutError, OSError) as exc:
            self._publish(self.status_pub, {"reachable": False, "t_ms": t_ms, "error": str(exc)})

    def _do_cmd(self, req: dict) -> None:
        self.commands += 1
        cid = req.get("id")
        command = str(req.get("command", ""))
        params = req.get("params") or {}
        t_ms = int(time.time() * 1000)
        t0 = time.monotonic()
        out: dict = {"id": cid, "command": command, "params": params, "t_ms": t_ms, "steps": []}
        try:
            if command == "connect" and (params.get("ip") or params.get("port")):
                # The protocol's connect(ip, port) retargets the node (rig YAML normally sets these).
                self.http.host = str(params.get("ip") or self.http.host)
                self.http.port = int(params.get("port") or self.http.port)
            steps = plan(command, params)
        except ValueError as exc:
            out.update({"ok": False, "outcome": "error", "error": str(exc)})
            self._publish(self.reply_pub, out)
            return
        value = None
        for http_cmd, val in steps:
            try:
                r = self.http.call(http_cmd, val)
            except TimeoutError as exc:
                # UNKNOWN: the command may or may not have taken effect. Reconcile before
                # anything else runs, and say so.
                out["steps"].append({"command": http_cmd, "ok": None, "error": f"timeout after {self.http.timeout_s}s"})
                out.update({"ok": False, "outcome": "unknown", "error": str(exc)})
                self._do_status()
                break
            except OSError as exc:
                out["steps"].append({"command": http_cmd, "ok": False, "error": str(exc)})
                out.update({"ok": False, "outcome": "error", "error": f"BIAS unreachable at {self.http.base}: {exc}"})
                break
            out["steps"].append(r)
            if not r["ok"]:
                out.update({"ok": False, "outcome": "error", "error": f"{http_cmd}: {r['message']}"})
                break
            value = r["value"]
            if command == "startRecording" and http_cmd == "get-video-file" and r["value"]:
                out["artifact"] = {"kind": "video", "path": r["value"], "program": "bias"}
        else:
            out.update({"ok": True, "outcome": "ok", "value": value})
        out["rtt_ms"] = round((time.monotonic() - t0) * 1000.0, 2)
        self._publish(self.reply_pub, out)
        # rclpy caches a severity per call site, so info and warning need separate statements.
        if out["ok"]:
            self.get_logger().info(f"{command} → ok ({out['rtt_ms']} ms)")
        else:
            self.get_logger().warning(f"{command} → {out['outcome']} ({out['rtt_ms']} ms): {out.get('error')}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="BIAS command-port host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=5010, help="BIAS command port (default: 5010)")
    p.add_argument("--timeout-s", type=float, default=2.0, help="per-HTTP-call timeout; a timeout is an UNKNOWN outcome (default: 2)")
    p.add_argument("--status-every", type=float, default=1.0, help="get-status poll period in s; 0 = off (default: 1)")
    args, ros_args = p.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = BiasNode(BiasHttp(args.host, args.port, args.timeout_s), args.status_every)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C: rclpy ≥ Jazzy raises ExternalShutdownException from spin
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:  # rclpy's own SIGINT handler may already have shut the context down (races in Jazzy)
            pass
        print(f"[bias-node] {node.commands} commands handled", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
