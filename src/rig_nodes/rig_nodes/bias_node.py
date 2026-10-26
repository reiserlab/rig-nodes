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
  /bias/frames  OUT  after stopRecording: every frame's BIAS timestamp from its stamp log,
                      {"video", "stamp_log", "chunk", "chunks", "first_index", "t": […]}

The `command` vocabulary is the protocol's `camera` plugin (js/plugin-registry.js
BiasPlugin): connect(ip, port, config_file), startRecording(filename), stopRecording,
startPreview, stopCapture, getTimestamp, disconnect — each mapped to the BIAS HTTP
sequence `BiasPlugin.m` performs. The bridge node logs every reply and status line into the
run log as `{"type":"connector","src":"bias",…}` events, so the video file identity
(`artifact`) and every command/reply pair sit in the same file as the FicTrac rows.

LAB-303 semantics carried here: `timeout_s` per HTTP call (`slow_timeout_s` for the camera /
writer commands in SLOW_HTTP — a real camera connect takes seconds); a timeout is an UNKNOWN outcome
(`outcome: "unknown"`), after which the node reconciles with `get-status` before anything
else. `critical` / `on_loss` are the runner's decision (the Studio side, shared by both
plans) — the node only reports.

    pixi run bias-node -- --host 127.0.0.1 --port 5010
    pixi run bias-cmd -- startRecording --filename trial_001.avi
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rig_nodes._common import RELIABLE_QOS, raise_timer_resolution, spin, wall_ms

CMD_TOPIC = "/bias/cmd"
REPLY_TOPIC = "/bias/reply"
STATUS_TOPIC = "/bias/status"
# Every frame's BIAS timestamp for a finished recording, read from BIAS's stamp_log_cam<N>.txt at
# stopRecording: {"video", "stamp_log", "chunk", "chunks", "first_index", "t": [s, …]}, ≤ STAMP_CHUNK each.
FRAMES_TOPIC = "/bias/frames"
STAMP_CHUNK = 20000
QOS = RELIABLE_QOS
PLUGIN_COMMANDS = ("connect", "startRecording", "stopRecording", "startPreview", "stopCapture", "getTimestamp", "disconnect")
# HTTP commands that touch the camera or the video writer and can legitimately take seconds:
# the Atlas10 BIAS on ww18 answers `connect` after 2.18 s (camera discovery + transport setup),
# past the 2 s per-call timeout, so a successful connect came back `outcome: "unknown"`.
SLOW_HTTP = frozenset({"connect", "disconnect", "load-configuration", "start-capture", "stop-capture", "close"})


class BiasHttp:
    """Blocking HTTP client for BIAS's command port. One command per request."""

    def __init__(self, host: str, port: int, timeout_s: float, slow_timeout_s: float | None = None) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.slow_timeout_s = max(timeout_s, slow_timeout_s if slow_timeout_s is not None else timeout_s)

    @property
    def base(self) -> str:
        return f"http://{self.host}:{self.port}/"

    def timeout_for(self, command: str) -> float:
        return self.slow_timeout_s if command in SLOW_HTTP else self.timeout_s

    def call(self, command: str, value: str | None = None) -> dict:
        """→ {"command", "ok", "message", "value", "rtt_ms"} or raises TimeoutError / OSError."""
        q = command if value is None else f"{command}={urllib.parse.quote(str(value), safe='/:\\')}"
        url = f"{self.base}?{q}"
        # perf_counter, not monotonic: on Windows monotonic ticks every 15.625 ms (GetTickCount64),
        # which turned every RTT into 0 / 15 / 16 / 31 ms (ww18 2026-10-15).
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_for(command)) as resp:
                body = resp.read()
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, TimeoutError) or "timed out" in str(exc.reason):
                raise TimeoutError(str(exc)) from exc
            raise OSError(str(exc)) from exc
        except TimeoutError:
            raise
        rtt = (time.perf_counter() - t0) * 1000.0
        try:
            arr = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise OSError(f"non-JSON reply from BIAS: {body[:80]!r}")
        rep = arr[0] if isinstance(arr, list) and arr else (arr if isinstance(arr, dict) else {})
        if not isinstance(rep, dict):
            raise OSError(f"unexpected reply element from BIAS: {rep!r}")
        return {
            "command": command,
            "ok": bool(rep.get("success", False)),
            "message": rep.get("message", ""),
            "value": rep.get("value", None),
            "rtt_ms": round(rtt, 2),
        }


def resolve_video(requested: str, since_epoch_s: float) -> str | None:
    """The file BIAS actually writes for `requested`, or None. BIAS's `get-video-file` answers
    `<dir>/<stem>.<ext>`, but its writer adds the auto-naming suffix (`<stem>_cam_0_date_…_v001.<ext>`
    on the Atlas10 build, a `…_v001` folder on the Mac port), so that path never exists (ww18
    2026-10-15: artifact `poc.ufmf`, file `poc_cam_0_date_2026_10_15_time_13_43_10_v001.ufmf`).
    Newest `<stem>*` entry in that folder touched since the recording started; the exact name
    wins when it exists (bias_sim, or BIAS with auto-naming off)."""
    if not requested:
        return None
    folder, base = os.path.split(os.path.normpath(requested))
    stem, ext = os.path.splitext(base)
    if os.path.exists(os.path.join(folder, base)):
        return os.path.join(folder, base)
    cands = [p for p in glob.glob(os.path.join(glob.escape(folder), glob.escape(stem) + "*"))
             if (os.path.isdir(p) or p.lower().endswith(ext.lower())) and os.path.getmtime(p) >= since_epoch_s - 5.0]
    return max(cands, key=os.path.getmtime) if cands else None


def read_stamp_log(folder: str, since_epoch_s: float, settle_s: float = 3.0) -> dict | None:
    """BIAS's per-frame timestamps for the recording that just stopped: the newest
    `stamp_log_cam*.txt` in the video folder written since the start (BIAS writes ONE per camera
    and overwrites it on the next recording, so it has to be read now). Waits until the size is
    stable — the logger drains its queue after stop-capture. → {path, t: [s…], sha256, bytes}."""
    cands = [p for p in glob.glob(os.path.join(glob.escape(folder), "stamp_log_cam*.txt"))
             if os.path.getmtime(p) >= since_epoch_s - 5.0]
    if not cands:
        return None
    path = max(cands, key=os.path.getmtime)
    end, last = time.monotonic() + settle_s, -1
    while time.monotonic() < end:
        size = os.path.getsize(path)
        if size == last and size > 0:
            break
        last = size
        time.sleep(0.15)
    with open(path, "rb") as fh:
        raw = fh.read()
    t = []
    for line in raw.decode("ascii", errors="replace").splitlines():
        try:
            t.append(float(line.split()[0]))
        except (ValueError, IndexError):
            continue
    return {"path": path, "t": t, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}


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
    def __init__(self, http: BiasHttp, status_every_s: float, video_dir: str | None = None) -> None:
        super().__init__("bias_camera")
        self.http = http
        # Where a RELATIVE startRecording filename lands (MATLAB's BiasPlugin resolves it
        # against the experiment folder; on a rig this comes from the rig YAML). None = pass
        # the name through and warn (BIAS then resolves it against ITS cwd).
        self.video_dir = video_dir
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, QOS)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, QOS)
        self.frames_pub = self.create_publisher(String, FRAMES_TOPIC, QOS)
        self._rec: dict | None = None  # the recording in progress: {"requested", "t0"} (startRecording → stopRecording)
        self.create_subscription(String, CMD_TOPIC, self._on_cmd, QOS)
        self._q: queue.Queue = queue.Queue()
        self._status_pending = False  # at most ONE get-status poll waits in the queue
        self._worker = threading.Thread(target=self._work, name="bias-http", daemon=True)
        self._worker.start()
        self._status_every = status_every_s
        if status_every_s > 0:
            self.create_timer(status_every_s, self._enqueue_status)
        self.commands = 0
        self.get_logger().info(f"BIAS at {http.base} ← {CMD_TOPIC} → {REPLY_TOPIC}, {STATUS_TOPIC} every {status_every_s:g}s")

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
        if not isinstance(req, dict):
            return
        self._q.put(("cmd", req))

    def _enqueue_status(self) -> None:
        # Coalesce: when BIAS is slow or black-holed each poll blocks up to timeout_s, and a
        # poll per period would build an unbounded line that commands then wait behind.
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
            except Exception as exc:  # a malformed BIAS body, HTTPException, … must not kill the worker
                self.get_logger().error(f"{kind} failed: {exc!r}")
                if kind == "cmd":
                    self._publish(self.reply_pub, {"id": (payload or {}).get("id"), "command": (payload or {}).get("command"),
                                                   "ok": False, "outcome": "error", "error": f"node error: {exc!r}"})

    def _do_status(self) -> None:
        t_ms = wall_ms()
        try:
            r = self.http.call("get-status")
            st = r["value"] if isinstance(r["value"], dict) else {}
            self._publish(self.status_pub, {"reachable": True, "t_ms": t_ms, "rtt_ms": r["rtt_ms"], **st})
        except (TimeoutError, OSError) as exc:
            self._publish(self.status_pub, {"reachable": False, "t_ms": t_ms, "error": str(exc)})

    def _finish_recording(self, cid, out: dict) -> None:
        """After a successful stopRecording: the final video file (+ size) and every frame's BIAS
        timestamp — the summary into the reply's `artifact` (so the run log names them), the
        timestamps themselves on /bias/frames (so the bag holds them)."""
        rec, self._rec = self._rec, None
        if rec is None:
            out["warning"] = "stopRecording without a startRecording from this node: video file and frame timestamps not collected"
            return
        video = resolve_video(rec["requested"], rec["t0"])
        art = {"kind": "video", "path": video or rec["requested"], "requested": rec["requested"], "resolved": video is not None,
               "program": "bias", "bytes": os.path.getsize(video) if video and os.path.isfile(video) else None}
        try:
            sl = read_stamp_log(os.path.dirname(os.path.normpath(video or rec["requested"])), rec["t0"])
        except OSError as exc:
            sl, art["stamp_log_error"] = None, str(exc)
        if sl is not None:
            t = sl["t"]
            chunks = max(1, -(-len(t) // STAMP_CHUNK))
            for k in range(chunks):
                part = t[k * STAMP_CHUNK:(k + 1) * STAMP_CHUNK]
                self._publish(self.frames_pub, {"id": cid, "video": art["path"], "stamp_log": sl["path"], "chunk": k, "chunks": chunks,
                                                "first_index": k * STAMP_CHUNK, "t": part})
            art["stamp_log"] = {"path": sl["path"], "frames": len(t), "first_s": t[0] if t else None, "last_s": t[-1] if t else None,
                                "sha256": sl["sha256"], "bytes": sl["bytes"], "chunks": chunks, "topic": FRAMES_TOPIC}
        out["artifact"] = art

    def _do_cmd(self, req: dict) -> None:
        self.commands += 1
        cid = req.get("id")
        command = str(req.get("command", ""))
        params = req.get("params") or {}
        t_ms = wall_ms()
        t0 = time.perf_counter()
        t_epoch = time.time()  # file mtimes are wall clock: the recording's start for resolve_video
        out: dict = {"id": cid, "command": command, "params": params, "t_ms": t_ms, "steps": []}
        try:
            if command == "connect" and (params.get("ip") or params.get("port")):
                # The protocol's connect(ip, port) retargets the node (rig YAML normally sets these).
                self.http.host = str(params.get("ip") or self.http.host)
                self.http.port = int(params.get("port") or self.http.port)
            if command == "startRecording" and params.get("filename") and not os.path.isabs(str(params["filename"])):
                if self.video_dir:
                    params = dict(params)
                    params["filename"] = os.path.join(self.video_dir, str(params["filename"]))
                    out["params"] = params
                else:
                    # Seen on the real Mac port (2026-10-14): a relative name resolves against BIAS's
                    # own cwd (`/` when launched with `open`), start-capture still answers success,
                    # and nothing captures. Say so in the reply instead of letting the run go dark.
                    out["warning"] = f"relative filename {params['filename']!r} resolves against BIAS's working directory, not the rig's; use an absolute path or --video-dir"
                    self.get_logger().warning(out["warning"])
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
                out["steps"].append({"command": http_cmd, "ok": None, "error": f"timeout after {self.http.timeout_for(http_cmd)}s"})
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
                self._rec = {"requested": str(r["value"]), "t0": t_epoch}
                found = resolve_video(self._rec["requested"], t_epoch)
                out["artifact"] = {"kind": "video", "path": found or self._rec["requested"], "requested": self._rec["requested"],
                                   "resolved": found is not None, "program": "bias"}
        else:
            out.update({"ok": True, "outcome": "ok", "value": value})
            if command == "stopRecording":
                self._finish_recording(cid, out)
        out["rtt_ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
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
    p.add_argument("--slow-timeout-s", type=float, default=15.0,
                   help=f"timeout for the camera/writer commands ({', '.join(sorted(SLOW_HTTP))}); default: 15")
    # 5 Hz: the (frameCount, BIAS timeStamp) samples, stamped with this node's precise wall clock,
    # map BIAS's clock onto the run's (with the /bias/frames timestamps: every frame on the run
    # timeline) and show drops live; ~10 ms of BIAS's GUI thread per poll.
    p.add_argument("--status-every", type=float, default=0.2, help="get-status poll period in s; 0 = off (default: 0.2 = 5 Hz)")
    p.add_argument("--video-dir", default=None, help="directory a RELATIVE startRecording filename is resolved against (default: pass through + warn)")
    args, ros_args = p.parse_known_args(argv)
    raise_timer_resolution()  # Windows: 1 ms system timer for this process (_common "clocks")
    rclpy.init(args=ros_args)
    node = BiasNode(BiasHttp(args.host, args.port, args.timeout_s, args.slow_timeout_s), args.status_every, video_dir=args.video_dir)
    spin(node)
    print(f"[bias-node] {node.commands} commands handled", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
