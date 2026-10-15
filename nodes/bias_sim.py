#!/usr/bin/env python3
"""bias_sim.py — a stand-in for the BIAS camera program's external-control HTTP interface.

BIAS (IO Rodeo) is controlled over HTTP GET on its command port (5010 by default in our
rig YAMLs): `http://host:port/?<command>[=<value>][&<command>…]`, answering a JSON ARRAY
with one object per command: `{"success": bool, "message": str, "value": …, "command": str}`.
This is the interface `BiasPlugin.m` (maDisplayTools) and BIAS's own `bias.py` client use.
The sim implements the subset bias_node.py needs, with a real state machine so sequencing
mistakes show up (start-capture before connect fails, as on the real program):

  connect / disconnect                        connected flag
  start-capture / stop-capture                capturing flag; frameCount grows at --fps
  enable-logging / disable-logging            logging flag (video written while capturing)
  set-video-file=<path> / get-video-file      the artifact: on stop-capture with logging on,
                                              a small file is written at that path (relative
                                              paths land in --video-dir) and reported as value
  get-status                                  {"connected","capturing","logging","frameCount",
                                               "framesPerSec","timeStamp","videoFile",
                                               "sim": true}  (so a run log says who answered)
  get-time-stamp / get-frame-count            scalars
  load-configuration=<file>                   accepted, echoed

VERIFY AGAINST THE REAL PROGRAM: the command names and the array response shape are from
BIAS's documented external interface; the exact `value` payloads of the Mac port must be
confirmed on Michael's Mac (bias_node.py treats them as opaque and logs them verbatim).

    pixi run bias-sim -- --port 5010 --video-dir /tmp/bias-videos
    curl 'http://127.0.0.1:5010/?connect'
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

DEFAULT_PORT = 5010


class BiasState:
    def __init__(self, fps: float, video_dir: str, latency_s: float) -> None:
        self.fps = fps
        self.video_dir = video_dir
        self.latency_s = latency_s
        self.lock = threading.Lock()
        self.connected = False
        self.capturing = False
        self.logging = False
        self.video_file = ""
        self.capture_t0: float | None = None
        self.frames_before = 0  # frames from earlier capture segments
        self.t0 = time.time()
        self.written: list[str] = []

    def frame_count(self) -> int:
        n = self.frames_before
        if self.capturing and self.capture_t0 is not None:
            n += int((time.time() - self.capture_t0) * self.fps)
        return n

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "capturing": self.capturing,
            "logging": self.logging,
            "frameCount": self.frame_count(),
            "framesPerSec": self.fps if self.capturing else 0.0,
            "timeStamp": round(time.time() - self.t0, 6),
            "videoFile": self.video_file,
            "sim": True,
        }

    def _video_path(self) -> str:
        p = self.video_file
        if p and not os.path.isabs(p):
            p = os.path.join(self.video_dir, p)
        return p

    def handle(self, cmd: str, value: str | None) -> dict:
        """One command → one response object. Mirrors BIAS's success/message/value shape."""
        ok, msg, val = True, "", ""
        if cmd == "connect":
            self.connected = True
            msg = "connected"
        elif cmd == "disconnect":
            if self.capturing:
                ok, msg = False, "camera is capturing"
            else:
                self.connected = False
                msg = "disconnected"
        elif cmd == "start-capture":
            if not self.connected:
                ok, msg = False, "camera not connected"
            elif self.capturing:
                ok, msg = False, "already capturing"
            else:
                self.capturing = True
                self.capture_t0 = time.time()
                msg = "capture started"
        elif cmd == "stop-capture":
            if not self.capturing:
                ok, msg = False, "not capturing"
            else:
                self.frames_before = self.frame_count()
                self.capturing = False
                self.capture_t0 = None
                msg = "capture stopped"
                if self.logging and self.video_file:
                    path = self._video_path()
                    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
                    with open(path, "wb") as fh:
                        fh.write(json.dumps({"bias_sim": True, "frames": self.frames_before, "fps": self.fps}).encode())
                    self.written.append(path)
                    val = path
                    msg = f"capture stopped, video written {path}"
        elif cmd == "enable-logging":
            self.logging = True
            msg = "logging enabled"
        elif cmd == "disable-logging":
            self.logging = False
            msg = "logging disabled"
        elif cmd == "set-video-file":
            if value is None:
                ok, msg = False, "set-video-file needs a value"
            else:
                self.video_file = value
                val = self._video_path()
                msg = "video file set"
        elif cmd == "get-video-file":
            val = self._video_path()
        elif cmd == "get-status":
            val = self.status()
        elif cmd == "get-time-stamp":
            val = round(time.time() - self.t0, 6)
        elif cmd == "get-frame-count":
            val = self.frame_count()
        elif cmd == "load-configuration":
            if value is None:
                ok, msg = False, "load-configuration needs a file"
            else:
                msg = f"configuration loaded from {value}"
                val = value
        elif cmd == "get-configuration":
            val = {"sim": True, "fps": self.fps}
        else:
            ok, msg = False, f"unknown command {cmd!r}"
        return {"success": ok, "message": msg, "value": val, "command": cmd}


def make_handler(state: BiasState):
    class Handler(BaseHTTPRequestHandler):
        server_version = "bias_sim/0.1"

        def log_message(self, fmt, *args):  # quiet; the node logs replies
            pass

        def do_GET(self):
            q = urlparse(self.path).query
            replies = []
            with state.lock:
                for part in q.split("&"):
                    if not part:
                        continue
                    cmd, _, value = part.partition("=")
                    replies.append(state.handle(unquote(cmd), unquote(value) if _ else None))
            if state.latency_s > 0:
                time.sleep(state.latency_s)
            body = json.dumps(replies).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


class _Server(ThreadingHTTPServer):
    # Windows: socketserver's SO_REUSEADDR would let a second sim bind OVER this one; bind
    # exclusively there instead (the two options cannot be combined).
    allow_reuse_address = os.name != "nt"

    def server_bind(self) -> None:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def answering(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.5):
            return True
    except OSError:
        return False


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--fps", type=float, default=100.0, help="simulated capture rate (default: 100)")
    p.add_argument("--video-dir", default="bias-videos", help="where relative video paths land (default: ./bias-videos)")
    p.add_argument("--latency-ms", type=float, default=0.0, help="artificial reply delay, for timeout tests")
    args = p.parse_args(argv)
    # Refuse to start next to a live program on the port. On Windows the real BIAS's listener
    # (Qt, 0.0.0.0) shares its address, so our bind would SUCCEED and every 127.0.0.1 client —
    # the rig's bias node included — would then talk to this sim (ww18 2026-10-15). Only a
    # connect sees that listener.
    if answering(args.host, args.port):
        print(f"[bias-sim] cannot start: something already answers on {args.host}:{args.port} — probably the real BIAS "
              f"(or another bias-sim); binding here would hide it from every local client. Close it, or pass --port.",
              file=sys.stderr)
        return 2
    state = BiasState(args.fps, args.video_dir, args.latency_ms / 1000.0)
    srv = _Server((args.host, args.port), make_handler(state))
    print(f"[bias-sim] http://{args.host}:{args.port}/?<command>  (fps {args.fps:g}, videos → {args.video_dir})", file=sys.stderr)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        print(f"[bias-sim] videos written: {state.written}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
