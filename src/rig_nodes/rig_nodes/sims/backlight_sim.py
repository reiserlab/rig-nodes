#!/usr/bin/env python3
"""backlight_sim.py — a stand-in for the LED backlight controller, on a TCP port instead of a serial port.

The real controller reads CR-terminated ASCII lines from a serial port (backlight.py). pyserial
opens `socket://host:port` like a serial port, so the node talks to this sim with
`--port socket://127.0.0.1:5020` and no virtual-COM driver. The sim keeps the state the lines
set and prints every line it receives, one per line on stderr:

  [backlight-sim] rx 'RED 5 2 1010'  ir=0 red=5 grn=0 blu=0 on=False

  IR <p>                       ir power
  RED|GRN|BLU <p> [<panel> [<pattern>]]   color power (panel and pattern only printed)
  ON 0,0 / OFF                 on flag
  anything else                printed as `rx … (not modeled)`

It never replies: the plugin commands get no acknowledgment from the real controller either
(LEDController.m reads replies only for the stored-experiment queries, which the sim does not
model). VERIFY AGAINST THE REAL CONTROLLER: the command words come from LEDController.m; what
the firmware does with malformed lines is unknown.

    pixi run backlight-sim -- --port 5020
    pixi run backlight-node -- --port socket://127.0.0.1:5020
"""

from __future__ import annotations

import argparse
import os
import socket
import socketserver
import sys
import threading

DEFAULT_PORT = 5020


class BacklightState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.ir = 0
        self.colors = {"RED": 0, "GRN": 0, "BLU": 0}
        self.on = False
        self.lines = 0

    def apply(self, line: str) -> str:
        """Apply one line; return the stderr record for it."""
        words = line.split()
        with self.lock:
            self.lines += 1
            modeled = True
            try:
                if words[:1] == ["IR"] and len(words) == 2:
                    self.ir = int(words[1])
                elif words[:1] and words[0] in self.colors and 2 <= len(words) <= 4:
                    self.colors[words[0]] = int(words[1])
                elif line == "ON 0,0":
                    self.on = True
                elif line == "OFF":
                    self.on = False
                else:
                    modeled = False
            except ValueError:
                modeled = False
            state = f"ir={self.ir} red={self.colors['RED']} grn={self.colors['GRN']} blu={self.colors['BLU']} on={self.on}"
        return f"[backlight-sim] rx {line!r}  " + (state if modeled else "(not modeled)")


def make_handler(state: BacklightState):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self) -> None:
            print(f"[backlight-sim] client {self.client_address[0]}:{self.client_address[1]} connected", file=sys.stderr, flush=True)
            buf = b""
            while True:
                chunk = self.request.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b"\r" in buf:
                    raw, buf = buf.split(b"\r", 1)
                    print(state.apply(raw.decode("ascii", errors="replace").strip("\n")), file=sys.stderr, flush=True)
            print("[backlight-sim] client disconnected", file=sys.stderr, flush=True)

    return Handler


class _Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    # Windows: SO_REUSEADDR would let a second sim bind OVER this one; bind exclusively there instead.
    allow_reuse_address = os.name != "nt"

    def server_bind(self) -> None:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = p.parse_args(argv)
    state = BacklightState()
    srv = _Server((args.host, args.port), make_handler(state))
    print(f"[backlight-sim] listening on {args.host}:{args.port}  (node: --port socket://{args.host}:{args.port})", file=sys.stderr, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        print(f"[backlight-sim] {state.lines} lines received", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
