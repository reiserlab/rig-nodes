#!/usr/bin/env python3
"""backlight_node.py — the LED backlight controller as a ROS 2 node: protocol plugin commands in, replies + status out.

The controller is a serial device (one ASCII command per line, CR, 115200 baud; see backlight.py)
that MATLAB drove through LEDController.m / LEDControllerPlugin.m. This node owns its port and
exposes it on three topics, all std_msgs/String JSON (no custom message types):

  /backlight/cmd     IN   {"id": <any>, "command": <plugin command>, "params": {…}}
  /backlight/reply   OUT  {"id", "command", "params", "ok", "outcome": "ok"|"error"|"unknown",
                           "value", "error", "warning", "steps": [...one per controller line...],
                           "rtt_ms", "t_ms", "rx_epoch_ms"}
  /backlight/status  OUT  every --status-every s: {"reachable", "t_ms", "port"} (+ "error")

The `command` vocabulary is the protocol's `backlight` plugin (LEDControllerPlugin.m;
js/plugin-registry.js is a client of it): setIRLEDPower(power), setRedLEDPower /
setGreenLEDPower / setBlueLEDPower(power[, panel_num[, pattern]]), turnOnLED, turnOffLED,
setVisibleBacklightsOff. backlight.plan() turns each into controller lines, byte for byte what
MATLAB sends; each line is one entry in `steps`.

What the outcome means here: the controller sends no acknowledgment for these commands, so `ok`
means every line was written to the port, and each step says `"confirmed": false`. A write that
times out is `unknown` (the bytes may have gone out), after which the node publishes a status
before anything else. A port that cannot be opened is `error`.

A write that fails with an error (the device behind the port is gone: unplugged, or a new
pseudo-terminal behind the same name) makes the node close the port, reopen it, and send the
same line again, once per command. The failed attempt stays in `steps` and `warning` says the
port was reopened; if the reopen or the second write fails too, the outcome is `error` and the
next command or status tick tries again. So a controller that was unplugged and plugged back in
is used again from the first command, without restarting the node. The retry is safe because
every command in the vocabulary sets a state (a level, on, off): sending a line twice changes
nothing. A timeout is never retried. `reachable` in the status means the port is open, not that
the controller answered: with no writes, the node cannot tell that the device went away. `critical` / `on_loss` are the executor's
decision (CONTRACT.md §4) — the node only reports.

Stopping the node closes the port and leaves the LEDs as they are (LEDController.m's behavior).

    pixi run backlight-node -- --port COM6
    pixi run backlight-cmd -- setIRLEDPower --power 50
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

from rig_nodes._common import RELIABLE_QOS, raise_timer_resolution, spin, wall_ms
from rig_nodes.backlight import WRITE_TIMEOUT_S, LEDController, plan

CMD_TOPIC = "/backlight/cmd"
REPLY_TOPIC = "/backlight/reply"
STATUS_TOPIC = "/backlight/status"
QOS = RELIABLE_QOS


class BacklightNode(Node):
    def __init__(self, port: str, status_every_s: float, write_timeout_s: float = WRITE_TIMEOUT_S) -> None:
        super().__init__("backlight")
        self.port = port
        self.write_timeout_s = write_timeout_s
        self.dev: LEDController | None = None  # touched only by the worker thread
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, QOS)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, QOS)
        self.create_subscription(String, CMD_TOPIC, self._on_cmd, QOS)
        # Serial writes block (up to write_timeout_s), so they run on one worker, in arrival order.
        self._q: queue.Queue = queue.Queue()
        self._status_pending = False  # at most ONE status tick waits in the queue
        self._worker = threading.Thread(target=self._work, name="backlight-serial", daemon=True)
        self._worker.start()
        if status_every_s > 0:
            self.create_timer(status_every_s, self._enqueue_status)
        self.commands = 0
        self._q.put(("status", None))  # open the port now, and say whether that worked
        self.get_logger().info(f"backlight controller on {port} ← {CMD_TOPIC} → {REPLY_TOPIC}, {STATUS_TOPIC} every {status_every_s:g}s")

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
        if not self._status_pending:
            self._status_pending = True
            self._q.put(("status", None))

    def _work(self) -> None:
        while True:
            kind, payload = self._q.get()
            if kind == "stop":
                return
            try:
                if kind == "status":
                    self._status_pending = False
                    self._do_status()
                else:
                    self._do_cmd(payload)
            except Exception as exc:  # nothing a command carries may kill the worker
                self.get_logger().error(f"{kind} failed: {exc!r}")
                if kind == "cmd":
                    try:  # at shutdown the publisher may already be gone
                        self._publish(self.reply_pub, {"id": payload.get("id"), "command": payload.get("command"),
                                                       "ok": False, "outcome": "error", "error": f"node error: {exc!r}"})
                    except Exception:
                        pass

    # ── the port ──

    def _ensure_open(self) -> LEDController:
        """The open controller, (re)opening the port if needed. Raises OSError (SerialException)."""
        if self.dev is None or not self.dev.is_open:
            self.dev = None
            self.dev = LEDController(self.port, write_timeout_s=self.write_timeout_s)
            self.get_logger().info(f"opened {self.port}")
        return self.dev

    def _drop_port(self) -> None:
        """Close a port that failed, so the next command or status tick reopens it."""
        if self.dev is not None:
            try:
                self.dev.close()
            except OSError:
                pass
            self.dev = None

    def close(self) -> None:
        """Stop the worker and close the port. The LEDs stay as they are."""
        self._q.put(("stop", None))
        self._worker.join(self.write_timeout_s + 1.0)
        self._drop_port()

    # ── status and commands ──

    def _do_status(self) -> None:
        t_ms = wall_ms()
        try:
            self._ensure_open()
            self._publish(self.status_pub, {"reachable": True, "t_ms": t_ms, "port": self.port})
        except OSError as exc:
            self._drop_port()
            self._publish(self.status_pub, {"reachable": False, "t_ms": t_ms, "port": self.port, "error": str(exc)})

    def _do_cmd(self, req: dict) -> None:
        self.commands += 1
        command = str(req.get("command", ""))
        params = req.get("params") or {}
        t0 = time.perf_counter()
        out: dict = {"id": req.get("id"), "command": command, "params": params, "t_ms": wall_ms(), "steps": []}
        try:
            if not isinstance(params, dict):
                raise ValueError("params must be a JSON object")
            lines, warnings = plan(command, params)
        except (ValueError, TypeError) as exc:
            out.update({"ok": False, "outcome": "error", "error": str(exc)})
            self._publish(self.reply_pub, out)
            self.get_logger().warning(f"{command} → error: {exc}")
            return
        for text in warnings:
            self._warn(out, f"{command}: {text}")
        self._write(lines, out)
        out["rtt_ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
        self._publish(self.reply_pub, out)
        # rclpy caches a severity per call site, so info and warning need separate statements.
        if out["ok"]:
            self.get_logger().info(f"{command} → ok ({out['rtt_ms']} ms)")
        else:
            self.get_logger().warning(f"{command} → {out['outcome']} ({out['rtt_ms']} ms): {out.get('error')}")

    def _warn(self, out: dict, text: str) -> None:
        """Add one warning to the reply (joined with '; ') and to the node's log."""
        out["warning"] = f"{out['warning']}; {text}" if out.get("warning") else text
        self.get_logger().warning(text)

    def _write(self, lines: list[str], out: dict) -> None:
        """Write the lines in order, one `steps` entry per attempt; stop at the first failure.
        A write error reopens the port and sends the line again, once per command."""
        try:
            dev = self._ensure_open()
        except OSError as exc:
            self._drop_port()
            out.update({"ok": False, "outcome": "error", "error": f"backlight controller unreachable on {self.port}: {exc}"})
            return
        reopened = False
        for line in lines:
            while True:
                t0 = time.perf_counter()
                try:
                    dev.send_line(line)
                except serial.SerialTimeoutException as exc:
                    # UNKNOWN: some of the line may have gone out. Never retried; reconcile first.
                    out["steps"].append({"command": line, "ok": None, "error": f"write timeout after {self.write_timeout_s:g}s"})
                    out.update({"ok": False, "outcome": "unknown", "error": str(exc) or "write timeout"})
                    self._do_status()
                    return
                except OSError as exc:
                    out["steps"].append({"command": line, "ok": False, "error": str(exc)})
                    self._drop_port()
                    if reopened:
                        out.update({"ok": False, "outcome": "error", "error": f"write to {self.port} failed again after reopening it: {exc}"})
                        return
                    reopened = True
                    try:
                        dev = self._ensure_open()
                    except OSError as reopen_exc:
                        self._drop_port()
                        out.update({"ok": False, "outcome": "error",
                                    "error": f"write to {self.port} failed ({exc}); reopening it failed: {reopen_exc}"})
                        return
                    self._warn(out, f"port {self.port} reopened after a write error ({exc}); {line!r} sent again")
                    continue
                out["steps"].append({"command": line, "ok": True, "confirmed": False,
                                     "rtt_ms": round((time.perf_counter() - t0) * 1000.0, 2)})
                break
        out.update({"ok": True, "outcome": "ok", "value": None})


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True,
                   help='serial port (COM6, /dev/ttyACM0) or pyserial URL ("socket://127.0.0.1:5020" for backlight-sim)')
    p.add_argument("--status-every", type=float, default=1.0, help="status period in s; 0 = off (default: 1)")
    p.add_argument("--write-timeout-s", type=float, default=WRITE_TIMEOUT_S,
                   help=f"a write that takes longer is an UNKNOWN outcome (default: {WRITE_TIMEOUT_S:g})")
    args, ros_args = p.parse_known_args(argv)
    raise_timer_resolution()  # Windows: 1 ms system timer for this process (_common "clocks")
    rclpy.init(args=ros_args)
    node = BacklightNode(args.port, args.status_every, args.write_timeout_s)
    try:
        spin(node)
    finally:
        node.close()
    print(f"[backlight-node] {node.commands} commands handled", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
