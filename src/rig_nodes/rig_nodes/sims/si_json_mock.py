#!/usr/bin/env python3
"""si_json_mock.py — a stand-in for ScanImage's built-in JSON server (`most.network.json.Server`).

Ported from `tests/remote-control/mock_servers.py` (`serve_json`) in reiserlab/scanimage-2026-bergamo
(Hannah Marie Santos, LAB-305), with attribution; the acquisition model below is ours. It imitates
the server's WIRE and its stock (unpatched) behaviors, not ScanImage's timing:

  * TCP, single client (listen(1), like AsyncServer); every message is `<Q` little-endian byte
    count + UTF-8 JSON, both ways.
  * {"command":"get","property":P}           → the same object + "value"
    {"command":"set","property":P,"value":V} → + "actual"
    {"command":"eval","function":F,"num_outputs":N} → + "outputs" (only hSI.startGrab(),
    hSI.startFocus(), hSI.abort() and AcquisitionMex(hSI.hScan2D.hAcq,'getAcqStatus') are
    understood; anything else is an error reply)
  * Stock bug B2 (default ON, `--no-b2` to patch it): a get of an OBJECT-valued property gets
    NO reply at all — the client must time out. The connector's property allowlist is what
    keeps a rig away from this; the harness uses it to exercise the UNKNOWN path.
  * Acquisition model: `startGrab` → acqState "grab" for framesPerSlice / scanFrameRate seconds
    (`--grab-s` overrides), logFileCounter += 1 when logging is enabled, lastFrameNumber counts
    up at the frame rate; `abort` → "idle" at once. `startGrab` while not idle is an error,
    as in ScanImage. The eval reply is delayed `--start-delay-s` (ScanImage's startGrab is
    synchronous and takes 1.2–2.4 s on the Bergamo PC, LAB-305).

    python tests/si_json_mock.py --port 5555
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time

STATE = {
    "acqState": "idle",
    "hRoiManager": {"scanZoomFactor": 1.0, "scanFrameRate": 29.08, "linesPerFrame": 512, "pixelsPerLine": 512},
    "hStackManager": {"framesPerSlice": 60, "numSlices": 1},
    "hChannels": {"loggingEnable": True, "channelSave": [1, 2]},
    "hScan2D": {"logFileStem": "file", "logFilePath": "F:\\ImagingData\\mock", "logFileCounter": 1, "bidirectional": False,
                "virtualChannelSettings": [{"laserFilterWindow": [0, 1]}, {"laserFilterWindow": [10, 11]}]},
    "hDisplay": {"lastFrameNumber": 0},
    "acqsPerLoop": 1,
}
OBJECT_VALUED = {"hScan2D", "hRoiManager", "hStackManager", "hChannels", "hDisplay", "hScan2D.virtualChannelSettings"}
_lock = threading.Lock()
STALL_ON: set = set()  # canonical property paths whose `get` gets NO reply (--stall-on; a B2-style silence on an allowed property)


class _Stall(Exception):
    """Drop this request without a reply."""
_acq = {"t_end": None, "t_start": None, "grabs": 0, "aborts": 0}


def _strip(path: str) -> str:
    return path[4:] if path.startswith("hSI.") else path


def _resolve(path: str):
    node = STATE
    parts = _strip(path).split(".")
    for p in parts[:-1]:
        if not isinstance(node, dict) or p not in node:
            raise ValueError(f'"{path}" is not a valid property name')
        node = node[p]
    if not isinstance(node, dict) or parts[-1] not in node:
        raise ValueError(f'"{path}" is not a valid property name')
    return node, parts[-1]


def get(path: str):
    _tick()
    if path in STALL_ON or ("hSI." + path) in STALL_ON:
        raise _Stall(path)
    if _strip(path) in OBJECT_VALUED:
        return object()  # unencodable → B2
    node, key = _resolve(path)
    return node[key]


def set_(path: str, value):
    node, key = _resolve(path)
    if _strip(path) == "acqState":
        raise ValueError("acqState is read-only")
    node[key] = value
    return value


def _tick() -> None:
    """Advance the acquisition model to `now`."""
    with _lock:
        if STATE["acqState"] in ("grab", "focus") and _acq["t_start"] is not None:
            fps = STATE["hRoiManager"]["scanFrameRate"]
            STATE["hDisplay"]["lastFrameNumber"] = int((time.monotonic() - _acq["t_start"]) * fps)
        if STATE["acqState"] == "grab" and _acq["t_end"] is not None and time.monotonic() >= _acq["t_end"]:
            STATE["acqState"] = "idle"
            _acq["t_end"] = None


def _eval(expr: str, args: argparse.Namespace):
    expr = expr.strip()
    if expr in ("hSI.startGrab()", "hSI.startGrab"):
        _tick()
        if STATE["acqState"] != "idle":
            raise ValueError(f"Cannot start a grab while acqState is '{STATE['acqState']}'")
        time.sleep(args.start_delay_s)
        with _lock:
            dur = args.grab_s if args.grab_s is not None else STATE["hStackManager"]["framesPerSlice"] / STATE["hRoiManager"]["scanFrameRate"]
            STATE["acqState"] = "grab"
            _acq["t_start"] = time.monotonic()
            _acq["t_end"] = _acq["t_start"] + dur
            _acq["grabs"] += 1
            STATE["hDisplay"]["lastFrameNumber"] = 0
            if STATE["hChannels"]["loggingEnable"]:
                STATE["hScan2D"]["logFileCounter"] += 1
        return []
    if expr in ("hSI.startFocus()", "hSI.startFocus"):
        _tick()
        if STATE["acqState"] != "idle":
            raise ValueError(f"Cannot start focus while acqState is '{STATE['acqState']}'")
        time.sleep(args.start_delay_s)
        with _lock:
            STATE["acqState"] = "focus"
            _acq["t_start"] = time.monotonic()
            _acq["t_end"] = None
        return []
    if expr in ("hSI.abort()", "hSI.abort"):
        with _lock:
            STATE["acqState"] = "idle"
            _acq["t_end"] = None
            _acq["aborts"] += 1
        return []
    if expr == "AcquisitionMex(hSI.hScan2D.hAcq,'getAcqStatus')":
        _tick()  # the vDAQ engine's status struct (fields as on 2026.0.0, simulated vDAQ 2026-10-21)
        with _lock:
            n = STATE["hDisplay"]["lastFrameNumber"]
            return [{"fifoOverflows": False, "dcOvervoltage": False, "acquisitionFinished": STATE["acqState"] == "idle",
                     "totalAcquiredFrames": n, "acqNumber": _acq["grabs"], "frameNumberAcq": n,
                     "acqTriggerTimestamp": -1, "nextFileMarkerTimestamp": -1, "numDroppedFramesDisplay": 0,
                     "numDroppedFramesLogger": 0, "ZMQIsThreadRunning": STATE["acqState"] != "idle", "ZMQErrorMessage": ""}]
    raise ValueError(f"mock cannot eval {expr!r} (only hSI.startGrab(), hSI.startFocus(), hSI.abort(), getAcqStatus)")


def _recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError
        buf += chunk
    return buf


def serve(args: argparse.Namespace, stop: threading.Event | None = None) -> None:
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)  # single client, like AsyncServer
    srv.settimeout(0.2)
    print(f"[si-json-mock] ScanImage JSON server stand-in on {args.host}:{args.port} (B2 {'on' if args.b2 else 'patched'}, "
          f"start delay {args.start_delay_s:g} s, grab {args.grab_s if args.grab_s is not None else 'framesPerSlice/fps'} s)", file=sys.stderr, flush=True)
    while not (stop and stop.is_set()):
        try:
            conn, peer = srv.accept()
        except socket.timeout:
            continue
        conn.settimeout(10.0)
        with conn:
            while not (stop and stop.is_set()):
                try:
                    n = struct.unpack("<Q", _recv_exact(conn, 8))[0]
                    body = _recv_exact(conn, n).decode("utf-8")
                except (ConnectionError, socket.timeout, OSError):
                    break
                try:
                    msg = json.loads(body)
                except ValueError as e:
                    rsp = json.dumps({"message": body, "error": str(e)})
                else:
                    try:
                        c = msg.get("command")
                        if c == "get":
                            msg["value"] = get(msg["property"])
                        elif c == "set":
                            msg["actual"] = set_(msg["property"], msg["value"])
                        elif c == "eval":
                            outs = _eval(msg["function"], args)
                            msg["outputs"] = {f"output{i + 1}": o for i, o in enumerate(outs)}
                        else:
                            raise ValueError(f"unknown command {c!r}")
                        rsp = json.dumps(msg)
                    except _Stall as e:
                        print(f"[si-json-mock] --stall-on: {e} → no reply", file=sys.stderr, flush=True)
                        continue
                    except TypeError:
                        if args.b2:
                            print(f"[si-json-mock] B2: object-valued {msg.get('property')!r} → no reply (stock behavior)", file=sys.stderr, flush=True)
                            continue  # stock bug B2: unencodable value → no reply at all
                        msg["error"] = "object-valued property cannot be encoded"
                        rsp = json.dumps(msg)
                    except Exception as e:  # noqa: BLE001
                        msg["error"] = str(e)
                        rsp = json.dumps(msg)
                data = rsp.encode("utf-8")
                try:
                    conn.sendall(struct.pack("<Q", len(data)) + data)
                except OSError:
                    break
    srv.close()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--no-b2", dest="b2", action="store_false", help="patched behavior: an error reply instead of silence on object-valued gets")
    p.add_argument("--start-delay-s", type=float, default=0.3, help="how long startGrab/startFocus block before replying (Bergamo: 1.2–2.4 s)")
    p.add_argument("--grab-s", type=float, default=None, help="grab duration override (default: framesPerSlice / scanFrameRate)")
    p.add_argument("--stall-on", action="append", default=[], help="property path whose get is never answered (harness: the UNKNOWN path on an allowlisted property)")
    args = p.parse_args(argv)
    STALL_ON.update(a if a.startswith("hSI.") else "hSI." + a for a in args.stall_on)
    try:
        serve(args)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
