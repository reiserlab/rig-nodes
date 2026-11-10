#!/usr/bin/env python3
"""uvc_camera_node.py — a USB (UVC) camera → ROS 2 image topic, for viewing and recording.

ffmpeg owns the camera (DirectShow) and writes frames to a pipe; this node cuts the pipe into
frames, stamps each when it arrived, and publishes. No OpenCV, no cv_bridge, no camera driver
package: rclpy + sensor_msgs + an ffmpeg executable.

    --mode mjpeg (default)  the camera's own JPEGs, untouched → /<src>/image/compressed
                            (sensor_msgs/CompressedImage, format "jpeg"). No decode in the node,
                            tens of KB per frame, Foxglove shows it directly.
    --mode gray             YUYV decoded by ffmpeg to 8-bit luma → /<src>/image_raw
                            (sensor_msgs/Image, mono8). For consumers that want pixels.

`/<src>/status` (CONTRACT §4) carries the MEASURED frame rate every `--status-every` s. That is
the point of it: a UVC camera silently picks the nearest rate it has, and its driver timestamps
are synthetic, so frames counted against the host clock are the only truth (docs/uvc-cameras.md).

One node per camera, each under its own `<src>` (`--src ezbox`, `--src ov9281`):

    pixi run uvc-camera -- --device "Arducam OV9281 USB Camera" --src ov9281 --size 1280x800 --fps 120
    pixi run uvc-camera -- --list                 # the DirectShow cameras on this PC
    pixi run uvc-camera -- --device "<name>" --list   # that camera's formats, sizes and rate ranges

Stamps are host arrival times (`header.stamp`, precise wall clock). ffmpeg's DirectShow reader
hands frames over in bursts of ~16 ms on Windows, so they order frames and align them roughly;
they are not exposure times. Windows only for now (DirectShow).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from rig_nodes._common import RELIABLE_QOS, raise_timer_resolution, spin, wall_ms, wall_ns
from rig_nodes.uvc import MODES, JpegSplitter, ffmpeg_argv, find_ffmpeg, parse_size, rate_note

# Uncompressed frames are large (307 KB at 640x480): a shorter queue than the 200 of RELIABLE_QOS.
RAW_IMAGE_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                           depth=30, durability=QoSDurabilityPolicy.VOLATILE)


def _child_env(ffmpeg: str) -> dict:
    """ffmpeg's own directory first on PATH: an MSYS2 build finds its DLLs only there."""
    env = dict(os.environ)
    d = os.path.dirname(os.path.abspath(ffmpeg))
    env["PATH"] = d + os.pathsep + env.get("PATH", "")
    return env


class UvcCameraNode(Node):
    def __init__(self, ffmpeg: str, device: str, src: str, size: str, fps: float, mode: str, status_every_s: float) -> None:
        super().__init__(f"{src}_camera")
        self.device, self.src, self.size, self.fps, self.mode = device, src, size, fps, mode
        self.width, self.height = parse_size(size)
        if mode == "mjpeg":
            self.topic = f"/{src}/image/compressed"
            self.pub = self.create_publisher(CompressedImage, self.topic, RELIABLE_QOS)
        else:
            self.topic = f"/{src}/image_raw"
            self.pub = self.create_publisher(Image, self.topic, RAW_IMAGE_QOS)
        self.status_pub = self.create_publisher(String, f"/{src}/status", RELIABLE_QOS)
        self.frames = 0
        self.bytes = 0
        self.error = ""
        self._stop = threading.Event()
        self.proc = subprocess.Popen(ffmpeg_argv(ffmpeg, device, size, fps, mode), stdout=subprocess.PIPE,
                                     stdin=subprocess.DEVNULL, env=_child_env(ffmpeg))
        self._thread = threading.Thread(target=self._read_loop, name="ffmpeg-read", daemon=True)
        self._thread.start()
        self.get_logger().info(f"{device!r} {size} @{fps:g} {mode} → {self.topic}")
        self._last = (0, time.monotonic())
        self.fps_measured: float | None = None
        self._warned = False
        self.create_timer(0.5, self._watch)
        if status_every_s > 0:
            self.create_timer(status_every_s, self._status)

    # ── the pipe ────────────────────────────────────────────────────────────────────────
    def _stamp(self, msg) -> None:
        ns = wall_ns()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.header.frame_id = self.src

    def _read_loop(self) -> None:
        out = self.proc.stdout
        try:
            if self.mode == "mjpeg":
                split = JpegSplitter()
                while not self._stop.is_set():
                    chunk = out.read1(1 << 16)  # what is there; never wait to fill the buffer
                    if not chunk:
                        break
                    for jpeg in split.feed(chunk):
                        msg = CompressedImage(format="jpeg")
                        self._stamp(msg)
                        msg.data = jpeg
                        self.pub.publish(msg)
                        self.frames += 1
                        self.bytes += len(jpeg)
            else:
                n = self.width * self.height
                while not self._stop.is_set():
                    frame = out.read(n)  # buffered pipe: returns short only at end of stream
                    if len(frame) < n:
                        break
                    msg = Image(height=self.height, width=self.width, encoding="mono8", is_bigendian=0, step=self.width)
                    self._stamp(msg)
                    msg.data = frame
                    self.pub.publish(msg)
                    self.frames += 1
                    self.bytes += n
        except Exception as exc:  # a publish after shutdown, a closed pipe
            if not self._stop.is_set():
                self.error = repr(exc)
        if not self._stop.is_set() and not self.error:
            try:
                rc = self.proc.wait(timeout=2)  # the pipe closes a moment before the process is gone
            except subprocess.TimeoutExpired:
                rc = None
            self.error = f"ffmpeg ended (exit {rc}): camera busy, unplugged, or a size/rate it does not offer"

    # ── status ──────────────────────────────────────────────────────────────────────────
    def _publish_status(self, extra: dict) -> None:
        body = {"reachable": not self.error, "t_ms": wall_ms(), "device": self.device, "mode": self.mode,
                "size": self.size, "fps_requested": self.fps, "frames": self.frames, "bytes": self.bytes}
        if self.fps_measured is not None:  # CONTRACT §4: a counter not yet known is absent, never 0
            body["fps_measured"] = round(self.fps_measured, 2)
        body.update(extra)
        self.status_pub.publish(String(data=json.dumps(body, separators=(",", ":"))))

    def _status(self) -> None:
        n0, t0 = self._last
        now = time.monotonic()
        self.fps_measured = (self.frames - n0) / max(1e-9, now - t0)
        self._last = (self.frames, now)
        self._publish_status({})
        self.get_logger().info(f"{self.frames} frames ({self.fps_measured:.1f}/s, {self.bytes / 1e6:.1f} MB)")
        note = rate_note(self.fps, self.fps_measured)
        if note and n0 > 0 and not self._warned:  # skip the first window: it includes start-up
            self._warned = True
            self.get_logger().warning(note)

    def _watch(self) -> None:
        if self.error:
            self._publish_status({"error": self.error})
            self.get_logger().error(self.error)
            raise SystemExit(1)  # a supervisor restarts the node; a dead camera must not look alive

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self.proc.kill()
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass
        return super().destroy_node()


def list_devices(ffmpeg: str, device: str | None) -> int:
    """Print what ffmpeg sees: the cameras, or one camera's formats / sizes / rate ranges."""
    argv = [ffmpeg, "-hide_banner", "-f", "dshow"]
    argv += ["-list_options", "true", "-i", f"video={device}"] if device else ["-list_devices", "true", "-i", "dummy"]
    r = subprocess.run(argv, capture_output=True, text=True, errors="replace", env=_child_env(ffmpeg))
    for line in r.stderr.splitlines():
        if "]" in line and "Error opening" not in line:
            print(line.split("]", 1)[1].rstrip())
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", help='DirectShow camera name, e.g. "Arducam OV9281 USB Camera" (see --list)')
    p.add_argument("--src", default="camera", help="device identity: topic prefix /<src>/… and node name; one per camera (default: camera)")
    p.add_argument("--size", default="640x480", help="frame size WxH (default: 640x480)")
    p.add_argument("--fps", type=float, default=60.0, help="frame rate to ask for; the status line reports what arrives (default: 60)")
    p.add_argument("--mode", choices=MODES, default="mjpeg", help="mjpeg: camera JPEGs untouched → CompressedImage; gray: mono8 Image (default: mjpeg)")
    p.add_argument("--ffmpeg", help="ffmpeg executable (default: $RIG_FFMPEG, PATH, then C:\\msys64\\ucrt64\\bin)")
    p.add_argument("--status-every", type=float, default=5.0, help="/<src>/status + rate line interval in s; 0 = off (default: 5)")
    p.add_argument("--list", action="store_true", help="list the cameras (or, with --device, its modes) and exit")
    args, ros_args = p.parse_known_args(argv)
    ffmpeg = find_ffmpeg(args.ffmpeg)
    if not ffmpeg:
        raise SystemExit("no ffmpeg found: pass --ffmpeg, set RIG_FFMPEG, or put it on PATH")
    if os.name != "nt":
        raise SystemExit("rig-uvc-camera reads cameras through DirectShow: Windows only for now")
    if args.list:
        return list_devices(ffmpeg, args.device)
    if not args.device:
        p.error("--device is required (try --list)")
    parse_size(args.size)
    raise_timer_resolution()
    rclpy.init(args=ros_args)
    node = UvcCameraNode(ffmpeg, args.device, args.src, args.size, args.fps, args.mode, args.status_every)
    spin(node)
    print(f"[uvc-camera] {node.src}: {node.frames} frames, {node.bytes / 1e6:.1f} MB", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
