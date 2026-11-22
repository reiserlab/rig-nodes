#!/usr/bin/env python3
"""uvc_sim.py — a stand-in for ffmpeg reading a UVC camera: frames on stdout, on any OS.

`rig-uvc-camera --sim` starts this instead of ffmpeg, with the same command line
(rig_nodes.uvc.ffmpeg_argv). It reads `-video_size`, `-framerate` and the output format from that
line and writes what ffmpeg would write to its pipe:

    -f mjpeg     one JPEG per frame. Every frame is the same small picture (64x48, whatever size
                 was asked for) with a comment segment that carries the frame number and the bytes
                 FF D9, so a splitter that stops at the first FF D9 fails on every frame.
    -f rawvideo  width*height bytes of 8-bit luma per frame, the value = frame number mod 256.

A real camera's quirks are switched on through the environment, so the node needs no test options:

    RIG_UVC_SIM_RATE=101      deliver this rate, whatever was asked for (a camera that picks its own)
    RIG_UVC_SIM_FRAMES=50     stop after this many frames with exit code 1 (camera unplugged)
    RIG_UVC_SIM_DROP_AT=20,21 before these frames, write ffmpeg's "frame dropped" line to stderr

    pixi run uvc-camera -- --sim --src camera --fps 60
"""

from __future__ import annotations

import base64
import os
import sys
import time

# A 64x48 8-bit gradient, baseline JPEG, 604 bytes (written by Pillow 12.2, quality 50).
_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDABALDA4MChAODQ4SERATGCgaGBYWGDEjJR0oOjM9PDkzODdASFxOQERXRTc4UG1RV19i"
    "Z2hnPk1xeXBkeFxlZ2P/wAALCAAwAEABAREA/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUF"
    "BAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVW"
    "V1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi"
    "4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/AOJgStCBK0IErQgStGBK0IErQgSq/iTVP7F0Ka5Q4nf93Dx/Geh6EcAE89cY715d"
    "AlaECVoQJWjAlaECVoQJWhAlebeOdU/tLXTbRnMFlmMcdX/jPTPUAd/u5HWs+BK0IErQgStGBK0IErQgSq/iTVP7F0Ka5Q4nf93D"
    "x/Geh6EcAE89cY715NAlbECVoQJWjAlaECVoQJWhAlebeOdU/tLXTbRnMFlmMcdX/jPTPUAd/u5HWseBK2IErQgStGBK0IErQgSq"
    "/iTVP7F0Ka5Q4nf93Dx/Geh6EcAE89cY715NAlaECVsQJWjAlaECVoQJWhAlebeOdU/tLXTbRnMFlmMcdX/jPTPUAd/u5HWseBK0"
    "IEr/2Q=="
)
DROP_LINE = "[dshow @ sim] real-time buffer [sim] [video input] too full or near too full (101% of size: 67108864 [rtbufsize parameter])! frame dropped!"


def jpeg_frame(n: int) -> bytes:
    """The picture with a COM segment after SOI: the frame number, then FF D9 inside the segment."""
    payload = f"rig-uvc-sim frame {n} ".encode() + b"\xff\xd9"
    return _JPEG[:2] + b"\xff\xfe" + (len(payload) + 2).to_bytes(2, "big") + payload + _JPEG[2:]


def frame_number(jpeg: bytes) -> int | None:
    """The frame number a jpeg_frame() carries; None for any other JPEG."""
    tag = b"rig-uvc-sim frame "
    i = jpeg.find(tag, 0, 64)
    if i < 0:
        return None
    try:
        return int(jpeg[i + len(tag): jpeg.index(b" ", i + len(tag))])
    except ValueError:
        return None


def _option(argv: list[str], name: str, default: str) -> str:
    return argv[argv.index(name) + 1] if name in argv[:-1] else default


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if "-h" in argv or "--help" in argv:
        print(__doc__)
        return 0
    w, h = (int(v) for v in _option(argv, "-video_size", "640x480").split("x"))
    raw = argv[-3:-1] == ["-f", "rawvideo"]
    rate = float(os.environ.get("RIG_UVC_SIM_RATE") or _option(argv, "-framerate", "60"))
    limit = int(os.environ.get("RIG_UVC_SIM_FRAMES") or 0)
    drop_at = {int(v) for v in (os.environ.get("RIG_UVC_SIM_DROP_AT") or "").split(",") if v.strip()}
    out = sys.stdout.buffer
    period = 1.0 / rate
    next_t = time.monotonic()
    n = 0
    try:
        while not limit or n < limit:
            n += 1
            if n in drop_at:
                print(DROP_LINE, file=sys.stderr, flush=True)
            out.write(bytes([n % 256]) * (w * h) if raw else jpeg_frame(n))
            out.flush()
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    except (BrokenPipeError, KeyboardInterrupt, OSError):
        return 0
    return 1  # the frame limit: a camera that went away


if __name__ == "__main__":
    raise SystemExit(main())
