"""uvc.py — the pure parts of the UVC camera node: the ffmpeg command line, the MJPEG stream
splitter and the rate check. No ROS, no camera; tests/test_pure.py covers all of it.
"""

from __future__ import annotations

import os
import shutil

SOI = b"\xff\xd8"  # JPEG start of image
EOI = b"\xff\xd9"  # JPEG end of image

MODES = ("mjpeg", "gray")
FFMPEG_ENV = "RIG_FFMPEG"
# Where the lab's Windows PCs have it (MSYS2 ucrt64); only consulted when PATH has none.
_FALLBACKS = (r"C:\msys64\ucrt64\bin\ffmpeg.exe",)


def find_ffmpeg(explicit: str | None = None) -> str | None:
    """`--ffmpeg`, else $RIG_FFMPEG, else PATH, else the MSYS2 location; None when none exists."""
    for cand in (explicit, os.environ.get(FFMPEG_ENV)):
        if cand:
            return cand if os.path.isfile(cand) else shutil.which(cand)
    found = shutil.which("ffmpeg")
    if found:
        return found
    for cand in _FALLBACKS:
        if os.path.isfile(cand):
            return cand
    return None


def ffmpeg_argv(ffmpeg: str, device: str, size: str, fps: float, mode: str) -> list[str]:
    """ffmpeg reading a DirectShow camera and writing frames to stdout.

    mjpeg: the camera's own JPEGs, stream-copied (no decode, no re-encode) as a bare MJPEG stream.
    gray:  the camera's YUYV decoded to 8-bit luma, `width*height` bytes per frame. `-fps_mode
           passthrough` matters: without it ffmpeg resamples to exactly `fps` and drops the frames
           of a camera that runs slightly fast (61.5 → 59.8 fps measured on ww18, 2026-11-10).
    """
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}, not {mode!r}")
    w, h = parse_size(size)
    argv = [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "warning", "-rtbufsize", "64M",
            "-f", "dshow", "-video_size", f"{w}x{h}", "-framerate", f"{fps:g}"]
    if mode == "mjpeg":
        argv += ["-vcodec", "mjpeg", "-i", f"video={device}", "-c", "copy", "-flush_packets", "1", "-f", "mjpeg", "-"]
    else:
        argv += ["-pixel_format", "yuyv422", "-i", f"video={device}", "-pix_fmt", "gray",
                 "-fps_mode", "passthrough", "-flush_packets", "1", "-f", "rawvideo", "-"]
    return argv


def parse_size(size: str) -> tuple[int, int]:
    """'640x480' → (640, 480)."""
    try:
        w, h = (int(v) for v in size.lower().split("x"))
    except ValueError:
        raise ValueError(f"size must look like 640x480, not {size!r}") from None
    if w <= 0 or h <= 0:
        raise ValueError(f"size must be positive, not {size!r}")
    return w, h


class JpegSplitter:
    """Cut a concatenated MJPEG byte stream into whole JPEGs.

    `feed(chunk)` returns the JPEGs completed by that chunk, each from SOI through EOI. Bytes
    before the first SOI are discarded (and counted in `skipped`). Inside entropy-coded data a
    0xFF is always followed by 0x00, so EOI cannot appear there; a JPEG that embeds another one
    in an APPn segment (an EXIF thumbnail) would be cut short — UVC cameras do not send those.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._scan = 0  # where the search for EOI resumes (never rescans a chunk)
        self.skipped = 0

    def feed(self, chunk: bytes) -> list[bytes]:
        buf = self._buf
        buf += chunk
        out: list[bytes] = []
        while True:
            if self._scan == 0:
                s = buf.find(SOI)
                if s < 0:
                    keep = 1 if buf[-1:] == b"\xff" else 0  # an SOI may be split across chunks
                    self.skipped += len(buf) - keep
                    del buf[: len(buf) - keep]
                    return out
                if s:
                    self.skipped += s
                    del buf[:s]
                self._scan = 2
            e = buf.find(EOI, self._scan)
            if e < 0:
                self._scan = max(2, len(buf) - 1)
                return out
            out.append(bytes(buf[: e + 2]))
            del buf[: e + 2]
            self._scan = 0


def rate_note(requested: float, measured: float, tolerance: float = 0.05) -> str:
    """'' when the measured rate is within `tolerance` of the requested one, else a sentence.

    UVC cameras pick the nearest rate they have without saying so, and neither the driver nor
    ffmpeg reports it (an Arducam at 640x480 MJPEG: asked 120 → 100, asked 150 → 200). Counting
    frames against the host clock is the only check.
    """
    if requested <= 0 or measured <= 0:
        return ""
    if abs(measured - requested) <= tolerance * requested:
        return ""
    return f"asked for {requested:g} fps, camera delivers {measured:.1f} fps"
