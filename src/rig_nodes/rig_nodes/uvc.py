"""uvc.py — the pure parts of the UVC camera node: the ffmpeg command line, the MJPEG stream
splitter and the rate check. No ROS, no camera; tests/test_pure.py covers all of it.
"""

from __future__ import annotations

import os
import re
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


# Inside a scan a 0xFF is followed by 0x00 (a stuffed data byte) or by RST0–RST7; anything else
# starts a marker. One C-level search finds the end of the scan.
_SCAN_END = re.compile(rb"\xff[^\x00\xd0-\xd7\xff]")
_NO_LENGTH = frozenset((0x01, *range(0xD0, 0xD8)))  # TEM, RST0–RST7: markers without a length
MAX_JPEG_BYTES = 32 << 20  # a "frame" that grows past this is not a JPEG: resynchronize


class JpegSplitter:
    """Cut a concatenated MJPEG byte stream into whole JPEGs.

    `feed(chunk)` returns the JPEGs completed by that chunk, each from SOI through EOI. The
    splitter follows the JPEG structure: marker segments are skipped by their length, so the
    bytes FF D9 inside a table, an APPn segment or a comment do not end a frame, and EOI is
    looked for only where a marker can stand. Bytes before an SOI are discarded and counted in
    `skipped`; a stream that stops being a JPEG (no marker where one must be, a nested SOI, a
    frame beyond MAX_JPEG_BYTES) is counted in `malformed` and the search restarts at the next SOI.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._pos = 0  # next byte to look at; 0 = no SOI yet (a chunk is never rescanned)
        self._in_scan = False
        self.skipped = 0
        self.malformed = 0

    def _resync(self, drop: int) -> None:
        self.malformed += 1
        self.skipped += drop
        del self._buf[:drop]
        self._pos, self._in_scan = 0, False

    def feed(self, chunk: bytes) -> list[bytes]:
        buf = self._buf
        buf += chunk
        out: list[bytes] = []
        while True:
            if self._pos == 0:
                s = buf.find(SOI)
                if s < 0:
                    keep = 1 if buf[-1:] == b"\xff" else 0  # an SOI may be split across chunks
                    self.skipped += len(buf) - keep
                    del buf[: len(buf) - keep]
                    return out
                if s:
                    self.skipped += s
                    del buf[:s]
                self._pos, self._in_scan = 2, False
            if len(buf) > MAX_JPEG_BYTES:
                self._resync(2)
                continue
            pos = self._pos
            if self._in_scan:
                m = _SCAN_END.search(buf, pos)
                if m is None:
                    self._pos = max(pos, len(buf) - 1)  # the last byte may be the 0xFF of a marker
                    return out
                self._pos, self._in_scan = m.start(), False
                continue
            if len(buf) < pos + 2:
                return out
            if buf[pos] != 0xFF:
                self._resync(2)  # drop this SOI; the next one starts a new attempt
                continue
            marker = buf[pos + 1]
            if marker == 0xFF:  # fill byte before a marker
                self._pos = pos + 1
            elif marker == 0xD9:
                out.append(bytes(buf[: pos + 2]))
                del buf[: pos + 2]
                self._pos = 0
            elif marker == 0xD8:  # a new image inside an unfinished one: the first was cut off
                self._resync(pos)
            elif marker in _NO_LENGTH:
                self._pos = pos + 2
            else:
                if len(buf) < pos + 4:
                    return out
                length = (buf[pos + 2] << 8) | buf[pos + 3]
                if length < 2:
                    self._resync(2)
                    continue
                if len(buf) < pos + 2 + length:
                    return out
                self._pos = pos + 2 + length
                self._in_scan = marker == 0xDA  # SOS: entropy-coded data follows its header


FRAME_DROPPED = "frame dropped"  # ffmpeg's dshow input: "real-time buffer [...] too full ...! frame dropped!"


def is_drop_line(line: str) -> bool:
    """True for the line ffmpeg's DirectShow input writes when its real-time buffer overflowed
    and it discarded a camera frame (the reader behind the pipe was too slow)."""
    return FRAME_DROPPED in line


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
