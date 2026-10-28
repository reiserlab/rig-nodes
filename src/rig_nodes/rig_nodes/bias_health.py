"""bias_health.py — BIAS rate and loss numbers from `/bias/status` polls (pure, no ROS).

Moved from webDisplayTools `rig/nodes/bias_health.py` (the okey-cokey camera rig, 2026-10-26).
Consumers: the arena visualization node live (`/viz/bias/fps_from_count`, `/viz/bias/drops`) and
the dashboard replay that rebuilds those topics from a finished run's bag.
"""

from __future__ import annotations

from collections import deque

# Loss counters the atlas-bench Atlas10 BIAS build reports in get-status (the FLIR/Spinnaker
# build may report none: absent = not reported, never 0).
BIAS_DROP_KEYS = ("frameIdGaps", "cameraSkipped", "incompleteFrames", "loggerSkipped", "writerSkipped")


class CountRate:
    """Frames per second from a running frame counter sampled at uneven times: the slope over the
    last `window_s` seconds. A counter that goes DOWN (a new recording) restarts the window."""

    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s = window_s
        self.samples: deque = deque()

    def add(self, t_s: float, count: float) -> float | None:
        if self.samples and (count < self.samples[-1][1] or t_s <= self.samples[-1][0] - 5.0):
            self.samples.clear()
        self.samples.append((t_s, count))
        while len(self.samples) > 2 and self.samples[0][0] < t_s - self.window_s:
            self.samples.popleft()
        (t0, c0), (t1, c1) = self.samples[0], self.samples[-1]
        return (c1 - c0) / (t1 - t0) if t1 - t0 > 0.2 else None


def bias_drops(st: dict) -> float | None:
    """Sum of the drop counters BIAS reports, None when it reports none of them."""
    vals = [st[k] for k in BIAS_DROP_KEYS if isinstance(st.get(k), (int, float))]
    return float(sum(vals)) if vals else None
