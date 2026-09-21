#!/usr/bin/env python3
"""fictrac_sim.py — emit FicTrac socket output: random data, a replayed log, or a model fly.

FicTrac (https://github.com/rjdmoore/fictrac) streams one record per camera frame
as a comma-space-separated, newline-terminated line of 25 fields. This tool stands
in for FicTrac so the bridge, the browser closed-loop UI and a REAL ARENA can be
exercised without a camera, a ball or a fly. Four sources:

  (default)            a smooth random walk (soak-harness knobs --turn-sigma / --jump-every)
  fictrac_sim.py X.csv replay a FicTrac data log verbatim, paced by its column 22
  --replay run.jsonl.gz  re-emit a REAL FLY's heading/position trace from one of our own
                       run logs (the behavior_v1/v2 rows [ms,fc,idx,ft,x,y,hd]) at the
                       original pacing — real saccade / bout / pause statistics, no model
  --model fly          a CLOSED-LOOP MODEL FLY: subscribes to the bridge's WebSocket, reads
                       the frame index the arena is showing, and turns toward the feature
                       (--kp, fixation) and/or with the world's motion (--kv, optomotor),
                       plus Ornstein–Uhlenbeck noise, Poisson saccades and walking bouts.
                       With a bias waveform installed (LAB-185) a --kp fly counter-turns;
                       with a mis-signed rig it drives the feature AWAY (anti-fixation) —
                       which is exactly what this mode is for.

Sign conventions (the part worth reading twice):
  * FicTrac's integrated heading (col 17) is a lab-frame rotation about +z, so it INCREASES
    when the fly turns LEFT (counter-clockwise from above). The model fly follows that.
  * Feature azimuth `az` is in the fly's view, degrees, positive = RIGHT, from the frame
    index: az = wrap180(frame_dir · idx · deg_per_frame + feature_az0). `--frame-dir +1`
    (default) means an index increase moves the feature rightward = the display turns
    clockwise — the on-arena sign Isabel confirmed for a positive bias at gain +1.8.
  * Fixation: turn TOWARD the feature → ω = −kp · az (az right → heading decreases = turn
    right). Optomotor: follow the world → ω = −kv · d(az)/dt. If the arena shows the
    feature running away from the model fly, the rig (or --frame-dir) has the wrong sign.

Transport roles mirror real FicTrac so `bridge.py` is identical for sim and real:
  --proto udp (default): FicTrac is the *sender*. This sim `sendto`s each line to
                         (host, port); the bridge binds and receives.
  --proto tcp:           FicTrac's TCP variant is a *server*. This sim listens and
                         accepts one client (the bridge), then streams to it.

FicTrac output columns (1-based), per doc/data_header.txt:
   1     frame counter (starts at 1)
   2-4   delta rotation vector, camera coords (rad)
   5     rotation-estimate error score
   6-8   delta rotation vector, lab coords (rad)
   9-11  absolute orientation, camera coords (rad)
  12-14  absolute orientation, lab coords (rad)
  15-16  integrated x/y position, lab coords (rad; scale by ball radius)
  17     integrated animal heading, lab coords (rad)
  18     instantaneous movement direction, lab coords (rad)
  19     movement speed (rad/frame)
  20-21  integrated forward/side motion (rad)
  22     timestamp (video position or epoch ms)
  23     sequence counter within the current tracking sequence
  24     delta timestamp (ms)
  25     alt. timestamp (ms since midnight)
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import random
import socket
import sys
import threading
import time

N_FIELDS = 25
DEFAULT_PORT = 60000
TWO_PI = 2.0 * math.pi


def fmt_record(fields: list[float]) -> str:
    """Format one record exactly like FicTrac: ', '-joined, newline-terminated.

    The frame counter (field 1) and sequence counter (field 23) are integers on
    the wire; everything else is a float.
    """
    out = []
    for i, v in enumerate(fields):
        if i in (0, 22):  # frame counter, sequence counter
            out.append(str(int(v)))
        else:
            out.append(f"{v:.6f}")
    return ", ".join(out) + "\n"


def wrap180(deg: float) -> float:
    """Wrap degrees into [-180, 180)."""
    return ((deg + 180.0) % 360.0) - 180.0


def wrap_pi(rad: float) -> float:
    """Wrap radians into [-π, π)."""
    return ((rad + math.pi) % TWO_PI) - math.pi


def pack_record(
    frame: int,
    dt_ms: float,
    heading: float,
    d_head: float,
    speed: float,
    move_dir: float,
    x: float,
    y: float,
    fwd: float,
    side: float,
    jitter=lambda sigma: 0.0,
    ts_ms: float | None = None,
) -> list[float]:
    """Pack the lab-frame state into the 25 FicTrac fields (all angles rad).

    `jitter(sigma)` fills the camera-frame / error columns nobody downstream reads with
    small noise (the Walker) or zeros (replay, model). `heading` is wrapped to [0, 2π)
    like FicTrac's col 17. `ts_ms` overrides the frame-derived timestamp (replay).
    """
    if ts_ms is None:
        ts_ms = frame * dt_ms
    fields = [0.0] * N_FIELDS
    fields[0] = frame  # 1
    fields[1] = jitter(0.02)  # 2-4 delta rot cam
    fields[2] = jitter(0.02)
    fields[3] = d_head + jitter(0.01)
    fields[4] = abs(jitter(0.005))  # 5 error score
    fields[5] = jitter(0.02)  # 6-8 delta rot lab
    fields[6] = jitter(0.02)
    fields[7] = d_head
    fields[8] = jitter(0.02)  # 9-11 abs rot cam
    fields[9] = jitter(0.02)
    fields[10] = heading + jitter(0.01)
    fields[11] = jitter(0.02)  # 12-14 abs rot lab
    fields[12] = jitter(0.02)
    fields[13] = heading
    fields[14] = x  # 15-16 integrated x/y
    fields[15] = y
    fields[16] = heading % TWO_PI  # 17 integrated heading
    fields[17] = move_dir % TWO_PI  # 18 movement direction
    fields[18] = speed  # 19 movement speed
    fields[19] = fwd  # 20-21 integrated fwd/side
    fields[20] = side
    # FicTrac writes field 22 in NANOSECONDS; the bridge divides it by FT_TS_NS_PER_MS to get ms
    # (every simulator log before 2026-09-13 had ft 1000× too small — found by the telemetry
    # logging review). Field 25 stays in ms as FicTrac's "ms since midnight".
    fields[21] = ts_ms * 1_000_000.0  # 22 timestamp (ns)
    fields[22] = frame  # 23 sequence counter
    fields[23] = dt_ms  # 24 delta ms
    fields[24] = ts_ms % 86_400_000.0  # 25 abs ms since midnight
    return fields


class Walker:
    """Smooth random walk producing deterministic FicTrac-like records.

    Deterministic given --seed: values come only from the seeded RNG and the
    frame counter, never from the wall clock, so two runs with the same seed
    emit byte-identical records regardless of pacing.
    """

    def __init__(
        self,
        rate_hz: float,
        seed: int | None,
        turn_sigma: float = 0.05,
        jump_every: int = 0,
        jump_deg: float = 90.0,
        noise: float = 1.0,
    ) -> None:
        self.rng = random.Random(seed)
        self.dt = 1.0 / rate_hz
        self.dt_ms = 1000.0 * self.dt
        self.noise = noise  # scales every random-walk sigma (0 = a perfectly still ball)
        # Soak-harness knobs (fw #50): `turn_sigma` is the per-frame heading step
        # (rad) — 0.05 rad ≈ 2.9° ≈ ±1.6 frames/sample at the 1.8°/frame default
        # gain, i.e. a FicTrac-like random walk that keeps the SD reads on the
        # sequential fast path most of the time. `jump_every` > 0 adds a ±jump_deg
        # heading jump every N frames — a wide seek that defeats that fast path.
        self.turn_sigma = turn_sigma
        self.jump_every = max(0, int(jump_every))
        self.jump_rad = math.radians(jump_deg)
        self.frame = 0
        self.heading = 0.0  # integrated heading (rad), field 17
        self.x = 0.0  # integrated x (rad), field 15
        self.y = 0.0  # integrated y (rad), field 16
        self.fwd = 0.0  # integrated forward motion, field 20
        self.side = 0.0  # integrated side motion, field 21

    def _gauss(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma * self.noise)

    def next_record(self) -> list[float]:
        self.frame += 1

        # Per-frame deltas: a small turn and a small forward step.
        d_head = self._gauss(self.turn_sigma)
        if self.jump_every and self.frame % self.jump_every == 0:
            d_head += self.jump_rad if self.rng.random() < 0.5 else -self.jump_rad
        speed = abs(self._gauss(0.03))  # rad/frame, field 19
        move_dir = self.heading + self._gauss(0.1)  # field 18

        # Integrate the lab-frame state.
        self.heading = (self.heading + d_head) % TWO_PI
        self.x += speed * math.cos(self.heading)
        self.y += speed * math.sin(self.heading)
        self.fwd += speed
        self.side += speed * math.sin(d_head)
        return pack_record(
            self.frame, self.dt_ms, self.heading, d_head, speed, move_dir,
            self.x, self.y, self.fwd, self.side, jitter=self._gauss,
        )


# ── model fly: a closed-loop agent that sees the displayed frame index ────────
class ModelFly:
    """A fly that turns toward a feature (fixation) and/or with the world (optomotor).

    Pure and deterministic given `seed`: `step(dt, az_deg)` advances the state by one
    FicTrac frame given the feature's azimuth in the fly's view (deg, right = +, None
    when the display has not been seen yet) and returns the packed record. All internal
    angles are radians; the public knobs are degrees because that is how people think
    about a fly on a ball.

        ω_target = −kp·az  −  kv·d(az)/dt          (deg/s; heading is CCW-positive)
        τ dω/dt = ω_target − ω                     (first-order turning response)
        + OU noise (sigma `noise_dps`, time constant `noise_tau`)
        + Poisson saccades (rate /s, amplitude ~ N(mean, 0.3·mean), fixed duration)
        + walking bouts: two-state Markov (mean bout / pause seconds); a paused fly has
          no fixation/optomotor drive, 20 % of the noise and zero forward speed

    kp is deg/s per deg of azimuth (2 ≈ brisk fixation), kv is dimensionless (1 = perfect
    optomotor following). Both default to 0 so the fly is a noisy walker until asked.
    """

    def __init__(
        self,
        rate_hz: float,
        seed: int | None = None,
        kp: float = 0.0,
        kv: float = 0.0,
        tau_s: float = 0.1,
        noise_dps: float = 30.0,
        noise_tau_s: float = 0.3,
        saccade_rate_hz: float = 0.5,
        saccade_deg: float = 45.0,
        saccade_ms: float = 80.0,
        bout_s: float = 4.0,
        pause_s: float = 0.0,
        speed_rad_s: float = 0.6,
        heading0_deg: float = 0.0,
    ) -> None:
        self.rng = random.Random(seed)
        self.dt = 1.0 / rate_hz
        self.dt_ms = 1000.0 * self.dt
        self.kp = kp
        self.kv = kv
        self.tau = max(tau_s, self.dt)
        self.noise_dps = noise_dps
        self.noise_tau = max(noise_tau_s, self.dt)
        self.saccade_rate = max(0.0, saccade_rate_hz)
        self.saccade_deg = saccade_deg
        self.saccade_frames = max(1, int(round(saccade_ms / 1000.0 / self.dt)))
        self.bout_s = max(bout_s, self.dt)
        self.pause_s = max(0.0, pause_s)
        self.speed = speed_rad_s
        self.omega_max = math.radians(1500.0)
        # state
        self.frame = 0
        self.heading = math.radians(heading0_deg)  # rad, unwrapped
        self.omega = 0.0  # rad/s, smooth turning
        self.noise = 0.0  # rad/s, OU component
        self.saccade_left = 0  # frames remaining in the current saccade
        self.saccade_omega = 0.0  # rad/s during it
        self.walking = True
        self.prev_az: float | None = None
        self.x = self.y = self.fwd = self.side = 0.0
        self.last_az: float | None = None
        self.saccades = 0

    # -- pieces (separately testable) --
    def _drive_dps(self, az_deg: float | None) -> float:
        """Fixation + optomotor drive in deg/s (0 when the display is unseen or paused)."""
        if az_deg is None or not self.walking:
            self.prev_az = az_deg
            return 0.0
        drive = -self.kp * az_deg
        if self.kv and self.prev_az is not None:
            v_az = wrap180(az_deg - self.prev_az) / self.dt  # deg/s, right = +
            drive -= self.kv * v_az
        self.prev_az = az_deg
        return drive

    def _maybe_saccade(self) -> None:
        if self.saccade_left > 0 or self.saccade_rate <= 0:
            return
        if self.rng.random() < self.saccade_rate * self.dt:
            amp = self.rng.gauss(self.saccade_deg, 0.3 * self.saccade_deg)
            sign = 1.0 if self.rng.random() < 0.5 else -1.0
            self.saccade_left = self.saccade_frames
            self.saccade_omega = math.radians(sign * amp) / (self.saccade_frames * self.dt)
            self.saccades += 1

    def _maybe_toggle_bout(self) -> None:
        if self.pause_s <= 0:
            self.walking = True
            return
        mean = self.bout_s if self.walking else self.pause_s
        if self.rng.random() < self.dt / mean:
            self.walking = not self.walking

    def step(self, az_deg: float | None) -> list[float]:
        self.frame += 1
        self.last_az = az_deg
        self._maybe_toggle_bout()
        target = math.radians(self._drive_dps(az_deg))
        # first-order turning response
        self.omega += (target - self.omega) * (self.dt / self.tau)
        # Ornstein–Uhlenbeck noise (exact discretisation)
        a = math.exp(-self.dt / self.noise_tau)
        sigma = math.radians(self.noise_dps) * (1.0 if self.walking else 0.2)
        self.noise = a * self.noise + sigma * math.sqrt(1.0 - a * a) * self.rng.gauss(0.0, 1.0)
        # saccades
        self._maybe_saccade()
        sacc = 0.0
        if self.saccade_left > 0:
            sacc = self.saccade_omega
            self.saccade_left -= 1
        # A fly cannot spin faster than a saccade peak (~1500 °/s); a runaway loop (mis-signed
        # optomotor) then reads as "spinning flat out" instead of an absurd number.
        omega_total = max(-self.omega_max, min(self.omega_max, self.omega + self.noise + sacc))
        d_head = omega_total * self.dt
        prev = self.heading
        self.heading += d_head
        speed = self.speed * self.dt * (1.0 if self.walking else 0.0)
        speed *= max(0.0, 1.0 + self.rng.gauss(0.0, 0.2))
        self.x += speed * math.cos(self.heading)
        self.y += speed * math.sin(self.heading)
        self.fwd += speed
        self.side += speed * math.sin(d_head)
        move_dir = self.heading + self.rng.gauss(0.0, 0.1)
        return pack_record(
            self.frame, self.dt_ms, self.heading % TWO_PI, wrap_pi(self.heading - prev),
            speed, move_dir, self.x, self.y, self.fwd, self.side,
        )


def feature_azimuth(idx: int, deg_per_frame: float, frame_dir: int, az0_deg: float) -> float:
    """Azimuth (deg, right = +) of the pattern's feature when the arena shows frame `idx`."""
    return wrap180(frame_dir * idx * deg_per_frame + az0_deg)


class BridgeFrameFeed:
    """Background WebSocket client: keeps the newest frame index the bridge published.

    The bridge broadcasts every `{"type":"frame","index":…}` to all clients, so the sim
    can watch the loop it is driving. Reconnects forever; `latest()` → (index | None, age_s).
    """

    def __init__(self, url: str) -> None:
        self.url = url
        self._idx: int | None = None
        self._t = 0.0
        self._connected = False
        self._thread = threading.Thread(target=self._run, name="bridge-feed", daemon=True)

    def start(self) -> "BridgeFrameFeed":
        self._thread.start()
        return self

    @property
    def connected(self) -> bool:
        return self._connected

    def latest(self) -> tuple[int | None, float]:
        return self._idx, (time.perf_counter() - self._t) if self._idx is not None else float("inf")

    def _run(self) -> None:
        import asyncio

        from websockets.asyncio.client import connect  # lazy: only the model fly needs it

        async def main() -> None:
            while True:
                try:
                    async with connect(self.url, max_size=16 * 1024 * 1024) as ws:
                        self._connected = True
                        print(f"[sim] watching frames on {self.url}", file=sys.stderr)
                        await ws.send(json.dumps({"type": "hello", "client": "fictrac_sim", "v": 1}))
                        async for raw in ws:
                            try:
                                m = json.loads(raw)
                            except ValueError:
                                continue
                            if isinstance(m, dict) and m.get("type") == "frame":
                                i = m.get("index")
                                if isinstance(i, (int, float)) and math.isfinite(i):
                                    self._idx = int(i)
                                    self._t = time.perf_counter()
                except Exception as exc:  # noqa: BLE001 — a dead bridge must not kill the sim
                    if self._connected:
                        print(f"[sim] bridge feed lost ({exc}); retrying", file=sys.stderr)
                self._connected = False
                await asyncio.sleep(2.0)

        asyncio.run(main())


# ── transports: drive a `send` callable from a source (`emit`) ──────────────
def run_udp(host: str, port: int, emit) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (host, port)
    print(f"[sim] UDP → {host}:{port} (Ctrl-C to stop)", file=sys.stderr)
    emit(lambda b: sock.sendto(b, dst))


def run_tcp(host: str, port: int, emit) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[sim] TCP server on {host}:{port} — waiting for a client …", file=sys.stderr)
    while True:
        conn, peer = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[sim] client connected from {peer[0]}:{peer[1]}", file=sys.stderr)
        try:
            emit(conn.sendall)
            return  # source finished
        except (BrokenPipeError, ConnectionResetError):
            print("[sim] client disconnected — waiting for a new one …", file=sys.stderr)
        finally:
            conn.close()


# ── sources: an `emit(send)` that decides what to send and when ─────────────
def _paced(rate_hz: float, count: int, make_record):
    """Fixed-rate emitter (perf-counter schedule): send(make_record()) until count reached."""
    def emit(send) -> None:
        dt = 1.0 / rate_hz
        start = time.perf_counter()
        n = 0
        while count <= 0 or n < count:
            rec = make_record()
            if rec is None:
                return
            send(fmt_record(rec).encode("ascii"))
            n += 1
            target = start + n * dt
            delay = target - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
    return emit


def emit_generated(walker: Walker, rate_hz: float, count: int):
    """Synthetic random-walk records at a fixed rate (perf-counter schedule)."""
    return _paced(rate_hz, count, walker.next_record)


def emit_model_fly(
    fly: ModelFly,
    feed: BridgeFrameFeed,
    rate_hz: float,
    count: int,
    deg_per_frame: float,
    frame_dir: int,
    az0_deg: float,
    stale_s: float = 1.0,
    status_every_s: float = 1.0,
):
    """Model-fly records at a fixed rate, steering on the frame the bridge last published.

    A frame older than `stale_s` (bridge not applying / no run) counts as "display unseen":
    the fly free-runs on noise + saccades, so it keeps the ball moving between trials.
    """
    state = {"next_status": 0.0}

    def make_record():
        idx, age = feed.latest()
        az = feature_azimuth(idx, deg_per_frame, frame_dir, az0_deg) if idx is not None and age < stale_s else None
        rec = fly.step(az)
        now = time.perf_counter()
        if status_every_s > 0 and now >= state["next_status"]:
            state["next_status"] = now + status_every_s
            seen = f"idx {idx:>3} az {az:+6.1f}°" if az is not None else "display unseen  "
            print(
                f"[fly] {seen} | heading {math.degrees(fly.heading) % 360.0:6.1f}° "
                f"ω {math.degrees(fly.omega + fly.noise):+6.1f}°/s {'walk' if fly.walking else 'pause'} "
                f"saccades {fly.saccades}",
                file=sys.stderr,
            )
        return rec

    return _paced(rate_hz, count, make_record)


def load_fictrac_csv(path: str) -> list[tuple[float, bytes]]:
    """Parse a FicTrac data log into (timestamp_ms, raw_line_bytes) rows.

    Rows are kept verbatim (re-sent byte-for-byte, original separators preserved).
    Lines without ≥22 comma-separated fields or a numeric column 22 (header rows,
    blanks, comments) are skipped.
    """
    rows: list[tuple[float, bytes]] = []
    skipped = 0
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            text = line.strip()
            if not text:
                continue
            parts = text.split(",")
            if len(parts) < 22:
                skipped += 1
                continue
            try:
                ts_ms = float(parts[21])  # column 22 (1-based) = timestamp (ms)
            except ValueError:
                skipped += 1  # header / non-numeric
                continue
            rows.append((ts_ms, (text + "\n").encode("ascii", errors="replace")))
    if skipped:
        print(f"[sim] skipped {skipped} non-data line(s)", file=sys.stderr)
    return rows


def emit_playback(rows: list[tuple[float, bytes]], speed: float):
    """Replay recorded rows, paced by the inter-row Δ of column 22 (ms).

    First row goes immediately; non-increasing timestamps clamp to no delay;
    `speed` > 1 plays faster. Plays once, then returns.
    """
    def emit(send) -> None:
        if not rows:
            return
        sched = time.perf_counter()
        prev_ts = rows[0][0]
        for ts_ms, raw in rows:
            d = (ts_ms - prev_ts) / 1000.0 / speed
            prev_ts = ts_ms
            if d > 0:
                sched += d
            delay = sched - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            send(raw)
    return emit


# ── run-log replay: a real fly's trace from one of OUR logs ─────────────────
def load_runlog_rows(path: str) -> list[tuple[float, int, float, float, float]]:
    """(ms, fc, x, y, hd) from a bridge run log (.jsonl or .jsonl.gz), any level.

    behavior_v1/v2 rows are `[ms, fc, idx, ft, x, y, hd]` (first element numeric);
    `full`-level frames are `{"type":"fictrac_frame","fictrac":[25 cols]}`. String-tagged
    arrays (`"a"`, `"cc"`, …) and every other object are skipped. `ms` restarts at 0 on
    every log activation; the replay pacer treats a backwards step as no delay.
    """
    rows: list[tuple[float, int, float, float, float]] = []
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if isinstance(o, list):
                if len(o) >= 7 and isinstance(o[0], (int, float)) and o[6] is not None:
                    rows.append((float(o[0]), int(o[1]), float(o[4]), float(o[5]), float(o[6])))
            elif isinstance(o, dict) and o.get("type") == "fictrac_frame":
                f = o.get("fictrac")
                if isinstance(f, list) and len(f) >= 22:
                    rows.append((float(f[21]) / 1e6, int(f[0]), float(f[14]), float(f[15]), float(f[16])))
    return rows


def replay_records(rows, loop: bool = False, max_gap_ms: float = 1000.0):
    """Generator of (delay_s, fields) from run-log rows, heading kept continuous across loops."""
    if not rows:
        return
    frame = 0
    head_off = 0.0  # keeps heading continuous when the file restarts
    prev_hd = None
    pass_no = 0
    while True:
        prev_ms = rows[0][0]
        first_hd = rows[0][4]
        if prev_hd is not None:
            head_off = prev_hd - first_hd  # new pass starts where the last one ended
        for ms, fc, x, y, hd in rows:
            gap = ms - prev_ms
            prev_ms = ms
            delay = min(max(gap, 0.0), max_gap_ms) / 1000.0
            frame += 1
            heading = hd + head_off
            d_head = 0.0 if prev_hd is None else wrap_pi(heading - prev_hd)
            prev_hd = heading
            yield delay, pack_record(frame, gap if gap > 0 else 0.0, heading, d_head, 0.0, heading,
                                     x, y, 0.0, 0.0, ts_ms=ms + pass_no * (rows[-1][0] + 20.0))
        pass_no += 1
        if not loop:
            return


def emit_runlog_replay(rows, speed: float, loop: bool):
    """Re-emit a real fly's heading trace at (scaled) original pacing."""
    def emit(send) -> None:
        sched = time.perf_counter()
        for delay, fields in replay_records(rows, loop=loop):
            sched += delay / speed
            wait = sched - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            send(fmt_record(fields).encode("ascii"))
    return emit


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file", nargs="?", help="FicTrac data log (CSV) to play back; omit to generate random data")
    p.add_argument("--proto", choices=("udp", "tcp"), default="udp", help="transport (default: udp)")
    p.add_argument("--host", default="127.0.0.1", help="UDP destination / TCP bind address (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default: {DEFAULT_PORT})")
    p.add_argument("--rate", type=float, default=50.0, help="generated/model mode: frames per second (default: 50)")
    p.add_argument("--seed", type=int, default=None, help="generated/model mode: RNG seed for reproducible output")
    p.add_argument(
        "--noise",
        type=float,
        default=1.0,
        help="generated mode: scales all random-walk noise sigmas (default: 1.0)",
    )
    p.add_argument("--count", type=int, default=0, help="generated/model mode: frames to emit then exit (0 = forever)")
    p.add_argument("--speed", type=float, default=1.0, help="playback/replay mode: speed multiplier (default: 1.0)")
    p.add_argument(
        "--turn-sigma",
        type=float,
        default=0.05,
        help="generated mode: per-frame heading step sigma in rad (default 0.05 ≈ ±1.6 frames/sample "
        "at gain 1.8; soak harness, fw #50)",
    )
    p.add_argument(
        "--jump-every",
        type=int,
        default=0,
        help="generated mode: add a ±JUMP_DEG heading jump every N frames (0 = never); a wide "
        "frame seek that defeats the SD sequential-read fast path",
    )
    p.add_argument("--jump-deg", type=float, default=90.0, help="generated mode: jump size in degrees (default 90)")

    g = p.add_argument_group("run-log replay (a real fly's trace from one of our .jsonl[.gz] logs)")
    g.add_argument("--replay", metavar="RUNLOG", help="re-emit the FicTrac rows of a bridge run log at original pacing")
    g.add_argument("--loop", action="store_true", help="replay: start over when the log ends (heading stays continuous)")

    m = p.add_argument_group("model fly (closed loop: steers on the frame index the bridge publishes)")
    m.add_argument("--model", choices=("fly",), help="use the model fly instead of the random walk")
    m.add_argument("--bridge", default="ws://127.0.0.1:8765", help="bridge WebSocket to watch frames on (default: ws://127.0.0.1:8765)")
    m.add_argument("--kp", type=float, default=0.0, help="fixation gain, deg/s per deg of feature azimuth (0 = off; try 2)")
    m.add_argument("--kv", type=float, default=0.0, help="optomotor gain, fly ω / world ω (0 = off; 1 = perfect following)")
    m.add_argument("--tau", type=float, default=0.1, help="turning-response time constant, s (default 0.1)")
    m.add_argument("--noise-dps", type=float, default=30.0, help="Ornstein–Uhlenbeck turning noise sigma, deg/s (default 30)")
    m.add_argument("--noise-tau", type=float, default=0.3, help="noise time constant, s (default 0.3)")
    m.add_argument("--saccade-rate", type=float, default=0.5, help="saccades per second (default 0.5; 0 = none)")
    m.add_argument("--saccade-deg", type=float, default=45.0, help="mean saccade amplitude, deg (default 45)")
    m.add_argument("--saccade-ms", type=float, default=80.0, help="saccade duration, ms (default 80)")
    m.add_argument("--bout-s", type=float, default=4.0, help="mean walking bout, s (default 4)")
    m.add_argument("--pause-s", type=float, default=0.0, help="mean pause between bouts, s (default 0 = never pauses)")
    m.add_argument("--speed-rad-s", type=float, default=0.6, help="forward speed while walking, ball rad/s (default 0.6)")
    m.add_argument("--heading0", type=float, default=0.0, help="initial heading, deg (default 0)")
    m.add_argument("--deg-per-frame", type=float, default=1.8, help="display degrees per frame index (default 1.8 = 360/200)")
    m.add_argument("--frame-dir", type=int, choices=(1, -1), default=1, help="+1: an index increase moves the feature RIGHT (display clockwise; the confirmed sign at gain +1.8). -1 flips it")
    m.add_argument("--feature-az0", type=float, default=0.0, help="feature azimuth at frame 0, deg, right = + (default 0 = frontal)")
    m.add_argument("--stale-s", type=float, default=1.0, help="a published frame older than this counts as 'display unseen' (default 1.0)")
    args = p.parse_args(argv)

    if args.replay is not None:
        # ── run-log replay: a real fly's heading trace from a bridge log ──
        if args.file is not None or args.model:
            p.error("--replay cannot be combined with a CSV file or --model")
        if not os.path.isfile(args.replay):
            p.error(f"file not found: {args.replay}")
        if args.speed <= 0:
            p.error("--speed must be > 0")
        rows = load_runlog_rows(args.replay)
        if not rows:
            p.error(f"no FicTrac rows found in {args.replay}")
        span = sum(min(max(b[0] - a[0], 0.0), 1000.0) for a, b in zip(rows, rows[1:])) / 1000.0 / args.speed
        print(
            f"[sim] replaying {os.path.basename(args.replay)}: {len(rows)} FicTrac rows over ~{span:.0f}s "
            f"(speed {args.speed:g}×{', looping' if args.loop else ''})",
            file=sys.stderr,
        )
        emit = emit_runlog_replay(rows, args.speed, args.loop)
    elif args.file is not None:
        # ── playback mode: replay a recorded FicTrac log, paced by column 22 ──
        if not os.path.isfile(args.file):
            p.error(f"file not found: {args.file}")
        if args.speed <= 0:
            p.error("--speed must be > 0")
        rows = load_fictrac_csv(args.file)
        if not rows:
            p.error(f"no FicTrac data rows found in {args.file}")
        span = (rows[-1][0] - rows[0][0]) / 1000.0 / args.speed
        print(
            f"[sim] playing back {args.file}: {len(rows)} rows over ~{span:.1f}s "
            f"(speed {args.speed:g}×)",
            file=sys.stderr,
        )
        emit = emit_playback(rows, args.speed)
    elif args.model == "fly":
        # ── model fly: closed loop through the real bridge / browser / arena ──
        if args.rate <= 0:
            p.error("--rate must be > 0")
        if args.deg_per_frame <= 0:
            p.error("--deg-per-frame must be > 0")
        fly = ModelFly(
            rate_hz=args.rate, seed=args.seed, kp=args.kp, kv=args.kv, tau_s=args.tau,
            noise_dps=args.noise_dps, noise_tau_s=args.noise_tau,
            saccade_rate_hz=args.saccade_rate, saccade_deg=args.saccade_deg, saccade_ms=args.saccade_ms,
            bout_s=args.bout_s, pause_s=args.pause_s, speed_rad_s=args.speed_rad_s, heading0_deg=args.heading0,
        )
        feed = BridgeFrameFeed(args.bridge).start()
        print(
            f"[sim] model fly: kp={args.kp:g} kv={args.kv:g} tau={args.tau:g}s noise={args.noise_dps:g}°/s "
            f"saccades {args.saccade_rate:g}/s×{args.saccade_deg:g}° pause={args.pause_s:g}s | "
            f"feature az = {args.frame_dir:+d}·idx·{args.deg_per_frame:g}° + {args.feature_az0:g}°",
            file=sys.stderr,
        )
        emit = emit_model_fly(fly, feed, args.rate, args.count, args.deg_per_frame, args.frame_dir,
                              args.feature_az0, stale_s=args.stale_s)
    else:
        # ── generated mode: synthetic random walk ──
        if args.rate <= 0:
            p.error("--rate must be > 0")
        if args.noise < 0:
            p.error("--noise must be >= 0")
        if args.turn_sigma < 0 or args.jump_every < 0:
            p.error("--turn-sigma and --jump-every must be >= 0")
        walker = Walker(
            rate_hz=args.rate,
            seed=args.seed,
            turn_sigma=args.turn_sigma,
            jump_every=args.jump_every,
            jump_deg=args.jump_deg,
            noise=args.noise,
        )
        emit = emit_generated(walker, args.rate, args.count)

    try:
        if args.proto == "udp":
            run_udp(args.host, args.port, emit)
        else:
            run_tcp(args.host, args.port, emit)
    except KeyboardInterrupt:
        print("\n[sim] stopped", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
