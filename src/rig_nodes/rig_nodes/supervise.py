"""supervise.py — process supervision primitives shared by every rig launcher and test harness.

Extracted from webDisplayTools' `rig/nodes/rig.py` and `_common.py` (2026-10-26). A launcher such
as the arena rig's `rig.py` composes a rig from these: which children to start, in which order and
what to do when one dies stay with the application (rig-software-repos-and-runners.md §3.2,
"application composition stays with the application"); the portable mechanics live here.

* `popen_group` / `signal_group` / `SIGKILL` — a child in its own process group on every platform,
  and a stop signal that reaches wrappers AND what they spawn (`ros2 run … rmw_zenohd` does not
  forward SIGINT to the router binary). Windows: CREATE_NEW_PROCESS_GROUP + CTRL_BREAK_EVENT.
* `wait_port` — poll a TCP port (the Zenoh router on 127.0.0.1:7447).
* `port_in_use` — the preflight probe: is something already bound here? Correct on Windows too
  (a plain bind lies there; see the function).
* `preflight` — turn a list of (proto, port, who) into the human-readable refusal a launcher
  prints before touching anything.
* `split_args` — `--x-args "…"` → argv, with the Windows backslash rule.
* `Child` — one supervised child: start (own process group, stderr prefixed with its name),
  alive, stop with SIGINT → SIGTERM → SIGKILL escalation.
"""

from __future__ import annotations

import os
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

# Windows' signal module has no SIGKILL; every "kill it" site uses this constant so the
# escalation paths do not raise AttributeError there.
SIGKILL = getattr(signal, "SIGKILL", 9)


def popen_group(argv: list, **kwargs) -> subprocess.Popen:
    """Popen with the child in its OWN process group, on every platform, so a stop signal
    reaches wrappers and what they spawn. POSIX: setsid; Windows: CREATE_NEW_PROCESS_GROUP,
    which is also what makes CTRL_BREAK_EVENT deliverable to that child."""
    if os.name == "nt":
        kwargs.setdefault("creationflags", subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs.setdefault("start_new_session", True)
    return subprocess.Popen(argv, **kwargs)


def signal_group(proc: subprocess.Popen, sig: int) -> None:
    """Send SIGINT / SIGTERM / SIGKILL to the child's whole group, portably.
    Windows has no SIGINT for other processes (send_signal raises ValueError): SIGINT maps to
    CTRL_BREAK_EVENT on the child's group, SIGTERM and SIGKILL to TerminateProcess."""
    if os.name == "nt":
        if sig == signal.SIGINT:
            try:
                proc.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
                return
            except (ValueError, OSError):
                pass
        proc.kill() if sig == SIGKILL else proc.terminate()
        return
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        proc.send_signal(sig)


def wait_port(host: str, port: int, timeout_s: float) -> bool:
    """True once a TCP connect to host:port succeeds, polling every 0.2 s up to timeout_s."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def port_in_use(proto: str, port: int, host: str = "127.0.0.1") -> bool:
    """True when binding host:port fails (something else holds it) — or, on Windows, when a
    TCP connect to it succeeds."""
    kind = socket.SOCK_STREAM if proto == "tcp" else socket.SOCK_DGRAM
    s = socket.socket(socket.AF_INET, kind)
    try:
        if proto == "tcp" and os.name != "nt":
            # Ignore TIME_WAIT leftovers from a just-stopped rig; a LIVE listener still fails.
            # Not on Windows: there SO_REUSEADDR lets this probe bind OVER a live listener (ww18
            # 2026-10-15: a running router on 7447 went unreported), and TIME_WAIT does not block
            # a plain bind.
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
    except OSError:
        return True
    finally:
        s.close()
    if proto == "tcp" and os.name == "nt":
        # Windows: a listener that shares its address blocks NO bind, wildcard or specific — the
        # real BIAS (Qt, 0.0.0.0:5010) on ww18 2026-10-15. Only a connect sees it.
        try:
            with socket.create_connection((host, port), timeout=0.3):
                return True
        except OSError:
            pass
    return False


def preflight(ports: list[tuple[str, int, str]], host: str = "127.0.0.1") -> list[str]:
    """Probe every (proto, port, who) and return one line per port already bound, e.g.
    `  tcp 127.0.0.1:7447 is in use — probably a Zenoh router`. Empty list = clear to start."""
    taken = []
    for proto, port, who in ports:
        if proto and port_in_use(proto, port, host):
            taken.append(f"  {proto} {host}:{port} is in use — probably {who}")
    return taken


def split_args(s: str) -> list[str]:
    """`--x-args "…"` → argv. POSIX shlex treats `\\` as an escape, so on Windows
    `--video-dir D:\\rec\\reiserm\\rig` became the drive-relative `D:recreisermrig` and BIAS
    wrote the run's video there (ww18 2026-10-15). Windows: split without escapes, then drop
    the quotes around a token ("C:\\My Videos\\x")."""
    if not s:
        return []
    if os.name != "nt":
        return shlex.split(s)
    return [t[1:-1] if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'" else t for t in shlex.split(s, posix=False)]


class Child:
    """One supervised child process. Its stdout+stderr are pumped to OUR stderr, each line
    prefixed `[name    ] `, so a rig's console interleaves every node's output."""

    def __init__(self, name: str, argv: list[str], env: dict | None = None, prefix_width: int = 8) -> None:
        self.name = name
        self.argv = argv
        self.env = env
        self.prefix_width = prefix_width
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        self.proc = popen_group(
            self.argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=(self.env if self.env is not None else os.environ.copy()),
        )
        threading.Thread(target=self._pump, name=f"pump-{self.name}", daemon=True).start()

    def _signal(self, sig: int) -> None:
        assert self.proc
        signal_group(self.proc, sig)

    def _pump(self) -> None:
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            sys.stderr.write(f"[{self.name:{self.prefix_width}}] {line}")
        sys.stderr.flush()

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    @property
    def returncode(self) -> int | None:
        return self.proc.returncode if self.proc else None

    def stop(self, grace_s: float = 3.0, term_s: float = 2.0) -> None:
        """SIGINT, wait grace_s; SIGTERM, wait term_s; SIGKILL."""
        if not self.alive():
            return
        assert self.proc
        try:
            self._signal(signal.SIGINT)
            self.proc.wait(grace_s)
        except subprocess.TimeoutExpired:
            self._signal(signal.SIGTERM)
            try:
                self.proc.wait(term_s)
            except subprocess.TimeoutExpired:
                self._signal(SIGKILL)
        except (ProcessLookupError, OSError, ValueError):
            pass
