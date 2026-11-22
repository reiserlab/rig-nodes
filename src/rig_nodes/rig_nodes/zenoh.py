"""zenoh.py — the loopback-only Zenoh configuration every rig graph runs with.

`rig_nodes/config/zenoh-router.json5` and `zenoh-session.json5` are the stock rmw_zenoh files with
three changes: the router listens on `tcp/127.0.0.1:7447` instead of `tcp/[::]:7447` (fails on a
host without IPv6), sessions connect to 127.0.0.1 instead of `localhost`, and the `routing.peer` /
`peers_failover_brokering` keys are removed (Zenoh reports them as deprecated on every node
start, first seen on RoboStack's Jazzy 0.19 set; nothing on a single-host loopback graph used them). A rig PC's
graph is therefore not reachable from the lab network (CONTRACT.md §1).

ONE pair ships, derived from the stock `DEFAULT_RMW_ZENOH_*_CONFIG.json5` of rmw_zenoh 0.10.6,
the version RoboStack packages for Lyrical (the only set since package set 0.5.0, 2026-11-18;
the stock files of Jazzy's 0.2.11 were byte-identical, so the pair did not change with the move).
Upstream's `lyrical` branch (rmw_zenoh 0.10.8) does change the schema — per-mode
`scouting.*.listen`, `autoconnect` lists with `client`, a `gateway` block, the shared-memory
`messages` list — so the day RoboStack rebuilds Lyrical on that version, re-derive the pair
(same three edits applied to the new stock files). `config_dir()` below still looks for a
`config/<ROS_DISTRO>/` pair first and falls back to the shared one: the lookup predates the move
to one distribution and is kept because consumers import this module and a re-derived pair can be
staged there beside the old one; with one distribution, replacing the shared pair is equivalent.

Shipped INSIDE the package so an installed consumer needs no repository checkout: importing
`rig_nodes._common` sets ZENOH_ROUTER_CONFIG_URI / ZENOH_SESSION_CONFIG_URI to these files
unless the environment already names others (`apply_env_defaults`), and `rig-router` starts
`rmw_zenohd` with them.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent / "config"
ROUTER_NAME = "zenoh-router.json5"
SESSION_NAME = "zenoh-session.json5"
# The shared pair.
ROUTER_CONFIG = CONFIG_DIR / ROUTER_NAME
SESSION_CONFIG = CONFIG_DIR / SESSION_NAME
ROUTER_PORT = 7447


def config_dir(distro: str | None = None, root: Path = CONFIG_DIR) -> Path:
    """The directory holding the pair for `distro` (default: $ROS_DISTRO): `root/<distro>/` when a
    distro-specific pair is checked in there (both files present), else `root` — the shared pair."""
    d = (distro if distro is not None else os.environ.get("ROS_DISTRO", "")).strip().lower()
    if d:
        cand = root / d
        if (cand / ROUTER_NAME).is_file() and (cand / SESSION_NAME).is_file():
            return cand
    return root


def router_config(distro: str | None = None) -> Path:
    return config_dir(distro) / ROUTER_NAME


def session_config(distro: str | None = None) -> Path:
    return config_dir(distro) / SESSION_NAME


def apply_env_defaults(env: dict | None = None) -> dict:
    """Set the Zenoh config URIs (and the rmw defaults) in `env` (default os.environ) where unset.
    The pair is chosen by the ROS_DISTRO of `env` (`config_dir`)."""
    e = os.environ if env is None else env
    distro = e.get("ROS_DISTRO")
    e.setdefault("ZENOH_ROUTER_CONFIG_URI", str(router_config(distro)))
    e.setdefault("ZENOH_SESSION_CONFIG_URI", str(session_config(distro)))
    e.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    e.setdefault("ZENOH_ROUTER_CHECK_ATTEMPTS", "0")
    return e


def router_argv() -> list[str]:
    return ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]


def main(argv: list[str] | None = None) -> int:
    """`rig-router`: the one Zenoh router a host runs, with the loopback config."""
    apply_env_defaults()
    print(f"[rig-router] config {os.environ['ZENOH_ROUTER_CONFIG_URI']}", file=sys.stderr)
    try:
        return subprocess.call(router_argv() + list(argv or []))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
