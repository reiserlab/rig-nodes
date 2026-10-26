"""zenoh.py — the loopback-only Zenoh configuration every rig graph runs with.

`rig_nodes/config/zenoh-router.json5` and `zenoh-session.json5` are the stock rmw_zenoh files with
ONLY the endpoint lines changed: the router listens on `tcp/127.0.0.1:7447` instead of
`tcp/[::]:7447` (fails on a host without IPv6) and sessions connect to 127.0.0.1 instead of
`localhost`. A rig PC's graph is therefore not reachable from the lab network (CONTRACT.md §1).

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
ROUTER_CONFIG = CONFIG_DIR / "zenoh-router.json5"
SESSION_CONFIG = CONFIG_DIR / "zenoh-session.json5"
ROUTER_PORT = 7447


def apply_env_defaults(env: dict | None = None) -> dict:
    """Set the Zenoh config URIs (and the rmw defaults) in `env` (default os.environ) where unset."""
    e = os.environ if env is None else env
    e.setdefault("ZENOH_ROUTER_CONFIG_URI", str(ROUTER_CONFIG))
    e.setdefault("ZENOH_SESSION_CONFIG_URI", str(SESSION_CONFIG))
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
