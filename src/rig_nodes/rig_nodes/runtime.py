"""runtime.py — what this process runs on: the ROS distribution, the middleware, the RoboStack
set, Python, the package set. One flat dict of strings.

`rig_nodes.recorder` writes it into every bag as rosbag2's `custom_data` (the MCAP metadata record
named `rosbag2`, beside the `ros_distro` rosbag2 records on its own), so a bag says what produced
it; the arena gateway can add the same dict to its session record. Nothing here needs a ROS
process: the values come from the environment (ROS_DISTRO, RMW_IMPLEMENTATION, as the
ros-environment package and the workspace activation set them), the ament index (package.xml
versions) and the conda prefix (`conda-meta/`, for the `ros2-distro-mutex` build that names the
RoboStack set). A lookup that fails leaves its key out rather than guessing.

    python -m rig_nodes.runtime     # prints the dict (`pixi run runtime`)
"""

from __future__ import annotations

import json
import os
import platform
import re
import sys
from pathlib import Path

# The ROS packages whose versions a bag should name: the client library, the recorder's writer and
# storage plugin, and the rmw (looked up under its RMW_IMPLEMENTATION name, so another middleware
# is recorded just the same).
_ROS_PACKAGES = ("rclpy", "rosbag2_py", "rosbag2_storage_mcap")


def _ament_version(package: str) -> str | None:
    """`<version>` of an installed ROS package's package.xml, via the ament index; None when absent."""
    try:
        from ament_index_python.packages import get_package_share_directory

        xml = Path(get_package_share_directory(package)) / "package.xml"
        m = re.search(r"<version>([^<]+)</version>", xml.read_text(encoding="utf-8"))
        return m.group(1) if m else None
    except Exception:
        return None


def _conda_meta(name: str, prefix: str | os.PathLike | None = None) -> dict | None:
    """The conda-meta record of an installed conda package (`version`, `build`, `channel`), or None.
    `prefix` defaults to this interpreter's prefix, which under pixi is the environment."""
    root = Path(prefix or os.environ.get("CONDA_PREFIX") or sys.prefix) / "conda-meta"
    try:
        hits = sorted(root.glob(f"{name}-*.json"))
    except OSError:
        return None
    for p in hits:
        try:
            rec = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if rec.get("name") == name:
            return rec
    return None


def runtime_info(prefix: str | os.PathLike | None = None) -> dict[str, str]:
    """The flat string dict described in the module docstring. Keys (each present only when known):
    ros_distro, ros_version, rmw_implementation, rmw (the rmw package's version), rclpy, rosbag2_py,
    rosbag2_storage_mcap, ros2_distro_mutex (`<version> <build>`, e.g. `0.19.0 jazzy_26`),
    robostack_channel, python, platform, rig_nodes, rig_interfaces."""
    out: dict[str, str] = {}
    for key, env in (("ros_distro", "ROS_DISTRO"), ("ros_version", "ROS_VERSION"), ("rmw_implementation", "RMW_IMPLEMENTATION")):
        v = os.environ.get(env)
        if v:
            out[key] = v
    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw:
        v = _ament_version(rmw)
        if v:
            out["rmw"] = v
    for pkg in _ROS_PACKAGES:
        v = _ament_version(pkg)
        if v:
            out[pkg] = v
    mutex = _conda_meta("ros2-distro-mutex", prefix)
    if mutex:
        out["ros2_distro_mutex"] = f"{mutex.get('version')} {mutex.get('build')}".strip()
        chan = mutex.get("channel")
        if chan:
            out["robostack_channel"] = str(chan).rstrip("/").split("/")[-1]
    out["python"] = platform.python_version()
    out["platform"] = platform.platform()
    try:
        from rig_nodes import __version__

        out["rig_nodes"] = __version__
    except Exception:
        pass
    v = _ament_version("rig_interfaces")
    if v:
        out["rig_interfaces"] = v
    return out


def main(argv: list[str] | None = None) -> int:
    print(json.dumps(runtime_info(), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
