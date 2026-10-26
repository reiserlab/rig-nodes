# channel/ — the published `rig-nodes` conda channel

The conda channel a rig installs the package set from: `rig_interfaces` (the typed ROS 2
messages: `FicTracFrame`, `ArenaFrame`, `ConnectorReply`) and `rig_nodes` (the Python package),
built by `.github/workflows/package-set.yml` on win-64, osx-arm64 and linux-64 and indexed with
`rattler-index`. Consumers read it over raw.githubusercontent.com pinned to a release tag
(`https://raw.githubusercontent.com/reiserlab/rig-nodes/vX.Y.Z/channel`), so a rig PC **without
a compiler** installs typed messages as a download (decision: `../docs/typed-distribution-decision.md`).

**The package payload is not in the tree between releases.** The `<subdir>/` directories are
committed ONCE, on the tagged release commit (README "Releasing a package set"), and removed again
in the next commit; git history serves the pinned URL. This README stays as the record of what was
built. Every package pins `ros2-distro-mutex` to the RoboStack set the rigs run (0.19) and Python 3.12 —
one set on every platform, never two (the 0.2.0 lesson below).

## Releases

| package set | tag | built from | pins | notes |
|---|---|---|---|---|
| 0.2.1 | `v0.2.1` (pending) | — | mutex 0.19, Python 3.12 | first set from this repository; rig_interfaces 0.2.1 = the 0.2.0 messages, rebuilt with the pins; rig_nodes 0.2.1 = the moved nodes |

## History (webDisplayTools, before the move)

- `rig_interfaces 0.2.0` (2026-10-26, webDisplayTools run 40401906061, commit `eb2bf01`, PR #270):
  win-64 `rig_interfaces-0.2.0-ha41a7fa_0.conda` (mutex `>=0.20.0,<0.21.0a0`, Python 3.14),
  osx-arm64 `h60d57d3_0` (mutex `>=0.16.0,<0.17.0a0`, Python 3.12), linux-64 `hb0f4dca_0`
  (mutex `>=0.20.0,<0.21.0a0`, Python 3.14). The pins were whatever each runner resolved that
  day, not the `0.19` the arena workspace locks — a rig running its default environment plus
  `typed-prebuilt` carried two RoboStack sets. Never tagged; superseded by 0.2.1.
- `arena_interfaces 0.1.0`, win-64 only (2026-10-16, built on ww18 with pixi 0.80.0 and
  pixi-build-ros 0.7.6; mutex `>=0.19,<0.20`, Python 3.12). The OLD package name: nothing that
  imports `rig_interfaces` may install it.
