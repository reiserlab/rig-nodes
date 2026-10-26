# rig/channel — the CI-built `rig_interfaces` conda channel

A conda channel with one package, `rig_interfaces` (the typed ROS 2 messages of the rig graph:
`FicTracFrame`, `ArenaFrame`, `ConnectorReply`), read by the opt-in `typed-prebuilt` environment in
`../pixi.toml` over raw.githubusercontent.com, pinned to the commit that holds the payload, so a
rig PC **without a compiler** installs typed messages as a download. Built by
`.github/workflows/rig-interfaces.yml` (decision: `../docs/typed-distribution-decision.md`); the
recipe for the next rebuild is in that note under "Releasing a new build".

**The package payload is not in the tree.** Like the 2026-10-18 removal of the first (win-64)
payload, the `<subdir>/` directories are committed ONCE (the "payload commit" the `typed-prebuilt`
URL in `../pixi.toml` pins) and removed again in the next commit; git history serves the pinned
URL. This README stays as the record of what was built.

## rig_interfaces 0.2.0 (2026-10-26)

Built by the `rig-interfaces` workflow run
[40401906061](https://github.com/reiserlab/webDisplayTools/actions/runs/40401906061) from commit
`eb2bf01` (the rename `ros2-poc/` → `rig/`, `arena_interfaces` → `rig_interfaces`; PR
[#270](https://github.com/reiserlab/webDisplayTools/pull/270)) with pixi 0.80.0 and the RoboStack
`pixi-build-ros` backend, indexed with `rattler-index`; three platforms plus an empty `noarch/`:

| subdir | file | sha256 | `ros2-distro-mutex` pin | Python |
|---|---|---|---|---|
| `win-64` | `rig_interfaces-0.2.0-ha41a7fa_0.conda` | `5104b3d9b166fae11d9b0f27baf70cb71afcea6514f0e56943387e7d75a6ed22` | `>=0.20.0,<0.21.0a0` | 3.14 |
| `osx-arm64` | `rig_interfaces-0.2.0-h60d57d3_0.conda` | `c5321504efbf9c53bc2460b3d716ec393b88b9b4202d19e91eca91989eb5c57d` | `>=0.16.0,<0.17.0a0` | 3.12 |
| `linux-64` | `rig_interfaces-0.2.0-hb0f4dca_0.conda` | `0d7489d26092ec4577b4bfff037381f4af4afa6fce7048eee8c265686c35d2b8` | `>=0.20.0,<0.21.0a0` | 3.14 |

The mutex pins are whatever RoboStack Jazzy the build backend resolved on each runner that day
(the `prefix.dev/robostack-jazzy` mirror), NOT the `ros2-distro-mutex 0.19` the `poc` solve group
locks. `typed-prebuilt` is its own solve group, so it resolves its own RoboStack set per platform
around these pins. The tag `rig-interfaces-v0.2.0` is pending.

## History

- `arena_interfaces 0.1.0`, win-64 only (`arena_interfaces-0.1.0-ha41a7fa_0.conda`, sha256
  `5501edc89c10f6ee7acca21273f26f716edf2b7686e7109fed54335a3d39c883`): built 2026-10-16 on ww18
  from commit 4632674 with pixi 0.80.0 and pixi-build-ros 0.7.6.20261014.1408.4ef64dd
  (`pixi publish --path interfaces\arena_interfaces\pixi.toml --target-channel C:\chan`), depends on
  `ros2-distro-mutex >=0.19,<0.20` and Python 3.12. Payload at commit `30512581`
  (`ros2-poc/channel`), removed from the tree 2026-10-18. The OLD package name: nothing that
  imports `rig_interfaces` may install it.
