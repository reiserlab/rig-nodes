# channel/ — the published `rig-nodes` conda channel

The conda channel a rig installs the package set from: `rig_interfaces` (the typed ROS 2
messages: `FicTracFrame`, `ArenaFrame`, `ConnectorReply`, `HostStats` + its four record types) and `rig_nodes` (the Python package),
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
| 0.4.0 | `v0.4.0` | [run 40800724499](https://github.com/reiserlab/rig-nodes/actions/runs/40800724499) from `144a0a8`, pixi 0.80.0, pixi-build-ros + pixi-build-python | mutex 0.19, Python 3.12 | rig_interfaces 0.4.0 = the 0.3.0 messages, unchanged, rebuilt under the set's version; rig_nodes 0.4.0 (noarch) = the ScanImage JSON connector removed (`scanimage_node`, `scanimage_cmd`, `sims/si_json_mock`, the `rig-scanimage-*` / `rig-si-mock` scripts; now private in scanimage-2026-bergamo `2p-rig-nodes/` — a contract change, CONTRACT §2/§4), `host_sampler.quantize` + `host_monitor.to_msg` quantization of the typed HostStats floats, the cheaper portable process sampling (`proc_rescan_s`, `--proc-rescan`); acceptance (clean-directory install + tests) passed on win-64 / osx-arm64 / linux-64 |
| 0.3.0 | `v0.3.0` | [run 40775301367](https://github.com/reiserlab/rig-nodes/actions/runs/40775301367) from `b5aebf7`, pixi 0.80.0, pixi-build-ros + pixi-build-python | mutex 0.19, Python 3.12 | rig_interfaces 0.3.0 = the 0.2.1 messages + `HostStats` with `DiskStats` / `NetStats` / `GpuStats` / `ProcStats` (the host monitor's `/rig/host` sample); rig_nodes 0.3.0 (noarch) = + `host_monitor` / `host_sampler`, `bias_health`, the `foxglove_json` derive hook, Zenoh configs without the deprecated routing keys; psutil run-dependency; acceptance (clean-directory install + tests) passed on win-64 / osx-arm64 / linux-64 |
| 0.2.1 | `v0.2.1` | [run 40427967719](https://github.com/reiserlab/rig-nodes/actions/runs/40427967719) from `451bd37`, pixi 0.80.0, pixi-build-ros + pixi-build-python | mutex 0.19, Python 3.12 | first set from this repository; rig_interfaces 0.2.1 = the 0.2.0 messages, rebuilt with the pins; rig_nodes 0.2.1 (noarch) = the moved nodes; acceptance (clean-directory install + tests) passed on win-64 / osx-arm64 / linux-64 |

### v0.4.0 files

| subdir | file | sha256 | pins |
|---|---|---|---|
| `linux-64` | `rig_interfaces-0.4.0-hb0f4dca_0.conda` | `cce3c8ee6176b15eb5c27651e16a902286ab22ebfde5117eabcaee023f8f63cd` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `noarch` | `rig_nodes-0.4.0-pyh4616a5c_0.conda` | `1b6a084aafdca0bdbec3508af9c98e57a7b152f03f676d2de24307131bf53e9d` | `python >=3.12`, `python *`, `ros2-distro-mutex 0.19.*` |
| `osx-arm64` | `rig_interfaces-0.4.0-h60d57d3_0.conda` | `2575c73130b6856a8041ed9bd501ffa889bc3602fc5eef398316f986862a383b` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `win-64` | `rig_interfaces-0.4.0-ha41a7fa_0.conda` | `31a04f0ad5dd07a6ed0ca69c8781065663b07302b8c738d08f0933e6a93c5cec` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |

### v0.3.0 files

| subdir | file | sha256 | pins |
|---|---|---|---|
| `linux-64` | `rig_interfaces-0.3.0-hb0f4dca_0.conda` | `47aceda59b9b620865ebc5e265651dee277d5b8e1c75086f5f4d693b1e29564d` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `noarch` | `rig_nodes-0.3.0-pyh4616a5c_0.conda` | `21e25cc26f888a92631c9699f17e0c7b03787e3ba91ae04de6a56694cb3fbfa5` | `python >=3.12`, `python *`, `ros2-distro-mutex 0.19.*` |
| `osx-arm64` | `rig_interfaces-0.3.0-h60d57d3_0.conda` | `330681b4e0fd635442a8e53a73f066fa3dce3492f145cd58d810503028f5ef65` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `win-64` | `rig_interfaces-0.3.0-ha41a7fa_0.conda` | `2af2122abbec43ada5a471c9b42f7d3f41d45b6078afb4f14cdde057b7369c41` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |

### v0.2.1 files

| subdir | file | sha256 | pins |
|---|---|---|---|
| `linux-64` | `rig_interfaces-0.2.1-hb0f4dca_0.conda` | `9eb49568b01263b22a193915e0102e9dc1a0597a5f4d3f6267f400c77700a3be` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `noarch` | `rig_nodes-0.2.1-pyh4616a5c_0.conda` | `e002109fbeb72c36489c850ddce0176df13e0e032b422b64ccf854465d5394a7` | `python >=3.12`, `python *`, `ros2-distro-mutex 0.19.*` |
| `osx-arm64` | `rig_interfaces-0.2.1-h60d57d3_0.conda` | `76339c539d4eaae37bb81d7efd84c5e6dfff438a89b722a6c57252b09ba9d32d` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |
| `win-64` | `rig_interfaces-0.2.1-ha41a7fa_0.conda` | `dfef21559ce477fd9d4ca7bbd4f9f24943b52405f82b79d075d2874202cb0d4f` | `ros2-distro-mutex`, `ros2-distro-mutex >=0.19.0,<0.20.0a0`, `python_abi 3.12.* *_cp312` |


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
