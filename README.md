# rig-nodes

Shared ROS 2 infrastructure for every Reiser Lab rig: the typed message contracts
(`rig_interfaces`), device adapters and their simulators, MCAP recording primitives, process
supervision primitives, the Zenoh configuration and the benchmarks behind the design choices.
Created 2026-10-26 by moving the shared part of webDisplayTools' `rig/` workspace here with its
history (decision record: webDisplayTools
[`docs/development/rig-software-repos-and-runners.md`](https://github.com/reiserlab/webDisplayTools/blob/main/docs/development/rig-software-repos-and-runners.md),
ticket LAB-338). What a graph may rely on is written down in [CONTRACT.md](CONTRACT.md).

**Dependencies point downward only.** This repository imports nothing of ours. The arena rig
(webDisplayTools: the Studio's gateway `bridge_node`, the arena composition `rig.py`, the arena
visualization) and the Python arena library + headless runner (LED-Display_pyDisplayTools) depend
on the packages built here. A package lives here only when it carries no arena semantics in its
code (message contracts may) and a second consumer confirms the abstraction; anything doubtful
stays with its application until a consumer asks.

**Setting up a machine** (install recipe, host table, per-host quirks for the Mac, ww18, the Bergamo PC and ww14): [`SETUP.md`](SETUP.md). Every install or QC pass updates its host's row there.

## Layout

```
pixi.toml                 the SOURCE workspace (RoboStack Jazzy + rmw_zenoh + the two packages built from src/)
src/rig_interfaces/       ament_cmake message package: FicTracFrame, ArenaFrame, ConnectorReply  (pixi-build-ros)
src/rig_nodes/            Python package (pixi-build-python, hatchling); console scripts rig-*
  rig_nodes/_common.py        clocks (precise on Windows), QoS, the adapter envelope, spin, typed-interfaces probe
  rig_nodes/supervise.py      process groups, portable signals, port preflight, Child supervision  (ex rig.py)
  rig_nodes/fictrac.py        FicTrac's line grammar + column names (pure)
  rig_nodes/fictrac_udp_adapter.py, kinefly_udp_adapter.py          stream adapters → /<src>/raw (+ /fictrac/frame typed)
  rig_nodes/bias_node.py, bias_cmd.py, bias_health.py               BIAS camera: /bias/cmd|reply|status|frames
  rig_nodes/scanimage_node.py, scanimage_cmd.py                     ScanImage JSON server: /scanimage/cmd|reply|status
  rig_nodes/recorder.py        one MCAP bag per run (rosbag2_py), a writer library — the lifecycle belongs to the application
  rig_nodes/mcap_index.py      reindex a finished bag; recover a torn one
  rig_nodes/cdr.py             decode CDR from the ros2msg schema text in a bag (no ROS process)
  rig_nodes/foxglove_json.py   String-topic bag → Foxglove-plottable JSON MCAP (generic; applications add `enrich` / `derive` hooks)
  rig_nodes/graph_dump.py      the live graph as Mermaid / DOT / a rate table
  rig_nodes/build_interfaces.py   colcon build of src/rig_interfaces (the .msg author's route) + `--check`
  rig_nodes/sims/              fictrac_sim, kinefly_sim, bias_sim, si_json_mock — the device programs, stood in
  rig_nodes/zenoh.py + config/ the loopback-only Zenoh configs, shipped in the package; `rig-router` starts rmw_zenohd with them
channel/                  the published conda channel: README always; payloads only on a tagged release commit
tests/                    test_pure.py (no graph), test_device_nodes.py (sims → nodes → topics → loss accounting)
bench/                    shm_bench.py, shm_bench_qpc.py, mcap_bench.py, sizing/ (measurements behind docs/)
docs/                     shm-bench.md, typed-distribution-decision.md, typed-windows.md
foxglove/                 generic Foxglove layouts (none yet; the arena layouts stay in webDisplayTools)
.github/workflows/package-set.yml   build both packages on win-64 / osx-arm64 / linux-64, index the channel, install-test it
```

## Developing here

```bash
pixi install          # solves RoboStack Jazzy and BUILDS src/rig_interfaces + src/rig_nodes in place (pixi-build)
pixi run test-pure    # the pure modules, no router needed
pixi run test-sim     # the FicTrac simulator (record packing, replay, model fly)
pixi run test         # the device-node conformance run (starts its own Zenoh router on 127.0.0.1:7447)
pixi run router       # a router for manual runs; then e.g. `pixi run fictrac-sim` + `pixi run fictrac-adapter`
pixi run graph        # the live graph as Mermaid
```

Building `rig_interfaces` from source needs a C/C++ compiler (rosidl generates C typesupport and
a Python extension): clang on macOS and GCC on Linux come from conda-forge's `cxx-compiler`;
Windows needs the MSVC v143 toolset and a short build path (`pixi config set --local
detached-environments C:\px`), see `docs/typed-windows.md`. Rigs never build it: they install the
channel packages.

Only one ROS graph per host at a time: Zenoh discovery crosses workspaces, so a device-node test
here and the arena harness in a webDisplayTools checkout must not run at the same time.

## Using the packages from a rig workspace

A consumer's `pixi.toml` adds the channel and pins the package set:

```toml
[workspace]
channels = ["conda-forge", "robostack-jazzy", "https://raw.githubusercontent.com/reiserlab/rig-nodes/v0.2.1/channel"]

[dependencies]
ros2-distro-mutex = "0.19.*"          # the RoboStack set the packages were built against (macOS ≥ 14 for osx-arm64)
rig_nodes = "==0.2.1"
rig_interfaces = "==0.2.1"            # optional: without it the nodes publish String JSON (`--typed auto`)
```

and starts nodes with their console scripts (`rig-fictrac-adapter`, `rig-bias-node`, …) or
`python -m rig_nodes.<module>`; launchers import the supervision primitives from
`rig_nodes.supervise`. **Developer path without a release:** a sibling checkout of this repository
and a feature that swaps the two pins for path dependencies (webDisplayTools' `rig/pixi.toml`
has one: `pixi install -e dev`). Package versions belong in every run's metadata (consumer side).

## Releasing a package set

One version for both packages, one tag, one channel directory (record §7.6, §7.9).

1. Bump the version in **four** places: `src/rig_interfaces/package.xml`,
   `src/rig_interfaces/pixi.toml`, `src/rig_nodes/pixi.toml`, `src/rig_nodes/pyproject.toml`
   (and `rig_nodes/__init__.py`, `pixi.toml`'s workspace version, the top of CONTRACT.md).
2. Push; the `package-set` workflow builds both packages on the three platforms, indexes the
   channel, checks every package's `ros2-distro-mutex` and Python pins against the expected set,
   and **installs the channel in a clean directory on each platform and runs the tests against
   the installed packages** (the release gate).
3. Download the `rig-nodes-channel` artifact of that run into `channel/` (`gh run download <id>
   -n rig-nodes-channel -D channel`), update `channel/README.md` (files, sha256s, pins), commit
   as `release: package set vX.Y.Z`, tag `vX.Y.Z` on that commit, push with the tag. The payload
   is committed **once**, on that commit, and removed again in the next commit; git history serves
   the pinned `raw.githubusercontent.com/…/vX.Y.Z/channel` URL. On the tag push the workflow also
   uploads to a prefix.dev channel when the repository variable `RIG_NODES_CHANNEL` and the
   secret `PREFIX_API_KEY` exist (the lab does not own one yet).
4. Consumers re-pin and re-lock (webDisplayTools: `rig/pixi.toml` + `pixi lock`, through its
   own release tiers), and their CI's cross-repository smoke runs against the new set.

Keep an installable copy of each released channel on the group share, so a channel outage never
blocks a rig rebuild.

## Testing split (record §7.8)

Device-node conformance lives here (`tests/test_device_nodes.py`: simulator → node → topics →
loss accounting, plus the recorder on real CDR). Runner conformance belongs to pyDisplayTools and
browser-contract conformance to webDisplayTools, whose harness (`rig/tests/test_ros2_poc.py`)
runs the whole arena graph against the **installed** packages and is the cross-repository smoke.

## History

Moved from webDisplayTools (`rig/`, formerly `ros2-poc/`; `fictrac-bridge/fictrac_sim.py`) on
2026-10-26 with `git filter-repo`; the move list is the record's §3.3. The `arena_interfaces` →
`rig_interfaces` rename (2026-10-26, webDisplayTools #270) preceded the move so that no released
bag carries a typed topic under the old name. The measurement records in `docs/` keep their
original paths (`rig/…`, `ros2-poc/…`) as history.
