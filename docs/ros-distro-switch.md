# The ROS 2 distribution is one switch

**Status 2026-11-05.** The rigs run ROS 2 **Jazzy** from RoboStack (mutex 0.19, Python 3.12); that
is the default of every workspace and the only set the channel publishes. **Lyrical** (ROS 2
Lyrical Luth, RoboStack mutex 0.22, Python 3.14) is an opt-in environment and an allowed-to-fail
CI job: the same source tree built and tested on the next distribution, so a migration is a
decision with evidence rather than a project. Nothing a rig runs changed. This page is the record
of what the switch is, what the rehearsal found and what still blocks a real move; the
measurements that led here are in webDisplayTools' `docs/development/rig-software-repos-and-runners.md`
(§7: one RoboStack set per platform, lock files, package versions in every run's record).

## 1. What differs between two RoboStack sets

| | Jazzy (the rigs) | Lyrical (rehearsal) |
|---|---|---|
| channel | `robostack-jazzy` | `robostack-lyrical` |
| `ros2-distro-mutex` | `0.19.*` (build `jazzy_26`; the channel's current rebuild is 0.20 = Jazzy on Python 3.14) | `0.22.*` (build `lyrical_30`) |
| Python | 3.12 | 3.14 (the only Python the Lyrical set is built for) |
| rmw_zenoh | 0.2.11 | 0.10.6 (≥ 0.10.6: 0.10.5 can hang in `rmw_wait`) |
| rclpy / rosbag2 | 7.1.12 / 0.26.11 | 10.0.11 / 0.33.4 |
| package names | `ros-jazzy-*` | `ros-lyrical-*` |

Everything else the package set touches — rclpy init/spin/Node/QoS/serialize, rosbag2_py's
SequentialWriter and Reindexer, four `.msg` types, the MCAP record — is the same API on both.
RoboStack now publishes each real package once under a distro-neutral name (`ros2-rclpy ==7.1.12`
on Jazzy, `ros2-rclpy ==10.0.11` on Lyrical, each pinned to its set's mutex) and the
`ros-<distro>-*` names are 2 KiB shims onto them; the switch below relies on that layout.

## 2. Where the switch lives

**The workspace (`pixi.toml`).** One `[feature.<distro>]` per set carries the channel (appended
behind conda-forge with `priority = -1`), the mutex pin, the Python pin and that distro's spelling
of the ROS packages the workspace itself installs (ros-base, rmw_zenoh, rosbag2, the rosidl
toolchain). An environment picks one feature: `default`, `bench`, `viz` → `jazzy`; `lyrical` →
`lyrical`. Adding a set is one more feature (+ one alternative in the variant below).

**The source packages name no distro.**
- `src/rig_interfaces/pixi.toml` has no `distro = …` and no `ros-<distro>-*` lines: pixi-build-ros
  takes the distro from the first `robostack-<distro>` channel of the environment that builds the
  package and maps package.xml's `ament_cmake` / `rosidl_default_generators` / `rosidl_default_runtime`
  to that distro's packages itself. The ROS host packages' run_exports pin the result to the set
  (`ros2-distro-mutex >=0.19.0,<0.20.0a0` on Jazzy, `>=0.22.0,<0.23.0a0` on Lyrical) and the set
  fixes the Python ABI (`python_abi 3.12.* *_cp312` vs `*_cp314`) — the lock shows both from one manifest.
- `src/rig_nodes/pixi.toml` (noarch Python) depends on the distro-neutral `ros2-rclpy`,
  `ros2-std-msgs`, `ros2-rosbag2-py`, `ros2-rosbag2-storage-mcap`, `ros2-rosidl-runtime-py`.
  pixi-build-python carries no run_exports, so the package names the mutex **bare** and takes the set
  from the consumer's own mutex pin through those dependencies: one noarch build serves every set
  (before 2026-11-05 it pinned `0.19.*`; CI's inspect step now checks the `ros2-*` dependencies
  instead of a pin for this package, and the pin for `rig_interfaces`).
- `[workspace.build-variants] ros2-distro-mutex = ["0.19.*|0.22.*"]` pins the packages' *host*
  solve to the set the environment uses. It is ONE value — a disjunction every channel narrows to
  its own set — because pixi solves every listed variant value in every environment, and `0.22.*`
  has no candidate in the Jazzy channel (a two-element list fails the Jazzy solve). Without it the
  Jazzy host solve takes mutex 0.20, the Python 3.14 rebuild of Jazzy, not the rigs' set.

**The code reads ROS_DISTRO** (set by the `ros-environment` package in both distros) in two places:
`rig_nodes.zenoh.config_dir` (§3) and `rig_nodes.runtime.runtime_info` (§4). The two remaining
mentions of "Jazzy" in `_common.py` and `bias_cmd.py` are comments about rclpy shutdown races and
the Windows clock, not logic.

**The consumer (webDisplayTools `rig/pixi.toml`)** mirrors the features: `default` / `typed` /
`viz` / `dev` stay Jazzy and resolve to the same packages; its `lyrical` environment builds the two
packages from this repository's git source, because the published channel is Jazzy-only (the
channel packages pin mutex 0.19). It carries the same `[workspace.build-variants]` line: a consumer
that builds the source packages must, or the Jazzy host solve takes mutex 0.20 once `main` has no
manifest pin. Two consumer lessons from ww18 (2026-11-05): a feature's `activation.env` takes
precedence over a workspace-level `[target.win-64.activation.env]`, so anything a distro feature sets
for POSIX (the Zenoh config URIs under `lib/python3.x/`) needs its Windows spelling in
`[feature.<distro>.target.win-64.activation.env]` — otherwise every environment on Windows gets the
literal `$CONDA_PREFIX/...` and no router starts; and a tool from another set cannot join the graph
(a Jazzy `foxglove_bridge` on a Lyrical router), so per-distro helpers get a per-distro environment
(`lyrical-viz`). The private 2P rig repository (`scanimage-2026-bergamo/2p-rig-nodes`) has its own
manifest and has not mirrored the switch.

## 3. The Zenoh configs: one pair, selected by distro when it has to be

`rig_nodes/config/zenoh-{router,session}.json5` are the stock rmw_zenoh files with three edits
(loopback listen/connect, the deprecated `routing.peer` / `peers_failover_brokering` keys removed).
Deriving a Lyrical pair the same way was the plan; the derivation found that the stock
`DEFAULT_RMW_ZENOH_*_CONFIG.json5` at rmw_zenoh **0.2.11 (Jazzy) and 0.10.6 (Lyrical) are
byte-identical**, so a second pair would be a copy. One pair ships; `rig_nodes.zenoh.config_dir()`
looks for `config/<ROS_DISTRO>/` first and falls back to the shared pair, and `apply_env_defaults`
uses it — so the day a divergent pair is needed, it is added under `config/<distro>/` with no code
change. That day is foreseeable: upstream's `lyrical` branch is at rmw_zenoh **0.10.8**, whose stock
files change the schema (per-mode `scouting.*.listen`, `autoconnect` lists including `client`, a
`gateway` block, the shared-memory `messages` list). Our pair is a subset of the stock keys, so it
keeps parsing on 0.10.8; when RoboStack rebuilds Lyrical on it, re-derive (download the stock files
at that tag, apply the three edits, drop them into `config/lyrical/`).

## 4. Every bag says what it ran on

`rig_nodes.runtime.runtime_info()` — `ros_distro`, `ros_version`, `rmw_implementation`, `rmw` (the
rmw package's version), `rclpy`, `rosbag2_py`, `rosbag2_storage_mcap`, `ros2_distro_mutex`
(`<version> <build>`, e.g. `0.19.0 jazzy_26`: the RoboStack set), `robostack_channel`, `python`,
`platform`, `rig_nodes`, `rig_interfaces` — is handed to rosbag2 by `Recorder.open()` as the
StorageOptions `custom_data`. rosbag2 writes it, with its own `ros_distro`, into the MCAP metadata
record named `rosbag2` (`serialized_metadata`, a YAML `BagMetadata`) when the bag closes, and into
`metadata.yaml`; `mcap_index.reindex` copies metadata records, so the reindexed bag keeps it, and
`Recorder.close()` returns the dict as `runtime`. `pixi run runtime` prints it. A bag the writer
never closed (crash) has no metadata record; the arena gateway's `/arena/session` open record is
the crash-safe copy of the package versions, and can carry this dict too (not done here).

Reading it back without ROS: `mcap.reader.make_reader(f).iter_metadata()` → the record named
`rosbag2` → `metadata["serialized_metadata"]` is YAML text with `custom_data:` and `ros_distro:`.

## 5. CI

`package-set.yml` keeps the release path (build → channel → clean-directory accept) on Jazzy, spelled
once per job matrix (`distro`, `mutex`, `python`). The new `lyrical` job (`continue-on-error`) runs
`pixi install -e lyrical` on win-64 / osx-arm64 / linux-64 — pixi-build compiles `rig_interfaces` and
packages `rig_nodes` for the Lyrical set in place — then the pure, host-sampler, FicTrac-simulator
and device-node conformance tests on it. It produces no channel payload. `PIXI_VERSION` moved to
0.83.0 (the rigs' floor since 2026-10-28 and the pixi that writes `pixi.lock`, which this job installs).

## 6. RoboStack Lyrical availability (checked 2026-11-05 against the channel's repodata)

| package | linux-64 | osx-arm64 | win-64 |
|---|---|---|---|
| `ros-lyrical-ros-base` 0.13.0 | ✓ | ✓ | ✓ |
| `ros-lyrical-rmw-zenoh-cpp` 0.10.6 | ✓ | ✓ | ✓ |
| `ros-lyrical-rosbag2-py` / `-rosbag2-storage-mcap` 0.33.4 | ✓ | ✓ | ✓ |
| `ros-lyrical-rclpy` 10.0.11, `-std-msgs` 5.9.3, `-rosidl-runtime-py` 0.15.2 | ✓ | ✓ | ✓ |
| `ros-lyrical-ament-cmake` 2.8.8, `-rosidl-default-generators` / `-runtime` 1.8.1, `-ros-environment` 4.5.1 | ✓ | ✓ | ✓ |
| `ros-lyrical-foxglove-bridge` 3.6.0, `-rqt-graph` 1.8.5, `-rqt-topic` 2.1.2, `-rqt-console` 2.4.4, `-demo-nodes-py` 0.37.9 | ✓ | ✓ | ✓ |
| `ros2-distro-mutex` | 0.21.0 `lyrical_28`, **0.22.0 `lyrical_30`** | same | same |

Nothing the two workspaces need is missing on any platform. All of it is Python 3.14
(`python_abi 3.14.* *_cp314`); the set is `_30` (2,015–2,612 packages per platform), and the
newest packages in it were uploaded on 2026-11-05 — the channel is still being filled in.

## 7. What blocks a real migration (and what does not)

1. **Python 3.14 for everything in the rig environment.** The ROS side is conda and resolves
   (this repository's `lyrical` lock on all three platforms). The arena workspace adds `pyzmq`,
   `websockets`, `mcap`, `psutil` — conda-forge builds for 3.14 exist, and its lyrical environment
   resolves too — but every Python the lab wrote has only ever run on 3.12: the Studio gateway
   (`bridge_node.py`, `rig.py`, `mcap_convert.py`), the QC driver, the dashboard scripts. Nothing
   here is known to break on 3.14; nothing has been run on it except this package set's tests.
2. **Channel maturity.** RoboStack's Lyrical channel is weeks old and still being rebuilt (`_28` →
   `_30` within the set we pinned; the newest packages landed the day this was checked). A `*` pin
   there moves under you; a rig would pin the mutex build (`0.22.* lyrical_30`) and re-lock
   deliberately, as it does on Jazzy.
3. **rmw_zenoh 0.10.6 is behind upstream's Lyrical branch (0.10.8)** with its new config schema
   (§3) and shared-memory defaults (`messages: ["put", "query", "reply"]`, the in-transit
   watchdog race fix). The first RoboStack rebuild onto 0.10.8 is when the Zenoh pair must be
   re-derived; our loopback graph does not use shared memory, so the SHM changes do not matter yet.
4. **One distro per host, not per process.** Zenoh's wire protocol and rmw_zenoh's key-expression
   layout differ between 0.2.x and 0.10.x; a Jazzy node and a Lyrical node on one router do not
   talk. The Studio's WebSocket and the MCAP record are unaffected (the browser never speaks ROS;
   the bag is self-describing). A rig moves whole, together with its private nodes (the 2P rig's
   ScanImage/SiPM nodes live in `scanimage-2026-bergamo`, Jazzy-spelled).
5. **Nothing has run on hardware under Lyrical.** The rehearsal drives every device node through
   its simulator (FicTrac, Kinefly, BIAS) and the recorder on real CDR; the arena harness
   (webDisplayTools `rig/tests/test_ros2_poc.py`), FicTrac on a camera, BIAS on a camera, the G6
   controller and a Studio run have not been tried. The `viz` tools (rqt, foxglove_bridge) exist
   for Lyrical but were not exercised.
6. **The channel publishes Jazzy only.** A dual-distro channel is possible — `rig_nodes` is one
   noarch package for both sets, and the two `rig_interfaces` builds differ by build string and
   mutex pin, so they coexist in one subdir — but the release workflow would gain a distro matrix
   and the acceptance job a second manifest. Not needed until a rig is scheduled to move.
7. **API drift hides in Python bindings, not in the ROS API.** The one Lyrical failure the rehearsal
   found was in `rosbag2_py`: the 3-argument `SequentialWriter.write` overload gained an optional
   `sequence_number`, so the recorder's positional 4-argument call bound the producer stamp to it and
   every record's publish_time silently equaled its log_time (the arena gateway writes the
   closed-loop sample time there; CONTRACT §6). Fixed in `recorder.py` by passing the stamps by
   keyword where the binding names them (Jazzy's does not). Expect more of this kind — bindings,
   `metadata.yaml` versions, CLI output — than rclpy-level breakage.
8. **Staying on Jazzy is not standing still.** RoboStack's current Jazzy rebuild is mutex **0.20 on
   Python 3.14**; the rigs' 0.19 set is a frozen snapshot that no longer receives rebuilds. The next
   re-pin on Jazzy would therefore bring Python 3.14 too — item 1 is on the path either way, which
   is why the switch was built before the migration rather than as part of it.

## 8a. The first rig PC (ww18, Windows 11, 2026-11-05)

Run from webDisplayTools #294's `lyrical` environment on the arena rig `rig5-mr` (pixi 0.83.0, VS
2022 Build Tools 19.44); record `rigs/ww18/lyrical-bench-2026-11-05.md` in `reiserlab/arena-experiments`.

| step | result |
|---|---|
| `pixi install --locked -e lyrical` | 43 s; `rig_interfaces` compiled for win-64 with vs2022 (16 harmless D9025 warnings) |
| runtime | `lyrical`, rmw 0.10.6, rclpy 10.0.11, rosbag2 0.33.4, mutex `0.22.0 lyrical_30`, Python 3.14.10 |
| check-typed · test-gateway · test-stimulus · test-viz | ✓ 8 types · 5/5 · 51/51 · 12/12 |
| arena harness (`pixi run -e lyrical test`) | **63/63** in 80 s (Jazzy baseline on the same PC: 60/60 in 56 s) |
| real run: FicTrac 2.1.2 on the FLIR camera at ~96 Hz, typed adapter, Production Studio, G6 on COM4, closed-loop protocol | gateway bag 4,182 records → 6,637 converted lines identical to the live file; `runlog-check` 4/4; controller 0x70 = host 0x70 = 1,543; shutdown parsed 29,472 / skipped 0 / bad 0, p99 0.77 ms; Console clean |
| soak smoke, simulator and real FicTrac (`soak -- --hours 0.1 --every 1`) | 6/6 ok each, `clean exit 0`; real FicTrac 35,044 frames, 0 bad, p50 0.41 / p99 0.61 ms; the Windows CTRL_BREAK stop path works |
| overnight `soak -- --hours 8 --every 10 --fictrac` | started 13:04 local; result in the record when it ends |

Findings: the Windows activation-env regression of #294 (fixed before merge, §2); `rig.py --foxglove`
hard-coded the Jazzy `viz` environment, so a `lyrical-viz` environment was added; Lyrical's
`foxglove_bridge` logs one benign `Failed to retrieve parameters from node '/rig_host_monitor'` on
connect. Not yet done on a rig: BIAS on the real Atlas10 under Lyrical (the soaks use the simulator),
a second night as the Jazzy baseline.

## 8. Rehearsal results (2026-11-05, Michael's Mac, osx-arm64, pixi 0.83.0)

| step | Jazzy (`default`) | Lyrical (`lyrical`) |
|---|---|---|
| lock (`pixi install` re-solves all 4 environments × 3 platforms) | ✓ | ✓ — `rig_interfaces` records `python_abi 3.14.* *_cp314` + `ros2-distro-mutex >=0.22.0,<0.23.0a0` on every platform; Jazzy's keep `cp312` + `>=0.19.0,<0.20.0a0` |
| install (builds both packages in place) | 28 s warm | 32 s warm (packages already cached by the solve) |
| `pixi run runtime` | `jazzy`, rmw 0.2.11, rclpy 7.1.12, rosbag2 0.26.11, mutex `0.19.0 jazzy_26`, Python 3.12.16 | `lyrical`, rmw 0.10.6, rclpy 10.0.11, rosbag2 0.33.4, mutex `0.22.0 lyrical_30`, Python 3.14.10 |
| `build_interfaces --check` (typed package imports + serializes) | ✓ 8 types | ✓ 8 types |
| `test-pure` | 50/50 | 50/50 |
| `test-host` | 52/52 | 52/52 |
| `test-sim` | 39/39 | 39/39 |
| `test` (sims → nodes → topics → recorder, own router) | 42/42 | 41/42 before the `rosbag2_py` keyword fix (§7 item 7), 42/42 after |
| `pixi build --path src/rig_nodes/pixi.toml` (the CI release path) | noarch `pyh44cc203_0`; run deps `ros2-rclpy`, `ros2-std-msgs`, `ros2-rosbag2-py`, `ros2-rosbag2-storage-mcap`, `ros2-rosidl-runtime-py`, bare `ros2-distro-mutex` | not built (the release path is Jazzy-only) |

The bags written by the recorder test now carry the `rosbag2` metadata record with
`ros_distro: jazzy` / `ros_distro: lyrical` and the `custom_data` dict of §4; the crashsafe →
reindexed rewrite keeps it (checked by the test on both distros). CI results for win-64 and
linux-64 are on the pull request that introduced this page.
