# Setting up a rig PC (and what each machine taught us)

The living setup note for every machine that runs the rig-nodes package set: arena rigs, the
2P rig, the robot station, a developer's laptop. It has two parts: the **install recipe**, which
is the same everywhere, and the **host table + per-host notes**, which is where the quirks go.
Anything that holds for a whole platform goes into the recipe; anything that holds for one
machine goes under that machine. Application-specific setup (Arena Studio, the course repo,
Web Serial) lives with the application: webDisplayTools `rig/SETUP.md`.

**Two rules.** (1) Every install or QC pass on a machine updates that machine's row (date,
version, result) and adds any new quirk under its heading, with the error text verbatim. The
dated result files stay where they are as records; this page links to them. (2) A quirk that
shows up on a second machine stops being prose: it becomes a check in the code (the port
preflight in `rig_nodes.supervise` is the model) or a fix in the package, and the note here
shrinks to one line that names the check.

## Install recipe

Prerequisite: [pixi](https://pixi.sh) (the lab's only developer tool; no conda, mamba, npm or
system ROS). RoboStack Jazzy and `rmw_zenoh` come from conda-forge/robostack through pixi.

1. **Pin the package set** in the workspace `pixi.toml` (the arena workspace is
   `webDisplayTools/rig/pixi.toml`; copy its shape for a new rig):

   ```toml
   [workspace]
   channels = ["conda-forge", "robostack-jazzy",
               "https://raw.githubusercontent.com/reiserlab/rig-nodes/v0.2.1/channel"]

   [system-requirements]
   macos = "14.0"                       # osx-arm64 otherwise resolves the older 0.16 RoboStack set

   [dependencies]
   ros2-distro-mutex = "0.19.*"         # the RoboStack set the packages were built against
   python = "3.12.*"
   rig_nodes = "==0.2.1"
   rig_interfaces = "==0.2.1"           # optional: without it nodes publish String JSON
   ```

2. `pixi install --locked` (first install: about 1 GB, 25–45 s on a warm cache, several minutes
   cold). pixi resolves **every** environment's source dependencies before it installs any of
   them, so a `path = …` dependency in any feature (a developer environment pointing at a sibling
   checkout) makes the plain install fail on a machine without that path; use `git = …` source
   dependencies for developer environments instead (webDisplayTools `rig/pixi.toml` `dev`,
   fixed 2026-10-27). **Install with `--locked` on a rig, always.** A plain `pixi install` on a
   newer pixi than the one that wrote the lock rewrites `pixi.lock` in place (ww18, pixi 0.80.0
   against a lock from 0.69.0: 1095 lines changed, `osx-arm64` renamed `osx-arm64-macos-14-0`,
   `virtual-packages:` blocks added, every `conda_source` hash recomputed) and leaves the
   checkout dirty, so the next `git checkout` refuses. `--locked` installs the committed lock
   unchanged on both versions. pixi ≥ 0.80 also warns that `[system-requirements]` is deprecated
   in favor of virtual packages on `platforms`; the fix for both is one re-lock from the newer
   pixi plus `requires-pixi` in the workspace manifest (webDisplayTools issue, after the
   2026-10-27 release). With `rig_interfaces` pinned nothing is compiled: the typed package is CI-built per
   platform and installed like any other conda package (9 s on ww18). Building the interfaces
   from source is the **developer path only** (see Windows below).
3. Check: `python -m rig_nodes.build_interfaces --check` (typed package importable and
   serializing), then the consumer's harness (`cd rig && pixi run test` in webDisplayTools,
   which is also the cross-repository smoke: record §7.8).
4. Ports the graph owns on one host: **7447** (Zenoh router, loopback only), **8765** (the
   Studio gateway), **60000** (FicTrac UDP in), **5010** (BIAS HTTP), **5555** (ScanImage
   JSON server), **8766** (foxglove_bridge). One router per host: never run two graphs or two
   harnesses (two worktrees) on one machine at the same time; Zenoh discovery crosses them.
5. Keep an installable copy of the released channel on the group share (README → "Releasing a
   package set"), so a channel outage never blocks a rig rebuild.

### Platform notes

**Windows** (most rig PCs)
- Activation paths use `%CONDA_PREFIX%`; `$PIXI_PROJECT_ROOT` stays literal in
  `[activation.env]` (ww18 W1). The Zenoh configs ship inside the package since 0.2.1, so no
  path in the workspace points at them.
- `foxglove_bridge` (the `viz` environment, `pixi run rig -- --foxglove`) exits `0xC0000135`
  (`Process exited with failure 3221225781`) unless `%CONDA_PREFIX%\Library\lib` is on `PATH`:
  robostack's `ros2-foxglove-bridge 3.6.0` puts `foxglove.dll` and `foxglove_cpp_shared.dll`
  there (ww14 2026-10-21, ww18 2026-10-28). The `[feature.viz.target.win-64.activation.env]`
  line that fixes it lands with webDisplayTools PR #269; until then prepend
  `<rig>\.pixi\envs\viz\Library\lib` to `PATH` by hand before starting the graph.
- `time.monotonic()` ticks every **15.6 ms** (`GetTickCount64`): any latency measured with it
  reads 0 / 16 / 31 ms. Use `time.perf_counter_ns()` for stamps (the package does).
- A connect to a **closed** localhost port is refused only after about 2 s (`WinError 10061`),
  so a dead device program makes every command wait its full timeout before `unknown`.
- Two programs can listen on one port (`0.0.0.0:5010` and `127.0.0.1:5010`); loopback clients
  reach the more specific one. Check `netstat -ano | findstr :5010` before blaming the node.
- Text files: everything the package writes is LF (`newline="\n"`); a Windows text-mode
  `open()` elsewhere produces CRLF and breaks byte-identity checks (ww18 W3/W4).
- Time stamps by hand: Git Bash has no tzdata and prints UTC labeled "ET"; use PowerShell.
- Building `rig_interfaces` from source needs the MSVC **v143** toolset (VS 2022 group; VS 2026
  Build Tools alone fail with `Could not find the compiler … cl.exe`), a short build path
  (`detached-environments`, paths past ~250 chars break), and the CMake workaround in
  `src/rig_interfaces` for pixi-build-ros' `Library/../Lib` install paths
  (`docs/typed-windows.md`). None of this applies when installing from the channel.

**macOS** (developer machines)
- `[system-requirements] macos = "14.0"` is required: `rmw_zenoh ≥ 0.2.11` needs `__osx >= 14`,
  and without the floor osx-arm64 silently resolves the 0.16 RoboStack set with a different
  Python.
- No `timeout` command; scripts that need one use Python.
- `pixi run` from a repository root with its own `pixi.toml` provisions **that** workspace
  (300 MB) instead of the rig one: always `cd` into the rig workspace or pass
  `--manifest-path`.
- The stock Zenoh IPv6 router config is unverified; the shipped loopback config is IPv4.

**Linux** (CI only so far): the package-set workflow installs and tests on `ubuntu-latest`
every push; no lab rig runs Linux yet.

## Host table

| Host | Role | OS / hardware | Installs | Last verified | Record |
|---|---|---|---|---|---|
| Michael's Mac (osx-arm64) | development, bench G6 2×10 controller when attached | macOS 14+, Chrome | webDisplayTools `rig/` default + typed from the v0.2.1 channel | 2026-10-27, PR #274 `ad98b2a`: harness 59/59 + 62/62, live FicTrac (sample video) → emulator runs, real Studio connected 20 Hz | webDisplayTools PR #274 comment (2026-10-27); `rig/README.md` → Mac results (2026-10-14) |
| ww18 (`rig5-mr`, `reiserlab-ww18`) | arena rig: G6 2×10 fly on ball, real FicTrac (FLIR USB3 camera, Spinnaker 4.4), BIAS on an Atlas10 camera | Windows 11 Enterprise 26200; Dell Pro Max Tower T2, Core Ultra 9 285K, 63 GB, RTX PRO 4500; pixi 0.80.0; FicTrac 2.1.2 pixi-built | webDisplayTools `rig/` default + typed from the v0.2.1 channel (rig_nodes 0.2.1, rig_interfaces 0.2.1, mutex 0.19.0, Python 3.12.16; 1.6 GB, 398 packages, 10 s warm) | 2026-10-27/28, PR #274 `ad98b2a` → `ca9f5c4`: plain install without a sibling checkout (13 s), typed + `check-typed`, harness 59/59, graph at 96 Hz on real FicTrac for 24.5 h unattended, two Production Studio v0.98 arena runs on real FicTrac (idx mismatch 0.00 %, runlog-check OK, controller 0x70 = host 0x70 = 1544), plus two on the simulator; export identical to the live file; Foxglove dashboard live during one run at no timing cost | webDisplayTools PR #274 comments [1](https://github.com/reiserlab/webDisplayTools/pull/274#issuecomment-6451162032), [2](https://github.com/reiserlab/webDisplayTools/pull/274#issuecomment-6479759516); `rig/windows-results-ww18-2026-10-15.md`; `docs/typed-windows.md` here |
| Bergamo PC (`reiserlab-bergamo1`) | 2P rig: ScanImage 2026.0.0 (simulated vDAQ), G6 controller at times | Windows, MATLAB R2025b | webDisplayTools `rig/` default + ScanImage connector | 2026-10-16 (connector day) and 2026-10-22 (ZMQ frames wrapper, PR #261); **released `main` 2026-10-26 pending (QC 2026-10-27)** | webDisplayTools `rig/scanimage-day-2026-10-16.md`, `rig/SCANIMAGE-DAY.md` |
| ww14 (okey-cokey PC) | behavior rig: BIAS v0.73.1 (Spinnaker) on a FLIR BFS-U3-51S5M, G6 controller on COM3, no FicTrac | Windows | webDisplayTools `rig/` default + viz | 2026-10-21/22 (PR #269 branch); **released `main` 2026-10-26 pending (QC 2026-10-27)** | webDisplayTools PR #269: `rig/okey-cokey-pc-2026-10-21.md`, `rig/okey-cokey-pc-results-2026-10-21.md` |
| Robot station (Isabel) | Mecademic robot + two BIAS cameras | — | not yet (LAB-245; shared pieces to come from here) | — | Linear LAB-245 / LAB-248 / LAB-251 |

## Per-host notes

### Michael's Mac
- **FicTrac:** `~/Documents/GitHub/Fictrac-MacOS`. The `build/macos` binary stopped loading on
  2026-10-27 after a Homebrew `abseil` upgrade (OpenCV 4.13's `libopencv_dnn` links
  `libabsl_*.2601.0.0`, which the new abseil no longer ships). Use `build/macos-opencv5/bin/fictrac`
  (self-contained OpenCV 5), or start the old build with
  `DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/abseil/20260107.1/lib` while that keg exists. The
  sample clip (`sample/sample.mp4`, 10 s, 300 frames) with `sock_host : 127.0.0.1`,
  `sock_port : 60000`, `src_fps : 20` is the standard no-hardware source for a live graph test.
- Chrome 155/156 showed **no** local-network-access prompt for `reiserlab.github.io` →
  `ws://localhost:8765` (2026-10-14, 2026-10-27). Do not assume LAB-304 bites until seen.
- Serving the Studio locally for a branch test: `python3 -m http.server 8080` from the
  repository root, then `http://localhost:8080/arena_studio.html?mode=run`. Without a
  controller the Studio connects to the bridge and shows frames; nothing is applied.

### ww18 (`rig5-mr`)
- `fictrac.exe` run directly exits `0xC0000135` (DLL not found); from Git Bash the message is
  `error while loading shared libraries: api-ms-win-crt-locale-l1-1-0.dll: cannot open shared
  object file` (rc 127). It is pixi-built: start it through `build\Release\fictrac.bat
  config.txt` from the config's directory, which puts its environment's `Library\bin` and
  `C:\Program Files\Teledyne\Spinnaker\bin64\vs2015` on `PATH`.
- **FicTrac camera dropped off USB on 2026-10-27, cleared by a PC reboot** (FLIR USB3 Vision,
  `USB\VID_1E10&PID_4000\21205235`, driver Teledyne 2.8.3.16; the PC had been up two weeks). The
  camera opened but delivered no frames: `Error grabbing frame!
  Error was: Spinnaker: Failed waiting for EventData on NEW_BUFFER_DATA event. (GenTL error code:
  -1011) [-1011]`, then FicTrac exits `0xC0000409`; SpinView showed no frames either. After a
  replug it enumerated for 16 s and dropped off the bus; on another port and cable it never
  enumerates (`Error! Could not find any connected PGR cameras!`, PnP `Present = False`), rear
  LED blinking. After the reboot the camera was fine; the replugs had moved it, so the FicTrac
  ball ROI was shifted 14 px down (chosen by the lowest match error, median 6.0e3 against 5.2e3 on
  the 2026-10-15 run). If FicTrac stops delivering frames: reboot first, then power the camera down
  for ≥ 30 s, then SpinView on another PC. Without a camera the graph runs on the simulator
  (`pixi run rig -- --sim --sim-args "--proto udp --model fly --kp 2 --seed 1 --rate 50"`).
- No system Python on `PATH`: `python` opens the Microsoft Store alias (`Python was not found;
  run without arguments to install…`). Use `pixi run python …` inside a workspace; nothing in
  the rig flow needs a system Python.
- 2026-10-27: a plain `pixi install` in `rig\` failed with `failed to solve requirements of
  environment 'dev' for platform 'win-64'` (the `dev` path dependencies, see the recipe). Fixed in
  webDisplayTools `31011dd`; the `C:\Lab\GitHub\rig-nodes` v0.2.1 shallow clone left on the PC as
  the workaround is no longer needed.
- BIAS on the Atlas10: `connect` takes about 2.2 s, past a 2 s per-call timeout, so a successful
  connect can come back `outcome: unknown`; `stop-capture` takes ~3 s on short recordings. Check
  the status afterwards instead of retrying blindly.
- BIAS writes `<stem>_cam_0_date_…_v001.ufmf` next to the requested path and keeps per-frame
  stamps only in `stamp_log_cam0.txt`, which the next recording overwrites.
- Ctrl-C on the launcher once killed the bridge node before it could close the bag (W6, fixed:
  the launcher now stops children in order). If a bag has no `metadata.yaml`, recover it with
  `rig-reindex` / the converter's `--recover`.
- Use PowerShell (`Measure-Command`) for timings, not Git Bash.

### Bergamo PC (`reiserlab-bergamo1`)
- Start MATLAB from a terminal, **never** from the MATLAB MCP connector's process tree:
  ScanImage's `AcquisitionMex` aborts in libzmq (`0x40000015`) inside that tree.
- ScanImage JSON server: `hJ = most.network.json.Server(5555)`; the connector talks to
  `127.0.0.1:5555`. `hSI.hDisplay.lastFrameNumber` throws `Index exceeds array bounds` until the
  first frame of a session; the connector tolerates it.
- `scanFrameRate` is **15.0 Hz** at 512×512 unidirectional; protocols sized for 29 Hz run long.
- The console code page is cp1252: set `PYTHONUTF8=1` (or avoid non-ASCII in printed text) or
  Python tools crash on `→`.
- The simulated vDAQ's GRAB never goes idle on its own; ABORT closes it.
- With the ZMQ frames wrapper, start the node **before** GRAB; frames queue up to libzmq's
  default high-water mark (1000 messages, about 1 GB at 512×512×2 channels).

### ww14 (okey-cokey PC)
- pixi was **0.70.2** on 2026-10-21 (older than the Mac's); update before an install from the
  channel if a lock-file feature is missing.
- BIAS v0.73.1 Spinnaker build at
  `C:\Users\labadmin\Documents\FlyBowl1Rig\BIAS_exe_spinnaker_win64_v0_73_1\v0.73.1\test_gui.exe`;
  listens on `0.0.0.0:5010`; its status reply has no drop counters (`frameIdGaps`,
  `cameraSkipped` absent), so drop-based health stays `None`. Its external-control setting is
  lost on restart; the saved camera config (`rig/config/bias/okey-cokey-bfs-u3-51s5m.json`)
  lands in the arena repository with webDisplayTools PR #269.
- Camera FLIR Blackfly S BFS-U3-51S5M (2448×2048 MONO8, ~73 fps, internal trigger); the G6
  controller enumerates as "USB Serial" on COM3.
- Full install (root + `--locked` + `viz`) took 45 s.

## When something here is wrong

Open an issue or PR against this file. If the note describes an application (Studio, course
repo, protocol), move it to that application's setup page instead and leave a link.
