# Typed messages on Windows — W2 follow-up (ww18, 2026-10-15)

LAB-303 Plan B. Follow-up to finding **W2** in `../windows-results-ww18-2026-10-15.md`: the `typed`
environment compiled and linked on ww18, then failed while packaging with
``paths in archives must not have `..` when setting path for Library``.

**Short answer.** Rig PCs can use typed messages without compiling anything: build
`arena_interfaces` once per platform in CI, publish it to a conda channel, and have the `typed`
environment depend on that package. Tested end to end on ww18 with a local file channel: a fresh
`pixi install --locked -e typed` took **9 s**, never started a build backend, and the harness
passed **48 / 48** with typed topics in the bag (§5). Building on the PC also works now: a
package-level CMake workaround (§2, commit 538cc85) plus a short build directory (§3) give a
35–36 s install and 48 / 48. That route still needs the MSVC v143 toolset and the `detached-environments`
setting on every PC. The upstream bug is known and a fix is open but not merged (§1, §4).

## What was measured

All on ww18: pixi 0.80.0, pixi-build-ros `0.7.6.20261014.1408.4ef64dd` (the newest build on
`prefix.dev/pixi-build-backends` today), MSVC 14.44.35207 (the v143 group inside VS 2026 Build
Tools), cmake 4.4.4, RoboStack Jazzy. Package cache warm.

| # | Setup | `pixi install … -e typed` | Then |
|---|---|---|---|
| T1 | as committed before today, checkout `C:\w\t\ros2-poc` (15 chars) | **FAIL** after 32 s (17 s build): the `..` error; 21 of 151 entries in `.rattler-build-package-files` read `<host>/Library/../Lib/site-packages/…` | — |
| T2 | + the CMakeLists.txt workaround (§2), same checkout, `--locked`, clean `bld/` | **PASS** 35 s (16 s build), lock unchanged; 151 files, 0 with `..` | `build_interfaces.py --check`: importable, serializes; harness **48 / 48** in 61 s, `typed topics: True` |
| T3 | T2 from this worktree (checkout path **79 chars**) + `detached-environments = 'C:\w\px'` | **PASS** 36 s; longest object path 253 chars | `--check`: importable, serializes |
| T4 | `pixi publish --path interfaces\arena_interfaces\pixi.toml --target-channel C:\w\chan` | 26 s → an indexed file channel, `win-64/arena_interfaces-0.1.0-ha41a7fa_0.conda` (101 KiB) | `pixi build --path … --output-dir` the same, 27 s |
| T5 | `typed` feature depends on `arena_interfaces` from that channel (no path dep, no toolchain), fresh env, `--locked` | **PASS 9 s, no build backend started** | `--check`: importable, serializes; harness **48 / 48** in 56 s, `typed topics: True` |

T1–T3 and T5 were run in a scratch detached worktree at `C:\w\t` (T5's manifest change is not committed;
§5 has it). Logs, T1's package-file list and the §4b recipes are on ww18 in `C:\w\logs-w2\` (not
committed); the scratch worktree and envs were removed.

## 1. Where the `..` comes from

Three pieces, each fine alone:

1. **pixi-build-ros' Windows template** (`crates/pixi_build_ros/templates/bld_ament_cmake.bat` in
   prefix-dev/pixi, embedded in `pixi-build-ros.exe`) sets
   `PYTHON_INSTALL_DIR = os.path.relpath(SP_DIR, LIBRARY_PREFIX)` = `../Lib/site-packages`.
   This line comes from RoboStack's vinca templates. On conda's Windows layout, C/C++ files go
   under `Library\` and Python under `Lib\site-packages`, so the relative value has to climb out.
   On Linux and macOS the value is `lib/python3.12/site-packages` (no `..`), which is why the Mac
   build always worked.
2. **prefix-dev/pixi#6691** (commit 40a25ff, 2026-07-28, "clean stale files from incremental
   ROS Python builds"): the backend now reuses its host prefix between builds, so the template
   stopped letting rattler-build diff the prefix. Instead it appends CMake's `install_manifest.txt`
   to `%RATTLER_BUILD_PACKAGE_FILES%`. CMake records each destination as spelled:
   `%PREFIX%/Library/../Lib/site-packages/arena_interfaces/msg/_arena_frame.py`.
3. **rattler-build's `Files::from_paths`** (`crates/rattler_build_core/src/packaging/file_finder.rs`,
   the `RATTLER_BUILD_PACKAGE_FILES` reader from rattler-build #2476) accepts an entry when
   `strip_prefix(prefix)` succeeds (a lexical check) and `exists()` is true (the OS resolves the
   `..`). It never normalizes the path, and the tar writer then refuses the `..` component.
   Reproduced with rattler-build 0.76.1 alone, no ROS and no compiler, in under 1 s (§4b). An entry
   outside the prefix, spelled `$PREFIX/../outside.txt`, passes the containment check the same way.

**Fixed upstream?** Not in any release. prefix-dev/pixi#6955 is open (since 2026-09-04); a
2026-10-09 comment reports exactly our error on `micro_ros_msgs`, plus the MAX_PATH problem.
prefix-dev/pixi#7226 (open since 2026-10-10, waiting for review) passes an absolute forward-slash
`PYTHON_INSTALL_DIR` instead. The newest backend build (2026-10-14) still has the `relpath` line;
I checked the template embedded in the binary. Nobody has filed the rattler-build side.

Note for reproducibility: the backend is not in `pixi.lock`. `version = "*"` in
`interfaces/arena_interfaces/pixi.toml` floats to the newest nightly-style build on every fresh
build. That will pick up #7226 automatically, but it also means two rigs can build with different
backends.

## 2. Package-level workaround (commit 538cc85)

`interfaces/arena_interfaces/CMakeLists.txt`, before `find_package(ament_cmake)`:

```cmake
if(WIN32 AND PYTHON_INSTALL_DIR MATCHES "^\\.\\./")
  cmake_path(ABSOLUTE_PATH PYTHON_INSTALL_DIR BASE_DIRECTORY "${CMAKE_INSTALL_PREFIX}"
    NORMALIZE OUTPUT_VARIABLE _arena_python_install_dir)
  set(PYTHON_INSTALL_DIR "${_arena_python_install_dir}")
endif()
```

This is the value #7226 would pass: absolute, normalized, forward slashes (backslashes would break
the generated `cmake_install.cmake`). A normal variable shadows the `-D` cache entry and reaches
rosidl's `arena_interfaces__py` subdirectory. The block is a no-op once upstream passes an
absolute path, and it never runs under colcon (`Lib/site-packages` relative to the install
prefix) or on Linux/macOS. Side effects, both harmless here:

- ament_cmake_python byte-compiles `"${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/<pkg>"`, so
  with an absolute value it logs `Can't list '%PREFIX%/Library/%PREFIX%/Lib/site-packages/arena_interfaces'`
  and compiles nothing. The package is unchanged either way: the 5 `.pyc` files in it come from
  rattler-build's own post-build step ("Compiling 5 .py files to .pyc", in the failing build and in
  the fixed one), not from ament's compileall, whose output never reaches `install_manifest.txt`.
  (Corrected 2026-10-16; an earlier version said no `.pyc` were packaged.)
- `share/arena_interfaces/environment/pythonpath.dsv` now holds an absolute path. rattler-build
  rewrites it to the install prefix (checked in the installed env); conda environments don't
  read these hooks.

## 3. MAX_PATH: `detached-environments`

`cl.exe` failed with `C1083` once an object path passed 260 characters (W2-2). The build adds
about 211 characters below the pixi directory. pixi 0.80 offers no override for `.pixi/bld`
alone, but `detached-environments` moves the whole pixi directory (`pixi_dir()` in pixi_core),
build workspaces included, to `<dir>\<workspace-name>-<xxh3 hash, up to 20 digits>`:

```powershell
pixi config set --local detached-environments C:\px   # once per checkout (.pixi\config.toml)
pixi install -e typed
```

With `C:\w\px` the longest object path was 253 characters, from a checkout 79 characters deep (T3).
**Keep the directory at 12 characters or fewer.** One script assumed `.pixi\envs`: `rig.py`'s
viz-env check now uses the running env's sibling (`CONDA_PREFIX`), so `--foxglove` works with
detached envs. The harness's `find_node()` looks in the *root* workspace, which this setting
doesn't touch. `LongPathsEnabled` was not tried (it is 0 on ww18, and changing it needs admin).

## 4. Upstream — filed 2026-10-16 (text below as posted)

### 4a. Comment on prefix-dev/pixi#7226 — [posted](https://github.com/prefix-dev/pixi/pull/7226#issuecomment-6217653890)

> Confirming this on a second machine with the newest backend build, plus one side effect.
>
> **Setup:** pixi 0.80.0, `pixi-build-ros 0.7.6.20261014.1408.4ef64dd` (newest on
> prefix.dev/pixi-build-backends as of 2026-10-15; still has the `relpath` line), win-64
> (Windows 11 26200), MSVC 14.44.35207 (v143 inside VS 2026 Build Tools), robostack-jazzy. The
> package is three `.msg` files: `ament_cmake` + `rosidl_generate_interfaces`, nothing else.
>
> **Before:** build and install succeed; packaging fails with
> ``paths in archives must not have `..` when setting path for Library``. 21 of the 151 lines in
> `.rattler-build-package-files` are `<host>/Library/../Lib/site-packages/…`: every Python file,
> the egg-info and the three `.pyd`.
>
> **With the value this PR passes:** we can't change the template, so we applied it from the
> package's own CMakeLists.txt. If `PYTHON_INSTALL_DIR` starts with `../`, replace it with
> `cmake_path(ABSOLUTE_PATH PYTHON_INSTALL_DIR BASE_DIRECTORY "${CMAKE_INSTALL_PREFIX}" NORMALIZE …)`.
> The package then builds, packages (151 files, none with `..`) and installs. The generated
> Python types import and serialize, and our end-to-end test passes with typed topics.
>
> **Side effect worth a line in the PR:** ament_cmake_python byte-compiles
> `"${CMAKE_INSTALL_PREFIX}/${ARG_DESTINATION}/<pkg>"` (`ament_python_install_package.cmake`). With an
> absolute `PYTHON_INSTALL_DIR` that becomes `%PREFIX%/Library/%PREFIX%/Lib/site-packages/<pkg>`, and the
> build logs `Can't list …` (not fatal). The package doesn't change: its 5 `.pyc` files come from
> rattler-build's own post-build step ("Compiling 5 .py files to .pyc", in the failing build and the fixed
> one alike), not from ament's compileall, whose output never reaches `install_manifest.txt`. If you'd rather keep the relative value the ament README asks for, you could instead normalize
> the manifest lines before appending them to `%RATTLER_BUILD_PACKAGE_FILES%`. Or rattler-build could
> normalize the list itself (prefix-dev/rattler-build#2875), which would cover every template.

Edited on GitHub 2026-10-16: the `.pyc` sentence first said none were packaged; the package does
hold 5, compiled by rattler-build. The quote above is the corrected text.
>
> **On the MAX_PATH note in #6955:** `detached-environments` moves `bld/` along with the envs. With
> `pixi config set --local detached-environments C:\px`, a checkout 79 characters deep built fine for
> us (longest object path 253 characters).

### 4b. Issue prefix-dev/rattler-build#2875 — [filed](https://github.com/prefix-dev/rattler-build/issues/2875) (links back to 4a)

**Title:** `RATTLER_BUILD_PACKAGE_FILES` entries are not normalized: a `..` inside the prefix breaks
packaging (and passes the outside-prefix check)

> `Files::from_paths` (`packaging/file_finder.rs`, the reader for `$RATTLER_BUILD_PACKAGE_FILES`
> from #2476) accepts an entry when `resolved.strip_prefix(prefix)` succeeds and `resolved.exists()`
> is true, then packages it under that spelling. `strip_prefix` is lexical but `exists()` resolves `..`,
> so:
>
> 1. a file inside the prefix spelled through `..` (`$PREFIX/Library/../Lib/x.py`) is accepted, then
>    rejected by the archive writer: ``paths in archives must not have `..` when setting path for Library``;
> 2. a file **outside** the prefix spelled `$PREFIX/../outside.txt` is also accepted ("Largest files:
>    9 B - ../outside.txt"). The build then fails the same way instead of with `PackageFileOutsidePrefix`.
>
> This is how pixi-build-ros fails on Windows for every ROS interface package (prefix-dev/pixi#6955).
> CMake writes `install_manifest.txt` with each destination as spelled
> (`%LIBRARY_PREFIX%/../Lib/site-packages/…`), and the build script appends that manifest to the
> list. A template fix is open (prefix-dev/pixi#7226), but any build script that records CMake, meson
> or setuptools output paths can hit the same thing.
>
> **Repro** (rattler-build 0.76.1, win-64, no compiler, < 1 s):
>
> ```yaml
> package:
>   name: dotdot-repro
>   version: "0.1.0"
> build:
>   number: 0
>   script:
>     - if: win
>       then:
>         - mkdir "%LIBRARY_PREFIX%" "%PREFIX%\Lib\site-packages\repro"
>         - echo hello> "%PREFIX%\Lib\site-packages\repro\hello.txt"
>         - echo %LIBRARY_PREFIX:\=/%/../Lib/site-packages/repro/hello.txt>> "%RATTLER_BUILD_PACKAGE_FILES%"
>       else:
>         - mkdir -p "$PREFIX/share" "$PREFIX/lib/repro"
>         - echo hello > "$PREFIX/lib/repro/hello.txt"
>         - echo "$PREFIX/share/../lib/repro/hello.txt" >> "$RATTLER_BUILD_PACKAGE_FILES"
> ```
>
> `rattler-build build --recipe recipe.yaml`:
>
> ```
>  │ │ Files in package:
>  │ │   ├─ Library/../Lib/site-packages/repro/hello.txt (7 B)
>  …
> Error:   × Could not open or create, or write to file
>   ╰─▶ paths in archives must not have `..` when setting path for Library
> ```
>
> Variant 2: write `"%PREFIX%\..\outside.txt"` and record `%PREFIX:\=/%/../outside.txt`. The result
> is `9 B - ../outside.txt` in the package listing, then ``… must not have `..` when setting path for
> ././@LongLink``. (Only win-64 was run; the `else:` branch is the same mechanism on Unix.)
>
> **Suggested fix:** before the containment check, normalize each entry lexically: drop `.` and
> resolve `..` against the previous component, with no filesystem access. Then `strip_prefix`, and
> return `PackageFileOutsidePrefix` if the result leaves the prefix. Normalizing also makes the
> `HashSet` dedupe one file listed under two spellings.

## 5. Fallback: build once in CI, publish to a channel

Rigs install `arena_interfaces` as a plain binary, like every RoboStack package. That means no
MSVC, no backend, no MAX_PATH and no `detached-environments`, and the `typed` install becomes a
download like the default one. T4 + T5 tested this on ww18 with a local file channel.

**Consumer side** (what T5 ran, with the real channel URL):

```toml
[feature.typed]
channels = ["https://prefix.dev/<lab-channel>"]

[feature.typed.dependencies]
arena_interfaces = { version = "0.1.*", channel = "https://prefix.dev/<lab-channel>" }
ros-jazzy-rosidl-default-runtime = "*"

# For whoever edits a .msg: today's source build, in its own solve group (the same package name
# from a path and from a channel cannot share one).
[feature.typed-src.dependencies]
arena_interfaces = { path = "interfaces/arena_interfaces" }
# … the ament/rosidl toolchain the typed feature carries today

[environments]
typed = { features = ["typed"], solve-group = "poc" }
typed-src = { features = ["typed-src"], solve-group = "typed-src" }
```

**Build side** — a sketch, not added to `.github/workflows/` (it would run on this PR):

```yaml
name: arena-interfaces
on:
  push:
    tags: ['arena-interfaces-v*']
  workflow_dispatch:
permissions:
  contents: read
  id-token: write          # prefix.dev trusted publishing (OIDC), no stored token
jobs:
  build:
    strategy:
      matrix:
        include:
          - { os: windows-2022, subdir: win-64 }    # VS 2022 → conda-forge vs2022 finds v143
          - { os: macos-14,     subdir: osx-arm64 }
          - { os: ubuntu-24.04, subdir: linux-64 }
    runs-on: ${{ matrix.os }}
    steps:
      - uses: actions/checkout@v5
      - uses: prefix-dev/setup-pixi@v0     # pin an exact version + pixi-version: v0.80.0
        with: { run-install: false }
      - if: runner.os == 'Windows'           # D:\a\webDisplayTools\webDisplayTools\ros2-poc is 45 chars: at the limit
        working-directory: ros2-poc
        run: pixi config set --local detached-environments C:\px
      - working-directory: ros2-poc
        run: pixi build --path interfaces/arena_interfaces/pixi.toml --output-dir ../dist
      - uses: actions/upload-artifact@v5
        with: { name: arena_interfaces-${{ matrix.subdir }}, path: dist/*.conda }
  publish:                                   # all three or nothing: see "partial channel" below
    needs: build
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/download-artifact@v5
        with: { path: dist, merge-multiple: true }
      - uses: prefix-dev/setup-pixi@v0
        with: { run-install: false }
      # trusted publishing: check the OIDC flag/behaviour of the rattler-build version you pin
      - run: pixi exec rattler-build upload prefix --channel <lab-channel> dist/*.conda
```

Notes and costs:

- **Channel host.** A prefix.dev channel (free for public ones) gets OIDC trusted publishing,
  so there is no token to rotate. It does add `prefix.dev` to a rig's network allowlist: today's
  default env reaches only `conda.anaconda.org`, and the source build already reaches prefix.dev
  for the backend. An anaconda.org org channel (`conda.anaconda.org/<org>`) keeps a single host but
  needs an API-token secret. A third option, not tried: commit the three `.conda` files (~100 KiB
  each) to the repo as a local channel.
- **Partial channel = no lock.** The workspace locks win-64, osx-arm64 and linux-64 together. With
  only the win-64 build in the channel, the lock could not resolve the other two platforms (seen in
  T5 until the scratch feature was limited to win-64). So publish all three platforms in one job.
- **When to rebuild.** On every `.msg` change: bump the version in `package.xml` and in
  `interfaces/arena_interfaces/pixi.toml`, tag, let CI publish, then `pixi update arena_interfaces` and commit the lock. Also
  rebuild when re-locking onto a newer RoboStack: the package pins `ros2-distro-mutex` (`>=0.19,<0.20`
  on win-64 and linux-64 in today's lock, `>=0.16,<0.17` on osx-arm64). Until someone re-locks, the
  lock keeps every rig on the old build.
- **One build for the whole graph.** rmw_zenoh puts the message type hash in its key expressions,
  so every node needs the same `.msg` version. The hash comes from the `.msg` text, so the Mac and
  a rig agree when both install the same channel version. A source build on one machine and a stale
  one on another would not see each other's typed topics. With the channel, the lock pins exact bytes
  (`sha256`) on every machine.
- **Effort.** About 50 lines of workflow, one channel to own, three runner jobs of about 2–4 min per
  `.msg` change. The CMakeLists workaround (§2) is still needed for the Windows CI build until
  #7226 ships.

## 6. What a Windows rig PC needs, per route

| | default env (String topics) | typed, built on the PC (today) | typed, prebuilt channel (§5) |
|---|---|---|---|
| install | `pixi install` | `pixi install -e typed` | `pixi install -e typed` |
| compiler | — | MSVC **v143** toolset (VS 2022 Build Tools, or the v143 group in VS 2026; UAC, < 5 min) | — |
| path setup | — | short checkout **or** `detached-environments` ≤ 12 chars | — |
| network | conda.anaconda.org | + prefix.dev (backend) | + the channel host |
| ww18 time (warm) | 25 s | 35–36 s | 9 s |
| harness | 47 / 47 | 48 / 48 | 48 / 48 |
