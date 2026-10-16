# Typed interfaces on Windows rigs — distribution decision (LAB-303, 2026-10-16)

**Decision: build `arena_interfaces` in CI and ship it as a conda channel; rigs never compile it.**
The default environment stays String-only and unchanged. The per-PC MSVC build remains available
for whoever edits a `.msg` (it is the same `pixi install -e typed` as today).

Measurements: `typed-windows.md` (ww18, 2026-10-15/16). This page is the decision and the recipe.

## The two routes, side by side

| | per-PC MSVC build (`typed` today) | CI-built channel (recommended) |
|---|---|---|
| what a rig PC installs | compiler + build backend + the ROS toolchain, then builds | one 100 KiB `.conda` like every RoboStack package |
| prerequisites on the PC | MSVC **v143** (VS 2022 Build Tools, or the v143 group added to VS 2026 — admin, ~5 min), `pixi config set --local detached-environments C:\px` (MAX_PATH), network to prefix.dev for the backend | none beyond the default env; network to the channel host |
| `pixi install -e typed`, ww18, warm cache | 35–36 s (T2/T3) | **5–9 s** (T5/T6), no build backend started |
| harness | 48 / 48 | 48 / 48 (T5), 49 / 49 (T6) |
| upstream exposure | the `..`-in-manifest bug needs our CMakeLists workaround until pixi#7226 / rattler-build#2875 ship; `version = "*"` on the backend floats, so two rigs can build with different backends | the workaround runs once, in CI; the lock pins the package by sha256 on every machine |
| type-hash agreement (rmw_zenoh keys typed topics by the hash of the `.msg` text) | guaranteed only if every machine built the same source | guaranteed: one build per version, same bytes everywhere |
| who can be surprised | every new rig PC, every VS update, every student who clones to a long path | the maintainer who forgets to rebuild after a `.msg` change (the version bump below is the guard) |
| cost to run | 0 infrastructure, N × 5 min of setup and a compiler per PC | one workflow (`.github/workflows/arena-interfaces.yml`), three runners × 2–4 min per `.msg` change, one channel to own |

The per-PC route works now (T2/T3) and is the right **developer** path. It is the wrong **rig**
path: the whole point of the two-environment split was that a rig PC always installs; a route
that needs an admin, a compiler, a path setting and an unmerged upstream fix on every PC
re-introduces exactly the install friction the decision procedure scored against.

## Channel host

Three options were measured or sketched (`typed-windows.md` §5). In order of preference:

1. **In-repo channel, read over raw.githubusercontent.com** (T6: committed `ros2-poc/channel/`,
   5 s install on ww18). No account, no secret, no new host beyond GitHub, works today. Cost: the
   repo grows by ~300 KiB per release (three platforms), and the channel URL must name an
   immutable ref. **Use a tag, not a commit sha:** the commit that adds the packages also edits
   `pixi.toml` to point at `arena-interfaces-vX.Y.Z`, and the tag is placed on that commit — one
   commit, consistent, and the lock still pins each package's sha256.
2. **prefix.dev lab channel** with trusted publishing. No repo growth, OIDC instead of a token,
   but someone has to own the channel and rigs gain `prefix.dev` in their allowlist (the source
   build already needs it for the backend). The workflow's last step uploads there as soon as the
   repository variable `ARENA_INTERFACES_CHANNEL` and the secret `PREFIX_API_KEY` exist; nothing
   else changes.
3. anaconda.org org channel: one host for everything, but an API-token secret to rotate. Not
   chosen.

Start with 1; move to 2 when the lab decides to own a channel. The workflow serves both.

## What is implemented (this commit)

`.github/workflows/arena-interfaces.yml`:

- `build (win-64 | osx-arm64 | linux-64)` — `windows-2022` (VS 2022 → v143 found by conda-forge's
  activation, no W2-1), `macos-14`, `ubuntu-24.04`; pixi `v0.80.0` (the ww18 pair; the backend is
  not in the lock, so the version is pinned on purpose); on Windows
  `pixi config set --local detached-environments C:\px` first (W2-2); then
  `pixi build --path interfaces/arena_interfaces/pixi.toml --output-dir ../dist`, the archive
  checked for its two inner tarballs, uploaded as `arena_interfaces-<subdir>`.
- `channel` — downloads the three, lays them out as `channel/<subdir>/`, runs
  `rattler-index fs channel` (the same indexer `pixi publish` used for the ww18 channel), prints
  each package's sha256 and `ros2-distro-mutex` pin, uploads **`arena-interfaces-channel`**
  (90 days). On an `arena-interfaces-v*` tag with the prefix.dev variable set it also uploads to
  the hosted channel.
- Triggers: `pull_request` on `ros2-poc/interfaces/**` or the workflow file (build-only; the first
  run is the push of this commit on PR #254 — the Windows job is the one to watch), tags
  `arena-interfaces-v*`, `workflow_dispatch`.

All three platforms are built because the workspace locks them together: a channel with only
win-64 cannot be re-locked (T5's first attempt). Nothing in `pixi.toml` changed: the default
environment is untouched, `typed` still builds from source, `typed-prebuilt` still reads the
ww18-built win-64 package committed on 2026-10-16. The CI build was verified locally on the
Mac before this commit (`pixi build` 17 s → `arena_interfaces-0.1.0-h60d57d3_0.conda`,
`rattler-index fs` → `repodata.json` + shards); the Windows runner is verified by the PR run.

## Releasing a new build (the recipe)

After any `.msg` change, or when re-locking onto a newer RoboStack (the package pins
`ros2-distro-mutex`):

1. Bump the version in `interfaces/arena_interfaces/package.xml` **and**
   `interfaces/arena_interfaces/pixi.toml` (same number).
2. Push; the PR run builds the three packages. Download the channel:
   `gh run download <run-id> -n arena-interfaces-channel -D ros2-poc/channel` (replace the
   directory; `git rm -r ros2-poc/channel` first).
3. Point the prebuilt environment at the new tag and version (`ros2-poc/pixi.toml`):

   ```toml
   [feature.typed-prebuilt]
   platforms = ["win-64", "osx-arm64", "linux-64"]          # all three are in the channel now
   channels = ["https://raw.githubusercontent.com/reiserlab/webDisplayTools/arena-interfaces-v0.2.0/ros2-poc/channel"]

   [feature.typed-prebuilt.dependencies]
   arena_interfaces = { version = "==0.2.0", channel = "https://raw.githubusercontent.com/reiserlab/webDisplayTools/arena-interfaces-v0.2.0/ros2-poc/channel" }
   ```

   `pixi lock` (or `pixi install -e typed-prebuilt`), commit packages + manifest + lock in ONE
   commit, then `git tag arena-interfaces-v0.2.0 && git push --tags` on that commit. The tag push
   runs the workflow once more (and uploads to prefix.dev when configured).
4. Rigs: `git pull && pixi install --locked -e typed-prebuilt`.

**Second step, once the first CI channel is committed:** rename the environments so the rig path
is the short name — `typed` = the channel (what `WINDOWS-TEST.md` 7b calls `typed-prebuilt`),
`typed-src` = today's source build in its own solve group (the sketch in `typed-windows.md` §5).
Not done in this commit: it changes which environment the Windows test notes refer to, so it
belongs with the next rebuild, after the Bergamo PC run.

## Not covered

A per-PC build stays documented in `WINDOWS-TEST.md` 1b / `typed-windows.md` §2–§3 for `.msg`
authors. The workflow does not run the harness against the built package (that needs the 1 GB
ROS environment per runner; the harness runs on the rig PCs and the Mac). Hardware runs with
`-e typed` / `-e typed-prebuilt` were not repeated on ww18 (ww18 results, "Still open").
