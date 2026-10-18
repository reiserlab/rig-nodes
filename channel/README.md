# ros2-poc/channel — a prebuilt arena_interfaces (LAB-303 Plan B test, win-64 only)

> **2026-10-18:** the package payload (`win-64/*.conda`, repodata) was removed from the tree —
> a binary in git history is enough for the pinned URL the `typed-prebuilt` environment reads
> (commit `30512581`, see `../pixi.toml`), and the CI-built channel replaces it. This README stays
> as the record of how the one-off package was built.

A conda channel with one package, read by the opt-in `typed-prebuilt` environment in
`../pixi.toml` over raw.githubusercontent.com (pinned to the commit that added these files), so a
Windows PC **without Visual Studio** can run the graph with typed messages. Stand-in for the
CI-built channel decided on 2026-10-16 (`../docs/typed-distribution-decision.md`): the next
rebuild comes from `.github/workflows/arena-interfaces.yml` (three platforms, `arena-interfaces-channel`
artifact) and replaces this directory; the env then points at an `arena-interfaces-v*` tag.

- `win-64/arena_interfaces-0.1.0-ha41a7fa_0.conda`, sha256 `5501edc89c10f6ee7acca21273f26f716edf2b7686e7109fed54335a3d39c883`
- Built 2026-10-16 on ww18 from commit 4632674 (the CMakeLists workaround included) with pixi 0.80.0
  and pixi-build-ros 0.7.6.20261014.1408.4ef64dd:
  `pixi publish --path interfaces\arena_interfaces\pixi.toml --target-channel C:\chan`
- Depends on `ros2-distro-mutex >=0.19,<0.20` (RoboStack Jazzy as locked today) and Python 3.12.

Rebuild and re-commit after any `.msg` change; the env pins `==0.1.0`, so bump the version too.
