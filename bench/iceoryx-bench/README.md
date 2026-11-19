# iceoryx-bench: rmw_iceoryx beside rmw_zenoh and ezmsg (Linux, 2026-11-18)

The scripts and raw results behind the Linux section of `docs/ezmsg-bench.md` and
`docs/ezmsg-and-rig-frameworks.html`. This is its own pixi project (ROS 2 Jazzy from RoboStack,
linux-64 only); nothing in the rig-nodes workspace depends on it.

```
pixi.toml, pixi.lock     default environment: ROS 2 Jazzy, rmw_zenoh, build tools; `ezmsg` environment: Python 3.12, ezmsg 3.9.0
bench/fetch.sh           clone iceoryx v2.0.6 and rmw_iceoryx (jazzy branch) into ws/src/
bench/build.sh           colcon build of iceoryx, rmw_iceoryx, bench_msgs and bench_cpp into ws/install/
bench/env.sh             source the built workspace; `iox_preload` lists the iceoryx 2.0.6 libraries to preload
bench/run.py             the ROS 2 matrix (rmw_zenoh, rmw_zenoh with shared memory, rmw_iceoryx) → bench/results.jsonl
bench/pubsub.py          rclpy publisher / subscriber of sensor_msgs/Image
bench/roudi_*.toml       RouDi memory pools, sized for a 63 MB /dev/shm
bench/ez_run.sh          the ezmsg matrix → bench/ez_results.jsonl
bench/ez_pubsub.py       ezmsg publisher / subscriber
ws/src/bench_msgs        fixed-size messages (64 kB, 2 MB, 8 MB) for the loaned-message path
ws/src/bench_cpp         sub_image (C++ subscriber of sensor_msgs/Image), fixed (C++ loan publisher / subscriber)
```

**ezmsg 3.9.0 does not run on Python 3.14,** the Python of the default environment: it installs
there but fails when a pipeline starts. That is why it has its own `ezmsg` environment on
Python 3.12. `docs/ezmsg-bench.md` lists the Python versions checked.

## Running it

From this directory:

```
bash bench/fetch.sh
pixi run bash bench/build.sh
pixi run bash -c 'source bench/env.sh && python bench/run.py'
bash bench/ez_run.sh
```

`run.py` takes transport names to run a subset (`zenoh`, `zenoh-shm`, `iceoryx`). Both runners
append to their results file, one JSON line per cell.

`run.py` deletes the user's files in `/dev/shm` after every cell (all but `lttng*`). Run it in a
container or on a machine where nothing else of yours uses shared memory.

## Results

`bench/results.jsonl` and `bench/ez_results.jsonl` are the run of 2026-11-18 (AMD Ryzen 7 7840U,
Fedora 44 container on Linux 6.18, `/dev/shm` limited to 63 MB, one 8 s run per cell). Latency is
receive time minus the publisher's wall-clock stamp, in ms. `n` is messages received and `sent`
messages published; `sent` below rate × 8 s means the publisher fell behind.
