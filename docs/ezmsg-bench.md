# ezmsg beside rmw_zenoh: a measurement, not a migration (2026-11-17, Linux and rmw_iceoryx added 2026-11-19)

[ezmsg](https://www.ezmsg.org/) is a pure-Python pub/sub framework for signal-processing
pipelines (JHU / JHU APL / Wyss Center / Blackrock Neurotech, MIT, derived from labgraph). It has
no relationship to ROS 2: no shared transport, message format, tools or bridge. The question was
whether it should sit beside ROS 2 on a rig, or replace it for the simplest ones.

**Decision: ROS 2 stays the rig graph. ezmsg is at most a library inside one node** (a multi-stage
online signal chain whose results the node publishes as ROS topics). Nothing in this repository
depends on it. **rmw_iceoryx**, a zero-copy transport for ROS 2 that Frank measured beside both
(below), does not help the rigs as they are either: its fast path needs fixed-size messages and
C++ nodes, and it is single-host.

## Why not instead of ROS 2

| need (CONTRACT.md) | ROS 2 + rmw_zenoh | ezmsg 3.9 |
|---|---|---|
| typed, language-neutral contract | `.msg`, hashed on the wire | pickled Python objects, Python only |
| recording | rosbag2 / MCAP, `mcap_index`, `cdr.py` | no MCAP; a JSON message logger |
| viewing | Foxglove, `ros2 topic`, `graph_dump` | Panel / vispy plots, Mermaid / Graphviz graph dumps |
| non-Python clients (Studio gateway, MATLAB, C++) | supported | only through websocket or ZeroMQ units |
| independently supervised device nodes | the native model | possible, but built around one launched pipeline |
| install on a rig PC | RoboStack + the CI-built channel (solved, `typed-distribution-decision.md`) | `pip install`, no compiler |
| community | the ROS ecosystem | about 30 GitHub stars, a few institutional backers |

The rig's traffic is small messages (FicTrac frames, JSON commands, 1 Hz host statistics), where
the measurement below shows no advantage for ezmsg.

## Measurement

`bench/ez_bench.py` repeats the cells of `bench/shm_bench_qpc.py` (same sizes, rates, 8 s per
cell after 1 s of warmup, the same latency span: stamp before publish → Python subscriber
entered). ezmsg is not in the pixi workspace; the script's docstring gives the `uv run` line.

Mac (Apple M5 Max, macOS 27.0.1, Python 3.12), 2026-11-17, one run each, nothing lost in any
cell. Median latency in ms (p99 in parentheses):

| message | rmw_zenoh, shm off | rmw_zenoh, shm on | ezmsg, 2 processes, `bytes` | ezmsg, 2 processes, numpy | ezmsg, forced TCP, numpy | ezmsg, same process |
|---|---|---|---|---|---|---|
| 1 kB @ 100 Hz | 0.37 (0.69) | 0.27 (0.71) | 0.22 (4.03) | 0.24 (2.92) | 0.34 (0.69) | 0.08 (0.20) |
| 64 kB @ 100 Hz | 0.39 (0.84) | 0.30 (0.61) | 0.19 (2.82) | 0.33 (4.44) | 0.43 (1.05) | 0.08 (0.18) |
| 1 MB @ 30 Hz | 1.28 (2.81) | 0.84 (1.40) | 0.55 (0.86) | 0.43 (2.59) | 1.30 (1.86) | 0.10 (0.29) |
| 2 MB @ 30 Hz | 2.28 (4.20) | 1.63 (2.37) | 1.06 (1.74) | 0.51 (0.92) | 2.52 (3.59) | 0.11 (0.24) |
| 8 MB @ 30 Hz | 5.45 (9.04) | 3.46 (5.47) | 2.10 (5.06) | 0.73 (6.33) | 6.19 (8.32) | 0.12 (0.22) |
| 2 MB @ 135 Hz | 2.18 (3.89) | 0.90 (8.45) | 0.74 (1.38) | 0.31 (2.80) | 2.07 (2.62) | 0.08 (0.17) |

rmw_zenoh: Jazzy (RoboStack), RELIABLE, depth 10, `sensor_msgs/Image`, rclpy on both ends.
ezmsg 3.9.0: a dataclass with the payload as `bytes` or a numpy `uint8` array.

- **Small messages:** medians are tied; ezmsg's p99 between processes was worse (2.8–4.4 ms
  against 0.6–0.7 ms).
- **Large arrays between processes:** ezmsg is 1.2 to 5 times faster at the median, most with
  numpy payloads (out-of-band pickle buffers). Its 8 MB p99 is no better than rmw_zenoh's.
- **Same process:** the object is handed over by reference, about 0.1 ms at any size. This is the
  case for a signal chain inside one node.
- **Shared memory off:** ezmsg over TCP is no faster than rmw_zenoh with shared memory off.
- **Subscriber CPU**, shared-memory cells: 1–3 % of a core (ezmsg) against 2–12 % (rclpy).

## Linux: rmw_iceoryx, rmw_zenoh and ezmsg on one machine (Frank, 2026-11-18)

[rmw_iceoryx](https://github.com/ros2/rmw_iceoryx) is not a framework but another transport under
ROS 2, in the slot rmw_zenoh fills on our rigs: the publisher writes into shared memory and the
subscriber reads the same memory. Frank built it under Jazzy and measured it, rmw_zenoh and ezmsg
on one Linux machine with the cells above. The full tables (Python → Python, Python → C++, C++
loan → C++) are in `ezmsg-and-rig-frameworks.html`; his benchmark scripts and raw results
(`iceoryx-bench/`) are **not in this repository yet**.

AMD Ryzen 7 7840U, Fedora 44 container on Linux 6.18 with shared memory limited to 63 MB, one
8 s run per cell. ROS 2 Jazzy from RoboStack on Python 3.14.10, rmw_zenoh 0.2.11, rmw_iceoryx
`jazzy` branch with iceoryx 2.0.6, RELIABLE, depth 10; ezmsg 3.9.0 on Python 3.12.16. Median
latency at 8 MB @ 30 Hz:

| path | nodes | median |
|---|---|---|
| ezmsg, same process | Python | 0.13 ms |
| rmw_iceoryx, fixed-size message, C++ loan → C++ (zero copy) | C++ | 0.13 ms |
| ezmsg, two processes, numpy | Python | 1.33 ms |
| ezmsg, two processes, `bytes` | Python | 2.62 ms |
| rmw_zenoh, fixed-size message, C++ → C++ | C++ | 6.2–6.8 ms |
| rmw_zenoh, `sensor_msgs/Image`, Python → C++ | Python, C++ | 6.5–7.8 ms |
| rmw_zenoh, `sensor_msgs/Image`, Python → Python | Python | 9.3–9.5 ms |
| rmw_iceoryx, `sensor_msgs/Image`, Python → C++ | Python, C++ | 34.1 ms (publisher fell behind) |
| rmw_iceoryx, `sensor_msgs/Image`, Python → Python | Python | 40.4 ms (publisher fell behind) |

rmw_zenoh ranges cover shared memory off and on.

- **rmw_iceoryx's zero-copy path is 0.09–0.14 ms from 64 kB to 8 MB** (fixed-size type, C++ loan
  API), against 2.0–6.8 ms for rmw_zenoh between the same C++ nodes at 2 and 8 MB.
- **For `sensor_msgs/Image` it is the slowest of the three from 64 kB up:** 2.5 to 5.3 times
  slower than rmw_zenoh at 1 MB and above, and the publisher could not hold 8 MB @ 30 Hz or
  2 MB @ 135 Hz. That is the path our rigs use (a variable-size type from Python nodes).
- **ezmsg between two Python processes was 4 to 7 times faster than rmw_zenoh** for 1 to 8 MB
  numpy arrays (1.2 to 5 times on the Mac).
- **rmw_iceoryx is one host only,** needs the RouDi daemon before any node, has no RoboStack
  package, and its `jazzy` branch (last commit January 2024) needed iceoryx 2.0.6 built from
  source. Its README lists `ros2 param list` and pub/sub events as unsupported and `ros2 bag` as
  untested; recording was not tested here.
- **ezmsg 3.9.0 did not run on Python 3.14** in this setup and ran in its own Python 3.12
  environment. A Lyrical-only rig is Python 3.14, so ezmsg inside a node would need a newer ezmsg
  or a separate environment.
- Turning shared memory on in rmw_zenoh made no clear difference on this machine, and it was not
  confirmed to be in use. The 63 MB shared-memory limit shaped every setup (four buffers at 8 MB
  for rmw_iceoryx and ezmsg, a 48 MB rmw_zenoh pool).
- These figures run from the publisher's timestamp to a typed subscriber callback, so they are
  not comparable with the ww18 recorder figure, which starts at the rmw publish call.

## Limits

- **Python subscribers only.** The gap is mostly rclpy's message conversion, not the Zenoh
  transport: on ww18 the C++ rosbag2 recorder received 8 MB in 0.18–0.19 ms (`shm-bench.md`),
  faster than ezmsg between processes here.
- **One Mac and one Linux container, one run per cell.** Not repeated on a Windows rig PC, where
  rclpy was 3–4 times slower than on the Mac. Compare columns within a machine, not across.
- **rmw_iceoryx was not tried on Windows,** and the C++ rosbag2 recorder was not measured on the
  Mac or the Linux machine.
- **Unloaded:** one publisher, one subscriber, and the subscriber only checks the payload length.
- ezmsg applies backpressure instead of dropping; no cell here reached the point where the two
  policies differ (every achieved rate equals the requested one).
