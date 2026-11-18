# ezmsg beside rmw_zenoh: a measurement, not a migration (2026-11-17)

[ezmsg](https://www.ezmsg.org/) is a pure-Python pub/sub framework for signal-processing
pipelines (JHU / JHU APL / Wyss Center / Blackrock Neurotech, MIT, derived from labgraph). It has
no relationship to ROS 2: no shared transport, message format, tools or bridge. The question was
whether it should sit beside ROS 2 on a rig, or replace it for the simplest ones.

**Decision: ROS 2 stays the rig graph. ezmsg is at most a library inside one node** (a multi-stage
online signal chain whose results the node publishes as ROS topics). Nothing in this repository
depends on it.

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

## Limits

- **Python subscribers only.** The gap is mostly rclpy's message conversion, not the Zenoh
  transport: on ww18 the C++ rosbag2 recorder received 8 MB in 0.18–0.19 ms (`shm-bench.md`),
  faster than ezmsg between processes here.
- **Mac only, one run per cell.** Not repeated on a Windows rig PC, where rclpy was 3–4 times
  slower than on the Mac.
- **Unloaded:** one publisher, one subscriber, and the subscriber only checks the payload length.
- ezmsg applies backpressure instead of dropping; no cell here reached the point where the two
  policies differ (every achieved rate equals the requested one).
