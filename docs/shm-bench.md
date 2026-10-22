# Shared-memory and MCAP recording benchmarks (LAB-303)

Three scripts in `ros2-poc/` measure rmw_zenoh with shared memory off vs on for image-sized
`sensor_msgs/Image` messages. None needs hardware. Each starts its own Zenoh router on
`127.0.0.1:7457` with `ROS_DOMAIN_ID` 42, so it never joins a rig graph on 7447 — but it loads the
CPU for a few minutes, so **never run it while an experiment is running on the PC** (Studio run,
FicTrac, BIAS or ScanImage acquisition).

| script | subscriber | use it for |
|---|---|---|
| `shm_bench.py` | rclpy | the original (Mac, 2026-10-21); latency stamps from `time.monotonic_ns()` |
| `shm_bench_qpc.py` | rclpy | the same with `time.perf_counter_ns()` stamps — **use this one on Windows** |
| `mcap_bench.py` | rosbag2 recorder (C++, MCAP) | what a rig recorder sees: loss, latency, recorder CPU |

**Why the `_qpc` copy on Windows:** with Python 3.12, `time.monotonic()` is `GetTickCount64()`, which
ticks every 15.625 ms, so every latency comes out as 0, 16, 31, … ms. `perf_counter_ns()` is
QueryPerformanceCounter counted from boot, the same in every process (on macOS and Linux it is
the system monotonic clock, so the copy is fine there too). Publish pacing still uses
`time.monotonic()`, so rows above ~64 Hz may send in small bursts.

## Running it on another Windows PC

1. Nothing rig-related running; port free: `netstat -ano | findstr ":7457"` prints nothing.
2. The default pixi environment is installed (`ros2-poc\.pixi\envs\default`); if not, see
   `WINDOWS-TEST.md` step 1a (`pixi install --locked`).
3. PowerShell in `ros2-poc`, with `<host>` = this PC's name (about 7 minutes in all):

```powershell
pixi run python shm_bench_qpc.py run --out <host>_shm_qpc.json
pixi run python shm_bench_qpc.py run --sizes 1048576,2097152,8388608 --rates 30,135,30 --out <host>_shm_qpc_repeat.json
pixi run python shm_bench_qpc.py run --sizes 8388608,8388608 --rates 100,200 --duration 5 --out <host>_shm_qpc_stress.json
pixi run python mcap_bench.py run --out <host>_mcap.json
pixi run python mcap_bench.py run --cells 8388608@100,8388608@200 --duration 5 --out <host>_mcap_stress.json
```

`mcap_bench.py` writes each bag to `%TEMP%\mcap_bench` (about 2.4 GB per 8 s cell, up to 8 GB per
stress cell) and deletes it after counting; `--bag-dir` tests another disk, `--keep-bags` keeps the
bags, `--preset fastwrite` uses rosbag2's no-CRC, no-index MCAP profile.

4. Afterwards `tasklist | findstr rmw_zenohd` must print nothing. Leave the JSON files untracked and
   send them back with the printed tables.

## What to check

- **Shared memory engaged:** the last column ("shm map") is each process's mapped shared memory after
  its run. With shared memory on it is about 49 MB higher than off (ww18: 8.7M → 58.0M; Mac:
  48K → 49.3M). If the on and off rows show the same value, shared memory did not engage: report it.
- **Loss:** `recv/sent` (`bagged/sent` for the recorder) and the `rec log:` lines, which say whether
  rosbag2 lost messages on the transport layer or in its cache.
- **Latency:** p50 / p99 / max. For `mcap_bench.py` this is rmw publish → recorder receive (MCAP
  `publish_time` → `log_time`); the Python-side conversion is in the `pub()` column.

## Reference: ww18 (Core Ultra 9 285K, Windows 11 25H2, 2026-10-21/22)

Median latency, shared memory off → on (two runs each):

| message | rclpy subscriber | rosbag2 recorder |
|---|---|---|
| 1 MB @ 30 Hz | 3.6 → 1.6–1.9 ms | — |
| 2 MB @ 135 Hz | 5.1–5.8 → 3.0–3.6 ms | 1.3 → 0.11–0.12 ms |
| 8 MB @ 30 Hz | 20–23 → 11–12 ms | 5.2–5.6 → 0.18–0.19 ms |

With shared memory off, p99 was 25–67 ms and both subscribers lost messages (RELIABLE, depth 10)
at 2 MB @ 135 Hz and 8 MB @ 30 Hz; with it on, p99 was 3–14 ms (rclpy) or at most 0.4 ms
(recorder) and nothing was lost below 8 MB @ 200 Hz. The recorder costs about 1.1 ms of CPU per MB (31 % of a core for
2 MB @ 135 Hz). The rclpy subscriber, not the transport, sets the Python numbers: about 8–10 ms of
CPU per 8 MB message, 3–4 times the Mac's.
