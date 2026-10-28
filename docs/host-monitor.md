> Moved from webDisplayTools `docs/development/rig-host-monitor.md` into rig-nodes with the node (0.3.0, 2026-10-28). The arena half (the gateway recording `/rig/host`, `rig.py --host`) stays in webDisplayTools.

# Rig host monitor — the PC's "pressure" in every run

Status: v1, 2026-10-28 (`rig_nodes.host_sampler` + `rig_nodes.host_monitor`, ww18). The message
spec sent to rig-nodes: `webDisplayTools rig/docs/host-stats-request.md (the request) / src/rig_interfaces/msg/HostStats.msg (normative)`.

## What and why

A generic node that publishes how loaded the rig PC is, like Task Manager or Performance Monitor
but recorded: CPU per core, memory, disk, network, GPU, and the processes responsible. It knows
nothing about FicTrac or the arena. `pixi run rig` starts it by default (`--no-host-monitor` turns
it off) because it costs little enough to leave on (§4). The arena gateway records `/rig/host` into
every run's MCAP bag, so a run that went wrong carries the PC's state alongside it, and a question
like "does Foxglove cost anything" is answered by comparing bags instead of a separate experiment.

The method is atlas-bench `tools/cpu_sampler.py` (commit 306fa6d, used on ww18 for the BIAS
CPU-spike work): cycle counters, 100 ms CPU sub-windows, one system call for all processes.

## What one sample holds (1 Hz, `rig_interfaces/HostStats`)

| area | fields | source (Windows · elsewhere) |
|---|---|---|
| CPU | per-core busy, window mean **and** busiest 100 ms sub-window; whole-PC busy cores (mean, peak); P/E-core class per CPU; how late the sampler woke (a PC-wide stall); context switches, interrupts, DPCs per s | idle-cycle counters (exact) + `GetSystemCpuSetInformation` + kernel DPC counts · psutil (tick-based) |
| memory | total, used, available, page file used, hard page faults per s | psutil + Σ per-process hard faults · psutil (no hard faults) |
| storage | per physical disk: read/write MB/s, IOPS, busy fraction; free space on the run-log volume | psutil |
| network | per adapter that is up (hardware, not loopback): rx/tx MB/s, errors + drops | `GetIfEntry2` per adapter · psutil |
| GPU | per NVIDIA GPU: core + memory-controller utilization, memory, NVENC/NVDEC load, temperature, power, P-state, throttle reasons | the driver's NVML through ctypes (no package) |
| processes | top 3 by CPU, by memory, by disk I/O (`--top`), plus one summed row per pinned group (router, FicTrac, BIAS `test_gui.exe`, MATLAB, browser, Foxglove app, foxglove_bridge, each graph node as `py:<script>`) | one `NtQuerySystemInformation` call · psutil scan |
| cost | `monitor_cpu_cores`: the node's own CPU over the window | its process cycle counter |

Granularity: CPU every 100 ms, published as mean + peak per 1 s window, because Task Manager's 1 s
all-core average hides a single thread pinned for a few hundred ms (cpu_sampler's BIAS finding: one
core at 100 % for 6.4 s read as ~4 %). Everything else is a 1 s rate from counter deltas.
`--period` and `--sub` change both.

Not measured, deliberately: temperatures beyond the GPU's (Windows needs a vendor source), loopback
traffic (Windows counts none, so Zenoh, the FicTrac UDP stream and the Studio WebSocket are not in
`nets`), per-thread CPU (cpu_sampler's job when one process needs a closer look), anything
arena-specific (the bridge and the Studio's telemetry own that).

## Where it lives

Generic, so it belongs in reiserlab/rig-nodes (`rig_nodes.host_monitor`, docs/development/
rig-software-repos-and-runners.md §3). It is prototyped in `rig/nodes/` because ww18 cannot push
there and is the PC the cost has to be measured on. It moves unchanged: `host_sampler.py` has no
ROS and no import from this repo; `host_monitor.py` imports only rclpy, std_msgs and the sampler.

The typed message needs `rig_interfaces` ≥ 0.3.0 (rig-nodes release in progress, win-64 build
included). Until that is installed (`pixi install -e typed`), the node publishes `std_msgs/String`
JSON with the same field names (NaN → null, floats to 3 decimals), the `--typed auto` pattern the
FicTrac adapter uses. The gateway picks the same type for its recording subscription.

## Cost and budget

Budget: ≤ 5 % of the PC's CPU and of the bag's size (the lab's limit), with a tighter self-check in
the acceptance harness: the node's own CPU ≤ 2 % of ONE core (median), `/rig/host` ≤ 5 % of the
run's compressed payload.

Measured on ww18 (Core Ultra 9 285K, 24 logical CPUs, ~300 processes), per 1 s window:

| piece | cost |
|---|---|
| 10 × per-core idle cycles | 0.06 ms each |
| process table (one call) + top-N selection | 1.9 ms (a psutil scan took 694 ms) |
| network, 3 adapters via `GetIfEntry2` | 0.19 ms (psutil: 5.3 ms) |
| memory, disks, log volume, GPU | < 0.3 ms together |
| **node total** | **≈ 0.85–0.9 % of one core ≈ 0.04 % of the PC** (the first window after start is higher: one-time cycle-rate calibration and process-name lookups) |

In the acceptance harness (the whole graph + simulators on ww18): median **0.9 % of one core**, max
3.7 % (a window with new processes to name).

Storage (String JSON, the form running today): 4.6 KB per sample raw, **≈ 1.0 MB/h zstd**, **4.9 %**
of the harness run's compressed payload. That run is a worst case for this ratio: ~14 pinned Python
processes (simulators, emulator, the harness itself) each get a row, and the bag is small (simulated
FicTrac at 50 Hz). Process rows are about half the bytes and the per-core arrays most of the rest;
the defaults (top 3 per list, floats to 3 decimals) came from that measurement — top 5 and 4
decimals measured 6.4 %. The typed form is re-measured when rig-nodes 0.3.0 is installed.

## Tests

- `tests/test_host_sampler.py` (root suite, CI, no ROS or psutil needed): every field and type
  against the `.msg` text in `webDisplayTools rig/docs/host-stats-request.md (the request) / src/rig_interfaces/msg/HostStats.msg (normative)`, counter-delta rates, CPU mean vs
  peak, pinned groups and top-N selection, strict JSON.
- `rig/tests/test_ros2_poc.py` (`cd rig && pixi run test`): the node runs in the graph; the bag
  holds ≥ 0.8 samples per second with exactly the spec's fields, per-core sanity, the graph's nodes
  pinned by script, and the cost and storage gates above.
