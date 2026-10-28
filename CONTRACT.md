# The graph contract

What every process on a Reiser Lab rig graph may rely on: topic names, QoS, the adapter envelope,
the command/reply pattern, the fields of the typed messages, the semantic invariants behind them
(units, directions, counter resets, clock domains) and the rules for changing any of it. This is
item 1(b) of the cross-repository checklist in webDisplayTools'
`docs/development/rig-software-repos-and-runners.md` §7 and plays the role that
`telemetry-logging-reference.md` plays for the arena run-log rows. The message definitions
themselves are `src/rig_interfaces/msg/*.msg`; the envelope is `rig_nodes._common.wrap_line`;
the FicTrac line grammar is `rig_nodes.fictrac`. Where this file and the code disagree, the code
is a bug and this file is the authority. Version: package set 0.4.0 (unreleased; 0.3.0 2026-10-28).

## 1. Middleware

- ROS 2 Jazzy (RoboStack), `rmw_zenoh_cpp`. One Zenoh router per host on `tcp/127.0.0.1:7447`
  (`rig_nodes/config/zenoh-router.json5`); sessions connect to it (`rig_nodes/config/zenoh-session.json5`); `rig_nodes.zenoh` applies them to the environment. Both
  configs are the stock rmw_zenoh files with only the endpoint lines changed to loopback, so a
  rig's graph is **not reachable from the lab network**. The graph is **single-host**: no
  contract below holds across machines (clock comparisons least of all) until host identity
  and clock alignment exist (checklist item 7).
- Under rmw_zenoh a typed topic is keyed by its type name **and the hash of the `.msg` text**.
  Two processes with different builds of `rig_interfaces` do not see each other's typed topics.
  Hence §7: a message change is a wire-contract change.

## 2. Topics

Names are absolute and fixed. `<src>` is a short lowercase device identity (`fictrac`,
`kinefly`, `bias`), the same string the envelope carries in `src`. Device nodes kept outside this
repository follow the same rules under their own `<src>` and own their vocabulary there (the 2P
rig's ScanImage connector, `/scanimage/cmd|reply|status`, moved out in 0.4.0).

| topic | type | QoS | producer | notes |
|---|---|---|---|---|
| `/<src>/raw` | `std_msgs/String` (envelope, §3) | RELIABLE, KEEP_LAST 200, VOLATILE | stream adapters: `/fictrac/raw`, `/kinefly/raw` | one message per device record, bytes untouched |
| `/fictrac/frame` | `rig_interfaces/FicTracFrame` | RELIABLE, KEEP_LAST 200, VOLATILE | FicTrac adapter when `rig_interfaces` is importable (`--typed auto`) | the parsed line beside the verbatim one; `/fictrac/raw` keeps publishing unless `--typed only` |
| `/<src>/cmd` | `std_msgs/String` (JSON, §4) | RELIABLE, KEEP_LAST 200 (publishers may use 50), VOLATILE | an executor (gateway, CLI tool, runner) | one command per message |
| `/<src>/reply` | `std_msgs/String` (JSON, §4) | RELIABLE, KEEP_LAST 200, VOLATILE | the device node | exactly one reply per command |
| `/<src>/status` | `std_msgs/String` (JSON, §4) | RELIABLE, KEEP_LAST 200, VOLATILE | the device node, every `--status-every` s | reachability + the device's own snapshot |
| `/bias/frames` | `std_msgs/String` (JSON) | RELIABLE, KEEP_LAST 200, VOLATILE | BIAS node after `stopRecording` | every frame's camera timestamp from BIAS's stamp log, chunked |
| `/rig/host` | `rig_interfaces/HostStats`, or `std_msgs/String` JSON with the same field names | RELIABLE, KEEP_LAST 10, VOLATILE | `rig_nodes.host_monitor` (default 1 Hz; `--typed auto\|on\|off`, `--py-root` names a rig's own script directory for the process groups) | one host-pressure sample per window (§5a); recorded by the arena gateway; absent values NaN (float) / 0 (int) |
| `/arena/frame` | `rig_interfaces/ArenaFrame`, or `std_msgs/String` JSON with the same field names | BEST_EFFORT, KEEP_LAST 1, VOLATILE | the arena gateway (webDisplayTools `rig/nodes/bridge_node.py`) | one closed-loop evaluation per FicTrac frame (§5); live viewing only, the recorder writes its own copy |
| `/arena/browser`, `/arena/event`, `/arena/session` | `std_msgs/String` JSON | recorded only (never published) | the arena gateway | gateway records in the bag; format authority: webDisplayTools `telemetry-logging-reference.md` |

Device nodes today use **one fixed instance per graph** (`/bias/*` means *the* BIAS). Two cameras
would both answer every command. Instance namespacing (`/<src>/<instance>/…`, routing by plugin
name, producer-restart identity, a two-camera isolation test) is LAB-340 and is **not** part of
this contract version; until it lands a graph runs at most one node per `<src>`.

## 3. The adapter envelope

Every `/<src>/raw` message is this compact JSON object and nothing else:

```json
{"src":"fictrac","seq":1234,"rx_epoch_ms":1761512345678,"rx_mono_ns":987654321000,"raw":"FT, 1234, …"}
```

| field | type | meaning |
|---|---|---|
| `src` | string | the device identity; equals the `<src>` of the topic |
| `seq` | integer | the adapter's own counter, **1-based, per adapter process**; restarts at 1 when the adapter restarts (a gap or a reset = the adapter, not the device, was lost) |
| `rx_epoch_ms` | integer | the adapter's wall clock at receipt, ms since the Unix epoch (precise on Windows: `_common.wall_ms`) |
| `rx_mono_ns` | integer | the adapter's monotonic clock at receipt, ns; comparable only with other monotonic stamps **on the same host** |
| `raw` | string | the datagram text verbatim, without the trailing newline; never parsed or normalized by the adapter |

The adapter moves bytes and stamps when it saw them. The one exception is protocol discrimination
where a wrong datagram would corrupt downstream parsing: the Kinefly adapter drops (and counts)
datagrams that are not KF/1 (`protocol: "Kinefly"`, `version: 1`). Nothing is ever padded from one
device's record into another's fields.

## 4. Commands, replies, status

**Command** on `/<src>/cmd`: `{"id": <any JSON>, "command": <name>, "params": {…}}`. `id` is
minted by the sender and echoed verbatim; the node never interprets it. `command` is the node's
own vocabulary (each device node owns it — BIAS: `connect`, `startRecording`, `stopRecording`,
`startPreview`, `stopCapture`, `getTimestamp`, `disconnect`). The protocol plugin registries in
webDisplayTools and MATLAB are clients of that vocabulary, not its source.

**Reply** on `/<src>/reply`, exactly one per command:

```json
{"id": …, "command": "startRecording", "ok": true, "outcome": "ok",
 "value": …, "error": "", "steps": [ {…per device request…} ],
 "rtt_ms": 12.3, "t_ms": 1761512345678, "rx_epoch_ms": 1761512345690,
 "artifact": {"kind": "video", "path": "D:/rec/trial_001.avi"}}
```

| field | meaning |
|---|---|
| `outcome` | **closed vocabulary**: `ok` (the device confirmed), `error` (the device or the node refused; `error` says why), `unknown` (a timeout: the command **may have taken effect**) |
| `ok` | `outcome == "ok"` |
| `value` | the device's answer, opaque to the node (its shape is the device's) |
| `steps` | the device-level requests the command expanded into, each with its own result and `rtt_ms` |
| `rtt_ms` | wall time the whole command took in the node |
| `t_ms` | the node's wall clock when the command started (ms since the epoch) |
| `rx_epoch_ms` | when the node received the command |
| `artifact` | present when the command created an output the record must name: `{"kind": "video"\|"tiff", "path", …}` |

After an `unknown` outcome the node **reconciles** (a status read) before it executes anything
else; the executor decides what an unknown outcome means for the run (`critical`, `on_loss` are
executor policy, never node policy). Typed replies use `rig_interfaces/ConnectorReply`, whose
`*_json` fields hold the same `value`, `artifact` and `steps` as JSON text.

**Status** on `/<src>/status`, every `--status-every` seconds: `{"reachable": true, "t_ms": …,
"rtt_ms": …, …the device's own snapshot…}` or `{"reachable": false, "t_ms": …, "error": "…"}`.
Device fields are passed through under the device's names (BIAS: `connected`, `capturing`,
`logging`, `frameCount`, `framesPerSec`, `videoFile`). A counter the device does not
report is **absent**, never 0.

## 5. `/arena/frame`

One message per FicTrac frame: the frame index the display was asked to show, evaluated by the
closed-loop node, plus the FicTrac columns the lab keeps. Since 2026-10-18 it is also **the**
FicTrac record in a run's bag (the raw FicTrac topics are recorded only on request). Fields
(`src/rig_interfaces/msg/ArenaFrame.msg` is normative; the String form uses the same names):

| field | type | meaning |
|---|---|---|
| `t_ms` | int64 | the evaluating node's wall clock at evaluation, ms since the epoch — **the** time of record; a recorder's receive time lags it |
| `ms` | int32 | the same instant relative to the run's t0 (the run log's `ms` column) |
| `fc` | int64 | FicTrac frame counter (col 1); 0 when an untracked frame carried none |
| `idx` | int32 | displayed frame index, 0-based |
| `has_ft`, `ft` | bool, float64 | FicTrac's camera timestamp relative to the run's first frame, ms; `has_ft` false when col 22 was absent |
| `x`, `y`, `hd` | float64 | integrated position (cols 15, 16) and heading (col 17), radians |
| `epoch` | uint32 | +1 per heading tare; 0 before the first |
| `bias_deg` | float64 | the bias angle applied at this evaluation; 0 when none |
| `seq` | uint64 | the gateway writer's emission order within the run; 0 when unknown |
| `turning_dps`, `forward_mms` | float64 | derived live-viewing channels (two-point differences on `ft`, else on `t_ms`); **NaN in the bag** since 2026-10-18 (recomputable; 16 B/frame otherwise) |
| `err`, `dr_lab_x/y/z`, `ft_seq`, `ft_ns` | float64 ×4, int64 ×2 | FicTrac cols 5, 6–8, 23, 22 as received (NaN / 0 when absent) |
| `flags` | uint8 | bit 0: untracked frame (an `FT_BADFR` line): no columns, no run-log row, but the bag says "untracked", not "missing" |

## 5a. `/rig/host`

One host-pressure sample per window (`rig_interfaces/msg/HostStats.msg` is normative; the String form
uses the same names): wall clock at the END of the window (`t_ms`), `window_s`, `sub_samples`,
`hostname`; CPU as whole-PC busy cores and per logical CPU, each as the window MEAN and the busiest
sub-window PEAK (`core_busy_mean`, `core_busy_peak`, 0..1), the P/E-core class per CPU (`core_class`,
empty when unknown), the sampler's own lateness (`sampler_late_ms_max`), context switches / interrupts /
DPCs per second; memory and page file in MB, hard page faults per second; the run-log volume and its
free space; and four arrays of record types — `DiskStats[] disks`, `NetStats[] nets`, `GpuStats[] gpus`
(NVML, incl. NVENC/NVDEC utilization), `ProcStats[] procs` (top N by CPU / memory / disk I/O plus
pinned names, deduplicated; `reasons` bitmask 1 CPU, 2 memory, 4 disk I/O, 8 pinned) — and the
monitor's own cost (`monitor_cpu_cores`). Absent values follow §6: NaN for a float, 0 for an integer,
`null` in the String form. Request and field rationale: webDisplayTools `rig/docs/host-stats-request.md`
(2026-10-28).

## 6. Semantic invariants

**Units.** Angles from FicTrac are radians; `x`, `y` are radians of ball rotation (multiply by
the ball **radius** for distance; the gateway's `--ball-mm` is the ball **diameter**, 9 mm by
default). `*_ms` fields are milliseconds; `rx_epoch_ms` and `t_ms` since the Unix epoch, `ms` and
`ft` relative to the run. `rx_mono_ns` and `ft_ns` are nanoseconds. `idx` is a 0-based frame
index of the displayed pattern. Degrees appear only in derived channels (`bias_deg`,
`turning_dps`) and are named so.

**Directions.** FicTrac's integrated heading is a lab-frame rotation about +z and **increases when
the animal turns left** (counter-clockwise seen from above); `turning_dps > 0` is the same sense.
Which way the display moves for a positive index step is an arena property (webDisplayTools
`closed-loop-bias.md`), not part of this contract.

**Counters and resets.** Adapter `seq`: 1-based per adapter process; a restart resets it. FicTrac
`fc` (col 1): monotonic within a FicTrac session; a FicTrac restart restarts it. `ft_seq` (col 23):
equals `fc` until FicTrac resets its tracking, then restarts at 1. Gateway `seq`: emission order
within one run, restarts per run. `epoch`: per run, 0 before the first tare. Kinefly `sequence`:
per Kinefly session (`session` UUID changes on a restart). None of these is durable across process
restarts; a reader that needs continuity joins on time, not on counters.

**Clock domains** (three, as in the arena run log): (1) **device clocks** — FicTrac col 22 (a ns
camera clock on our rigs; FicTrac's doc says ms), Kinefly `timestamp_s`, BIAS's `timeStamp`;
(2) **adapter host clocks** — `rx_epoch_ms` (wall) and `rx_mono_ns` (monotonic) at receipt;
(3) **evaluator clock** — `t_ms` / `ms`, the gateway's wall clock at evaluation. In a bag, MCAP
`log_time` is the recorder's receive wall clock and `publish_time` is the producer's own stamp
when it had one (the gateway writes `t_ms` there). Monotonic stamps compare only within one host;
wall clocks on one host are one clock. Windows: every wall stamp comes from
`GetSystemTimePreciseAsFileTime` and every monotonic one from `QueryPerformanceCounter`
(`rig_nodes._common`), never from the 15.6 ms tick.

**Absent values.** Typed messages carry NaN for an absent float and 0 for an absent integer (a
`.msg` field cannot be null); JSON payloads carry `null`. The Foxglove rewriter maps NaN to `null`
because JSON has no NaN.

**Loss accounting.** An adapter counts what it received and what it published (and what it
rejected, for Kinefly); a device node reports `reachable` in every status; the recorder counts
records written and records dropped while closed. A consumer that needs "was anything lost" reads
these, not the absence of a message.

## 7. Evolution rules

1. **A `.msg` change is a wire-contract change.** Under Zenoh type hashing every process must run
   the same `rig_interfaces` build. Any `.msg` edit bumps the package-set version and goes
   through a release (§8 of the README). Append fields at the end; never rename, reorder or
   retype a field; a new meaning is a new field or a new message.
2. **JSON payloads evolve by addition only.** A new key may appear; an existing key never changes
   its meaning, unit or type (a new meaning gets a new key). Consumers ignore unknown keys.
   The envelope's five keys are fixed.
3. **Topic names are stable.** A new device adds `/<src>/raw` (streams) or `/<src>/cmd|reply|status`
   (commands) with a new `<src>`; an existing topic never changes type outside a package-set
   release that says so.
4. **The `outcome` vocabulary is closed** (`ok | error | unknown`). A node that cannot express a
   result in it reports `error` with a message rather than inventing a fourth value.
5. **One RoboStack set per platform.** The packages pin `ros2-distro-mutex` (0.19 and Python 3.12 on all three
   platforms as of 0.2.1; macOS ≥ 14). Moving the pin is a
   package-set release and every rig re-locks.
6. **One package-set version.** `rig_interfaces` and `rig_nodes` are released together under one
   tag (`vX.Y.Z`), tested together; a consumer pins that version and records it with every run
   (checklist item 7, done in the consumer).
7. **Deprecations retire on evidence**, not after "one release": rigs sit unchanged for months.
   A retired field or topic keeps being accepted by readers until no deployed producer emits it.
8. **This file changes with the code**, in the same pull request, with the package-set version
   at the top updated.
