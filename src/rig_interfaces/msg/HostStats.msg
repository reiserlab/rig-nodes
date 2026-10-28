# One host-pressure sample (rig_nodes.host_monitor, /rig/host, default 1 Hz). CPU is measured from
# cycle counters every 1/sub_samples of the window (Windows: QueryIdleProcessorCycleTime, exact; Task
# Manager's tick-based numbers step at 15.6 ms) and reported as the window mean AND the busiest
# sub-window, so a short burst on one core is not averaged away. Absent value: NaN (float) / 0 (int).
int64 t_ms                      # wall clock at the END of the window (ms since the Unix epoch)
float32 window_s                # window length, s (1.0 at 1 Hz)
uint32 sub_samples              # CPU sub-windows in this window (10 at the default 10 Hz)
string hostname
# CPU
float32 cpu_busy_cores_mean     # whole-PC busy cores (0..n_cpu), mean over the window
float32 cpu_busy_cores_peak     # busiest sub-window
float32[] core_busy_mean        # per logical CPU, 0..1
float32[] core_busy_peak        # per logical CPU, busiest sub-window, 0..1
uint8[] core_class              # efficiency class per logical CPU (higher = faster: P- vs E-cores); empty if unknown
float32 sampler_late_ms_max     # how late the monitor woke for a sub-window (a PC-wide stall or starvation)
float32 context_switches_per_s
float32 interrupts_per_s
float32 dpcs_per_s              # deferred procedure calls (Windows; soft interrupts elsewhere)
# memory
float32 mem_total_mb
float32 mem_used_mb
float32 mem_available_mb
float32 pagefile_used_mb
float32 hard_page_faults_per_s  # page reads from disk (paging pressure)
# storage of the run log
string log_volume               # the volume holding the run logs / bags
float32 log_volume_free_gb
# devices and processes
DiskStats[] disks               # one per physical disk
NetStats[] nets                 # one per network adapter that is up (Windows does not count loopback)
GpuStats[] gpus                 # one per NVIDIA GPU (NVML); empty without one
ProcStats[] procs               # top N by CPU, by memory and by disk I/O, plus pinned names; deduplicated
float32 monitor_cpu_cores       # this node's own CPU over the window (its cost, always reported)
