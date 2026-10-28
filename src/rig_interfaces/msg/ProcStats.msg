# One process over a HostStats window (rig_nodes.host_monitor): the top N by CPU, memory and disk I/O
# plus the pinned names, deduplicated. Absent value: NaN / 0.
uint32 pid
string name                     # executable image name
string group                    # pinned group label ("fictrac", "bias", "py:bridge_node", …) or "" for top-N only
float32 cpu_cores               # mean over the window (1.0 = one core busy)
float32 rss_mb                  # working set
float32 io_read_mb_per_s
float32 io_write_mb_per_s
uint8 reasons                   # bitmask: 1 top CPU, 2 top memory, 4 top disk I/O, 8 pinned
