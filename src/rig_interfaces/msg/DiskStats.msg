# One physical disk's activity over a HostStats window (rig_nodes.host_monitor). Absent value: NaN.
string name                     # PhysicalDrive0 / nvme0n1 / disk0
float32 read_mb_per_s
float32 write_mb_per_s
float32 reads_per_s
float32 writes_per_s
float32 busy_frac               # 0..1, (read + write time) / window; NaN where the OS gives none
