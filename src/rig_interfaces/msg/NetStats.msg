# One network adapter's activity over a HostStats window (rig_nodes.host_monitor). Absent value: NaN / 0.
string name                     # adapter name as the OS reports it
float32 rx_mb_per_s
float32 tx_mb_per_s
uint32 errors                   # receive + transmit errors and drops in the window
