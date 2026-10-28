# One NVIDIA GPU over a HostStats window (rig_nodes.host_monitor, NVML). Absent value: NaN / 0.
uint32 index
string name
float32 util_frac               # 0..1, GPU core
float32 mem_util_frac           # 0..1, memory controller
float32 mem_used_mb
float32 mem_total_mb
float32 encoder_util_frac       # NVENC, 0..1 (BIAS MP4 recording)
float32 decoder_util_frac       # NVDEC, 0..1
float32 temperature_c
float32 power_w
uint8 pstate                    # 0 = maximum performance … 15; 255 = unknown
uint64 throttle_reasons         # NVML clocks-event (throttle) reason bitmask; 0 = none
