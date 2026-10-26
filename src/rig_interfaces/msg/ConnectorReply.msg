# A device node's answer to one connector_command (LAB-303: BIAS today). Replies are typed so a
# ROS-native consumer (MATLAB, ros2 topic echo) sees named fields; the JSON-valued parts stay
# JSON strings because their shape is the device's, not ours.
string id             # the command id the browser (or bias_cmd.py) minted
string connector      # "bias"
string command        # protocol plugin command, e.g. "startRecording"
bool ok
string outcome        # "ok" | "error" | "unknown" (a timeout: the command may have taken effect)
string error          # empty when ok
string value_json     # the device's value, JSON-encoded ("null" when none)
string artifact_json  # {"kind":"video","path":…} for startRecording, else "null"
string steps_json     # per-HTTP-step results, JSON array
float64 rtt_ms
int64 t_ms            # device-node wall clock when the command started (ms since the Unix epoch)
