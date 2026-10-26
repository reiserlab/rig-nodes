# One FicTrac record as the UDP adapter received it (LAB-303 Plan B, typed topic /fictrac/frame).
# The adapter parses the line; `raw` keeps the verbatim bytes so the closed-loop math can stay
# byte-for-byte bridge 3.4's (Pipeline.handle_line) and nothing is lost if a column is added.
uint64 seq            # adapter counter, 1-based
int64 rx_epoch_ms     # adapter wall clock at receipt (ms since the Unix epoch)
int64 rx_mono_ns      # adapter monotonic clock at receipt
string tag            # "FT" (good frame), "FT_BADFR" (untracked), "" (offline / simulator line)
float64[] fields      # the numeric columns after the tag (25 for FicTrac 2.x); empty when unparseable
string raw            # the line, verbatim, without the newline
