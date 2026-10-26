# One closed-loop evaluation: the frame index the display is asked to show for one FicTrac
# frame (LAB-303 Plan B, /arena/frame). THE field the record of truth needs is t_ms — the
# closed-loop node's OWN sample time, the instant idx, the bias phase and a pending tare were
# evaluated. A recorder's receive time lags it; offline reconstruction must use t_ms / ms.
int64 t_ms            # the node's wall clock at evaluation (ms since the Unix epoch)
int32 ms              # the same instant relative to the run's t0 (behavior_v1 column `ms`)
int64 fc              # FicTrac frame counter (col 1)
int32 idx             # displayed frame index (0-based)
bool has_ft           # false when FicTrac's col-22 timestamp was absent
float64 ft            # FicTrac camera timestamp relative to the first frame, ms (behavior_v1 `ft`)
float64 x             # integrated x position, radians (col 15)
float64 y             # integrated y position, radians (col 16)
float64 hd            # integrated heading, radians (col 17)
uint32 epoch          # +1 per heading tare; 0 before the first
float64 bias_deg      # bias angle applied at this evaluation (0 when the bias is `none`)
uint64 seq            # the bridge writer's emission order within the run (0 when unknown)
# Derived channels for live viewing (Foxglove) and ROS-native consumers ONLY — the hub message
# to the Studio and the behavior_v1 columns do not carry them. Two-point differences over
# consecutive frames on the FicTrac camera clock `ft` (falling back to the bridge clock when
# ft is absent), the way js/kinematics.js derives the same channels; NaN on the first frame
# of a run, when the clock does not advance, or when FicTrac restarted.
float64 turning_dps   # heading rate, deg/s (positive = increasing FicTrac heading)
float64 forward_mms   # forward velocity, mm/s (x/y step projected onto the heading × ball radius; --ball-mm, 9 mm default)
