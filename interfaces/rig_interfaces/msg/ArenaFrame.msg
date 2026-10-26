# One closed-loop evaluation: the frame index the display is asked to show for one FicTrac
# frame (LAB-303 Plan B, /arena/frame). THE field the record of truth needs is t_ms — the
# closed-loop node's OWN sample time, the instant idx, the bias phase and a pending tare were
# evaluated. A recorder's receive time lags it; offline reconstruction must use t_ms / ms.
#
# Since 2026-10-18 this is also THE FicTrac record in the bag (docs/development/
# fictrac-columns-in-the-bag.md): the raw FicTrac topics are no longer recorded by default, and
# the 10 FicTrac columns the lab keeps ride here — cols 1, 15, 16, 17, 22 as before, plus
# cols 5, 6-8 and 23 below. One message per frame; one type for the live topic and the bag.
int64 t_ms            # the node's wall clock at evaluation (ms since the Unix epoch)
int32 ms              # the same instant relative to the run's t0 (behavior_v1 column `ms`)
int64 fc              # FicTrac frame counter (col 1); 0 when an untracked frame carried none
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
# of a run, when the clock does not advance, or when FicTrac restarted. In the BAG these two
# are always NaN (live-only since 2026-10-18): exactly recomputable offline, and 16 B per frame
# of incompressible doubles otherwise — a fifth of the record.
float64 turning_dps   # heading rate, deg/s (positive = increasing FicTrac heading)
float64 forward_mms   # forward velocity, mm/s (x/y step projected onto the heading × ball radius; --ball-mm, 9 mm default)
# The FicTrac columns kept beyond the behavior row (NaN / 0 when the line did not carry them).
float64 err           # col 5: delta-rotation error score (FicTrac's optimizer residual; its only per-frame quality signal)
float64 dr_lab_x      # cols 6-8: delta rotation vector in laboratory coordinates, radians/frame —
float64 dr_lab_y      #   the primary measurement everything else is integrated from
float64 dr_lab_z      #   (FicTrac: heading -= dr_lab_z; velx = dr_lab_y; vely = -dr_lab_x)
int64 ft_seq          # col 23: sequence counter — equals fc until FicTrac's tracking is reset, then restarts at 1
int64 ft_ns           # col 22 as received, truncated to an integer (a ns camera clock on our rigs; FicTrac's doc says ms)
uint8 flags           # bit 0: untracked frame (an FT_BADFR line — no columns; recorded so the bag says "untracked", not "missing"; no run-log row)
