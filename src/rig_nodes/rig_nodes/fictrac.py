"""fictrac.py — what FicTrac's socket output looks like, in ONE place (pure, no ROS).

FicTrac (https://github.com/rjdmoore/fictrac) streams one record per camera frame as a
comma-separated line: an optional tag (`FT` = tracked, `FT_BADFR` = the frame could not be
tracked; offline .dat lines and `sims/fictrac_sim.py` carry none) followed by 25 numeric columns
(doc/data_header.txt). The UDP adapter publishes the line verbatim and, on the typed topic, the
parsed `(tag, fields)` from `parse_fictrac_line`; every downstream consumer (the arena gateway's
closed-loop pipeline, its run-log converter, the Foxglove rewriter) parses the SAME way, so a
column added upstream cannot be read differently by two of them.

Units and directions (CONTRACT.md "FicTrac"): angles in radians; the integrated heading (col 17)
is a lab-frame rotation about +z and INCREASES when the animal turns left (counter-clockwise from
above); x/y (cols 15-16) are integrated position in radians of ball rotation (multiply by the
ball radius for mm); col 22 is the camera timestamp (ns on our rigs, ms per FicTrac's doc); col 23
is the sequence counter, equal to the frame counter until FicTrac's tracking is reset.
"""

from __future__ import annotations

# FicTrac's 25 output columns by name (1-based positions in the comments), per doc/data_header.txt.
FICTRAC_COLS = [
    "frame",                                    # 1  frame counter (starts at 1)
    "dr_cam_x", "dr_cam_y", "dr_cam_z",         # 2-4  delta rotation vector, camera frame (rad)
    "dr_err",                                   # 5  delta rotation error score
    "dr_lab_x", "dr_lab_y", "dr_lab_z",         # 6-8  delta rotation vector, lab frame (rad)
    "r_cam_x", "r_cam_y", "r_cam_z",            # 9-11 absolute rotation vector, camera frame
    "r_lab_x", "r_lab_y", "r_lab_z",            # 12-14 absolute rotation vector, lab frame
    "x", "y",                                   # 15-16 integrated x/y position, lab (rad)
    "heading",                                  # 17 integrated animal heading, lab (rad)
    "direction",                                # 18 animal movement direction, lab (rad)
    "speed",                                    # 19 animal movement speed (rad/frame)
    "forward", "side",                          # 20-21 integrated forward / side motion (rad)
    "timestamp",                                # 22 frame timestamp (ms per the doc; ns on our rigs)
    "seq",                                      # 23 sequence counter
    "dt",                                       # 24 delta timestamp (ms)
    "alt_timestamp",                            # 25 alternative timestamp (ms)
]

TAG_TRACKED = "FT"


def parse_fictrac_line(raw: str) -> tuple[str, list[float]]:
    """(tag, fields): a leading FT* token is the tag ('' when absent, as in offline .dat lines
    and the simulator); fields = the floats after it, or [] when the line is not numeric (a bad
    frame has no usable data)."""
    parts = raw.strip().split(",")
    tag = ""
    if parts and parts[0].strip()[:2].upper() == "FT":
        tag = parts[0].strip()
        parts = parts[1:]
    try:
        return tag, [float(p) for p in parts]
    except ValueError:
        return tag, []


def is_bad_frame_tag(tag: str) -> bool:
    """FicTrac's socket output tags every record: "FT" = tracked, anything else starting with
    FT (seen: "FT_BADFR") = a frame it could not track."""
    return tag[:2].upper() == "FT" and tag != TAG_TRACKED


def columns_by_name(fields: list[float]) -> dict[str, float]:
    """The numeric columns keyed by FICTRAC_COLS (a short line gives fewer keys)."""
    return {k: v for k, v in zip(FICTRAC_COLS, fields)}
