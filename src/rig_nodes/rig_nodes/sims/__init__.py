"""rig_nodes.sims — stand-ins for the device programs, so a graph runs without hardware.

* `fictrac_sim`  — FicTrac's socket output: random walk, a replayed log, or a closed-loop model fly
* `kinefly_sim`  — a synthetic KF/1 producer (mbreiser/Kinefly-MacOS `--udp`)
* `bias_sim`     — the BIAS camera program's external-control HTTP interface

Each is a plain program (`python -m rig_nodes.sims.<name> --help`); none imports ROS.
"""
