"""rig_nodes — shared ROS 2 infrastructure for the Reiser Lab rigs (reiserlab/rig-nodes).

Device adapters and their simulators (FicTrac, Kinefly, BIAS, ScanImage), recording primitives
(MCAP recorder, indexer, CDR decoder, Foxglove JSON rewriter), supervision primitives (process
groups, port preflight, child supervision) and the clocks every node shares. The typed message
contracts live in the sibling package `rig_interfaces`; the topic and envelope contract is
CONTRACT.md at the repository root.

Importing this package pulls in nothing: each module imports what it needs (`rclpy` only in the
nodes and in `_common`; the pure modules `fictrac`, `cdr`, `supervise`, `mcap_index`,
`foxglove_json`, `bias_health`, `zmq_latency_report`, `thorsipm` run without a ROS process).
"""

__version__ = "0.3.0"
