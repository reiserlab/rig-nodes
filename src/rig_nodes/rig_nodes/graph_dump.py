#!/usr/bin/env python3
"""graph_dump.py — what the running graph looks like: nodes, topics, who publishes/subscribes.

    pixi run graph                 # Mermaid diagram of the LIVE graph on stdout (paste into a .md / GitHub)
    pixi run graph -- --dot        # Graphviz DOT instead (dot -Tsvg graph.dot > graph.svg)
    pixi run graph -- --text       # plain table: topic, type, publishers, subscribers, rate (1 s sample)
    pixi run graph -- --out docs/graph.md

The ROS 2 answer to "what/how are we using ROS": the graph is discoverable at run time, so this
asks the middleware (rclpy's node/topic introspection) instead of reading our code. Run it while
`pixi run rig` is up (any environment of this workspace). The interactive equivalents are
`rqt_graph` (live, GUI) and Foxglove Studio (live via foxglove_bridge, or offline on the
MCAP bag) — both in the opt-in `viz` environment (`pixi install -e viz`).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

OWN = {"graph_dump", "_ros2cli"}  # introspection nodes to hide


def collect(node: Node, sample_s: float) -> dict:
    nodes = [(n, ns) for n, ns in node.get_node_names_and_namespaces() if not any(n.startswith(o) for o in OWN)]
    topics: dict[str, dict] = {}
    for name, typ in node.get_topic_names_and_types():
        if name.startswith("/parameter_events") or name.startswith("/rosout"):
            continue
        topics[name] = {"types": typ, "pubs": [], "subs": [], "hz": None}
    for n, ns in nodes:
        full = (ns.rstrip("/") + "/" + n) if ns != "/" else "/" + n
        for t, _ in node.get_publisher_names_and_types_by_node(n, ns):
            if t in topics:
                topics[t]["pubs"].append(full)
        for t, _ in node.get_subscriber_names_and_types_by_node(n, ns):
            if t in topics:
                topics[t]["subs"].append(full)
    if sample_s > 0:
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        from rosidl_runtime_py.utilities import get_message

        counts = {t: 0 for t in topics}
        subs = []
        for t, info in topics.items():
            try:
                cls = get_message(info["types"][0])
            except Exception:
                continue
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
            subs.append(node.create_subscription(cls, t, (lambda tt: (lambda _m: counts.__setitem__(tt, counts[tt] + 1)))(t), qos))
        end = time.monotonic() + sample_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        for s in subs:
            node.destroy_subscription(s)
        for t in topics:
            topics[t]["hz"] = round(counts[t] / sample_s, 1)
    return {"nodes": sorted(("/" + n) if ns == "/" else ns.rstrip("/") + "/" + n for n, ns in nodes), "topics": topics}


def _id(s: str) -> str:
    return s.strip("/").replace("/", "_").replace("-", "_")


def mermaid(g: dict) -> str:
    out = ["```mermaid", "flowchart LR"]
    for n in g["nodes"]:
        out.append(f'    {_id(n)}(["{n}"])')
    for t, info in sorted(g["topics"].items()):
        label = t + ("\\n" + info["types"][0].split("/")[-1] if info["types"] else "") + (f"\\n{info['hz']} Hz" if info["hz"] else "")
        out.append(f'    {_id(t)}[/"{label}"/]')
        for p in info["pubs"]:
            out.append(f"    {_id(p)} --> {_id(t)}")
        for s in info["subs"]:
            out.append(f"    {_id(t)} --> {_id(s)}")
    out.append("```")
    return "\n".join(out)


def dot(g: dict) -> str:
    out = ["digraph arena {", "  rankdir=LR; node [fontname=Helvetica];"]
    for n in g["nodes"]:
        out.append(f'  "{n}" [shape=ellipse,style=filled,fillcolor="#e8f5e9"];')
    for t, info in sorted(g["topics"].items()):
        label = t + ("\\n" + info["types"][0] if info["types"] else "") + (f"\\n{info['hz']} Hz" if info["hz"] else "")
        out.append(f'  "{t}" [shape=box,label="{label}"];')
        for p in info["pubs"]:
            out.append(f'  "{p}" -> "{t}";')
        for s in info["subs"]:
            out.append(f'  "{t}" -> "{s}";')
    out.append("}")
    return "\n".join(out)


def text(g: dict) -> str:
    rows = [("topic", "type", "publishers", "subscribers", "Hz")]
    for t, info in sorted(g["topics"].items()):
        rows.append((t, info["types"][0] if info["types"] else "?", ", ".join(info["pubs"]) or "—", ", ".join(info["subs"]) or "—", str(info["hz"]) if info["hz"] is not None else ""))
    w = [max(len(r[i]) for r in rows) for i in range(5)]
    return "nodes: " + ", ".join(g["nodes"]) + "\n" + "\n".join("  ".join(c.ljust(w[i]) for i, c in enumerate(r)) for r in rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dot", action="store_true")
    ap.add_argument("--text", action="store_true")
    ap.add_argument("--sample", type=float, default=1.0, help="seconds to sample message rates (0 = skip)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--wait", type=float, default=2.0, help="seconds to let discovery settle")
    a, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Node("graph_dump")
    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    g = collect(node, a.sample)
    node.destroy_node()
    rclpy.try_shutdown()
    s = dot(g) if a.dot else text(g) if a.text else mermaid(g)
    if a.out:
        open(a.out, "w", encoding="utf-8").write(s + "\n")
        print(f"[graph] {len(g['nodes'])} nodes, {len(g['topics'])} topics → {a.out}", file=sys.stderr)
    else:
        print(s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
