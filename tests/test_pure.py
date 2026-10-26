#!/usr/bin/env python3
"""test_pure.py — the rig_nodes modules that need no graph: envelope, FicTrac grammar, CDR
decoding, the Foxglove JSON rewriter, the MCAP indexer, the supervision primitives.

    pixi run test-pure            # in the rig-nodes checkout
    python tests/test_pure.py     # in any environment where rig_nodes is installed

Standalone harness (no pytest): every check prints PASS/FAIL, exit 0 when all pass. Runs in a
few seconds; the acceptance job runs it against the INSTALLED package in a clean directory.
"""

from __future__ import annotations

import json
import math
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, info: str = "") -> bool:
    results.append((name, bool(ok), info))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'' if ok else ' — ' + str(info)[:300]}", file=sys.stderr)
    return bool(ok)


def cdr_of_string(s: str) -> bytes:
    """std_msgs/String as rmw serializes it: CDR header 00 01 00 00, uint32 length incl. NUL, bytes, NUL."""
    b = s.encode("utf-8") + b"\x00"
    return b"\x00\x01\x00\x00" + struct.pack("<I", len(b)) + b


def test_envelope() -> None:
    from rig_nodes._common import wrap_line

    line = wrap_line("fictrac", 7, "FT, 1, 2", 1761512345678, 987654321)
    obj = json.loads(line)
    check("envelope: exactly the five contract keys, in order", list(obj) == ["src", "seq", "rx_epoch_ms", "rx_mono_ns", "raw"], line)
    check("envelope: compact JSON (no spaces outside raw)", " " not in line.replace(obj["raw"], ""), line)
    check("envelope: raw verbatim", obj["raw"] == "FT, 1, 2" and obj["seq"] == 7)


def test_fictrac() -> None:
    from rig_nodes.fictrac import FICTRAC_COLS, columns_by_name, is_bad_frame_tag, parse_fictrac_line

    check("fictrac: 25 named columns", len(FICTRAC_COLS) == 25 and FICTRAC_COLS[16] == "heading" and FICTRAC_COLS[21] == "timestamp")
    tag, f = parse_fictrac_line("FT, 12, 0.1, 0.2, 0.3, 0.01, 0.4, 0.5, 0.6, 0,0,0, 0,0,0, 1.5, -2.5, 3.14, 0, 0, 0, 0, 1761512345678000000, 12, 10, 0")
    check("fictrac: tagged line → tag FT + 25 floats", tag == "FT" and len(f) == 25 and f[0] == 12 and abs(f[16] - 3.14) < 1e-9, (tag, len(f)))
    check("fictrac: columns_by_name maps col 17 → heading", columns_by_name(f)["heading"] == f[16] and columns_by_name(f)["frame"] == 12)
    tag, f = parse_fictrac_line("1, 0.1, 0.2")
    check("fictrac: untagged (simulator / .dat) line → tag ''", tag == "" and f == [1.0, 0.1, 0.2])
    tag, f = parse_fictrac_line("FT_BADFR, nan, x")
    check("fictrac: FT_BADFR → bad-frame tag, no fields", is_bad_frame_tag(tag) and f == [], (tag, f))
    check("fictrac: FT is not a bad frame; '' is not a bad frame", not is_bad_frame_tag("FT") and not is_bad_frame_tag(""))


def test_cdr() -> None:
    from rig_nodes.cdr import cdr_string, decode_cdr, parse_msgdef

    check("cdr: std_msgs/String round trip", cdr_string(cdr_of_string('{"a":1}')) == '{"a":1}')
    check("cdr: decode_cdr without a schema name = String", decode_cdr(None, None, cdr_of_string("hi")) == {"data": "hi"})
    # a small typed message: int64 t_ms, int32 ms, bool has_ft, float64 ft, string tag, float64[] fields
    schema = "int64 t_ms   # comment\nint32 ms\nbool has_ft\nfloat64 ft\nstring tag\nfloat64[] fields\n"
    fields = parse_msgdef(schema)
    check("cdr: parse_msgdef strips comments, keeps order", [n for _, n in fields] == ["t_ms", "ms", "has_ft", "ft", "tag", "fields"], fields)
    body = struct.pack("<q", 1761512345678) + struct.pack("<i", 1234) + struct.pack("<?", True) + b"\x00" * 3  # align to 8
    body += struct.pack("<d", 2.5)
    tag = b"FT\x00"
    body += struct.pack("<I", len(tag)) + tag + b"\x00"  # pad to 4 for the next uint32
    body += struct.pack("<I", 2) + b"\x00" * 4 + struct.pack("<dd", 1.0, -1.0)  # sequence length, align 8, two doubles
    data = b"\x00\x01\x00\x00" + body
    try:
        out = decode_cdr("x/msg/Y", schema, data)
        ok = out["t_ms"] == 1761512345678 and out["ms"] == 1234 and out["has_ft"] is True and out["ft"] == 2.5 and out["tag"] == "FT" and out["fields"] == [1.0, -1.0]
        check("cdr: decode_cdr aligns primitives, strings and sequences (XCDR1)", ok, out)
    except Exception as exc:
        check("cdr: decode_cdr aligns primitives, strings and sequences (XCDR1)", False, repr(exc))
    try:
        decode_cdr("x/msg/Z", "geometry_msgs/Point p\n", data)
        check("cdr: unsupported field type is refused loudly", False, "no error")
    except ValueError as exc:
        check("cdr: unsupported field type is refused loudly", "not supported" in str(exc), str(exc))


def write_string_bag(path: Path, topic_msgs: list[tuple[str, int, str]], torn: bool = False) -> None:
    """A minimal MCAP of std_msgs/String records (topic, log_time_ns, json_text) with the ros2msg
    schema the recorder writes; `torn` truncates the file after the last message (no footer)."""
    from mcap.writer import CompressionType, Writer

    with open(path, "wb") as f:
        w = Writer(f, compression=CompressionType.NONE, use_chunking=False)
        w.start(profile="ros2", library="test_pure")
        sid = w.register_schema(name="std_msgs/msg/String", encoding="ros2msg", data=b"string data")
        chans = {}
        for i, (topic, t, text) in enumerate(topic_msgs):
            if topic not in chans:
                chans[topic] = w.register_channel(topic=topic, message_encoding="cdr", schema_id=sid)
            w.add_message(channel_id=chans[topic], log_time=t, publish_time=t, sequence=i, data=cdr_of_string(text))
        if torn:
            f.flush()
            size = f.tell()
            w.finish()
            f.truncate(size)  # drop the footer/summary finish() wrote → a torn bag
        else:
            w.finish()


def test_foxglove_json(tmp: Path) -> None:
    from rig_nodes.foxglove_json import convert, finish_schema, finite, merge_schema

    s = merge_schema({}, {"a": 1, "b": "x", "c": [1, 2], "d": None})
    s = merge_schema(s, {"a": 2.5, "b": True, "e": {"f": 1}})
    fs = finish_schema(s)
    check("foxglove: schema merges types, widest wins, null dropped", fs["type"] == "object" and fs["properties"]["a"]["type"] == "number"
          and fs["properties"]["b"]["type"] == "string" and "d" not in fs["properties"] and fs["properties"]["c"]["items"]["type"] == "number", fs)
    check("foxglove: NaN → null", finite({"x": float("nan"), "y": [1.0, math.inf]}) == {"x": None, "y": [1.0, None]})
    src = tmp / "in.mcap"
    env = json.dumps({"src": "k", "seq": 1, "rx_epoch_ms": 1, "rx_mono_ns": 2, "raw": json.dumps({"protocol": "Kinefly", "sequence": 9})})
    write_string_bag(src, [("/k/raw", 10, env), ("/x/status", 20, '{"reachable":true,"v":1.5}'), ("/x/status", 30, '{"reachable":false,"err":"x"}'), ("/drop", 40, '{"z":1}')])
    rep = convert(src, tmp / "out.mcap", enrich=lambda topic, obj: None if topic == "/drop" else {**obj, "twice": obj.get("v", 0) * 2})
    check("foxglove: convert writes JSON messages, enrich adds + drops", rep["messages"] == 3 and rep["skipped"] == 1 and rep["topics"] == {"/k/raw": 1, "/x/status": 2}, rep)
    from mcap.reader import make_reader

    with open(tmp / "out.mcap", "rb") as f:
        r = make_reader(f)
        msgs = list(r.iter_messages())
        schemas = {ch.topic: json.loads(sc.data) for sc, ch, _ in msgs}
        bodies = {ch.topic: json.loads(m.data) for _, ch, m in msgs if ch.topic == "/k/raw"}
        meta = list(r.iter_metadata())
    check("foxglove: envelope raw parsed into msg, raw dropped", bodies["/k/raw"].get("msg", {}).get("sequence") == 9 and "raw" not in bodies["/k/raw"], bodies)
    check("foxglove: jsonschema per topic + metadata record", schemas["/x/status"]["properties"]["twice"]["type"] == "number" and any(m.name == "foxglove_json" for m in meta), list(schemas))


def test_mcap_index(tmp: Path) -> None:
    from mcap.reader import make_reader

    from rig_nodes.mcap_index import needs_recovery, recover_bag, reindex

    bag = tmp / "run.bag"
    bag.mkdir()
    write_string_bag(bag / "run_0.mcap", [("/a/raw", t * 1_000_000, json.dumps({"n": t})) for t in range(1, 51)])
    rep = reindex(bag / "run_0.mcap")
    with open(bag / "run_0.mcap", "rb") as f:
        r = make_reader(f)
        n = sum(1 for _ in r.iter_messages())
        summ = r.get_summary()
    check("mcap_index: reindex keeps every record and writes a summary/index", n == 50 and summ is not None and summ.statistics.message_count == 50, rep)
    torn = tmp / "torn.bag"
    torn.mkdir()
    write_string_bag(torn / "torn_0.mcap", [("/a/raw", t * 1_000_000, json.dumps({"n": t})) for t in range(1, 21)], torn=True)
    why = needs_recovery(torn)
    check("mcap_index: a torn bag (no metadata.yaml / footer) is detected", bool(why), why)
    rec = recover_bag(torn)
    with open(sorted(torn.glob("*.mcap"))[0], "rb") as f:
        r = make_reader(f)
        topics: dict[str, int] = {}
        for _, ch, m in r.iter_messages():
            topics[ch.topic] = topics.get(ch.topic, 0) + 1
    check("mcap_index: recover_bag salvages the records and appends ONE recovery marker",
          topics.get("/a/raw") == 20 and sum(v for k, v in topics.items() if k.endswith("/session")) == 1, (topics, rec))


def test_supervise() -> None:
    from rig_nodes.supervise import SIGKILL, Child, popen_group, port_in_use, preflight, signal_group, split_args, wait_port

    check("supervise: split_args splits, empty → []", split_args("--a 1 --b 'x y'") == ["--a", "1", "--b", "x y"] and split_args("") == [])
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    s.listen(1)
    port = s.getsockname()[1]
    try:
        check("supervise: port_in_use sees a live listener", port_in_use("tcp", port))
        lines = preflight([("tcp", port, "the test listener")])
        check("supervise: preflight names the taken port", len(lines) == 1 and f"127.0.0.1:{port}" in lines[0] and "test listener" in lines[0], lines)
        check("supervise: wait_port returns once connectable", wait_port("127.0.0.1", port, 2.0))
    finally:
        s.close()
    free = socket.socket()
    free.bind(("127.0.0.1", 0))
    fp = free.getsockname()[1]
    free.close()
    check("supervise: wait_port times out on a closed port", not wait_port("127.0.0.1", fp, 0.6))
    child = Child("sleeper", [sys.executable, "-c", "import time; print('up', flush=True); time.sleep(30)"])
    child.start()
    time.sleep(0.5)
    alive = child.alive()
    t0 = time.monotonic()
    child.stop(grace_s=1.0, term_s=1.0)
    check("supervise: Child starts in its own group and stops within the escalation budget", alive and not child.alive() and time.monotonic() - t0 < 4.0, child.returncode)
    p = popen_group([sys.executable, "-c", "import time; time.sleep(30)"])
    signal_group(p, SIGKILL)
    try:
        p.wait(3)
        check("supervise: signal_group(SIGKILL) ends the group", True)
    except subprocess.TimeoutExpired:
        p.kill()
        check("supervise: signal_group(SIGKILL) ends the group", False, "still alive")


def main() -> int:
    import rig_nodes

    print(f"[test_pure] rig_nodes {rig_nodes.__version__} from {rig_nodes.__file__}", file=sys.stderr)
    tmp = Path(tempfile.mkdtemp(prefix="rig-nodes-pure-"))
    groups = [("envelope", test_envelope), ("fictrac", test_fictrac), ("cdr", test_cdr),
              ("foxglove_json", lambda: test_foxglove_json(tmp)), ("mcap_index", lambda: test_mcap_index(tmp)), ("supervise", test_supervise)]
    for name, fn in groups:
        try:
            fn()
        except Exception as exc:  # a crashed group is a failure, not an abort
            check(f"{name}: group raised", False, repr(exc))
    n_ok = sum(1 for _, ok, _ in results if ok)
    print(f"[test_pure] {n_ok}/{len(results)} passed", file=sys.stderr)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
