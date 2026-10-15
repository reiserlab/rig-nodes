"""recorder.py — one MCAP bag per run, written with rosbag2_py (LAB-303 Plan B).

The bridge node records EVERY topic it touches into `<log-dir>/<run>.bag/` while a run's log
is open (log_control enabled → off / log_export / shutdown), so the bag — not the live NDJSON —
is the record of truth. mcap_convert.py derives the behavior_v2 run log from it.

Two kinds of records go in:
  * device topics as received (raw CDR bytes from the subscriptions — `/fictrac/raw` or
    `/fictrac/frame`, `/kinefly/raw`, `/bias/cmd`, `/bias/reply`, `/bias/status`), stamped
    with the bridge's receive time (MCAP log_time);
  * bridge-authored records, serialized here (`/arena/browser` = every browser message,
    `/arena/event`, `/arena/frame`, `/arena/session`), each carrying the writer's emission
    sequence number `seq`, so the converter can reproduce the live writer's line order.

Profiles (`--bag-profile`): `default` = rosbag2's MCAP defaults (768 KiB chunks — a chunk is
only flushed when full or at close, so a short run is ONE chunk written at close — plus the
writer's 100 MiB message cache); `crashsafe` = no chunking, no cache — every message reaches
the file as it is written, so a SIGKILL loses at most the record in progress (both measured in
tests/test_ros2_poc.py's crash phase). Thread-safe: subscriptions write from the executor
thread, the asyncio side from the loop thread. Records that arrive while the bag is still
opening (rosbag2 takes ~150 ms to open one) are buffered and written first.

Timestamps: rosbag2_py's 4-argument write is (topic, data, recv_timestamp, send_timestamp)
→ MCAP log_time = recv, publish_time = send (checked by reading the bag back).
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

STRING_TYPE = "std_msgs/msg/String"

PROFILES = {
    # name: (storage_preset_profile, storage_config (dict → YAML), max_cache_size bytes)
    "default": ("", None, 100 * 1024 * 1024),
    "crashsafe": ("", {"noChunking": True, "noChunkCRC": True, "compression": "None"}, 0),
}


def _yaml_from(d: dict) -> str:
    # rosbag2_storage_mcap's storage_config_uri is a YAML mapping of its own option names.
    return "".join(f"{k}: {json.dumps(v)}\n" for k, v in d.items())


class Recorder:
    def __init__(self, bag_dir: str | os.PathLike, topics: dict[str, str], profile: str = "default") -> None:
        """`topics`: topic name → ROS type name (e.g. 'std_msgs/msg/String')."""
        self.bag_dir = Path(bag_dir)
        self.topics = dict(topics)
        self.profile = profile
        self._w = None
        self._lock = threading.Lock()
        self.count = 0
        self.bytes = 0
        self.dropped = 0  # writes attempted while closed
        self.opened_mono = None
        self._pending: list | None = None  # records that arrived while open() was running

    @property
    def active(self) -> bool:
        return self._w is not None

    def open(self) -> None:
        preset, cfg, cache = PROFILES[self.profile]
        cfg_path = ""
        if cfg:
            cfg_path = str(self.bag_dir.parent / f".{self.bag_dir.name}.storage.yaml")
            Path(cfg_path).write_text(_yaml_from(cfg), encoding="utf-8")
        so = rosbag2_py.StorageOptions(uri=str(self.bag_dir), storage_id="mcap", max_cache_size=cache,
                                       storage_preset_profile=preset, storage_config_uri=cfg_path)
        co = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        with self._lock:
            self._pending = []
            self.count = 0
            self.bytes = 0
        w = rosbag2_py.SequentialWriter()
        w.open(so, co)
        for i, (name, typ) in enumerate(sorted(self.topics.items())):
            w.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=typ, serialization_format="cdr"))
        with self._lock:
            self._w = w
            self.opened_mono = time.monotonic()
            pending, self._pending = self._pending, None
            for topic, data, log_ns, pub_ns in pending or []:
                self._write_locked(topic, data, log_ns, pub_ns)

    def write_raw(self, topic: str, data: bytes, t_ns: int | None = None, pub_ns: int | None = None) -> bool:
        """Record already-serialized CDR bytes (a raw subscription's payload). `t_ns` = the
        recorder's receive time (MCAP log_time; default now); `pub_ns` = the producer's own
        stamp when it has one (MCAP publish_time; the closed-loop node's sample time)."""
        log_ns = time.time_ns() if t_ns is None else t_ns
        with self._lock:
            if self._w is None:
                if self._pending is not None:  # open() in progress: keep, write once open
                    self._pending.append((topic, data, log_ns, pub_ns))
                    return True
                self.dropped += 1
                return False
            self._write_locked(topic, data, log_ns, pub_ns)
            return True

    def _write_locked(self, topic: str, data: bytes, log_ns: int, pub_ns: int | None) -> None:
        if pub_ns is None:
            self._w.write(topic, data, log_ns)
        else:
            self._w.write(topic, data, log_ns, pub_ns)  # (recv_timestamp, send_timestamp)
        self.count += 1
        self.bytes += len(data)

    def write_json(self, topic: str, obj, t_ns: int | None = None, pub_ns: int | None = None) -> bool:
        """Record a JSON object as std_msgs/String (compact separators, like the live log)."""
        m = String()
        m.data = json.dumps(obj, separators=(",", ":"))
        return self.write_raw(topic, serialize_message(m), t_ns, pub_ns)

    def write_msg(self, topic: str, msg, t_ns: int | None = None, pub_ns: int | None = None) -> bool:
        """Record a typed ROS message object."""
        return self.write_raw(topic, serialize_message(msg), t_ns, pub_ns)

    def close(self, reindex: bool = True) -> dict:
        with self._lock:
            w, self._w = self._w, None
        if w is None:
            return {}
        if hasattr(w, "close"):
            w.close()
        else:  # older rosbag2_py: the destructor closes
            del w
        files = sorted(self.bag_dir.glob("*.mcap"))
        indexed = None
        if reindex and self.profile == "crashsafe":
            # The run closed cleanly: trade the crash-safe layout for an indexed, compressed one
            # (same records; Foxglove and every seeking reader then open it without a scan).
            try:
                from mcap_index import reindex as _reindex

                indexed = [_reindex(f) for f in files]
            except Exception as exc:  # never let a post-close rewrite lose the run
                indexed = [{"error": repr(exc)}]
        return {
            "indexed": indexed,
            "bag": str(self.bag_dir),
            "files": [f.name for f in files],
            "bytes_on_disk": sum(f.stat().st_size for f in files),
            "records": self.count,
            "payload_bytes": self.bytes,
            "profile": self.profile,
            "open_s": round(time.monotonic() - self.opened_mono, 3) if self.opened_mono else None,
        }
