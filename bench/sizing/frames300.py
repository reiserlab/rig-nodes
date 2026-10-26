"""One hour of controller frame records at 300 Hz in the telemetry ring's cf shape:
   ["cf", rx_epoch_ms, t_us, seq, idx, pattern, sd_load_us, spi_us, req_age_us, superseded, flags]
Realistic jitter: t_us steps 3333 µs ± a few µs; sd_load/spi vary; idx follows a 50 Hz Mode-3 loop
(changes every 6th frame on average). 'changes only' = one record per idx change."""
import io, json, gzip, random, zstandard
import mcap.writer as MW
from mcap.writer import Writer, CompressionType, IndexType

random.seed(1)
HZ, HOURS = 300, 1
N = HZ * 3600 * HOURS
t0_ms = 1_792_000_000_000
rows, changes = [], []
t_us = 0; idx = 0; rx = t0_ms
for i in range(N):
    t_us += 3333 + random.randint(-3, 3)
    if i % 6 == 0:                      # a 0x70 landed (≈50 Hz): idx steps by -1..+2
        idx = (idx + random.choice((-1, 0, 1, 1, 2))) % 200
        changed = True
    else:
        changed = False
    rx = t0_ms + (t_us // 1000) + random.randint(0, 2)   # host epoch ms at drain
    r = ["cf", rx, t_us, i, idx, 3, 180 + random.randint(0, 40), 95 + random.randint(0, 6),
         random.randint(50, 900), 0, 0]
    rows.append(r)
    if changed:
        changes.append(r)

def ndjson_gz(rs):
    return len(gzip.compress("".join(json.dumps(r, separators=(",", ":")) + "\n" for r in rs).encode(), 9))

def mcap_size(rs, batch_s=None, level=19, msg_index=False):
    saved = MW.zstandard.compress
    MW.zstandard.compress = lambda d, _l=level: zstandard.ZstdCompressor(level=_l).compress(d)
    try:
        buf = io.BytesIO()
        w = Writer(buf, compression=CompressionType.ZSTD,
                   index_types=IndexType.ALL if msg_index else (IndexType.ALL & ~IndexType.MESSAGE))
        w.start(profile="", library="frames300")
        cid = w.register_channel("controller/frame", "json", w.register_schema("cf", "jsonschema", b"{}"))
        if batch_s is None:
            for k, r in enumerate(rs):
                ts = r[1] * 1_000_000
                w.add_message(cid, log_time=ts, publish_time=ts, data=json.dumps(r, separators=(",", ":")).encode(), sequence=k)
        else:
            cur, bucket, k = None, [], 0
            for r in rs:
                b = int(r[1] // (batch_s * 1000))
                if cur is not None and b != cur:
                    ts = bucket[0][1] * 1_000_000; k += 1
                    w.add_message(cid, log_time=ts, publish_time=ts, data=json.dumps(bucket, separators=(",", ":")).encode(), sequence=k)
                    bucket = []
                cur = b; bucket.append(r)
            if bucket:
                ts = bucket[0][1] * 1_000_000
                w.add_message(cid, log_time=ts, publish_time=ts, data=json.dumps(bucket, separators=(",", ":")).encode(), sequence=k + 1)
        w.finish()
        return len(buf.getvalue())
    finally:
        MW.zstandard.compress = saved

MB = 1024 * 1024
print(f"{N} frame records/hour at {HZ} Hz; {len(changes)} idx changes/hour ({len(changes)/3600:.0f} Hz)")
print(f"{'variant':58s} {'every frame':>12s} {'changes only':>13s}")
for name, fn in [
    ("NDJSON gzip-9 (today's format)", lambda rs: ndjson_gz(rs)),
    ("MCAP per-record, rosbag2/SDK defaults (zstd-3, msg index)", lambda rs: mcap_size(rs, level=3, msg_index=True)),
    ("MCAP per-record, tuned (zstd-19, no msg index)", lambda rs: mcap_size(rs)),
    ("MCAP batched 1 s per message, zstd-19", lambda rs: mcap_size(rs, batch_s=1.0)),
    ("MCAP batched 10 s per message, zstd-19", lambda rs: mcap_size(rs, batch_s=10.0)),
]:
    a, b = fn(rows), fn(changes)
    print(f"{name:58s} {a/MB:9.1f} MB {b/MB:10.1f} MB")
