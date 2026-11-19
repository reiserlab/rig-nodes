"""ezmsg publisher / subscriber for the transport latency benchmark (counterpart of pubsub.py).

One publisher unit and one subscriber unit. The message carries the wall-clock send time;
the subscriber reports latency = receive time - send time as one JSON line.

Usage: python ez_pubsub.py bytes|numpy <bytes> <rate_hz> <seconds> two|same
  two  = subscriber in its own process (shared-memory transport)
  same = both units in one process (object handed over by reference)
"""
import asyncio
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, AsyncGenerator

import ezmsg.core as ez
import numpy as np

KIND, SIZE, RATE, SECS, MODE = (sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5])
TOTAL = int(RATE * SECS)
# The container's /dev/shm holds 63 MB, so the number of shared buffers shrinks with message size.
NBUF = 4 if SIZE > 4_000_000 else 8 if SIZE >= 1_000_000 else 32


@dataclass
class Msg:
    stamp_ns: int
    data: Any


class Pub(ez.Unit):
    OUT = ez.OutputStream(Msg, num_buffers=NBUF, buf_size=max(65536, SIZE + 4096))

    @ez.publisher(OUT)
    async def run(self) -> AsyncGenerator:
        payload = bytes(SIZE) if KIND == "bytes" else np.zeros(SIZE, dtype=np.uint8)
        await asyncio.sleep(2.0)  # let the subscriber connect
        period, nxt = 1.0 / RATE, time.perf_counter()
        for _ in range(TOTAL):
            yield self.OUT, Msg(time.time_ns(), payload)
            nxt += period
            await asyncio.sleep(max(0.0, nxt - time.perf_counter()))
        await asyncio.sleep(1.5)
        raise ez.NormalTermination


class Sub(ez.Unit):
    IN = ez.InputStream(Msg)

    async def initialize(self) -> None:
        self.lat: list[int] = []

    @ez.subscriber(IN)
    async def on_msg(self, m: Msg) -> None:
        self.lat.append(time.time_ns() - m.stamp_ns)
        len(m.data)
        if len(self.lat) == TOTAL:
            self.dump()

    def dump(self) -> None:
        lat = sorted(self.lat)
        out = {"n": len(lat), "sent": TOTAL,
               "median_ms": round(lat[len(lat) // 2] / 1e6, 3),
               "p99_ms": round(lat[int(0.99 * (len(lat) - 1))] / 1e6, 3)}
        with open(os.environ["EZ_OUT"], "w") as f:
            f.write(json.dumps(out))

    async def shutdown(self) -> None:
        if self.lat and len(self.lat) != TOTAL:
            self.dump()


if __name__ == "__main__":
    pub, sub = Pub(), Sub()
    ez.run(PUB=pub, SUB=sub, connections=((pub.OUT, sub.IN),),
           process_components=(sub,) if MODE == "two" else None)
