"""rclpy publisher / subscriber for the transport latency benchmark.

Publishes sensor_msgs/Image (RELIABLE, depth 10) with the wall-clock send time in header.stamp.
The subscriber reports latency = receive time - header.stamp as one JSON line.

Usage: python pubsub.py pub <bytes> <rate_hz> <seconds>
       python pubsub.py sub <seconds>
"""
import json
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def publish(size: int, rate: float, secs: float) -> None:
    node = rclpy.create_node("bench_pub")
    pub = node.create_publisher(Image, "bench", QOS)
    msg = Image()
    msg.data = bytes(size)
    time.sleep(2.0)  # let subscribers match
    period, sent = 1.0 / rate, 0
    nxt, end = time.perf_counter(), time.perf_counter() + secs
    while time.perf_counter() < end:
        now = time.time_ns()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 1_000_000_000)
        pub.publish(msg)
        sent += 1
        nxt += period
        time.sleep(max(0.0, nxt - time.perf_counter()))
    print(json.dumps({"sent": sent}), flush=True)
    node.destroy_node()


def subscribe(secs: float) -> None:
    node = rclpy.create_node("bench_sub_py")
    lat: list[int] = []

    def on_msg(m: Image) -> None:
        t = time.time_ns()
        lat.append(t - (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec))
        len(m.data)

    node.create_subscription(Image, "bench", on_msg, QOS)
    end = time.perf_counter() + secs
    while time.perf_counter() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    lat.sort()
    out: dict = {"sub": "rclpy", "n": len(lat)}
    if lat:
        out["median_ms"] = round(lat[len(lat) // 2] / 1e6, 3)
        out["p99_ms"] = round(lat[int(0.99 * (len(lat) - 1))] / 1e6, 3)
    print(json.dumps(out), flush=True)
    node.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    if sys.argv[1] == "pub":
        publish(int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    else:
        subscribe(float(sys.argv[2]))
    rclpy.shutdown()
