"""Run the transport latency matrix and append one JSON line per cell to results.jsonl.

Run from the project root as:  pixi run bash -c 'source bench/env.sh && python bench/run.py'
Each cell is one publisher and one subscriber on this machine, 8 s of publishing.
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results.jsonl"
DUR = 8.0
CASES = [(1_000, 100), (64_000, 100), (1_000_000, 30), (2_000_000, 30), (8_000_000, 30), (2_000_000, 135)]
FIXED = [("64k", 100), ("2m", 30), ("2m", 135), ("8m", 30)]
PRELOAD = subprocess.run(["bash", "-c", f"source {HERE}/env.sh; iox_preload"],
                         capture_output=True, text=True).stdout.strip()


def env_for(transport: str) -> dict:
    e = dict(os.environ)
    if transport == "iceoryx":
        e.update(RMW_IMPLEMENTATION="rmw_iceoryx_cpp", LD_PRELOAD=PRELOAD)
    else:
        e.update(RMW_IMPLEMENTATION="rmw_zenoh_cpp")
        if transport == "zenoh-shm":
            e.update(ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=true",
                     ZENOH_SHM_ALLOC_SIZE=str(48 * 1024 * 1024))
    return e


def daemon(transport: str, big: bool, env: dict) -> subprocess.Popen:
    if transport == "iceoryx":
        cmd = ["iox-roudi", "-c", str(HERE / ("roudi_8m.toml" if big else "roudi_small.toml"))]
    else:
        cmd = ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    time.sleep(3.0)
    return p


def stop(p: subprocess.Popen) -> None:
    try:
        os.killpg(p.pid, signal.SIGTERM)
        p.wait(timeout=5)
    except Exception:
        os.killpg(p.pid, signal.SIGKILL)
    time.sleep(0.5)
    for f in Path("/dev/shm").iterdir():  # segments left behind by this benchmark only
        if not f.name.startswith("lttng") and f.owner() == os.environ.get("USER", "dev"):
            f.unlink(missing_ok=True)


def last_json(text: str, key: str) -> dict:
    for line in reversed(text.splitlines()):
        if line.startswith("{") and key in line:
            return json.loads(line)
    return {}


def cell(transport: str, sub_cmd: list, pub_cmd: list, big: bool, sub_env_extra: dict | None = None) -> dict:
    env = env_for(transport)
    d = daemon(transport, big, env)
    try:
        sub = subprocess.Popen(sub_cmd, env={**env, **(sub_env_extra or {})}, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        time.sleep(1.0)
        pub = subprocess.run(pub_cmd, env=env, text=True, capture_output=True, timeout=DUR + 30)
        try:
            sub_out = sub.communicate(timeout=DUR + 30)[0]
        except subprocess.TimeoutExpired:
            sub.kill()
            sub_out = ""
        res = last_json(sub_out, '"n"')
        res.update(last_json(pub.stdout, "sent"))
        if not res.get("sent"):
            res["pub_error"] = (pub.stderr or pub.stdout)[-300:]
        return res
    finally:
        stop(d)


def main() -> None:
    only = set(sys.argv[1:])
    sub_secs = str(DUR + 5)
    with OUT.open("a") as out:
        def emit(rec: dict) -> None:
            print(json.dumps(rec), flush=True)
            out.write(json.dumps(rec) + "\n")
            out.flush()

        for transport in ("zenoh", "zenoh-shm", "iceoryx"):
            if only and transport not in only:
                continue
            for size, rate in CASES:
                pub_cmd = [sys.executable, str(HERE / "pubsub.py"), "pub", str(size), str(rate), str(DUR)]
                for name, sub_cmd in (("rclpy", [sys.executable, str(HERE / "pubsub.py"), "sub", sub_secs]),
                                      ("rclcpp", ["ros2", "run", "bench_cpp", "sub_image", sub_secs])):
                    r = cell(transport, sub_cmd, pub_cmd, size > 4_000_000)
                    emit({"transport": transport, "msg": "Image", "bytes": size, "hz": rate, "pub": "rclpy", **r, "sub": name})
            for size, rate in FIXED:
                pub_cmd = ["ros2", "run", "bench_cpp", "fixed", "pub", size, str(rate), str(DUR + 2)]
                sub_cmd = ["ros2", "run", "bench_cpp", "fixed", "sub", size, str(rate), sub_secs]
                # Jazzy only takes loans on the subscription side when this is set to 0.
                r = cell(transport, sub_cmd, pub_cmd, size == "8m", {"ROS_DISABLE_LOANED_MESSAGES": "0"})
                emit({"transport": transport, "msg": "Fixed" + size, "hz": rate, "pub": "rclcpp-loan", **r, "sub": "rclcpp-loan"})


if __name__ == "__main__":
    main()
