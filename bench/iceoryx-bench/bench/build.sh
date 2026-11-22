#!/usr/bin/env bash
# Build iceoryx 2.0.6 and rmw_iceoryx (jazzy branch) in ws/. Run as: pixi run bash bench/build.sh
# rmw_iceoryx does not compile against the iceoryx 2.95.8 that RoboStack Jazzy ships, hence the source build.
# The IOX_MAX_* limits shrink RouDi's management segment (65.7 MB by default) to fit a 63 MB /dev/shm.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../ws"
W="$PWD/install"
COMMON=(-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
        "-DCMAKE_CXX_FLAGS=-include cstdint -Wno-error")
colcon build --packages-select iceoryx_hoofs iceoryx_posh --cmake-force-configure --cmake-args "${COMMON[@]}" \
  -DIOX_MAX_PUBLISHERS=64 -DIOX_MAX_SUBSCRIBERS=64 -DIOX_MAX_SUBSCRIBERS_PER_PUBLISHER=8
colcon build --packages-select rmw_iceoryx_cpp bench_msgs bench_cpp --cmake-force-configure --cmake-clean-first --cmake-args "${COMMON[@]}" \
  -Diceoryx_posh_DIR="$W/iceoryx_posh/lib64/cmake/iceoryx_posh" \
  -Diceoryx_hoofs_DIR="$W/iceoryx_hoofs/lib64/cmake/iceoryx_hoofs"
