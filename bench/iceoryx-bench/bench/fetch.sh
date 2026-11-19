#!/usr/bin/env bash
# Fetch the upstream sources that bench/build.sh compiles, into ws/src/. Run once as: bash bench/fetch.sh
# iceoryx v2.0.6 and the rmw_iceoryx `jazzy` branch at the commit the results were measured with.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../ws/src"
[ -d iceoryx ] || git clone --depth 1 --branch v2.0.6 https://github.com/eclipse-iceoryx/iceoryx
if [ ! -d rmw_iceoryx ]; then
  git clone --branch jazzy https://github.com/ros2/rmw_iceoryx
  git -C rmw_iceoryx checkout --detach 1d03b7eea197dd8980989bf89daa4ef37b6be5e8
fi
