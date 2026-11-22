# Source inside `pixi shell` / `pixi run bash`: adds the locally built rmw_iceoryx workspace.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ws/install/setup.bash"
# iceoryx 2.0.6 installs to lib64, which colcon does not add; it must come before the env's iceoryx 2.95.8.
export LD_LIBRARY_PATH="$HERE/../ws/install/iceoryx_posh/lib64:$HERE/../ws/install/iceoryx_hoofs/lib64:$LD_LIBRARY_PATH"
export PATH="$HERE/../ws/install/iceoryx_posh/bin:$PATH"
# The conda toolchain bakes an RPATH to the env's lib dir, which wins over LD_LIBRARY_PATH,
# so the 2.0.6 libraries are preloaded. Use this environment only with RMW_IMPLEMENTATION=rmw_iceoryx_cpp.
iox_preload() {
  local P="$HERE/../ws/install/iceoryx_posh/lib64" H="$HERE/../ws/install/iceoryx_hoofs/lib64"
  echo "$H/libiceoryx_platform.so:$H/libiceoryx_hoofs.so:$P/libiceoryx_posh.so:$P/libiceoryx_posh_config.so:$P/libiceoryx_posh_roudi.so"
}
