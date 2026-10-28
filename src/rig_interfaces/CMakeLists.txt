cmake_minimum_required(VERSION 3.8)
project(rig_interfaces)

# Windows + pixi-build-ros (<= 0.7.6, 2026-10-14): the backend passes PYTHON_INSTALL_DIR as
# ../Lib/site-packages (relative to %LIBRARY_PREFIX%), CMake records that spelling in
# install_manifest.txt, the backend hands the manifest to rattler-build as the package's file
# list, and the archive writer refuses `Library/../Lib/...` ("paths in archives must not have
# `..`", prefix-dev/pixi#6955). Use the absolute, normalized, forward-slash path instead — the
# same value the pending upstream fix (prefix-dev/pixi#7226) passes, so this is a no-op once it
# ships. The installed tree is unchanged. colcon and Linux/macOS never take this branch.
if(WIN32 AND PYTHON_INSTALL_DIR MATCHES "^\\.\\./")
  cmake_path(ABSOLUTE_PATH PYTHON_INSTALL_DIR BASE_DIRECTORY "${CMAKE_INSTALL_PREFIX}"
    NORMALIZE OUTPUT_VARIABLE _arena_python_install_dir)
  set(PYTHON_INSTALL_DIR "${_arena_python_install_dir}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FicTracFrame.msg"
  "msg/ArenaFrame.msg"
  "msg/ConnectorReply.msg"
  # host-pressure sample (rig_nodes.host_monitor, /rig/host; 0.3.0) — the four record types, then the sample that nests them
  "msg/DiskStats.msg"
  "msg/NetStats.msg"
  "msg/GpuStats.msg"
  "msg/ProcStats.msg"
  "msg/HostStats.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
