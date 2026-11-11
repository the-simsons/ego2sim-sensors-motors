#!/usr/bin/env bash
# Start the bridge with Humble and the backend's custom message types in scope.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
backend_setup="${EGO2SIM_SETUP:-$repo_root/install/setup.bash}"
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -r "$backend_setup" ]]; then
  echo "Backend workspace setup not found: $backend_setup" >&2
  echo "Build from the repository root, or set EGO2SIM_SETUP to your workspace install/setup.bash." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$backend_setup"
if ! ros2 interface show ego2sim_interfaces/msg/ScalarStamped >/dev/null; then
  echo "ego2sim_interfaces is unavailable in the sourced workspace; build both backend packages first." >&2
  exit 1
fi
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml "$@"
