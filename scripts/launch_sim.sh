#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PROCESS_PATTERN='gzserver|gzclient|gazebo|ros2 launch humanoid_delivery_sim|fleet_orchestrator|navigation_server|fake_nav_server|robot_executive|speech_llm_interface|safety_controller|sim_perception|manipulation_server|static_map_publisher|motion_recorder|joint_motion_demo|spawn_entity.py|robot_state_publisher'

cleanup() {
  pkill -f "$PROCESS_PATTERN" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$PROCESS_PATTERN" 2>/dev/null || true
}

GUI="true"
if [[ "${1:-}" == "--headless" ]]; then
  GUI="false"
  shift
elif [[ "${1:-}" == "--gui" ]]; then
  GUI="true"
  shift
fi

cleanup
trap cleanup EXIT

unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=Copy
export __GL_THREADED_OPTIMIZATIONS=0

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ ! -f install/setup.bash ]]; then
  ./scripts/build.sh
fi

set +u
source install/setup.bash
set -u
ros2 launch humanoid_delivery_sim humanoid_delivery_sim.launch.py gui:="$GUI" "$@"
