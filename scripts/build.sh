#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=Copy

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${1:-}" == "--clean" ]]; then
  rm -rf build install log
  shift
fi

colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
