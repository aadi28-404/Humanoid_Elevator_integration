#!/usr/bin/env bash
set -euo pipefail

if ! pgrep -x gzserver >/dev/null; then
  echo "No gzserver is running."
  echo "Start the simulation in another terminal first:"
  echo "  cd /media/aadi/Extras/ba0ba0/assesment"
  echo "  ./scripts/launch_sim.sh --headless"
  exit 1
fi

unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=Copy
export __GL_THREADED_OPTIMIZATIONS=0
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MODEL_DATABASE_URI=""
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export SVGA_VGPU10=0

exec gzclient --verbose
