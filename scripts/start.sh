#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the atlas bridge. NO nav2 spawn here — `ros2 launch
# nav2_bringup navigation_launch.py …` runs inside Driver(CMD_INIT).
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-py 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

exec python3 -m nav2_wrapper.atlas_bridge
