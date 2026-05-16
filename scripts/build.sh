#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: rbnx codegen only. We do NOT vendor or build nav2
# itself — it's installed system-wide via apt:
#   sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
# (This is the same path the robot was already using.)
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[nav2_wrapper/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/data

# Sanity check that nav2_bringup is on the system. Soft-warn on miss
# (build still succeeds; user can apt install before `rbnx boot`).
if ! ros2 pkg list 2>/dev/null | grep -q "^nav2_bringup$"; then
    echo "[nav2_wrapper/build] NOTE: nav2_bringup not found in 'ros2 pkg list'."
    echo "                     Install with:"
    echo "                       sudo apt install ros-humble-nav2-bringup ros-humble-navigation2"
fi

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[nav2_wrapper/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[nav2_wrapper/build] done."
