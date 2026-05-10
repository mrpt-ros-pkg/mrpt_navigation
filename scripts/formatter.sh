#!/usr/bin/env bash
# Usage: scripts/formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    mrpt_map_server \
    mrpt_msgs_bridge \
    mrpt_navigation \
    mrpt_nav_interfaces \
    mrpt_pf_localization \
    mrpt_pointcloud_pipeline \
    mrpt_reactivenav2d \
    mrpt_tps_astar_planner \
    mrpt_tutorials \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
