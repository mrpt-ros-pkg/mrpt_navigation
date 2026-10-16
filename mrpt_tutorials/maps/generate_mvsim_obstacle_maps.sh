#!/bin/bash
# Generates the static obstacle maps (mp2p_icp .mm files, with one point layer "raw")
# of the mvsim demo worlds used by
# demo_astar_trajectory_follower_mvsim_3d.launch.py: the 2D cells occupied by
# the world geometry (except vehicles, people and animated objects) in the same
# height band as the live obstacles (params/point-cloud-pipeline-3d-obstacles-*.yaml).
#
# Requires mvsim (with the "mvsim slice" command) and mp2p_icp (txt2mm) in the
# environment. Usage (writes the .mm files next to this script):
#
#   bash generate_mvsim_obstacle_maps.sh
#
set -e

OUT_DIR=$(dirname "$(realpath "$0")")
WORLDS_DIR=$(ros2 pkg prefix --share mvsim)/mvsim_tutorial
TMP_FILE=$(mktemp --suffix=.txt)
trap 'rm -f "$TMP_FILE"' EXIT

for WORLD in greenhouse warehouse; do
  mvsim slice "$WORLDS_DIR/demo_$WORLD.world.xml" -o "$TMP_FILE" \
    --slice-z 0.10 --slice-z-max 1.80 --slice-resolution 0.10
  txt2mm -i "$TMP_FILE" -o "$OUT_DIR/mvsim_${WORLD}_obstacles.mm" -f xyz
done
