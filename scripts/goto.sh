#!/usr/bin/env bash
# Send Solara to a spot in the room (map frame, meters).
#   ./goto.sh 2.0 1.5
# Find a spot's coordinates: drive there, then run
#   ros2 run tf2_ros tf2_echo map base_link
set -e
if [ $# -ne 2 ]; then
  echo "usage: $0 <x> <y>"; exit 1
fi
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}"
