#!/usr/bin/env bash
# Save the map after driving around in mode:=mapping.
#   ./save_map.sh            -> maps/room.*
#   ./save_map.sh kitchen    -> maps/kitchen.*
# Writes two things:
#   <name>.posegraph/.data : what SLAM Toolbox loads in mode:=localization
#   <name>.pgm/.yaml       : a normal image of the map you can open on your laptop
set -e
NAME=${1:-room}
MAPS_DIR="$(cd "$(dirname "$0")/../maps" && pwd)"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAPS_DIR/$NAME'}"
ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$NAME"
echo "Saved to $MAPS_DIR/$NAME"
