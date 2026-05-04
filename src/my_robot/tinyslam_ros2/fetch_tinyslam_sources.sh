#!/usr/bin/env bash
# fetch_tinyslam_sources.sh
# Downloads the original CoreSLAM .c files into tinyslam_lib/
# Run this once before colcon build.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$SCRIPT_DIR/tinyslam_lib"
BASE_URL="https://raw.githubusercontent.com/OpenSLAM-org/openslam_tinyslam/master"

FILES=(
  "CoreSLAM.c"
  "CoreSLAM_ext.c"
  "CoreSLAM_random.c"
  "CoreSLAM_state.c"
  "CoreSLAM_loop_closing.c"
)

echo "Downloading TinySLAM source files into $LIB_DIR ..."
mkdir -p "$LIB_DIR"

for f in "${FILES[@]}"; do
  echo "  -> $f"
  curl -fsSL "$BASE_URL/$f" -o "$LIB_DIR/$f"
done

echo ""
echo "Done! You can now run:"
echo "  cd ~/ros2_ws && colcon build --packages-select tinyslam_ros2"
