#!/usr/bin/env bash
# =============================================================================
#  setup_hector_lib.sh
#  
#  Clones the original hector_slam repo (noetic-devel branch) and copies ONLY
#  the pure-C++ algorithm headers into hector_slam_lib/include/.
#
#  These headers have NO ROS1 dependency – they only use Eigen3.
#  This lets us compile the algorithm on ROS2 Humble without any ROS1 overlay.
#
#  Run from the package root:
#    bash scripts/setup_hector_lib.sh
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
DEST="${PKG_DIR}/hector_slam_lib"
TMP_DIR="/tmp/hector_slam_clone_$$"

echo "=== Hector SLAM lib setup ==="
echo "Package root : $PKG_DIR"
echo "Destination  : $DEST"
echo ""

# ── Clone repo (shallow, only what we need) ──────────────────────────────────
if [ -d "$DEST" ]; then
    echo "[INFO] hector_slam_lib already exists – skipping clone."
    echo "       Delete ${DEST} and re-run to refresh."
    exit 0
fi

echo "[1/3] Cloning hector_slam (noetic-devel, shallow) ..."
git clone \
    --depth 1 \
    --branch noetic-devel \
    https://github.com/tu-darmstadt-ros-pkg/hector_slam.git \
    "$TMP_DIR"

# ── Copy only the algorithm include tree ─────────────────────────────────────
echo "[2/3] Copying hector_slam_lib headers ..."

SRC_INCLUDE="${TMP_DIR}/hector_mapping/include"

if [ ! -d "$SRC_INCLUDE/hector_slam_lib" ]; then
    echo "[ERROR] Expected include path not found: $SRC_INCLUDE/hector_slam_lib"
    echo "        The hector_slam repo layout may have changed."
    rm -rf "$TMP_DIR"
    exit 1
fi

mkdir -p "${DEST}/include"
cp -r "${SRC_INCLUDE}/hector_slam_lib" "${DEST}/include/"

# ── Cleanup ───────────────────────────────────────────────────────────────────
echo "[3/3] Cleaning up temporary clone ..."
rm -rf "$TMP_DIR"

echo ""
echo "=== Done! ==="
echo "Headers installed at: ${DEST}/include/hector_slam_lib/"
echo ""
echo "Key headers:"
find "${DEST}/include" -name "*.h" | sort | head -20
echo ""
echo "You can now build:"
echo "  cd <your_workspace>"
echo "  colcon build --packages-select hector_slam_ros2"