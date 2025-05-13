#!/usr/bin/env bash
# ------------------------------------------------------------------
# orbslam3_install.sh – Local OpenCV + Pangolin + ORB‑SLAM3 (+ROS2)
# Ubuntu 22.04 • ROS 2 Humble • rev‑12
# ------------------------------------------------------------------
set -euo pipefail
NPROC=$(nproc)

# ---------- 0.  CONFIGURABLE PATHS --------------------------------
WS=~/src/tello_ws                 # your colcon workspace
LIBS_DIR="$WS/libs"
OPENCV_DIR="$LIBS_DIR/opencv"     # will contain opencv/$VERSION/{src,install}
OPENCV_VERSION=4.6.0
PANGO_DIR="$LIBS_DIR/Pangolin"    # will contain Pangolin/{src,install}
CORE_DIR="$LIBS_DIR/ORB_SLAM3"
WRAP_DIR="$LIBS_DIR/ORB_SLAM3_ROS2"

# ---------- Helper -------------------------------------------------
install_if_missing() {
  local PKG="$1"
  if dpkg-query -W -f='${Status}' "$PKG" 2>/dev/null | grep -q "ok installed"; then
    echo " ✔ $PKG already installed (APT)"
  else
    echo " ➜ Installing $PKG (APT)"
    sudo apt-get update -qq
    sudo apt-get install -y "$PKG"
  fi
}

pkg_config_missing() {
  # return 0 (true) if the given .pc file is NOT found
  ! pkg-config --exists "$1"
}

add_path_if_missing() {
  # idempotently prepend a path to an env‑var
  local var="$1" path="$2"
  if [[ -d "$path" ]] && [[ ":${!var}:" != *":${path}:"* ]]; then
    export "$var"="$path:${!var:-}"
  fi
}

mkdir -p "$LIBS_DIR"

# ---------- 1.  ROS 2 environment ---------------------------------
set +u
source /opt/ros/humble/setup.bash
set -u

# ---------- 2.  APT dependencies ----------------------------------
echo "=== Installing required APT packages ==="
for PKG in \
    git build-essential cmake pkg-config ninja-build \
    python3-ament-package python3-numpy python3-opencv \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    libyaml-cpp-dev libblas-dev liblapack-dev libssl-dev \
    libglew-dev libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev \
    libxxf86vm-dev libxkbcommon-dev ; do
  install_if_missing "$PKG"
done

# ---------- 3.  OpenCV $OPENCV_VERSION ----------------------------
if pkg-config --exists opencv4; then
  echo "✔ OpenCV already available via pkg‑config – skipping local build"
else
  echo "=== Building OpenCV $OPENCV_VERSION locally ==="
  OPENCV_SRC="$OPENCV_DIR/src"
  OPENCV_INSTALL="$OPENCV_DIR/install"
  if [[ -d "$OPENCV_SRC" ]]; then
    cd "$OPENCV_SRC" && git fetch --quiet && git checkout "tags/$OPENCV_VERSION" --quiet
  else
    git -C "$LIBS_DIR" clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv.git "$OPENCV_SRC"
  fi
  mkdir -p "$OPENCV_SRC/build" && cd "$OPENCV_SRC/build"
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_CUDA=OFF \
    -DCMAKE_INSTALL_PREFIX="$OPENCV_INSTALL" \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -GNinja
  ninja -j"$NPROC"
  ninja install
  # Expose to the current shell
  add_path_if_missing PKG_CONFIG_PATH "$OPENCV_INSTALL/lib/pkgconfig"
  add_path_if_missing LD_LIBRARY_PATH "$OPENCV_INSTALL/lib"
  add_path_if_missing CMAKE_PREFIX_PATH "$OPENCV_INSTALL"
fi

# ---------- 4.  Pangolin ------------------------------------------
if pkg-config --exists pangolin; then
  echo "✔ Pangolin already available via pkg‑config – skipping local build"
else
  echo "=== Building Pangolin locally ==="
  PANGO_SRC="$PANGO_DIR/src"
  PANGO_INSTALL="$PANGO_DIR/install"
  if [[ -d "$PANGO_SRC" ]]; then
    cd "$PANGO_SRC" && git pull --ff-only
  else
    git -C "$LIBS_DIR" clone --recursive https://github.com/stevenlovegrove/Pangolin.git "$PANGO_SRC"
  fi
  mkdir -p "$PANGO_SRC/build" && cd "$PANGO_SRC/build"
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PANGO_INSTALL" \
    -GNinja
  ninja -j"$NPROC"
  ninja install
  # Expose to the current shell
  add_path_if_missing PKG_CONFIG_PATH "$PANGO_INSTALL/lib/pkgconfig"
  add_path_if_missing LD_LIBRARY_PATH "$PANGO_INSTALL/lib"
  add_path_if_missing CMAKE_PREFIX_PATH "$PANGO_INSTALL"
fi

# ---------- 5.  ORB‑SLAM3 core ------------------------------------
echo "=== Preparing ORB_SLAM3 core ==="
if [[ -d "$CORE_DIR" ]]; then
  echo " ✔ ORB_SLAM3 repo already present"
else
  git -C "$LIBS_DIR" clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$CORE_DIR"
fi

# Patch CMakeLists to C++14 once only
if ! grep -q "CXX_STANDARD 14" "$CORE_DIR/CMakeLists.txt"; then
  echo " ➜ Patching ORB_SLAM3 CMakeLists.txt for C++14"
  sed -i 's/++11/++14/g' "$CORE_DIR/CMakeLists.txt"   # :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}
fi

echo "=== Building ORB_SLAM3 ==="
cd "$CORE_DIR"
chmod +x build.sh
NUM_BUILD_TRIES=3
for ((i=1; i<=NUM_BUILD_TRIES; i++)); do
  if ./build.sh -j"$NPROC"; then
    echo " ✔ ORB_SLAM3 built successfully"
    break
  else
    echo " ✖ ORB_SLAM3 build failed (attempt $i/$NUM_BUILD_TRIES) – retrying"
    sleep 1
  fi
done

# ---------- 6.  (Optional) ROS2 wrapper ---------------------------
# Uncomment when you’re ready:
# echo "=== Cloning ROS2 wrapper ==="
# if [[ -d "$WRAP_DIR" ]]; then
#   echo " ✔ ORB_SLAM3_ROS2 repo already present"
# else
#   git -C "$LIBS_DIR" clone https://github.com/ToniRV/ORB_SLAM3-ROS2.git "$WRAP_DIR"
# fi
# cd "$WRAP_DIR"
# colcon build --merge-install --cmake-args \
#   -DPangolin_DIR="$PANGO_INSTALL" \
#   -DOpenCV_DIR="$OPENCV_INSTALL"

# ---------- 7.  User hints ----------------------------------------
echo ""
echo "==================================================================="
echo "  ✓  All done!  To make these local libs discoverable in new shells,"
echo "  add the following lines to your ~/.bashrc (or source them now):"
echo ""
echo "    export PKG_CONFIG_PATH=\"${PKG_CONFIG_PATH:-}\""
echo "    export LD_LIBRARY_PATH=\"${LD_LIBRARY_PATH:-}\""
echo "    export CMAKE_PREFIX_PATH=\"${CMAKE_PREFIX_PATH:-}\""
echo "==================================================================="
