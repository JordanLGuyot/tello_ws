#!/usr/bin/env bash
# orbslam.sh • rev‑19 • wrapper from JordanLGuyot fork (humble)
set -euo pipefail
NPROC=$(nproc)

# ── paths ─────────────────────────────────────────────────────────
WS=~/src/tello_ws
LIBS_DIR="$WS/libs"
PANGO_DIR="$LIBS_DIR/Pangolin"
CORE_DIR="$LIBS_DIR/ORB_SLAM3"
WRAP_DIR="$LIBS_DIR/ORB_SLAM3_ROS2"

# ── helpers ───────────────────────────────────────────────────────
install_if_missing() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "ok installed" \
    || { echo " ➜ Installing $1"; sudo apt-get update -qq; sudo apt-get install -y "$1"; }
}
BRC="$HOME/.bashrc"
add_export() { grep -qxF "$1" "$BRC" || echo "$1" >> "$BRC"; }

# ── 1. ROS 2 env & APT deps ───────────────────────────────────────
set +u; source /opt/ros/humble/setup.bash; set -u
for P in git build-essential cmake pkg-config \
         ros-humble-rclcpp ros-humble-message-filters ros-humble-vision-opencv \
         python3-ament-package \
         libeigen3-dev libopencv-dev libboost-all-dev libsuitesparse-dev \
         libyaml-cpp-dev libblas-dev liblapack-dev \
         libglew-dev libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev \
         libxxf86vm-dev libxkbcommon-dev; do install_if_missing "$P"; done
mkdir -p "$LIBS_DIR"

# ── 2. Pangolin (skip if already installed) ───────────────────────
if ! pkg-config --exists pangolin; then
  git -C "$LIBS_DIR" clone --recursive https://github.com/stevenlovegrove/Pangolin.git "$PANGO_DIR" 2>/dev/null || true
  cd "$PANGO_DIR"; git pull --ff-only
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"$NPROC"; sudo make install; sudo ldconfig
  touch "${PANGO_DIR}/COLCON_IGNORE"
fi

# ─── 3. Sophus -----------------------------------------------------
echo ">>> Verifying Sophus headers ..."
cmake -P - <<'EOF' 2>/dev/null && SOPHUS_OK=1 || SOPHUS_OK=0
find_package(Sophus CONFIG QUIET)
EOF


if [ "$SOPHUS_OK" -eq 1 ]; then
 echo ">>> Sophus already present – skipping."
else
 echo ">>> Sophus not found – installing from Thirdparty/Sophus ..."
 pushd "${LIBS_DIR}/ORB_SLAM3/Thirdparty/Sophus" >/dev/null


   # Configure + build (header‑only, so this is quick)
   cmake -B build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local     # use $HOME/.local if you prefer
   cmake --build build -j"${NPROC}"


   # Install (headers + SophusConfig.cmake)
   sudo cmake --install build


 popd >/dev/null
 echo ">>> Sophus installation complete."
fi


# ── 4. ORB‑SLAM3 core (gnu++14) ───────────────────────────────────
git -C "$LIBS_DIR" clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$CORE_DIR" 2>/dev/null || true
cd "$CORE_DIR"; git submodule update --init --recursive

if [ ! -f build/libORB_SLAM3.so ]; then
  sed -i '1i set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")' CMakeLists.txt
  sed -i 's/cmake .. -DCMAKE_BUILD_TYPE=Release.*/cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS=-std=gnu++14/' build.sh
  grep -rl --exclude-dir=.git '\-std=c\+\+11\|\-std=c\+\+17\|\-std=gnu\+\+17\|\-std=c\+\+0x' . |
      xargs -r sed -i -E 's/-std=(c\+\+(11|17|0x)|gnu\+\+17)/-std=gnu++14/g' || true
  sed -i '1i add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable -Wno-aggressive-loop-optimizations -Wno-sign-compare -Wno-reorder)' CMakeLists.txt
  grep -rl --exclude-dir=.git '\-Werror' . | xargs -r sed -i 's/-Werror//g' || true
  rm -rf build
  echo " • Building ORB_SLAM3 core…"
  chmod +x build.sh; ./build.sh
  touch "${CORE_DIR}/COLCON_IGNORE"
else
  echo " ✔ ORB_SLAM3 core already built – skipping"
fi

# ── 5. ORB_SLAM3_ROS2 wrapper (fork • branch humble) ──────────────
if [ ! -d "$WRAP_DIR" ]; then
  git -C "$LIBS_DIR" clone -b humble git@github.com:JordanLGuyot/ORB_SLAM3_ROS2.git "$WRAP_DIR"
else
  cd "$WRAP_DIR"
  git remote set-url origin git@github.com:JordanLGuyot/ORB_SLAM3_ROS2.git
  git fetch origin humble
  git checkout humble
  git pull --ff-only
fi
cd "$WRAP_DIR"; git submodule update --init --recursive

# 5a. Vocabulary folder
VOCAB_DIR="$WRAP_DIR/Vocabulary"
[ -e "$VOCAB_DIR" ] && [ ! -d "$VOCAB_DIR" ] && rm -f "$VOCAB_DIR"
mkdir -p "$VOCAB_DIR"
[ -f "$VOCAB_DIR/ORBvoc.txt" ] || cp "$CORE_DIR/Vocabulary/ORBvoc.txt" "$VOCAB_DIR/"

# 5b. Launch file for Tello
if [ -f launch/mono_inertial.launch.py ]; then
  install -Dm644 launch/mono_inertial.launch.py launch/mono_inertial_tello.launch.py
fi

# 5c. Exports (temp + bashrc)
export ORB_SLAM3_ROOT_DIR="$CORE_DIR"
export ORB_SLAM3_ROOT="$CORE_DIR"
export CMAKE_PREFIX_PATH="$CORE_DIR/Thirdparty/g2o:$CORE_DIR:$CMAKE_PREFIX_PATH"

add_export "export ORB_SLAM3_ROOT_DIR=\"$CORE_DIR\""
add_export "export ORB_SLAM3_ROOT=\"$CORE_DIR\""
add_export "export CMAKE_PREFIX_PATH=\"$CORE_DIR/Thirdparty/g2o:\$CMAKE_PREFIX_PATH\""

if [ ! -d install/orbslam3_ros2 ]; then
  echo "=== Building ORB_SLAM3_ROS2 (humble) ==="
  colcon build --merge-install --parallel-workers "$NPROC" \
               --cmake-args -DCMAKE_BUILD_TYPE=Release \
                            -DORB_SLAM3_ROOT="$CORE_DIR"
else
  echo " ✔ ORB_SLAM3_ROS2 already built – skipping"
fi

# ── 6. Done ───────────────────────────────────────────────────────
echo -e "\n✅ All components built."
echo "👉 New terminal or 'source ~/.bashrc', then run:"
cat <<'EOF'
ros2 launch orbslam3_ros2 mono_inertial_tello.launch.py \
     vocab:=$HOME/src/tello_ws/libs/ORB_SLAM3_ROS2/Vocabulary/ORBvoc.txt \
     config:=$HOME/tello_ws/config/tello_mono_imu.yaml \
     image_topic:=/camera/camera_raw \
     imu_topic:=/imu
EOF
