#!/bin/bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [1/6] Locale ==="
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== [2/6] ROS2 Humble apt repository ==="
sudo apt update -qq
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -qq

echo "=== [3/6] ROS2 Humble + project dependencies ==="
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-mujoco-ros2-control \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip
# Note: ros-humble-mujoco-ros2-control may need to be built from source if not in apt

echo "=== [4/6] GPU / rendering libraries ==="
# Mesa OpenGL + D3D12 backend (WSL2 GPU passthrough already provides libcuda/libd3d12)
# libgl1-mesa-dri provides the D3D12 gallium driver for OpenGL over WSL2 GPU
sudo apt install -y \
  libglfw3-dev \
  libegl1-mesa-dev \
  libgl1-mesa-dri \
  mesa-utils \
  libglvnd-dev \
  x11-apps   # xeyes/glxgears for quick sanity checks

echo "=== [5/6] MuJoCo Python bindings ==="
pip3 install --user --break-system-packages mujoco

echo "=== [6/6] Build ROS2 workspace ==="
cd "$REPO_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== Setup complete! ==="
echo ""
echo "Add this to your ~/.bashrc (run once):"
echo "  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo "  echo 'export MUJOCO_GL=glfw' >> ~/.bashrc"
echo "  echo '[ -f $REPO_DIR/install/setup.bash ] && source $REPO_DIR/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Then in a new terminal:"
echo "  source install/setup.bash"
echo "  ros2 launch m3pro_mujoco_sim sim.launch.py"
