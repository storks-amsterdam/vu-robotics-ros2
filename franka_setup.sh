# --- This script already assumes ROS2 is installed and sourced ---
# Setup script for Franka Emika Panda robot with ROS2 (jazzy)

# libfranka dependency
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

sudo apt-get install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio

# Franka ROS2 setup
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws  # not into src

# clone branch jazzy
git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src
vcs import src < src/franka.repos --recursive --skip-existing

rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# use the --symlinks option to reduce disk usage, and facilitate development.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
source install/setup.sh



